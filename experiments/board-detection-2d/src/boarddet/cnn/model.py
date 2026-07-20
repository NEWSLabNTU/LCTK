"""Task 32: the lightweight U-Net board-segmenter.

Input `(B, 3, 32, W)` (see `cnn.data.channels_to_input_array` for the 3
channels' meaning) -> per-pixel board-probability logits `(B, 1, 32, W)`.
The CNN's job is SELECTION (is this pixel part of an isolated, board-shaped
blob?); geometry (Task 33) does POSE from the pixels it selects.

Design (approved, see the Task-32 brief):
  - encoder channels 16 -> 32 -> 64
  - CIRCULAR padding on the width (azimuth) axis only -- azimuth wraps at
    the 0/2*pi seam, so a board straddling column 0 must see the same
    neighborhood a board at any other column sees. The row (elevation)
    axis does NOT wrap (there is no laser above the top channel or below
    the bottom one), so it gets ordinary zero padding. `CircularWidthConv2d`
    below implements this split explicitly: `F.pad(..., mode="circular")`
    with a zero height-pad amount for the width dim, then a *second*
    `F.pad(..., mode="constant")` for the height dim -- NOT a single
    `padding_mode="circular"` conv, which would wrap the height axis too.
  - "gentle" vertical pooling: with only 32 rows, the encoder halves H
    just twice (32 -> 16 -> 8) while W keeps halving a third time
    (-> W/8), so the bottleneck keeps real elevation resolution.
  - dilated bottleneck (dilation 2, then 4) for wide azimuthal context --
    a pixel needs to "see" far enough around it in azimuth to tell
    whether its blob is isolated (board) or continues into a wall
    (clutter), which a local 3x3 receptive field cannot do.
  - decoder mirrors the encoder with skip connections (concat), matching
    upsample factors (width-only, then both) back to (2, 2)+(2, 2)+(1, 2)
    reversed, ending at full (32, W) resolution.
  - 1-channel logit output; `forward` returns LOGITS (no sigmoid) so
    training can use `F.binary_cross_entropy_with_logits` (numerically
    stable); apply `torch.sigmoid` at inference/eval time.

Width must be divisible by 8 (three width-halvings) for the skip
connections to line up exactly; `cnn.data`'s `DEFAULT_AZIMUTH_STEPS` (1800)
and the test suite's reduced width (360) both satisfy this.
"""
from __future__ import annotations

import torch
import torch.nn.functional as F
from torch import nn


class CircularWidthConv2d(nn.Module):
    """`Conv2d` whose "same"-shape padding is CIRCULAR on the width (last)
    dim and ZERO (constant) on the height (second-to-last) dim.

    Implemented as two `F.pad` calls rather than a single
    `padding_mode="circular"` conv: `nn.Conv2d(padding_mode="circular")`
    pads every spatial dim it's given the same way, which would wrap the
    elevation axis too (physically wrong -- there's no laser wraparound
    from the top channel to the bottom one).
    """

    def __init__(self, in_channels: int, out_channels: int,
                kernel_size: int = 3, dilation: int = 1):
        super().__init__()
        self.pad = dilation * (kernel_size - 1) // 2
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size,
                             padding=0, dilation=dilation)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        p = self.pad
        if p:
            x = F.pad(x, (p, p, 0, 0), mode="circular")
            x = F.pad(x, (0, 0, p, p), mode="constant", value=0.0)
        return self.conv(x)


class ConvBlock(nn.Module):
    """Two `CircularWidthConv2d(3x3) -> BatchNorm -> ReLU` layers."""

    def __init__(self, in_channels: int, out_channels: int, dilation: int = 1):
        super().__init__()
        self.net = nn.Sequential(
            CircularWidthConv2d(in_channels, out_channels, 3, dilation),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            CircularWidthConv2d(out_channels, out_channels, 3, dilation),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class BoardUNet(nn.Module):
    """Lightweight U-Net range-image board segmenter.

    `forward(x) -> logits (B, 1, 32, W)`, `x: (B, 3, 32, W)`. `W` must be
    divisible by 8.
    """

    def __init__(self, in_channels: int = 3, base_channels: int = 16):
        super().__init__()
        c1, c2, c3 = base_channels, base_channels * 2, base_channels * 4

        self.enc1 = ConvBlock(in_channels, c1)          # (32, W)   @ c1
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)          # -> (16, W/2)
        self.enc2 = ConvBlock(c1, c2)                    # (16, W/2) @ c2
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)          # -> (8, W/4)
        self.enc3 = ConvBlock(c2, c3)                    # (8, W/4)  @ c3
        self.pool3 = nn.MaxPool2d(kernel_size=(1, 2), stride=(1, 2))  # -> (8, W/8), height untouched

        # dilated bottleneck: wide azimuthal context without shrinking H further
        self.bottleneck = nn.Sequential(
            CircularWidthConv2d(c3, c3, kernel_size=3, dilation=2),
            nn.BatchNorm2d(c3),
            nn.ReLU(inplace=True),
            CircularWidthConv2d(c3, c3, kernel_size=3, dilation=4),
            nn.BatchNorm2d(c3),
            nn.ReLU(inplace=True),
        )

        # decoder: upsample (nearest, so it stays exactly the inverse of the
        # matching MaxPool factor) -> concat skip -> ConvBlock back down to
        # the skip's channel count.
        self.up3 = nn.Upsample(scale_factor=(1, 2), mode="nearest")   # (8, W/8) -> (8, W/4)
        self.dec3 = ConvBlock(c3 + c3, c2)               # concat with enc3 skip (c3)
        self.up2 = nn.Upsample(scale_factor=(2, 2), mode="nearest")   # (8, W/4) -> (16, W/2)
        self.dec2 = ConvBlock(c2 + c2, c1)               # concat with enc2 skip (c2)
        self.up1 = nn.Upsample(scale_factor=(2, 2), mode="nearest")   # (16, W/2) -> (32, W)
        self.dec1 = ConvBlock(c1 + c1, c1)               # concat with enc1 skip (c1)

        self.head = nn.Conv2d(c1, 1, kernel_size=1)      # 1x1, no padding needed

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.shape[-1] % 8 != 0:
            raise ValueError(
                f"width must be divisible by 8 for exact skip alignment, got {x.shape[-1]}"
            )

        s1 = self.enc1(x)
        s2 = self.enc2(self.pool1(s1))
        s3 = self.enc3(self.pool2(s2))
        b = self.bottleneck(self.pool3(s3))

        d3 = self.dec3(torch.cat([self.up3(b), s3], dim=1))
        d2 = self.dec2(torch.cat([self.up2(d3), s2], dim=1))
        d1 = self.dec1(torch.cat([self.up1(d2), s1], dim=1))

        return self.head(d1)


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)
