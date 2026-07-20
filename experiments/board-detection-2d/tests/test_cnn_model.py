"""Tests for the Task 32 U-Net (`boarddet.cnn.model`): forward shape/dtype,
parameter budget, and the circular-width-padding property (azimuth-roll
equivariance) that's the whole point of `CircularWidthConv2d`."""
from __future__ import annotations

import pytest
import torch

from boarddet.cnn.model import BoardUNet, CircularWidthConv2d, count_parameters

WIDTH = 64  # small, divisible by 8 (three width-halvings) -- keeps tests fast


def test_forward_shape_and_finite():
    model = BoardUNet()
    x = torch.randn(2, 3, 32, WIDTH)
    logits = model(x)
    assert logits.shape == (2, 1, 32, WIDTH)
    assert torch.isfinite(logits).all()


def test_param_count_under_budget():
    model = BoardUNet()
    n = count_parameters(model)
    assert n < 2_000_000, f"expected <~2M params (target <~1M), got {n}"


def test_width_must_be_divisible_by_eight():
    model = BoardUNet()
    x = torch.randn(1, 3, 32, 63)
    with pytest.raises(ValueError):
        model(x)


def test_circular_width_conv_wraps_the_seam():
    """A single circular-width conv layer must be equivariant to an
    arbitrary azimuth roll (no pooling involved, so this holds for ANY
    shift, unlike the whole-network test below which needs a
    downsample-factor-aligned shift)."""
    torch.manual_seed(0)
    conv = CircularWidthConv2d(3, 5, kernel_size=3, dilation=1)
    conv.eval()
    x = torch.randn(1, 3, 32, WIDTH)
    shift = 17
    with torch.no_grad():
        y = conv(x)
        y_of_rolled = conv(torch.roll(x, shifts=shift, dims=-1))
    expected = torch.roll(y, shifts=shift, dims=-1)
    assert torch.allclose(y_of_rolled, expected, atol=1e-5)


def test_full_network_translation_equivariant_at_downsample_aligned_shift():
    """`BoardUNet` pools width by a total factor of 8 (2*2*2 across the
    three encoder stages). A circular roll of the INPUT by a multiple of
    8 keeps every pooling window's column grouping intact (mod the
    circular wrap), so the whole network -- not just one conv -- must
    produce the correspondingly-rolled output. `model.eval()` avoids any
    BatchNorm batch-statistics noise (its running-stats affine transform
    in eval mode is a fixed per-channel elementwise op, which trivially
    commutes with a spatial permutation)."""
    torch.manual_seed(0)
    model = BoardUNet()
    model.eval()
    x = torch.randn(1, 3, 32, WIDTH)
    shift = 8 * 3  # multiple of the total width-downsample factor (8)
    with torch.no_grad():
        y = model(x)
        y_of_rolled = model(torch.roll(x, shifts=shift, dims=-1))
    expected = torch.roll(y, shifts=shift, dims=-1)
    assert torch.allclose(y_of_rolled, expected, atol=1e-4)
