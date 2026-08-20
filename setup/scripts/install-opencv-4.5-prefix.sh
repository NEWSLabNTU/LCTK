#!/usr/bin/env bash
# Side-install OpenCV 4.5.4 headers into a private prefix, without touching
# any system package.
#
# Why: this repo needs OpenCV <= 4.6 (the aruco module moved into objdetect in
# 4.7 and estimate_pose_single_markers was removed), and ROS Humble's cv_bridge
# links libopencv_*.so.4.5d. On JetPack 6 the NVIDIA libopencv-dev 4.8.0 package
# owns /usr/include/opencv4, so Ubuntu's libopencv-*-dev cannot be installed
# alongside it - apt would swap one for the other and drop NVIDIA's CUDA build.
#
# What this does instead: download the Ubuntu 4.5.4 -dev debs and unpack (not
# install) their headers into $PREFIX. $PREFIX/lib holds only symlinks to the
# 4.5.4 runtime that is already installed system-wide, so nothing is duplicated
# and dpkg never sees this tree. .envrc points the opencv crate at $PREFIX.
#
# Undo: rm -rf "$PREFIX"

set -euo pipefail

PREFIX="${LCTK_OPENCV_PREFIX:-$HOME/opt/opencv-4.5.4}"
OPENCV_VERSION="${LCTK_OPENCV_DEB_VERSION:-4.5.4+dfsg-9ubuntu4}"
MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
SYS_LIBDIR="/usr/lib/$MULTIARCH"

MODULES=(
    core imgproc imgcodecs calib3d features2d flann highgui objdetect
    videoio dnn ml photo video shape stitching superres videostab viz contrib
)

echo "Installing OpenCV $OPENCV_VERSION headers into $PREFIX"

# The runtime must already be present - we only ever symlink to it.
if [ ! -e "$SYS_LIBDIR/libopencv_core.so.4.5d" ]; then
    echo "Error: OpenCV 4.5.4 runtime not found at $SYS_LIBDIR/libopencv_core.so.4.5d" >&2
    echo "  install it with: sudo apt-get install -y libopencv-core4.5d libopencv-contrib4.5d" >&2
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pkgs=("libopencv-dev=$OPENCV_VERSION")
for m in "${MODULES[@]}"; do
    pkgs+=("libopencv-$m-dev=$OPENCV_VERSION")
done

(cd "$WORK" && apt-get download "${pkgs[@]}")

for deb in "$WORK"/*.deb; do
    dpkg-deb -x "$deb" "$WORK/extract"
done

rm -rf "$PREFIX"
mkdir -p "$PREFIX/lib"
cp -a "$WORK/extract/usr/include" "$PREFIX/include"

# The debs ship libopencv_X.so -> libopencv_X.so.4.5d dev symlinks. Copy those,
# then resolve their second hop against the installed runtime.
cp -a "$WORK/extract/usr/lib/$MULTIARCH/"*.so "$PREFIX/lib/"
for so in "$PREFIX"/lib/*.so; do
    soname="$(readlink "$so")"
    ln -sfn "$SYS_LIBDIR/$soname" "$PREFIX/lib/$soname"
done

dangling="$(find "$PREFIX/lib" -xtype l)"
if [ -n "$dangling" ]; then
    echo "Error: dangling symlinks in $PREFIX/lib:" >&2
    echo "$dangling" >&2
    exit 1
fi

version_h="$PREFIX/include/opencv4/opencv2/core/version.hpp"
minor="$(awk '/#define CV_VERSION_MINOR/ {print $3}' "$version_h")"
if [ "$minor" -gt 6 ]; then
    echo "Error: prefix holds OpenCV 4.$minor, but this repo needs <= 4.6" >&2
    exit 1
fi

echo "OpenCV 4.5.4 prefix ready at $PREFIX"
echo "Run 'direnv allow' (or re-source .envrc), then 'just build'."
