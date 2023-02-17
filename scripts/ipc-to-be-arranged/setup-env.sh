OPENCV_VERSION=4.3.0
CUDA_VERSION=10.2
PYTHON_VERSION=3.6

# check vglgroup
if $(groups "$USER" | grep -v -q vglusers); then
    echo "Please add your account to vglusers group to enable CUDA device permission"
fi

# rust
export RUST_BACKTRACE=1
export RUST_LOG=info
export PATH="$HOME/.cargo/bin:$PATH"

# pytorch
export LD_LIBRARY_PATH="$HOME/.local/lib/python${PYTHON_VERSION}/site-packages/torch/lib:$LD_LIBRARY_PATH"

# opencv 4.x
export LIBRARY_PATH="/opt/opencv${OPENCV_VERSION}/lib:$LIBRARY_PATH"
export LD_LIBRARY_PATH="/opt/opencv${OPENCV_VERSION}/lib:$LD_LIBRARY_PATH"
export PKG_CONFIG_PATH="/opt/opencv${OPENCV_VERSION}/lib/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/opt/opencv${OPENCV_VERSION}/lib/python3.6/dist-packages:$PYTHONPATH"

# cuda
export PATH="/usr/local/cuda-${CUDA_VERSION}/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-${CUDA_VERSION}/lib64:$LD_LIBRARY_PATH"
export LIBRARY_PATH="/usr/local/cuda-${CUDA_VERSION}/lib64:$LIBRARY_PATH"
