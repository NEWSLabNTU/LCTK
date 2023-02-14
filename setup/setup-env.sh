script_dir=$(dirname -- $(realpath -- "${BASH_SOURCE[0]}"))
OPENCV_VERSION=4.6.0
CUDA_VERSION=11.3
PYTHON_VERSION=3.8

# rust
export RUST_BACKTRACE=1
export RUST_LOG=info
source "$HOME/.cargo/env"

# pytorch
export LD_LIBRARY_PATH="$HOME/.local/lib/python${PYTHON_VERSION}/site-packages/torch/lib:$LD_LIBRARY_PATH"
export LIBTORCH="$HOME/.local/lib/python${PYTHON_VERSION}/site-packages/torch"
export LIBTORCH_CXX11_ABI=0

# opencv 4.x
export LIBRARY_PATH="/opt/opencv${OPENCV_VERSION}/lib:$LIBRARY_PATH"
export LD_LIBRARY_PATH="/opt/opencv${OPENCV_VERSION}/lib:$LD_LIBRARY_PATH"
export PKG_CONFIG_PATH="/opt/opencv${OPENCV_VERSION}/lib/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/opt/opencv${OPENCV_VERSION}/lib/python3.6/dist-packages:$PYTHONPATH"

# cuda
export PATH="/usr/local/cuda-${CUDA_VERSION}/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-${CUDA_VERSION}/lib64:$LD_LIBRARY_PATH"
export LIBRARY_PATH="/usr/local/cuda-${CUDA_VERSION}/lib64:$LIBRARY_PATH"

# python
export PATH="$HOME/.local/bin:$PATH"

# poetry
export PATH="$HOME/.poetry/bin:$PATH"

# cargo
export PATH="$HOME/.cargo/bin:$PATH"

