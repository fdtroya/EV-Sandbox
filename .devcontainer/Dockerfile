# 1. Define the build argument
ARG BASE_IMAGE=althack/ros2:jazzy-full
FROM ${BASE_IMAGE}

# 2. Switch to root
USER root

# 3. Network Repository Installation (Official NVIDIA Method)
# This installs the keyring which automatically configures the repo and keys
RUN apt-get update && apt-get install -y wget && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb

# 4. Install CUDA Toolkit and System Utils
# Now that the keyring is installed, 'apt' will find the packages easily.
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-toolkit-12-6 \
    nvidia-utils-535 \
    ros-jazzy-tf-transformations \
    ros-jazzy-plotjuggler-ros \
    python3-pip \
    mesa-utils \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# 5. Set CUDA Environment Variables
# Pathing is usually /usr/local/cuda-12.6 for the latest Noble repo
ENV PATH=/usr/local/cuda-12.6/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# 6. Install PyTorch & Math Libraries
RUN pip3 install --no-cache-dir --break-system-packages \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124 && \
    pip3 install --no-cache-dir --break-system-packages \
    numpy \
    matplotlib \
    scipy \
    cupy-cuda12x \
    numba \
    transforms3d \
    pandas  \
    ipykernel \
    casadi \
    matplotlib-inline==0.1.7

# 7. Switch back to the ROS user
USER ros
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc