FROM osrf/ros:humble-desktop

# 기본 툴 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# PyTorch 및 의존성 설치 (CUDA 11.8 호환)
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# numpy 버전을 2.0 미만으로 강제 지정
RUN pip3 install scipy "numpy<2.0" opencv-python matplotlib

# 환경 변수 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws
