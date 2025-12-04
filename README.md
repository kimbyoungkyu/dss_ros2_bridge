
1. ROS 2 Humble 설치
   1.1 Ubuntu 22.04 기준 로컬 또는 WSL2 환경에서 다음을 실행합니다.
     sudo apt update && sudo apt upgrade -y
     sudo apt install -y curl gnupg lsb-release

   1.2 ROS 2 Humble 설치
     sudo apt install software-properties-common
     sudo add-apt-repository universe
     sudo apt update
     sudo apt update && sudo apt install curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
     sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
     sudo apt update

     sudo apt install ros-humble-desktop
     sudo apt install python3-colcon-common-extensions

   1.3 설치 완료 후 환경 설정:
     echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
     source ~/.bashrc

2. 의존성 설치
  sudo apt update
  sudo apt install libnats-dev
  sudo apt install -y git cmake build-essential pkg-config autoconf automake libtool
  sudo apt install nlohmann-json3-dev
  cd ~
  git clone https://github.com/protocolbuffers/protobuf.git
  cd protobuf
  git checkout v3.15.8
  git submodule update --init --recursive
  ./autogen.sh
  ./configure
  make -j$(nproc)
  sudo make install
  sudo ldconfig
  ldconfig -p | grep protobuf

3. 저장소 Clone
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/kimbyoungkyu/dss_ros2_bridge.git

4. 빌드하기
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install


     
   
    
   
   
   
   
   
   

      
     
   
   
   


SSH 공개키 없이 저장소를 HTTPS 방식으로 Clone 하기
git clone https://github.com/kimbyoungkyu/dss_ros2_bridge.git

