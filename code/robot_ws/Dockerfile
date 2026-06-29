FROM gongdol2robot/auto-dump-bot:humble

SHELL ["/bin/bash", "-c"]

# ROS 환경 설정
# 도메인 아이디는 각자 상황에 맞게 설정
ENV ROS_DISTRO=humble \
    ROS_DOMAIN_ID=83 \
    ROS_LOCALHOST_ONLY=0 \
    QT_X11_NO_MITSHM=1 \
    BUILD_ON_START=1

WORKDIR /root/ros2_ws/auto-dump-bot/code/robot_ws

# 저장소의 최신 로봇 패키지를 이미지에 포함한다.
COPY code/robot_ws/src ./src
COPY code/robot_ws/docker/entrypoint.sh /ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh && /ros_entrypoint.sh true

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
