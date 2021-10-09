xhost +local:`docker inspect --format='{{ .Config.Hostname }}' eye_tracked_arm`
docker stop eye_tracked_arm
docker rm eye_tracked_arm
docker run -it \
    --name="eye_tracked_arm" \
    -v "$EYE_TRACKED_ROOT/robot_ws/src:/workspace/eye_tracked_arm/robot_ws/src/" \
    -v "$EYE_TRACKED_ROOT/package_ws/src:/workspace/eye_tracked_arm/package_ws/src/" \
    --env="DISPLAY" \
    --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    --privileged \
    --runtime=nvidia \
    eye_tracked_arm