#!/bin/bash
EXTRA=''
while getopts ":n" option; do
  case $option in
    n) # Run without cache
      echo "test"
      EXTRA="--no-cache";;
    \?) # Invalid option
      echo "Error: Invalid option"
      exit;;
  esac
done

docker build -t eye_tracked_arm -f ~/ros2_eye_tracked_arm/docker/Dockerfile ~/ros2_eye_tracked_arm/ ${EXTRA}
