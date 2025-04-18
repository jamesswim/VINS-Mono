#!/bin/bash
trap : SIGTERM SIGINT

function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 LAUNCH_FILE" >&2
  exit 1
fi


source /opt/ros/noetic/setup.bash  # 新增: 確保設置ROS環境
source /root/catkin_ws/devel/setup.bash  # 新增: 確保設置工作空間環境

roscore &
ROSCORE_PID=$!
sleep 3

rviz -d ../config/vins_rviz_config.rviz &
RVIZ_PID=$!

VINS_MONO_DIR=$(abspath "..")

docker run \
  -it \
  --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${VINS_MONO_DIR}:/root/catkin_ws/src/VINS-Mono/ \
  vins-mono \
  /bin/bash -c "cd /root/catkin_ws/; \
  source /opt/ros/noetic/setup.bash; \
  catkin config --env-cache --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release; \
  catkin build; \
  source devel/setup.bash; \
  roslaunch vins_estimator ${1}"

wait $ROSCORE_PID
wait $RVIZ_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
fi
