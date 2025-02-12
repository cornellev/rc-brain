#!/bin/bash
set -e

TARGET_USER=$1
TARGET_CAR=$2
PACKAGES=$3
FOLDER_NAME=rc-brain

BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

if [ -z "${TARGET_CAR}" ]; then
    echo "usage: $0 TARGET_USER TARGET_CAR [PACKAGE1,PACKAGE2,...]"
    echo "please do not add spaces between packages"
    exit 1
fi

printf "${BLUE}Copying files...${NC}\n"
rsync -avzP --stats --exclude .git \
    --rsync-path="mkdir -p /home/$TARGET_USER/ws/src/$FOLDER_NAME && rsync" \
    . $TARGET_USER@$TARGET_CAR:/home/$TARGET_USER/ws/src/$FOLDER_NAME

if [ $? -ne 0 ]; then
    printf "${RED}Failed to copy files.${NC}\n"
    exit 1
fi

printf "${BLUE}Building on car $TARGET_CAR...${NC}\n"
if [ -z "${PACKAGES}" ]; then
    ssh -tt $TARGET_USER@$TARGET_CAR << EOF
source /opt/ros/humble/setup.bash
cd ws
colcon build
exit 0
EOF
else
    ssh -tt $TARGET_USER@$TARGET_CAR << EOF
source /opt/ros/humble/setup.bash
cd ws
colcon build --packages-select autonomy
exit 0
EOF
fi


