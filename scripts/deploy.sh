#!/bin/bash
set -e

TARGET_USER=$1
TARGET_CAR=$2
FOLDER_NAME=rc-brain

BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

if [ -z "${TARGET_CAR}" ]; then
    echo "usage: $0 TARGET_USER TARGET_CAR"
    exit 1
fi

printf "${BLUE}Copying files...${NC}\n"
rsync -avzP --stats --mkpath . $TARGET_USER@$TARGET_CAR:/home/$TARGET_USER/ws/src/$FOLDER_NAME
if [ $? -ne 0 ]; then
    printf "${RED}Failed to copy files.${NC}\n"
    exit 1
fi

printf "${BLUE}Building on car $TARGET_CAR...${NC}\n"
ssh -tt $TARGET_USER@$TARGET_CAR << EOF
source /opt/ros/humble/setup.bash
cd ws
colcon build
exit 0
EOF

printf "${BLUE}Opening interactive ssh...${NC}\n"
ssh $TARGET_USER@$TARGET_CAR
