#!/bin/bash

TARGET_USER=cev
TARGET_CAR=$1
IMAGE_NAME=mini-deploy
CONTAINER_NAME=mini-deployed
SERVER_NAME=mini-server
SERVER_PORT=5000
SERVER_USER=cev
IMAGE_TAG=$SERVER_NAME:$SERVER_PORT/$SERVER_USER/$IMAGE_NAME

if [ -z "${TARGET_CAR}" ]; then
    echo "usage: $0 TARGET_CAR"
    exit 1
fi

echo "Building image..."
docker build --platform=linux/arm64 -t $IMAGE_TAG -f Dockerfile.deploy .
if [ $? -ne 0 ]; then
    echo "Failed to build image."
     exit 1
fi

echo "Pushing to local registry $SERVER_NAME..."
docker push $IMAGE_TAG 
if [ $? -ne 0 ]; then
    echo "Failed to push to registry."
    exit 1
fi

echo "Pulling down on car $TARGET_CAR and starting..."
ssh -tt $TARGET_USER@$TARGET_CAR << EOF
    docker stop $CONTAINER_NAME 2>/dev/null || true
    docker container rm $CONTAINER_NAME 2>/dev/null || true
    docker pull $IMAGE_TAG && docker run $IMAGE_TAG --name $CONTAINER_NAME --network=host
    exit 0
EOF
