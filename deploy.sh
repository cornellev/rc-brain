#!/bin/bash

TARGET_USER=cev
TARGET_CAR=$0
IMAGE_NAME=mini-deploy
CONTAINER_NAME=mini-deployed
SERVER_NAME=mini-server
SERVER_PORT=5000
SERVER_USER=cev

echo "Building image..."
docker build --platform=linux/arm64 -t $SERVER_NAME:$SERVER_PORT/$IMAGE_NAME -f Dockerfile.prod .

echo "Pushing to local registry $SERVER_NAME..."
docker push $SERVER_NAME:$SERVER_PORT/$SERVER_USER/$IMAGE_NAME

echo "Pulling down on car $TARGET_CAR and starting..."
ssh $TARGET_USER@$TARGET_CAR << EOF
    docker stop $CONTAINER_NAME &&
    docker container rm $CONTAINER_NAME &&
    docker pull $SERVER_NAME:$SERVER_PORT/$IMAGE_NAME &&
    docker run $IMAGE_NAME --name $CONTAINER_NAME --network=host
EOF
