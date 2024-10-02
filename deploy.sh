#!/bin/bash

TARGET_USER="cev"
TARGET_IP="mini-uno-v-2"
IMAGE_NAME="mini-deploy"

echo "Building Docker image."
docker build -t ${IMAGE_NAME} -f Dockerfile.prod .

echo "Saving and transferring."
docker save ${IMAGE_NAME} |
    gzip | 
    ssh ${TARGET_USER}@${TARGET_IP} -tt `docker load; docker run -t ${IMAGE_NAME}`
