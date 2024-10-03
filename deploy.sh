#!/bin/bash

TARGET_USER="cev"
TARGET_IP="mini-uno-v-2"
IMAGE_NAME="mini-deploy"

# echo "Building Docker image."
# docker build -t ${IMAGE_NAME} -f Dockerfile.prod .

echo "Saving image..."
docker save ${IMAGE_NAME} -o ${IMAGE_NAME}.tar

echo "Transferring image..."
rsync -avz --progress ${IMAGE_NAME}.tar rsync://${TARGET_USER}@${TARGET_IP}:/cev_home

echo "Loading image on mini car..."
ssh -tt ${TARGET_USER}@${TARGET_IP} "docker image rm ${IMAGE_NAME}; docker load < /home/${TARGET_USER}/${IMAGE_NAME}.tar"

# rm ${IMAGE_NAME}.tar
