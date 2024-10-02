#!/bin/bash

# Variables
IMAGE_NAME="mini_cars_dev"
TARGET_USER="cev"
TARGET_IP="mini-uno-v-2"
TARGET_DIR="/home/cev"
CONTAINER_NAME="mini_cars_deploy"
HOST_PORT="4567"
CONTAINER_PORT="4567"

echo "Saving to Docker image."

# Save the Docker image as a tarball
docker save -o ${IMAGE_NAME}.tar ${IMAGE_NAME}

echo "Saved."

# Transfer the tarball to the target device
scp ${IMAGE_NAME}.tar ${TARGET_USER}@${TARGET_IP}:${TARGET_DIR}

# SSH into the target device and load/run the Docker image
ssh -tt ${TARGET_USER}@${TARGET_IP} << 'EOF'
    docker load -i ${TARGET_DIR}/${IMAGE_NAME}.tar
    docker run -d --name ${CONTAINER_NAME} -p ${HOST_PORT}:${CONTAINER_PORT} ${IMAGE_NAME}
EOF

# Clean up tar file on local machine
rm ${IMAGE_NAME}.tar
