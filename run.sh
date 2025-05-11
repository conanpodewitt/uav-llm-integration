#!/bin/bash

########################################
# Requirements
########################################
# Ensure the script is executed on a Linux machine
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Error: This script can only be executed on a Linux machine. Aborting..."
    exit 1
fi

# Define Docker image and container names
IMAGE_NAME="uav-llm-integration"
CONTAINER_NAME="uav-llm-integration-container"

# Define environment configuration files
CONF_FILE="conf/.env.conf"
ENV_FILE=".env"

# Ensure the required environment files exist
if [ ! -f "$ENV_FILE" ]; then
    echo "Error: Missing .env file (contains LLM_API_KEY). Aborting..."
    exit 1
fi

if [ ! -f "$CONF_FILE" ]; then
    echo "Error: Missing conf/.env.conf file (contains configuration variables). Aborting..."
    exit 1
fi

# Check if a joystick is connected
if [ ! -e "/dev/input/js0" ]; then
    echo "Error: No joystick detected. Please connect a controller before launching. Aborting..."
    exit 1
fi

########################################
# Docker Setup
########################################
# Allow Docker access to the X server for GUI applications
xhost +local:docker

# Remove any existing container with the same name
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f $CONTAINER_NAME
fi

# Remove any existing image with the same name
if [ "$(docker images -q $IMAGE_NAME)" ]; then
    echo "Removing existing image: $IMAGE_NAME"
    docker rmi -f $IMAGE_NAME
fi

# Load configuration variables from the conf/.env.conf file
export $(grep -v '^#' $CONF_FILE | xargs)

# Load API key and other environment variables from the .env file
export $(grep -v '^#' $ENV_FILE | xargs)

########################################
# Build the Docker image
########################################
echo "Building Docker image: $IMAGE_NAME"
docker build \
    --build-arg SAFETY_STOP_DISTANCE=${SAFETY_STOP_DISTANCE} \
    --build-arg MAX_FORWARD_SPEED=${MAX_FORWARD_SPEED} \
    --build-arg MAX_REVERSE_SPEED=${MAX_REVERSE_SPEED} \
    --build-arg MAX_TURN_LEFT_SPEED=${MAX_TURN_LEFT_SPEED} \
    --build-arg MAX_TURN_RIGHT_SPEED=${MAX_TURN_RIGHT_SPEED} \
    --build-arg AREA_THRESHOLD=${AREA_THRESHOLD} \
    --build-arg AREA_MATCH_TOL=${AREA_MATCH_TOL} \
    --build-arg MAX_TRACKED=${MAX_TRACKED} \
    --build-arg LLM_URL=${LLM_URL} \
    --build-arg LLM_MODEL=${LLM_MODEL} \
    --build-arg LLM_TEMPERATURE=${LLM_TEMPERATURE} \
    --build-arg LLM_MAX_RETRIES=${LLM_MAX_RETRIES} \
    --build-arg TIME_DIFF_THRESHOLD=${TIME_DIFF_THRESHOLD} \
    --build-arg SYSTEM_INTERVAL=${SYSTEM_INTERVAL} \
    --build-arg POISON_RATE=${POISON_RATE} \
    --build-arg MIRAGE_RATE=${MIRAGE_RATE} \
    -t $IMAGE_NAME .

########################################
# Run the Docker container
########################################
# Detect serial device to decide on video+serial support
if [ -e "/dev/ttyUSB0" ]; then
    echo "Serial device detected. Running Docker container with serial and video support..."
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --env-file $ENV_FILE \
        -e DISPLAY=$DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e NO_AT_BRIDGE=1 \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --device /dev/ttyUSB0:/dev/ttyUSB0 \
        --device /dev/video0:/dev/video0 \
        --device /dev/input:/dev/input \
        $IMAGE_NAME
else
    echo "No serial device detected. Running Docker container without serial and video support..."
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --env-file $ENV_FILE \
        -e DISPLAY=$DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e NO_AT_BRIDGE=1 \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --device /dev/input:/dev/input \
        $IMAGE_NAME
fi

########################################
# Cleanup
########################################
# Reset X server access after container exits
xhost -local:docker