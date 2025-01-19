#!/bin/bash

# Allow X server connection
xhost +local:docker

# Build and run using docker-compose
docker-compose up --build

# Clean up X server access when done
xhost -local:docker