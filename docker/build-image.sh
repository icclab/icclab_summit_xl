TAG=${DOCKER_IMAGE_TAG:-"latest"}
sudo docker build -t="icclab/turtlebot:${TAG}" .
