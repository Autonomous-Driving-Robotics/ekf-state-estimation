docker run \
    --gpus all \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --name=adas \
    -it adas:0.1 \
    bash