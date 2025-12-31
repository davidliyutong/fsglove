#!/bin/bash
CONFIG_DIR=/config
if [ ! -f $CONFIG_DIR/config.yaml ]; then
    echo "creating config.yaml"
    ./rfmocap-app init --print > "$CONFIG_DIR/config.yaml"
    cat "$CONFIG_DIR/config.yaml"
fi

while true; do
    ./rfmocap-app serve --config "$CONFIG_DIR/config.yaml"
    echo "rfmocap-app exited with code $?, restarting in 5 seconds, press ctrl+c to exit"
    sleep 5
done
