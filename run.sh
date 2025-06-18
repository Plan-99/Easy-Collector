docker run -it --rm \
    -v "./src":"/root/src" \
    -v "./catkin_ws":"/root/catkin_ws" \
    -p 5173:5173 \
    --name easy_collector_container easy-collector:latest