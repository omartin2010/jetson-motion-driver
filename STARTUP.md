To start the motion driver, run the vscode command using the docker container start task (see [`tasks.json`](./.vscode/tasks.json)), and the [`launch.json`](./.vscode/launch.json) configuration.

To start the motion driver, use this :
```
docker run --rm -d \
    --name motion-driver \
    --privileged \
    -d \
    --name motiondriver \
    --network host \
    motiondriver:latest
```

This `motiondriver:latest` image the result of the [Dockerfile](./docker/Dockerfile) created in this project.