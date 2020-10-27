# How to use?


### Steps:

1. Navigate to where Dockerfile locates.

2. run ```./slrbot_docker.sh``` to build an image and bring up a container. (Note that a interactive terminal will be brought up, and closing it would lead to the shutdown and removal of the container!!!)

3. Use VSCode Remote - SSH plugin to connect to the container: Ctrl + Shift + P and search for "Connect to Host", then input:
```
$ ssh root@127.0.0.1 -p 2333
```

<strike>
2. Build docker image.
```
$ docker image build --rm --tag <IMAGE-NAME> .
```

3. Run a container, while allowing GUI programs to launch from the container.
```
$ docker run -it --gpus all \
--env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--name <CONTAINER-NAME> <IMAGE-NAME>

$ export containerId=$(docker ps -l -q)

$ xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`

$ docker start $containerId
```

4. Update the Dockerfile used inside sign_language_robot_ws, and use VSCode to Reopen in Container.

5. Modify launch.json to add task...
</strike>

### Testing:

Get a new terminal for the container,
```
$ docker container exec -it <CONTAINER-NAME> bash
```
or simply use the one that ```./slrbot_docker.sh``` brings up.

A simple test is to run ```xarclock```, a very lightweight clock app.

Then run ```roscore``` in one terminal, and ```rosrun rviz rviz``` in the other, and you should see RViz successfully launched in your local machine.
