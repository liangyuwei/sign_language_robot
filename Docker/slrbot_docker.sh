### Note that docker should have root permission.

# Arguments
IMAGE_NAME="liangyuwei:slrbot_image"
CONTAINER_NAME="slrbot"
LOCAL_PROJ_PATH=`cd ../../ && pwd`
CONTAINER_PROJ_PATH='/root/vscode-workspace/sign_language_robot_ws'

# Build an image(note that this would remove the currently existing image that has the same name!!!)
docker image build --rm --tag ${IMAGE_NAME} .

# Run the image (note that --rm specifies that the container be removed as soon as it's stopped!!!) in detached mode
# forward local port 2333 to port 4399 of the container 
# link local code project folder to container (so that it can be opened after ssh connected)
# set $DISPLAY environment variable in container for displaying GUIs (note that this env var won't be set when connecting to container via SSH, which creates a new shell session without this env var set)
docker run -id --rm --gpus all \
-p 2333:4399 \
--volume ${LOCAL_PROJ_PATH}:${CONTAINER_PROJ_PATH} \
--env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--name ${CONTAINER_NAME} ${IMAGE_NAME}

# Set up Forwarding SSH in local machine (edit local file)
#msg="ForwardX11 yes"
#if cat ~/.ssh/config | grep "$msg" > /dev/null
#then
#echo "Already set up Forwarding X11"
#else 
#echo $msg >> ~/.ssh/config
#fi

# Set up graphics
export containerId=$(docker ps -l -q)
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`

# Start the docker
docker start $containerId

# Enter the docker
docker container exec -it ${CONTAINER_NAME} bash

# Stop the docker (it would also be removed after stopped, since we use --rm for docker run)
docker container stop ${CONTAINER_NAME}



