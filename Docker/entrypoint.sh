#!/bin/bash
# /etc/profile would be run before **login shell**, which can be used to set environment variables
echo "export DISPLAY=$DISPLAY" >> /etc/profile
echo "export QT_X11_NO_MITSHM=1" >> /etc/profile
exec "$@"