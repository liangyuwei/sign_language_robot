#!/bin/bash
echo "export DISPLAY=$DISPLAY" >> /etc/profile
echo "export QT_X11_NO_MITSHM=1" >> /etc/profile
exec "$@"