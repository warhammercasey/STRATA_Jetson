#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/strata/git/RADAR/STRATA_Jetson/catkin/src/radar"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/strata/git/RADAR/STRATA_Jetson/catkin/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/strata/git/RADAR/STRATA_Jetson/catkin/install/lib/python2.7/dist-packages:/home/strata/git/RADAR/STRATA_Jetson/catkin/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/strata/git/RADAR/STRATA_Jetson/catkin/build" \
    "/usr/bin/python2" \
    "/home/strata/git/RADAR/STRATA_Jetson/catkin/src/radar/setup.py" \
     \
    build --build-base "/home/strata/git/RADAR/STRATA_Jetson/catkin/build/radar" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/strata/git/RADAR/STRATA_Jetson/catkin/install" --install-scripts="/home/strata/git/RADAR/STRATA_Jetson/catkin/install/bin"
