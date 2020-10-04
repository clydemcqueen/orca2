### Build the orca2 image
~~~
bash build_orca2.sh
~~~

### Run the simulation

Use rocker to set up an environment and create the container:
~~~
rocker --x11 --nvidia --device=/dev/input/js0 --user orca2_build
~~~

Inside the container:
~~~
cd ros2_ws
source setup.sh
ros2 launch orca_gazebo sim_launch.py
~~~

You'll need several bash sessions inside the container to use `rqt`, `rviz2`, etc.
They can be started with `docker exec`:
~~~
docker ps  # Get container id
docker exec -it CONTAINER_ID bash
~~~
