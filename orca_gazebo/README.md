# Simulation time in ROS2 Dashing and Gazebo 9

## Setting up simulation time

* load gazebo_ros_init to publish a time messages on the `/clock` topic: `gazebo -s libgazebo_ros_init.so myworld`
* set `use_sim_time: True` in all nodes, this will cause the node to subscribe to `/clock`
* call `node->now()` to get the current time, either wall time or simulation time

Check out the various launch files to see examples.

## Notes, challenges and workarounds

A time message is always published on `/clock` at 10Hz wall time, even if the Gazebo simulation rate is running faster
or slower than 100%.
When you call `now()` on the node, `rclcpp` will extrapolate past the most recent clock message to generate
a unique timestamp.

If the Gazebo simulation rate slows below 1% the `/clock` publish rate starts to slow down.
At a 0.1% simulation rate the rate is 1Hz wall time.
in wall time.

ROS timers use wall time, so the code needs to guard against the case where a wall timer
fires but the simulation time has moved much more or less than expected.
In fact simulation time might be stopped.

The Gazebo camera sensor (and other sensors?) tries to publish at the target frame rate even when the simulation
is slowed down.
Image frames may be duplicated (with duplicate timestamps) when the simulation is slowed.
These duplicates may be generated in bunches, and these bunches may overflow queues.
[This is a known bug](https://bitbucket.org/osrf/gazebo/issues/1966/camera-output-appears-to-depend-on-real),
[with a fix in the works](https://bitbucket.org/osrf/gazebo/pull-requests/2502/make-sure-cameras-fps-is-strictly-applied/diff). 

Workaround: Orca wraps sensor topics in `Monotonic<>` which ignores messages with duplicate timestamps.
It also guarantees that messages always move forward in time so that the rest of the code doesn't
need to worry about negative deltas.