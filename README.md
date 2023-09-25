# moveit_collision_check

This package checks for collisions while offline with its own model. This is useful especially while checking collisions for situations where collision links are disabled for experiments (allowing moveit to collide links in the simulation, e.g. while testing DawnIK Solver).

## Save your robot model

Run your simulation with **all collisions are enabled.** And then run:

```bash
$ rosrun moveit_collision_check save_current_robot_description.py
```

This will save your robot URDF into the `urdf` folder.

## Start the node

**Use the `/moveit_collision_check/check_collision` service for normal use!**

```bash
$ roslaunch moveit_collision_check start.launch
```

## Demo (with lite6)

The node subscribes to the `joint_states` topic just for debugging purposes. **Use the service for normal use.**

```bash
# Start the node
$ roslaunch moveit_collision_check start.launch robot_name:=lite6

# Start RViz for this demo
$ rosrun rviz rviz -d $(rospack find moveit_collision_check)/rviz.rviz

# Start service caller for this demo
$ rosrun rqt_service_caller rqt_service_caller
```

Using the `rqt_service_caller`, select our service `/moveit_collision_check/check_collision` and set `name` as below:

```
["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
```

Also set `position` as below:

```
[0,0,0,0,0,0]
```

Now **call** the service. You should see `lite6` robot without collision state.

Set `position` as below and **call** the service again:

```
[0,0.3,0,0,0,0]
```

You should see there is a collision and service returned `collision_state` value as True.

## Neural Network

Robot models for NN needs more disabled collisions. Use `generate_acm.py` for this purpose. Run `train_collision_model.py` to train a model.
