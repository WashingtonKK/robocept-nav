# Robocept Nav — TODO

## Obstacle Avoidance
- [ ] Test in Gazebo simulation with test world obstacles
- [ ] Tune `min_obstacle_distance` (0.35m) — may need to be larger for faster speeds
- [ ] Tune `slowdown_distance` (0.8m) — depends on robot stopping distance
- [ ] Add side obstacle detection (currently only checks front sector)
- [ ] Consider adding depth camera data for obstacles not visible to LiDAR (e.g., table legs below scan plane)

## Waypoint Navigation
- [ ] Add a ROS 2 service or action for sending goal poses
- [ ] Add a topic subscriber for `geometry_msgs/PoseStamped` goals (compatible with RViz "2D Nav Goal")
- [ ] Implement waypoint queue management (add, clear, skip)
- [ ] Test goal-reaching accuracy and tune `goal_tolerance_xy`

## Integration
- [ ] Test full pipeline: waypoint_nav → obstacle_avoider → base_driver
- [ ] Verify obstacle avoidance overrides waypoint commands correctly
- [ ] Test with teleop through obstacle_avoider (remap teleop output to `/nav_cmd_vel`)

## Future
- [ ] Evaluate Nav2 (`ros-humble-navigation2`) for production navigation with costmaps + AMCL
- [ ] Add path planning (A*, RRT) for complex environments
- [ ] Add recovery behaviors (back up, spin) when stuck
- [ ] Integrate AI detections — stop/avoid detected objects that LiDAR can't see
