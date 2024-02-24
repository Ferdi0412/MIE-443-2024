# Contest 2
Here we are building files to traverse an unknown environtment, to move and image targets at known locations.

## Contest files
The following files were provided with the contest package:
1. boxes
2. imagePipeline
3. navigation
4. robot_pose
5. contest2.cpp
6. webcam_publisher.cpp

## Header files
Add these to the [include](include) directory.

## Function files
Add these to the [src](src) directory.

## Testing
To test, modify the [src/priv-test.cpp](src/priv-test.cpp) file. This will **NOT** be commited to Git, and so you have free range to do with it as you like.

```shell
rosrun mie443_contest2 priv_test
```

## Gazebo
To open gazerbo, run the following command:

```shell
roslaunch mie443_contest2 turtlebot_world.launch world:=1
```

