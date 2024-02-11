# Ferdi TODO
Ignore this everyone else - this is such that I don't forget what I decide to do immediately after...

## Team1Robot
I will try to make a robot class for controlling the robot.
| Deadline        | Description                                                      |
| :------:        | :----------                                                      |
| Feb. 2 (Fri)    | ~~Create the initial robot class.~~                              |
| Feb. 2 (Fri)    | ~~Go through class with Parin to ensure sufficient function.   ~~|
| Feb. 6 (Tues)   | ~~Fix and finish robot class.                                  ~~|
| Feb. 6 (Tues)   | ~~Figure out Open-Space navigation                             ~~|
| Feb. 8 (Thurs)  | ~~Initial revision of codebase (make flag in GIT for rollback).~~|
| Feb. 15 (Thurs) | Competition day!                                                 |

### TODO:
- [x] Bumper subscription and getters
- [x] Odometry subscription and getters
- [x] Laser scan subscription and getters
- [x] Linear velocity setters and publishers
- [x] Rotational velocity setters and publishers
- [x] Blocking linear travel (with errors on collisions) setters and publishers
- [x] Blocking rotational travel (with errors on collisions) setters and publishers
- [x] Bumper error class
- [x] ~~Setup header-file, and CMakeLists.txt for easier use~~
- [x] Implement angleClockwiseTo(double orientation) -> clockwise angle between 2 points (similar to distanceToPoint())
- [x] Implement angleClockwiseToPoint( double target_x, double target_y );
- [x] ~~Add some subscription_online methods???~~
- [x] ~~Get laser scan from angle x to angle y function (0 as center)~~
- [x] Figure out/understand how NaN appears in laser scan data
- [x] ~~Figure out travel towards opening logic~~
- [ ] Add Mean Squared Error for the linear approximation
