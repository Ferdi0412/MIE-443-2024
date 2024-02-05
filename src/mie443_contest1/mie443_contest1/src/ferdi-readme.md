# Ferdi TODO
Ignore this everyone else - this is such that I don't forget what I decide to do immediately after...

## Team1Robot
I will try to make a robot class for controlling the robot.
| Deadline        | Description                                                   |
| :------:        | :----------                                                   |
| Feb. 2 (Fri)    | ~~Create the initial robot class.~~                           |
| Feb. 2 (Fri)    | ~~Go through class with Parin to ensure sufficient function.~~|
| Feb. 6 (Tues)   | Fix and finish robot class.                                   |
| Feb. 6 (Tues)   | Figure out Open-Space navigation                              |
| Feb. 8 (Thurs)  | Initial revision of codebase (make flag in GIT for rollback). |
| Feb. 15 (Thurs) | Competition day!                                              |

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
- [ ] Implement angleClockwiseTo(double orientation) -> clockwise angle between 2 points (similar to distanceToPoint())
- [ ] Implement angleClockwiseToPoint( double target_x, double target_y );
- [ ] Add some subscription_online methods???
- [ ] Get laser scan from angle x to angle y function (0 as center)
- [ ] Figure out/understand how NaN appears in laser scan data
- [ ] Figure out travel towards opening logic

### Navigate to opening
Implement using several functions the following (maybe also use some form of state-checking), but each should be a function, such that time-tracking can be done in some form of while(1) loop in main(...) for readability.

1. Retrieve some LaserScan data
2. Define a minimum distance and potentially max. distance (threshold) to navigate towards
3. Find region starting from edge with great opening OR region at center that's wide enough with great opening
4. [IFF] region found, navigate towards it
5. [ELSE] turn and try again
6. When moving, use custom loop to:
7. [1] track bumpers
8. [2] track openings
9. [3] check robot is actually moving as expected (not stuck)
9. [4] check for better paths

