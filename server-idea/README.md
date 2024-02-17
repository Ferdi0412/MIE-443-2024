# Server Idea
It should be possible to create a server that we can connect to with our phones/laptops to monitor the robot during operation. This should be able to provide:
1. Live data/topics
2. Live debugging/logger messages
3. Control (for testing ONLY)

## Requirements
This would require our devices to connect to the machine on the robot, which means that we need:
1. [SUDO REQUIRED] NodeJS libraries installed (or some alternate C++ setup... NodeJS would be easier...)
2. [SUDO REQUIRED] Firewall allowing the ports to be accessed through the router
3. [TESTING] For the UofT WiFi routers to not have some mechanism that would prevent us from connecting to a local device.
4. [TESTING] For the required packages to exist for Ubuntu 16.04 and ROS Kinetic

## Test 1
The first test we would need to run, would check that we can use the WiFi routers, and that the required packages exist for Ubuntu 16.04 and ROS Kinetic. This code was derived from an example in the [rosnodejs_examples](https://github.com/RethinkRobotics-opensource/rosnodejs_examples/blob/kinetic-devel/scripts/listener.js) repository.

```JavaScript
const rosnodejs = require('rosnodejs'); // "Import" library

const std_msgs = rosnodejs.require('std_msgs').msg; // Get the std_msgs classes

const nav_msgs = rosnodejs.require('nav_msgs').msg; // Ensure this is available...

function odomCallback ( msg ) {
    Console.log('Data: ' + msg... + '\n');
}

if ( require.main === module ) { // TODO: Check what this if statement checks for...

    /**
     * Set up subscriptions... 
    */
    rosnodejs.initNode('/direct_from_computer').then((rosNode) => {
        let sub = rosNode.subscribe('/odom', nav_msgs.Odometry, odomCallback);
    });
}
```

[NOTE] The above will only work if ALL ports are exposed... Eventually, there should be a 'broker' type node that exposes all of these through a single endpoint using UDP. This could potentially be made using a pseudo-MQTT style program, if one exists/is easy enough to make.... This could be implemented using the [paho-mqtt](https://pypi.org/project/paho-mqtt/) library in conjunction with the [rospy](http://wiki.ros.org/rospy) library...