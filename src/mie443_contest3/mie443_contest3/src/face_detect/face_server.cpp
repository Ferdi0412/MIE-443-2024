int main( int argc, char **argv ) {
    ros::init( argc, argv, "face_server" );
    ros::NodeHandle nh;

    ros::Rate loop_rate( 10 ); // Cycle rate

    // To publish face count:
    ros::Publisher  face_count_pub = nh.advertise<std_msgs::Int32>( "face_count", 1 );
    std_msgs::Int32 face_count;

    // To publish face detected Bool:
    ros::Publisher face_detected_pub = nh.advertise<std_msgs::Bool>( "face_detected", 1 );
    std_msgs::Bool face_detected;

    /* Face Detector Setup */
    KinectFaceDetector face_detector;

    while ( ros::ok() ) {
        ros::spinOnce();

        // For publish face detected:
        face_detected.data = face_detector.isFaceDetected();
        face_detected_pub.publish( face_detected );

        // For publishing faces counted:
        // face_count.data = face_detector.countFacesDetected();
        // face_count_pub.publish( face_count );

        // Can add visualization stuff here if you want...

        loop_rate.sleep();
    }
}
