#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <cstdlib>  // Needed for rand()
#include <ctime>    // Needed to seed random number generator with a time value
#include "tf/LinearMath/Quaternion.h" // Needed to convert rotation ...
#include "tf/LinearMath/Matrix3x3.h"  // ... quaternion into Euler angles
#include "tf/LinearMath/Vector3.h"

struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  Pose static sub(Pose p1, Pose p2){
    Pose p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    p.heading = p1.heading - p2.heading;
    return p;
  }

  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };

// Process incoming pose message for current robot
  void poseEKFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };
};

class RandomWalk {
 public:
  // Construst a new RandomWalk object and hook up this ROS node
  // to the simulated robot's velocity control and laser topics
  RandomWalk(ros::NodeHandle& nh)
      : fsm(FSM_ROTATE),
        rotateStartTime(ros::Time::now()),
        rotateDuration(0.f) {
    // Initialize random time generator
    srand(time(NULL));
    std::cout << "Start routine" << std::endl;

    // Advertise a new publisher for the simulated robot's velocity command
    // topic
    // (the second argument indicates that if multiple command messages are in
    // the queue to be sent, only the last command will be sent)
    commandPub =
        nh.advertise<geometry_msgs::Twist>("/robot_5/mobile_base/commands/velocity", 1);

    std::cout << "Teleop publisher" << std::endl;

    // Subscribe to the simulated robot's laser scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    //laserSub = nh.subscribe("/robot_5/scan", 1, &RandomWalk::commandCallback, this);
      subPose = nh.subscribe("/robot_5/odom", 1, &Pose::poseCallback, &pose);
      subPose_ekf = nh.subscribe("/robot_5/robot_pose_ekf/odom_combined", 1, &Pose::poseEKFCallback, &pose_ekf);

    std::cout << "Laser subscriber" << std::endl;
  };
  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist
        msg;  // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };
  //returns the time to go forward
   double moveStraight(double dist) {
    return dist/FORWARD_SPEED_MPS;
   }
   //returns the time to rotate
   double rotateRads(double rads){
   return rads/ROTATE_SPEED_RADPS;
   }
  
   double rotateAbsRads(double absRads) {
	return (absRads - pose.heading)/ROTATE_SPEED_RADPS;


   }

  // Process the incoming laser scan message
  void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (fsm == FSM_MOVE_FORWARD) {
      // Compute the average range value between MIN_SCAN_ANGLE and
      // MAX_SCAN_ANGLE

      //
      // NOTE: ideally, the following loop should have additional checks to
      // ensure
      // that indices are not out of bounds, by computing:
      //
      // - currAngle = msg->angle_min + msg->angle_increment*currIndex
      //
      // and then ensuring that currAngle <= msg->angle_max
      unsigned int minIndex =
          ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
      unsigned int maxIndex =
          ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
      float closestRange = 100000;  // msg->ranges[minIndex];
      for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex;
           currIndex++) {
        if (msg->ranges[currIndex] < closestRange) {
          closestRange = msg->ranges[currIndex];
        }
      }

      std::cout << "Range: " << closestRange << std::endl;

      // TODO: if range is smaller than PROXIMITY_RANGifE_M, update fsm and
      // rotateStartTime,
      // and also choose a reasonable rotateDuration (keeping in mind of the
      // value
      // of ROTATE_SPEED_RADPS)
      //
      // HINT: you can obtain the current time by calling:
      //
      // - ros::Time::now()
      //
      // HINT: you can set a ros::Duration by calling:
      //
      // - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)
      //
      // HINT: you can generate a random number between 0 and 99 by calling:
      //
      // - rand() % 100
      //
      // see http://www.cplusplus.com/reference/clibrary/cstdlib/rand/ for more
      // details
      /////////////////////// ANSWER CODE BEGIN ///////////////////

      if (closestRange < PROXIMITY_RANGE_M) {
        fsm = FSM_ROTATE;
        rotateStartTime = ros::Time::now();

        // rotate randomly from 45 to 315 degrees taking rotation speed into
        // account

        // time to rotate 180 degrees
        double piTime = M_PI / ROTATE_SPEED_RADPS;
        double angleRange =
            (MAX_ROTATE_ANGLE_DEG - MIN_ROTATE_ANGLE_DEG) * DEG2RAD;

        // Pick a random factor that will get an angle in the acceptable range
        double randomFactor = (angleRange / M_PI) * (rand() / (double)RAND_MAX);

        // Minimum time factor to offset random range value
        double minFactor = MIN_ROTATE_ANGLE_DEG * DEG2RAD / M_PI;

        // Rotate (180 degrees) * (some random factor) to get min and max range
        double dur = piTime * (minFactor + randomFactor);

        rotateDuration = ros::Duration(dur);
      }

      /////////////////////// ANSWER CODE END ///////////////////
    }
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(10);  // Specify the FSM loop rate in Hz

    ros::Time startTime = ros::Time::now();
    rotateStartTime = ros::Time::now();
    double dur = 1 / FORWARD_SPEED_MPS;
    double durR = (M_PI/2)/ROTATE_SPEED_RADPS;
    ros::Duration runTime = ros::Duration(dur);
    rotateDuration = ros::Duration(durR);
    int count = 0;
    Pose startPose, startPoseEKF, pErr, pEkfErr;
    while (ros::ok()) {  // Keep spinning loop until user presses Ctrl+C
      // TODO: Either call:
      //
      // - move(0, ROTATE_SPEED_RADPS); // Rotate right
      //
      // or
      //
      // - move(FORWARD_SPEED_MPS, 0); // Move foward
      //
      // depending on the FSM state; also change the FSM state when appropriate
      /////////////////////// ANSWER CODE BEGIN ///////////////////

      if(count == 10){
          startPose = pose;
          startPoseEKF = pose_ekf;
          std::cout << "Pose: (x,y,heading)     (" << pose.x << ", " << pose.y << ", " << pose.heading << ")" << std::endl;
          std::cout << "Pose_EKF: (x,y,heading) (" << pose_ekf.x << ", " << pose_ekf.y << ", " << pose_ekf.heading << ")" << std::endl;
      }

      switch (fsm) {
        case FSM_STOP:
          move(0,0);
          std::cout << "Results:" << std::endl;
          pErr = Pose::sub(pose, startPose);
          pEkfErr = Pose::sub(pose_ekf, startPoseEKF);
          std::cout << "Pose: (x,y,heading)     (" << pose.x << ", " << pose.y << ", " << pose.heading << ")" << std::endl;
          std::cout << "Pose_EKF: (x,y,heading) (" << pose_ekf.x << ", " << pose_ekf.y << ", " << pose_ekf.heading << ")" << std::endl;
          std::cout << "Pose Distance:    (x,y,heading)     (" << pErr.x << ", " << pErr.y << ", " << pErr.heading << ")" << " Dist: " << sqrt(pErr.x*pErr.x + pErr.y*pErr.y) << std::endl;
          std::cout << "PoseEKF Distance: (x,y,heading)     (" << pEkfErr.x << ", " << pEkfErr.y << ", " << pEkfErr.heading << ")" << " Dist: " << sqrt(pEkfErr.x*pEkfErr.x + pEkfErr.y*pEkfErr.y)<< std::endl;
          return;
          break;

        case FSM_ROTATE:

          if (rotateStartTime + rotateDuration < ros::Time::now()) {
            fsm = FSM_STOP;
          }

          move(0, ROTATE_SPEED_RADPS);
          break;

        case FSM_MOVE_FORWARD:
          move(moveStraight, 0);
          break;
      }

      if(startTime + runTime < ros::Time::now()){
          fsm = FSM_STOP;   
      }

      //std::cout << "Pose: (x,y,heading)     (" << pose.x << ", " << pose.y << ", " << pose.heading << ")" << std::endl;
      //std::cout << "Pose_EKF: (x,y,heading) (" << pose_ekf.x << ", " << pose_ekf.y << ", " << pose_ekf.heading << ")" << std::endl;

      // move(0, ROTATE_SPEED_RADPS);
      /////////////////////// ANSWER CODE END ///////////////////
      ros::spinOnce();  // Need to call this function often to allow ROS to
                        // process incoming messages
      rate.sleep();  // Sleep for the rest of the cycle, to enforce the FSM loop
                     // rate
      count++;
    }
  };

  void translate(double d){
    ros::Rate rate(10);
    ros::Time startTime = ros::Time::now();
    double dur = 1 / FORWARD_SPEED_MPS;
    ros::Duration runTime = ros::Duration(dur);

    while(ros.ok() && startTime + runTime > ros::Time::now()){
      move(FORWARD_SPEED_MPS, 0);
      ros::spinOnce();
      rate.sleep();
    }
  }

  enum FSM { FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STOP };

  // Tunable parameters
  // TODO: tune parameters as you see fit
  const static double MIN_SCAN_ANGLE_RAD = -30.0 / 180 * M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +30.0 / 180 * M_PI;
  const static float PROXIMITY_RANGE_M =
      1;  // Should be smaller than sensor_msgs::LaserScan::range_max
  const static double FORWARD_SPEED_MPS = 0.2;
  const static double ROTATE_SPEED_RADPS = M_PI / 2.0;

  const static double DEG2RAD = M_PI / 180.0;
  const static double MAX_ROTATE_ANGLE_DEG = 315;
  const static double MIN_ROTATE_ANGLE_DEG = 45;

 protected:
  ros::Publisher
      commandPub;  // Publisher to the simulated robot's velocity command topic
//  ros::Subscriber laserSub;  // Subscriber to the simulated robot's laser scan topic
  ros::Subscriber subPose;
  ros::Subscriber subPose_ekf;

  Pose pose;
  Pose pose_ekf;

  enum FSM fsm;  // Finite state machine for the random walk algorithm
  ros::Time rotateStartTime;     // Start time of the rotation
  ros::Duration rotateDuration;  // Duration of the rotation
};

int main(int argc, char** argv) {
  std::cout << "MAIN" << std::endl
            << std::flush;
  std::cerr << "MAIN" << std::endl;
  ros::init(
      argc, argv,
      "robot_5_error_measurement");  // Initiate new ROS node named "random_walk"
  std::cout << "MAIN" << std::endl;
  ros::NodeHandle n;
  RandomWalk walker(n);  // Create new random walk object
  //walker.spin();         // Execute FSM loop

  

  return 0;
};
