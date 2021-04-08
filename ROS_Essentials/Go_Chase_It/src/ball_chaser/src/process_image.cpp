#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
   ROS_INFO_STREAM("Driving the robot");
    // Request the robot velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    // Call the command_robot service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera.

    const int WHITE_PIXEL = 255;
    enum BALL_POSITION_TP {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    };
    BALL_POSITION_TP ball_position = NONE;
    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i+=3) {
        // Compare RGB values
        if (img.data[i] == WHITE_PIXEL
                && img.data[i+1] == WHITE_PIXEL
                && img.data[i+2] == WHITE_PIXEL) {
            int n_column = i % img.width;
            if (n_column < (img.width * 1/3)) {
                ball_position = LEFT;
            } else if (n_column > (img.width * 2/3)) {
                ball_position = RIGHT;
            } else {
                ball_position = MIDDLE;
            }
            break;
        }
    }
    // Drive the robot based on the ball position
    switch (ball_position)
    {
    case LEFT:
        //ROS_INFO("LEFT");
        drive_robot(0.5, 1.0);
        break;
    case MIDDLE:
        //ROS_INFO("MIDDLE");
        drive_robot(0.5, 0.0);
        break;
    case RIGHT:
        //ROS_INFO("RIGHT");
        drive_robot(0.5, -1.0);
        break;
    default:
        //ROS_INFO("NONE");
        drive_robot(0.0, 0.0);
        break;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}