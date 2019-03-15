#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
}

void move_left()
{
    drive_robot(0.0, 0.5);
}

void move_right()
{
    drive_robot(0.0, -0.5);
}

void move_forward()
{
    drive_robot(0.5, 0.0);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_count = 0;
    int right_count = 0;
    int middle_count = 0;

    int test = 0;

    for (uint i = 0; i < img.height; i++) {
        for (uint j = 0; j < img.width; j++) {
            int pixel_idx = j + (i * img.width);
            int pixel = img.data[pixel_idx];

            if (test < 10) {
                ROS_INFO_STREAM("j: " + std::to_string(j));
                ROS_INFO_STREAM("i: " + std::to_string(i));
                ROS_INFO_STREAM("pixel_idx: " + std::to_string(pixel_idx));
                test++;
            }

            if (pixel == white_pixel) {
                ROS_INFO_STREAM("FOUND PIXEL");
                uint third_width = img.width / 3;

                if (j < third_width) {
                    left_count++;
                } else if (j > third_width && j < third_width * 2) {
                    middle_count++;
                } else if (j > third_width * 2 && j < img.width) {
                    right_count++;
                }
            }
        }
    }

    if (left_count != 0 || right_count != 0 || middle_count != 0) {
        if (left_count > middle_count && left_count > right_count) {
            move_left();
        } else if (middle_count > left_count && middle_count > right_count) {
            move_forward();
        } else {
            move_right();
        }
    }


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
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
