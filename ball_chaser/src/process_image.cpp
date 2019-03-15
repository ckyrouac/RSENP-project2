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

void stop()
{
    drive_robot(0, 0);
}

void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_count = 0;
    int right_count = 0;
    int middle_count = 0;
    int pixel_size = img.step / img.width;
    
    for (uint row = 0; row < img.height; row++) {
        for (uint col = 0; col < img.width; col++) {
            uint pixel_idx = (col * pixel_size) + (row * img.step);

            if (img.data[pixel_idx] == white_pixel &&
                img.data[pixel_idx + 1] == white_pixel &&
                img.data[pixel_idx + 2] == white_pixel) {

                uint third_width = img.width / 3;

                if (col < third_width) {
                    left_count++;
                } else if (col > third_width && col < third_width * 2) {
                    middle_count++;
                } else if (col > third_width * 2 && col < img.width) {
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
    } else {
        stop();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ros::spin();

    return 0;
}
