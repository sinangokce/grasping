#include <grasping/current_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_current_listener");
  currentListener current_listener;
  ros::Rate rate(50.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}