#include "ros/ros.h"

#include "joytoctrl.h"
#include "param.h"

int main(int argc, char** argv){
  ros::init(argc,argv,NODE_NAME);
  JoyToCtrl joy_to_ctrl;

  ros::Rate loop(L_RATE);

  while(ros::ok()){
    joy_to_ctrl.publish();

    ros::spinOnce();
    loop.sleep();
  }

  return(0);
}




