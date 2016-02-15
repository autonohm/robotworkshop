// this is an example for a ros subscriber

                                              
#include <ros/ros.h>
#include <std_msgs/String.h>

void stringCallback(const std_msgs::String& msg)
{
   // Inhalt der Nachricht im Terminal ausgeben
   std::cout << "Nachricht empfangen: " << msg.data << std::endl;
}


int main(int argc, char **argv)
{
   // initialisierung des ros knotens
   ros::init(argc, argv, "my_string_subscriber");

   // initialisierung des node handles zur Kommunikation mit dem ROSCORE
   ros::NodeHandle n;

   // Initialisierung für den ROS-Publisher zum veröffentlichen von String-Nachrichten
   ros::Subscriber string_sb = n.subscribe("hello_world_topic", 1, stringCallback);


   // Aufruf für Endlosschleife
   ros::spin();

   return 0;
}
