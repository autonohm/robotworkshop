// this is a example for a ros publisher

#include <ros/ros.h>
#include <std_msgs/String.h>


int main(int argc, char **argv)
{
   // initialisierung des ros knotens
   ros::init(argc, argv, "my_string_publisher");

   // initialisierung des node handles zur Kommunikation mit dem ROSCORE
   ros::NodeHandle n;

   // Initialisierung für den ROS-Publisher zum veröffentlichen von String-Nachrichten
   ros::Publisher string_pub = n.advertise<std_msgs::String>("hello_world_topic", 1);

   // Initialisierung der Wiederholfrequenz des Programms
   ros::Rate loop_rate(10);

   // Programm wird solange ausgeführt, bis die Funktion ros::ok() false zurückliefert
   while(ros::ok())
   {
      std_msgs::String msg;               // Anlegen der Standard-Nachricht String
      msg.data = "Hello World";           // Füllen der Nachricht
      string_pub.publish(msg);            // Absenden der Nachricht

      // Ausgabe der Nachricht im Terminal
      std::cout << "Publish: " << msg.data << std::endl;

      loop_rate.sleep();                  // Warten bis zum nächsten Durchlauf der Schleife
   }

   return 0;
}
