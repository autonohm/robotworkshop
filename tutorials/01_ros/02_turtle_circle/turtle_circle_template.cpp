#include "ros/ros.h"

#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
  /**
   * Name des eigenen Knotens
   */
  char node[64];
  sprintf(node, "%s%s", argv[1], "_node");
  printf("Neuer Knoten: %s\n", node);

  /**
   * Name des publishers
   */
  char pub[64];
  sprintf(pub, "%s/%s", argv[1], "command_velocity");
  printf("Publisher: %s\n", pub);

  /**
   * Implementieren Sie hier den Aufruf des Dienstes spawn
   */


  /**
   * Implementieren Sie hier das Versenden von Nachrichten
   * zur Steuerung der Schildkröte
   */


  return 0;
}

