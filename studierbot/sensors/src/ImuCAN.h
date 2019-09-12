#ifndef _IMUCAN_H_
#define _IMUCAN_H_

#include "SocketCAN.h"
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

/**
 * @class ImuCAN
 * @brief CAN interface for Evocortex IMU board.
 * @author Stefan May
 * @date 06.08.2019
 */
class ImuCAN : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] verbosity verbosity output flag
   */
  ImuCAN(SocketCAN* can, bool verbosity=0);

  /**
   * Destructor
   */
  ~ImuCAN();

protected:

private:

  /**
   * Implementation of inherited method from SocketCANObserver. This class is getting notified by the SocketCAN,
   * as soon as messages of interest arrive (having the desired CAN ID).
   * @param[in] frame CAN frame
   */
  void notify(struct can_frame* frame);

  SocketCAN*       _can;

  can_frame        _cf;

  std::vector<float> _q;

  ros::NodeHandle _n;

  ros::Publisher _pubPoseGyro;
};

#endif /* _<IMUCAN_H_ */
