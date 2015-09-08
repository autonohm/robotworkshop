/**
* @file JoyPanTilt.h
* @autor christian
* @date  10.06.2013
*
*
*/

#ifndef JOYPANTILT_H_
#define JOYPANTILT_H_

#include "ohm_joyToVel/PanTiltCtrl.h"
#include "ohm_joyToVel/SensorHeadCtrl.h"

/**
 * @class   JoyPanTilt
 * @author  Christian Pfitzner
 * @date    2013-06-10
 */
class JoyPanTilt
{
public:
  /**
   * Defautl constructor
   * @param[in]  useVelCtrl
   */
  JoyPanTilt(bool useVelCtrl = true);
  /**
   * Default destructor
   */
  ~JoyPanTilt(void) { }
  /**
   * Function to set limits for paning
   * @param[in]     min   minimum angle in rad
   * @param[in]     max   maximum angle in rad
   */
  void setPanAnglesLimit(const double& min = -1.0, const double& max = 1.0);
  /**
   * Function to set limits for tilting
   * @param[in]     min   minimum angle in rad
   * @param[in]     max   maximum angle in rad
   */
  void setTiltAnglesLimit(const double& min = -0.5, const double& max = 0.5);
  /**
   * Function to set homing position
   * @param[in]   panAngle    pan home angle
   * @param[in]   tiltAngle   tilt home angle
   */
  void setHomePos(const double& panAngle = 0.0, const double& tiltAngle = 0.0);
  void setSpeedFactor(const double& speed) {_speed = speed;}
  /**
   * Function to set mode of panning and tiltig
   */
  void setMode(bool useVelCtrl)  { _useVelCtrl = useVelCtrl; }
  /**
   * Function to get current mode
   * @return  TRUE if velocity mode used
   */
  bool velUsed(void) const { return _useVelCtrl; }
  /**
   * Function to set via velocity
   * @param[in]     panValue      panning value from joy stick
   * @param[in]     tiltValue     tilting value from joy stick
   * @param[out]    camCtrl       message for pan tilt control
   */
  void panTiltVel(const double& panValue, const double& tiltValue, ohm_joyToVel::PanTiltCtrl& camCtrl);
  /**
   * Function to set via position
   * @param[in]     panValue      panning value from joy stick
   * @param[in]     tiltValue     tilting value from joy stick
   * @param[out]    camCtrl       message for pan tilt control
   */
  void panTiltPos(const double& panValue, const double& tiltValue, ohm_joyToVel::PanTiltCtrl& camCtrl);

  /**
   * Function to set via velocity
   * @param[in]     panValue      panning value from joy stick
   * @param[in]     tiltValue     tilting value from joy stick
   * @param[out]    camCtrl       message for pan tilt control
   */
  void panTiltVel(const double& panValue, const double& tiltValue, ohm_joyToVel::SensorHeadCtrl& ctrl);
  /**
   * Function to set via position
   * @param[in]     panValue      panning value from joy stick
   * @param[in]     tiltValue     tilting value from joy stick
   * @param[out]    camCtrl       message for pan tilt control
   */
  void panTiltPos(const double& panValue, const double& tiltValue, ohm_joyToVel::SensorHeadCtrl& ctrl);


  /**
   * Function to go to home position
   * @return  TRUE if possible (position mode)
   */
  bool homePos(void);
  /**
   * Function to fix camera position
   * @param   active    TRUE to activate
   * @return  TRUE if possible (velocity mode)
   */
  bool fixPos(bool button);
   /**
   * Function to look back
   * @param    camCtrl            message for pan tilt control
   * @return
   */
  bool lookBack(void);
private:
  /**
   * Function to limit values for angles
   * @param       angle
   * @param[in]   minTH
   * @param[in]   maxTH
   */
  void limitAngles(double& angle, const double& minTH, const double& maxTH);

  bool    _useVelCtrl;                  //!< TRUE if pan tilt unit is controlled by velocity
  double  _speed;

  // state members
  bool    _cameraFixed;                 //!< TRUE if pan tilt unit is fixed in position
  bool    _homingPressed;               //!< TRUE if homing button pressed
  bool    _backPressed;                 //!< TRUE if looking back set

  struct Angle                          //!< @struct Angle
  {
    double pan;                         //!< panning angle
    double tilt;                        //!< tilting angle
  };
  Angle   _homePos;                     //!< homing position
  Angle   _oldAngle;                    //!< last angle

  struct RangeAngles                    //!< @struct RangeAngles
  {
    struct Pan                          //!< @struct Pan
    {
      double min;                       //!< minimum panning angle
      double max;                       //!< maximum panning angle
    };
    struct Tilt                         //!< @struct Tilt
    {
      double min;                       //!< minimum tilting angle
      double max;                       //!< maximum tilting angle
    };
    Pan   pan;                          //!< member for panning
    Tilt  tilt;                         //!< member for tilting
  };
  RangeAngles _range;                   //!< range of pan titl control
  bool _debug;
};



#endif /* JOYPANTILT_H_ */
