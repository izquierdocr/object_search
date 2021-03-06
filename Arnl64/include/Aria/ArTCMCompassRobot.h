#ifndef ARTCMCOMPASSROBOT_H
#define ARTCMCOMPASSROBOT_H

#include "ariaUtil.h"
#include "ArCommands.h"
#include "ArFunctor.h"
#include "ArRobot.h"
#include "ArTCM2.h"

/** Interface to a TCM 2/2.5/2.6 3-axis compass through the robot microcontroller.
 *  When a Pioneer robot is equipped with a TCM compass, it is typically connected
 *  to the robot microcontroller, which returns compass information in 
 *  compass data packets upon request (set compass mode/type to 2 or 3 in
 *  firmware configuration; mode 1 (data in SIP) is not supported by ARIA).  This class communicates
 *  with the robot microcontroller to configure the compass and recieve data
 *  from it. 
 *
*/
class ArTCMCompassRobot : public virtual ArTCM2
{
public:

  AREXPORT ArTCMCompassRobot(ArRobot *robot);
  AREXPORT virtual ~ArTCMCompassRobot();

  virtual void commandOff(void) { myRobot->comInt(ArCommands::TCM2, 0); }
  virtual void commandJustCompass(void) { myRobot->comInt(ArCommands::TCM2, 1); }
  virtual void commandOnePacket(void) { myRobot->comInt(ArCommands::TCM2, 2); }
  virtual void commandContinuousPackets(void) { myRobot->comInt(ArCommands::TCM2, 3); }
  virtual void commandUserCalibration(void) { myRobot->comInt(ArCommands::TCM2, 4); }
  virtual void commandAutoCalibration(void) { myRobot->comInt(ArCommands::TCM2, 5); }
  virtual void commandStopCalibration(void) { myRobot->comInt(ArCommands::TCM2, 6); }
  virtual void commandSoftReset(void) { myRobot->comInt(ArCommands::TCM2, 7); }

private:  
  ArRobot *myRobot;
  ArRetFunctor1C<bool, ArTCMCompassRobot, ArRobotPacket*> myPacketHandlerCB;
  bool packetHandler(ArRobotPacket *packet);
};


#endif 
