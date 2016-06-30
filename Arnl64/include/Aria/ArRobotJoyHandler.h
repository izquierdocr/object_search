#ifndef ARROBOTJOYHANDLER_H
#define ARROBOTJOYHANDLER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

class ArRobot;
class ArRobotPacket;

/// Interfaces to a joystick on the robot's microcontroller
/** 
    This is largely meant to be about the same as the normal joy
    handler but gets the data back from the robot about the joystick,
    but this sameness is why it reports things as it does.

    Also note that x is usually rotational velocity (since it right/left),
    whereas Y is translational (since it is up/down).

  When created, this class requests continuous joystick packets from the robot if already
  connected, or if not, requests joystick packets upon robot connection.  In
  requests that the joystick data be stopped upon normal robot disconnection, ARIA
  program exit, or when this object is destroyed.

  @ingroup OptionalClasses
**/
class ArRobotJoyHandler
{
 public:
  /// Constructor
  AREXPORT ArRobotJoyHandler(ArRobot *robot);
  /// Destructor
  AREXPORT ~ArRobotJoyHandler();
  /// Gets the adjusted reading, as floats
  AREXPORT void getDoubles(double *x, double *y, double *z);
  /// Gets the first button 
  bool getButton1(void) { return myButton1; }
  /// Gets the second button 
  bool getButton2(void) { return myButton2; }
  /// Gets the time we last got information back
  AREXPORT ArTime getDataReceivedTime(void) { return myDataReceived; }
  /// If we've ever gotten a packet back
  AREXPORT bool gotData(void) { return myGotData; }
  /// Adds to a section in a config
  AREXPORT void addToConfig(ArConfig *config, const char *section);
  /// Gets the X value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawX(void) { return myRawX; }
  /// Gets the Y value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawY(void) { return myRawY; }
  /// Gets the throttle value (only use for information, or with the robot locked, getDoubles is preferred)
  int getRawThrottle(void) { return myRawThrottle; }

 protected:
  AREXPORT bool handleJoystickPacket(ArRobotPacket *packet);
  AREXPORT void connectCallback(void);

  ArRobot *myRobot;
  ArTime myDataReceived;
  bool myButton1;
  bool myButton2;
  double myJoyX;
  double myJoyY;
  double myThrottle;
  bool myGotData;

  int myJoyXCenter;
  int myJoyYCenter;

  int myRawX;
  int myRawY;
  int myRawThrottle;

  ArTime myStarted;
  ArRetFunctor1C<bool, ArRobotJoyHandler,
      ArRobotPacket *> myHandleJoystickPacketCB;
  ArFunctorC<ArRobotJoyHandler> myConnectCB;

  ArFunctorC<ArRobotJoyHandler> myStopPacketsCB;

  void stopPackets();

};


#endif // ARJOYHANDLER_H

