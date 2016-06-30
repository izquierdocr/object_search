#ifndef ARROBOTPACKETSENDER_H
#define ARROBOTPACKETSENDER_H

#include "ariaTypedefs.h"
#include "ArRobotPacket.h"

class ArDeviceConnection;

/// Given a device connection this sends commands through it to the robot

class ArRobotPacketSender
{
public:
  /// Constructor without an already assigned device connection
  AREXPORT ArRobotPacketSender(unsigned char sync1 = 0xfa,
			       unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection
  AREXPORT ArRobotPacketSender(ArDeviceConnection *deviceConnection,
			       unsigned char sync1 = 0xfa,
			       unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection and tracking
  AREXPORT ArRobotPacketSender(ArDeviceConnection *deviceConnection,
			       unsigned char sync1,
			       unsigned char sync2,
						bool tracking,
						const char *trackingLogName);
  /// Destructor
  AREXPORT virtual ~ArRobotPacketSender();

  /// Sends a command to the robot with no arguments
  AREXPORT bool com(unsigned char command);
  /// Sends a command to the robot with an int for argument
  AREXPORT bool comInt(unsigned char command, short int argument);
  /// Sends a command to the robot with two bytes for argument
  AREXPORT bool com2Bytes(unsigned char command, char high, char low);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStr(unsigned char command, const char *argument);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStrN(unsigned char command, const char *str, int size);
  /// Sends a command containing exactly the data in the given buffer as argument
  AREXPORT bool comDataN(unsigned char command, const char *data, int size);
  /// Sends a ArRobotPacket
  AREXPORT bool sendPacket(ArRobotPacket *packet);
  
  /// Sets the device this instance sends commands to
  AREXPORT void setDeviceConnection(ArDeviceConnection *deviceConnection);
  /// Gets the device this instance sends commands to
  AREXPORT ArDeviceConnection *getDeviceConnection(void);

  /// Sets the callback that gets called with the finalized version of
  /// every packet set... this is ONLY for very internal very
  /// specialized use
  AREXPORT void setPacketSentCallback(ArFunctor1<ArRobotPacket *> *functor);

  void setTracking(bool v = true)
  {
    myTracking = v;
  }
  void setTrackingLogName(const char *n)
  {
    myTrackingLogName = n;
  }
protected:
  bool connValid(void);
  ArDeviceConnection * myDeviceConn;
  ArRobotPacket myPacket;

	bool myTracking;
	std::string myTrackingLogName;

  ArMutex mySendingMutex;

  ArFunctor1<ArRobotPacket *> *myPacketSentCallback;

  enum { INTARG = 0x3B, NINTARG = 0x1B, STRARG = 0x2B };
};


#endif //ARROBOTPACKETSENDER_H
