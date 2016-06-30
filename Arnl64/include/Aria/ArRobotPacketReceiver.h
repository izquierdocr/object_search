#ifndef ARROBOTPACKETRECEIVER_H
#define ARROBOTPACKETRECEIVER_H

#include "ariaTypedefs.h"
#include "ArRobotPacket.h"


class ArDeviceConnection;

/// Given a device connection it receives packets from the robot through it
class ArRobotPacketReceiver
{
public:
  /// Constructor without an already assigned device connection
  AREXPORT ArRobotPacketReceiver(bool allocatePackets = false,
				 unsigned char sync1 = 0xfa, 
				 unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection
  AREXPORT ArRobotPacketReceiver(ArDeviceConnection *deviceConnection, 
				 bool allocatePackets = false,
				 unsigned char sync1 = 0xfa, 
				 unsigned char sync2 = 0xfb);
  /// Constructor with assignment of a device connection and tracking
  AREXPORT ArRobotPacketReceiver(ArDeviceConnection *deviceConnection, 
				 bool allocatePackets,
				 unsigned char sync1, 
				 unsigned char sync2,
					bool tracking,
					const char *trackingLogName);
  /// Destructor
  AREXPORT virtual ~ArRobotPacketReceiver();
  
  /// Receives a packet from the robot if there is one available
  AREXPORT ArRobotPacket *receivePacket(unsigned int msWait = 0);

  /// Sets the device this instance receives packets from
  AREXPORT void setDeviceConnection(ArDeviceConnection *deviceConnection);
  /// Gets the device this instance receives packets from
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  
  /// Gets whether or not the receiver is allocating packets
  AREXPORT bool isAllocatingPackets(void) { return myAllocatePackets; }
  /// Sets whether or not the receiver is allocating packets
  AREXPORT void setAllocatingPackets(bool allocatePackets) 
    { myAllocatePackets = allocatePackets; }

#ifdef DEBUG_SPARCS_TESTING
  AREXPORT void setSync1(unsigned char s1) { mySync1 = s1; }
  AREXPORT void setSync2(unsigned char s2) { mySync2 = s2; }
#endif

	void setTracking(bool tracking)
  {
    myTracking = tracking;
  }

	void setTrackingLogName(const char *trackingLogName)
  {
    myTrackingLogName = trackingLogName;
  }

  /// Sets the callback that gets called with the finalized version of
  /// every packet set... this is ONLY for very internal very
  /// specialized use
  AREXPORT void setPacketReceivedCallback(ArFunctor1<ArRobotPacket *> *functor);
protected:
  ArDeviceConnection *myDeviceConn;
	bool myTracking;
	std::string myTrackingLogName;

  bool myAllocatePackets;
  ArRobotPacket myPacket;
  enum { STATE_SYNC1, STATE_SYNC2, STATE_ACQUIRE_DATA };
  unsigned char mySync1;
  unsigned char mySync2;

  ArFunctor1<ArRobotPacket *> *myPacketReceivedCallback;
};

#endif // ARROBOTPACKETRECEIVER_H
