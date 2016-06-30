#ifndef ARROBOTBATTERYPACKETREADER_H
#define ARROBOTBATTERYPACKETREADER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArRobot;
class ArRobotPacket;

/// This class will read a config packet from the robot
class ArRobotBatteryPacketReader
{
public:
  /// Constructor
  AREXPORT ArRobotBatteryPacketReader(ArRobot *robot);
  /// Destructor
  AREXPORT ~ArRobotBatteryPacketReader();
  /// Request a single packet.. 
  AREXPORT void requestSinglePacket(void);
  /// Request a continous stream of packets
  AREXPORT void requestContinuousPackets(void);
  /// Stop the stream of packets
  AREXPORT void stopPackets(void);
  /// See if we've requested packets
  AREXPORT bool haveRequestedPackets(void);
  /// See if we've gotten the data
  bool hasPacketArrived(void) const { return myPacketArrived; }
  /// Gets the number of batteries
  int getNumBatteries(void) const { return myNumBatteries; }
  /// Gets the number of bytes per battery
  int getNumBytesPerBattery(void) const { return myNumBytesPerBattery; }
  /// Gets the flags1 for a particular battery
  int getFlags1(int battery) { return myFlags1[battery]; }
  /// Gets the flags2 for a particular battery
  int getFlags2(int battery) { return myFlags2[battery]; }
  /// Gets the flags3 for a particular battery
  int getFlags3(int battery) { return myFlags3[battery]; }
  /// Gets the relative state of charge for a particular battery
  int getRelSOC(int battery) { return myRelSOC[battery]; }
  /// Gets the absolute state of charge for a particular battery
  int getAbsSOC(int battery) { return myAbsSOC[battery]; }


protected:
  /// internal, packet handler
  AREXPORT bool packetHandler(ArRobotPacket *packet);
  /// internal, packet handler
  AREXPORT void connectCallback(void);

  // the robot
  ArRobot *myRobot;

  int myNumBatteries;
  int myNumBytesPerBattery;
  std::map<int, int> myFlags1;
  std::map<int, int> myFlags2;
  std::map<int, int> myFlags3;
  std::map<int, int> myRelSOC;
  std::map<int, int> myAbsSOC;


  // whether our data has been received or not
  bool myPacketArrived;
  // last time we requested a packet (we'll only ask every 200 ms)
  ArTime myLastPacketRequest;

  bool myRequestedBatteryPackets;

  // the callback
  ArRetFunctor1C<bool, ArRobotBatteryPacketReader, ArRobotPacket *> myPacketHandlerCB;
  ArFunctorC<ArRobotBatteryPacketReader> myConnectCB;
};

#endif // ARROBOTBATTERYPACKETREADER_H
