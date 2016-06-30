#ifndef ARSZSERIES_H
#define ARSZSERIES_H

#include "ariaTypedefs.h"
#include "ariaOSDef.h"
#include "ArRobotPacket.h"
#include "ArLaser.h"   
#include "ArFunctor.h"

/** @internal */
class ArSZSeriesPacket : public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArSZSeriesPacket();
  /// Destructor
  AREXPORT virtual ~ArSZSeriesPacket();
  
  /// Gets the time the packet was received at
  AREXPORT ArTime getTimeReceived(void);
  /// Sets the time the packet was received at
  AREXPORT void setTimeReceived(ArTime timeReceived);

  AREXPORT virtual void duplicatePacket(ArSZSeriesPacket *packet);
  AREXPORT virtual void empty(void);

  AREXPORT virtual void byteToBuf(ArTypes::Byte val);

  AREXPORT virtual ArTypes::Byte bufToByte(void);
  
  void setDataLength(int x)
  { myDataLength = x; }
  int getDataLength()
  { return myDataLength; }
  void setNumReadings(int x)
  { myNumReadings = x; }
  int getNumReadings()
  { return myNumReadings; }
  void setScanFrequency(int x)
  { myScanFrequency = x; }
  int getScanFrequency()
  { return myScanFrequency; }

  void setPrevScanFrequency(int x)
  { myPrevScanFrequency = x; }
  int getPrevScanFrequency()
  { return myPrevScanFrequency; }


  unsigned char getCrcByte1()
  { return myCrcByte1; }
  void setCrcByte1(unsigned char c)
  { myCrcByte1 = c; }
  unsigned char getCrcByte2()
  { return myCrcByte2; }
  void setCrcByte2(unsigned char c)
  { myCrcByte2 = c; }

protected:
  int deascii(char c);

  ArTime myTimeReceived;

  // SZS specific
  int myDataLength;
  int myNumReadings;
  int myScanFrequency;
  unsigned char myCrcByte1;
  unsigned char myCrcByte2;
  
  int myPrevScanFrequency;


};


/// Given a device connection it receives packets from the sick through it
/// @internal
class ArSZSeriesPacketReceiver
{
public:
  /// Constructor with assignment of a device connection
  AREXPORT ArSZSeriesPacketReceiver();
  /// Destructor
  AREXPORT virtual ~ArSZSeriesPacketReceiver();
  
  /// Receives a packet from the robot if there is one available
  AREXPORT ArSZSeriesPacket *receivePacket(unsigned int msWait = 0,
					 bool shortcut = false);

  /// Sets the device this instance receives packets from
  AREXPORT void setDeviceConnection(ArDeviceConnection *conn);
  /// Gets the device this instance receives packets from
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  unsigned short CRC16(unsigned char *, int);

  // PS - added to pass info to this class
  AREXPORT void	setmyInfoLogLevel(ArLog::LogLevel infoLogLevel)
  { myInfoLogLevel = infoLogLevel; }
  AREXPORT void setmyIsSZ00(bool isSZ00)
  { myIsSZ00 = isSZ00; }
  AREXPORT void setmyName(const char *name )
  { strcpy(myName, name); }

protected:
  ArDeviceConnection *myConn;
  ArSZSeriesPacket myPacket;
  
  char myName[1024];
  unsigned int myNameLength;
  unsigned char myReadBuf[100000];
  int myReadCount;
  bool myIsSZ00;
  ArLog::LogLevel myInfoLogLevel;

  unsigned short myPrevCrc;


};

/**
  @since Aria 2.7.4
  @see ArLaserConnector
  Use ArLaserConnector to connect to a laser, determining type based on robot and program configuration  parameters.
*/
class ArSZSeries : public ArLaser
{
public:
  /// Constructor
  AREXPORT ArSZSeries(int laserNumber,
		 const char *name = "SZSeries");
  /// Destructor
  AREXPORT ~ArSZSeries();
  AREXPORT virtual bool blockingConnect(void);
  AREXPORT virtual bool asyncConnect(void);
  AREXPORT virtual bool disconnect(void);
  virtual bool isConnected(void) { return myIsConnected; }
  virtual bool isTryingToConnect(void) 
    { 
      if (myStartConnect)
	return true;
      else if (myTryingToConnect)
	return true; 
      else
	return false;
    }  

  /// Logs the information about the sensor
  AREXPORT void log(void);
protected:
  AREXPORT virtual void laserSetName(const char *name);
  AREXPORT virtual void * runThread(void *arg);
  AREXPORT virtual void setRobot(ArRobot *robot);
  void sensorInterp(void);
  void failedToConnect(void);
  void clear(void);
  bool myIsConnected;
  bool myTryingToConnect;
  bool myStartConnect;

  int myNumChans;


  ArLog::LogLevel myLogLevel;

  ArSZSeriesPacketReceiver myReceiver;

  ArMutex myPacketsMutex;
  ArMutex myDataMutex;

  std::list<ArSZSeriesPacket *> myPackets;
  
  ArTime myPrevSensorIntTime;

  ArFunctorC<ArSZSeries> mySensorInterpTask;
  ArRetFunctorC<bool, ArSZSeries> myAriaExitCB;
};

#endif 
