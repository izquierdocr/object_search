#ifndef ARURG_H
#define ARURG_H

#include "ariaTypedefs.h"
#include "ArLaser.h"
#include "ArDeviceConnection.h"

/** Hokuyo Urg laser range device.
 *  Connects using the Urg's serial port connector (not USB).
 *  Supports URG-04LX using SCIP 1.1 protocol only. See ArLaserConnector for instructions on 
 *  using lasers in a program.
    @sa ArUrg_2_0
    @sa ArLaserConnector
    @sa ArLaser
 *  @since 2.7.0
 */
class ArUrg : public ArLaser
{
public:
  /// Constructor
  AREXPORT ArUrg(int laserNumber,
		 const char *name = "urg");
  /// Destructor
  AREXPORT ~ArUrg();
  AREXPORT virtual bool blockingConnect(void);
  AREXPORT virtual bool asyncConnect(void);
  AREXPORT virtual bool disconnect(void);
  AREXPORT virtual bool isConnected(void) { return myIsConnected; }
  AREXPORT virtual bool isTryingToConnect(void) 
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
  /// Sets the parameters that control what data you get from the urg
  AREXPORT bool setParams(
	  double startingDegrees = -135, double endingDegrees = 135,
	  double incrementDegrees = 1, bool flipped = false);
  /// Sets the parameters that control what data you get from the urg
  AREXPORT bool setParamsBySteps(
	  int startingStep = 0, int endingStep = 768, int clusterCount = 3,
	  bool flipped = false);
  AREXPORT virtual void * runThread(void *arg);
  /// internal call to write a string to the urg
  bool writeLine(const char *str);
  /// internal call to read a string from the urg
  bool readLine(char *buf, unsigned int size, unsigned int msWait);

  /// internal call to write a command and get the response back into the buf
  bool sendCommandAndRecvStatus(
	  const char *command, const char *commandDesc, 
	  char *status, unsigned int size, unsigned int msWait);
  
  void sensorInterp(void);
  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT virtual bool laserCheckParams(void);
  AREXPORT virtual void laserSetName(const char *name);
  
  void failedToConnect(void);
  ArMutex myReadingMutex;
  ArMutex myDataMutex;

  ArTime myReadingRequested;
  std::string myReading;

  int myStartingStep;
  int myEndingStep;
  int myClusterCount;
  bool myFlipped;
  char myRequestString[1024];
  double myClusterMiddleAngle;

  bool internalConnect(void);

  bool internalGetReading(void);

  void clear(void);
  bool myIsConnected;
  bool myTryingToConnect;
  bool myStartConnect;
  
  std::string myVendor;
  std::string myProduct;
  std::string myFirmwareVersion;
  std::string myProtocolVersion;
  std::string mySerialNumber;
  std::string myStat;
  
  bool myLogMore;
  
  ArFunctorC<ArUrg> mySensorInterpTask;
  ArRetFunctorC<bool, ArUrg> myAriaExitCB;
};

#endif // ARURG_H
