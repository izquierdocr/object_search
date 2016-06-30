#ifndef ARROBOTPACKETREADER_H
#define ARROBOTPACKETREADER_H


#include "ariaTypedefs.h"
#include "ArASyncTask.h"


class ArRobot;


class ArRobotPacketReaderThread : public ArASyncTask
{
public:

  AREXPORT ArRobotPacketReaderThread();
  AREXPORT virtual ~ArRobotPacketReaderThread();

  AREXPORT void setRobot(ArRobot *robot);

  AREXPORT void stopRunIfNotConnected(bool stopRun);
  AREXPORT virtual void * runThread(void *arg);

  AREXPORT virtual const char *getThreadActivity(void);


protected:
  bool myStopRunIfNotConnected;
  ArRobot *myRobot;
  bool myInRun;
};


#endif // ARSYNCLOOP_H
