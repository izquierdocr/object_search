/*
Adept MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.9.0

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006-2009 MobileRobots Inc.
Copyright (C) 2010-2014 Adept Technology, Inc.

All Rights Reserved.

Adept MobileRobots does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
Adept MobileRobots
10 Columbia Drive
Amherst, NH 03031
+1-603-881-7960

*/
#ifndef ARCCENTRALDOCKHELPER_H
#define ARCCENTRALDOCKHELPER_H

#include "Aria.h"
#include "ArNetworking.h"

/**
   Class for helping gather which robots are at which docks
**/
class ArCentralDockHelper
{
public:
  /// Constructor
  AREXPORT ArCentralDockHelper(ArCentralManager *manager);

  /// Destructor
  AREXPORT virtual ~ArCentralDockHelper();
  /// Called when a new forwarder is added
  AREXPORT void forwarderAdded(ArCentralForwarder *forwarder);
  /// Called when a new forwarder is added
  AREXPORT void forwarderRemoved(ArCentralForwarder *forwarder);
protected:
  ArCentralManager *myManager;

  void gotData(void);

  class ClientDockHelper
  {
  public:
    /// Constructor
    ClientDockHelper(ArCentralForwarder *forwarder, ArFunctor *gotDataCB);
    /// Destructor
    ~ClientDockHelper();
    void dockingAtToServer(ArNetPacket *packet);
    
    char myMapName[10000];
    char myDockName[10000];
    char myOverrideMapName[10000];
    ArPose myDockPose;

  protected:
    ArCentralForwarder *myForwarder;
    ArFunctor *myGotDataCB;
    ArFunctor1C<ClientDockHelper, ArNetPacket *> myDockingAtToServerCB;
  };
  
  std::map<ArCentralForwarder *, ClientDockHelper *> myClientDockHelpers;

  ArNetPacket myDockingAtFromServerPacket;
  ArMutex myDataMutex;

  ArFunctor1C<ArCentralDockHelper, 
	      ArCentralForwarder *> myForwarderAddedCB;
  ArFunctor1C<ArCentralDockHelper, 
	      ArCentralForwarder *> myForwarderRemovedCB;
  ArFunctorC<ArCentralDockHelper> myGotDataCB;
};

#endif // ARSERVERSWITCHFILEUTILS_H
