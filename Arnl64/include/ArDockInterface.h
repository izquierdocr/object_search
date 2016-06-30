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

#ifndef ARDOCKINTERFACE_H
#define ARDOCKINTERFACE_H

#include "Aria.h"
#include "ArNetworking.h"

/// Interface for objects that provide information about a robot's docking status. 
/**
 *  ArDockInterface defines the methods that enable the AramScheduler to 
 *  determine the docking status of a robot.  For a local robot, these methods are
 *  implemented by ArServerModeDock.  With the central server and remote robots, 
 *  these methods are implemented by AramCentralDockProxy.  
**/
class ArDockInterface  
{
public:
	enum State {
    UNDOCKED,
    DOCKING,
    DOCKED,
    UNDOCKING
  };

	AREXPORT static const char *toString(State s) {
    switch (s) {
		case UNDOCKED:
			return "UNDOCKED";
		case DOCKING:
			return "DOCKING";
		case DOCKED:
			return "DOCKED";
		case UNDOCKING:
			return "UNDOCKING";
		} // end switch state

		return "unknown";

	} // end method toString


  /// Constructor
  AREXPORT ArDockInterface() {}
  /// Destructor
  AREXPORT virtual ~ArDockInterface() {}

  /// Gets the docking state we're in
  AREXPORT virtual State getState() const = 0;

	/// Gets whether our docking is forced or not
  AREXPORT virtual bool getForcedDock() = 0;

	/// Gets whether ForcedDock flag is available
  AREXPORT virtual bool isForcedDockAvailable() = 0;

	/// Gets whether our docking is autodock or not
  AREXPORT virtual bool getAutoDock() = 0;

	/// Gets whether AutoDock flag is available
  AREXPORT virtual bool isAutoDockAvailable() = 0;

	/// Gets wheather a gotodock has been sent
  AREXPORT virtual bool hasGoToDockBeenSent() = 0;

	/// Gets whether our docking is forced or not
  AREXPORT virtual void gotoDock(bool force = true) = 0;

}; // end class ArDockInterface


#endif // ARDOCKINTERFACE_H
