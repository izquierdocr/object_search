/*

Copyright (c) 2014 Adept Technology Inc.
All rights reserved.  

Redistribution of this example source code, with or without modification, is 
permitted provided that the following conditions are met:   
-    Redistributions must retain the above copyright notice, 
     this list of conditions and the following disclaimer.  
-    Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Adept Technology, Inc.

Adept Technology, Inc. makes no warranty as to the suitability of this material
for use by the recipient, and assumes no responsibility for any consequences
resulting from such use. 

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/
#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"


/** @example centralServerExample.cpp Example of a central server which allows 
 *   clients to communicate with multiple robot servers through one connection,
 *   and for the multiple robot servers to share information.
 *
 * This is a simple "central robot server": it uses ArCentralManager for
 * ArNetworking to multiplex several ArNetworking connections through one
 * ArNetworking server.  Servers (e.g. guiServer) running on several robots 
 * can then be invoked with the "-centralServer" argument (and others), and
 * they will accept client connections through this central server. 
 * Clients supporting multiple robots, such as recent version of MobileEyes,
 * can connect to the central server. 
 *
 * The central server can also hold common configuration options and/or a 
 * common map.
 *
 * You use this example program as-is, or add to it to implement
 * multi-robot coordination etc. 
 *
 * Note, this server uses the default server port, 7272. Any other servers
 * running on the same computer will need to use a different port (e.g. use
 * the -serverPort argument).
 *
 * @todo describe how to use ArCentralManager to send things like range device
 *  readings, or commands, to multiple robots.
 */


// Called whenever a new robot server connects and a forwarder is created for
// it. (Defined after main() below.)
void robotForwarderAdded(ArCentralForwarder *forwarder);

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);

  /* This server accepts connections from the several robot servers. It's port
   * number is set using the -<name>ServerPort argument, where <name> is given
   * here as "robot". (A default of 5000 set below, this number should always
   * be used). */

  ArServerSimpleOpener robotServersOpener(&parser, "robot");
  ArServerBase robotServer(true, "RobotServer", false);

  // Set robotServerPort to 5000, to avoid conflicting with the
  // server port used for clients, which will use the normal
  // default of 7272.
  parser.addDefaultArgument("-robotServerPort 5000", 1);


  /* This server accepts connections from clients: */

  ArServerSimpleOpener clientOpener(&parser, "client");
  ArServerBase clientServer(true, "ClientServer", false, "", "", true);

  // Load other defaults from files and environment
  parser.loadDefaultArguments(1);


  /* Parse arguments */
  if(!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(0);
  }

  /* Open servers */

  ArLog::log(ArLog::Normal, "Opening connection point for robot servers...");
  if (!robotServersOpener.open(&robotServer, "", 120) || !robotServersOpener.checkAndLog())
  {
    ArLog::log(ArLog::Terse, "Could not open connection point (server) for robot servers");
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "Opening server for clients...");
  if (!clientOpener.open(&clientServer, "", 120) || !clientOpener.checkAndLog())
  {
    ArLog::log(ArLog::Terse, "Could not open server for clients");
    Aria::exit(1);
  }


  /* This keeps track of the several robot servers. When a robot server
   * connects to the robotsServer connection point, an ArCentralForwarder
   * object is created for that robot server. When a client
   * connects to the clientServer, it is informed of ports it can
   * connect to to reach the robot servers via the forwarder objects.
   */
  ArCentralManager centralManager(&robotServer, &clientServer);

  /* ArCentralManager also can invoke callbacks whenever a new robot server
   * connects and its ArCentralForwarder object is created. Commands can
   * be sent to the robots via these forwarders.
   * Here is an example:
   */
  ArGlobalFunctor1<ArCentralForwarder*> exampleNewForwarderCB(&robotForwarderAdded);
  centralManager.addForwarderAddedCallback(&exampleNewForwarderCB);

  /* Clients can send config requests to/from the robot servers, but
   * this allows you to also modify this central server's global configuration
   * (if you or any Aria class adds options to the global Aria ArConfig
   * object).
   */
  ArServerHandlerConfig handlerConfig(&clientServer, Aria::getConfig());

  /* monitor communication between the server and client. */
  ArServerHandlerCommMonitor commMonitor(&clientServer);
  
  /* custom/simple commands, but give them different names than the robot
   * servers' simple commands:
   */
  ArServerHandlerCommands commands(&clientServer);
  commands.setPrefix("CentralServer");
  
  ArServerSimpleServerCommands *serverCommands = NULL;
  serverCommands = new ArServerSimpleServerCommands(&commands, &clientServer);

  /* Drawings for the client, used to draw information about multiple robots. */
  ArServerInfoDrawings drawings(&clientServer);

  /* Keep track of each robot, and send positions to other robots to use, and
   * drawings to clients showing points on the robots */
  ArCentralMultiRobot *centralMultiRobot = NULL;
  centralMultiRobot = new ArCentralMultiRobot(&centralManager, Aria::getConfig(), &drawings);

  /* Load a config file if given */
/*
  char errorBuffer[1024];
  if (!Aria::getConfig()->parseFile(parser.getArg(1), true, false, 
				    errorBuffer, sizeof(errorBuffer)))
  {
    ArLog::log(ArLog::Terse, "### Warning: Error loading configuration file \"%s\": %s", argv[1], errorBuffer);
  }

  // write it back in case new items need to be added:
  Aria::getConfig()->writeFile(Aria::getConfig()->getFileName());
*/

  /* Run servers */
  clientServer.runAsync();
  robotServer.run();
  Aria::exit(0);
}


// Called whenever a new robot server connects and a forwarder is created for
// it.
void robotForwarderAdded(ArCentralForwarder *forwarder)
{
  // Note that when a robot server connects to this central server, the central
  // server takes on the "client" role with respect to that server. So to send
  // requests or get information from a robot server, use the forwarder's
  // "client" object. (The forwarder's "server" object is the local port that 
  // clients such as MobileEyes connect to.)
  ArLog::log(ArLog::Normal, "centralServerExample: New forwarder added for robot server named \"%s\" at host %s.", forwarder->getRobotName(), forwarder->getClient()->getHost());
}
