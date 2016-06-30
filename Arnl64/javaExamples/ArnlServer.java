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

// This is an example robot server written in Java, using ARNL for
// path planning and localization (it is a slightly simplified version of
// guiServer.cpp written in Java).

import com.mobilerobots.Aria.*;
import com.mobilerobots.ArNetworking.*;
import com.mobilerobots.BaseArnl.*;
import com.mobilerobots.Arnl.*;

public class ArnlServer {

  /* This loads the Arnl native wrapper library when this class in loaded */

  static {
    try {
      System.loadLibrary("AriaForArnlJava");
    } catch(UnsatisfiedLinkError e) {
      System.err.println("ArnlServer: Error: failed to load Aria native code library (libAriaForArnlJava): "+e.toString());
      System.exit(1);
    }

    try {
      System.loadLibrary("ArNetworkingForArnlJava");
    } catch(UnsatisfiedLinkError e) {
      System.err.println("ArnlServer: Error: failed to load ArNetworking Arnl native code library (libArNetworkingForArnlJava): "+e.toString());
      System.exit(1);
    }

    try {
      System.loadLibrary("BaseArnlJava");
    } catch(UnsatisfiedLinkError e) {
      System.err.println("ArnlServer: Error: failed to load BaseArnl native code library (libBaseArnlJava): "+e.toString());
      System.exit(1);
    }

    try {
      System.loadLibrary("ArnlJava");
    } catch(UnsatisfiedLinkError e) {
      System.err.println("ArnlServer: Error: failed to load Arnl native code library (libArnlJava): "+e.toString());
      System.exit(1);
    }
  }

  public static void main(String argv[]) {


    // Global library initialization, ARIA then ARNL (required)
    Aria.init();
    Arnl.init();


    // Create robot and device objects:
    ArRobot robot = new ArRobot("robot1", true, true, true);
    ArAnalogGyro gyro = new ArAnalogGyro(robot);
    ArSonarDevice sonar = new ArSonarDevice();
    ArSick laser = new ArSick();
    robot.addRangeDevice(sonar);
    robot.addRangeDevice(laser);

    // make the server for remote clients (e.g. MobileEyes)
    ArServerBase server = new ArServerBase();

    // Create a "simple connector" object and connect to either the simulator
    // or the robot. 
    // Unlike the C++ API which takes int and char** arguments, 
    // the Java constructor just takes the argv list.
    System.out.println( "Connecting to the robot..." );
    ArSimpleConnector connector = new ArSimpleConnector(argv);

    if(!Aria.parseArgs())
    {
      Aria.logOptions();
      System.exit(1);   
    }

    if (! connector.connectRobot(robot))
    {
      System.out.println( "Could not connect to robot, exiting");
      System.exit(1);
    }

    // Start the laser reading thread and connect
    laser.configureShort(true);
    laser.runAsync();
    if(! laser.blockingConnect())
    {
      System.out.println( "Could not connect to laser, exiting");
      System.exit(1);
    }


    // You can also use ArLog in Java. But unlike in C++, it only takes one string
    // which you can form by the Java string concatenation operator +.
    ArLog.log(ArLog.LogLevel.Terse, "Connected to robot \"" + robot.getName() + "\" and devices; Will look for map files in \"" + Aria.getDirectory() + "examples/\"; now creating network services...");

    // Start with an empty map. Use Arnl examples directory as the default for
    // map files (Arnl.init() changed Aria's default directory from .../Aria to
    // .../Arnl).  Allow a file name to be empty as well.  When the
    // configuration file is loaded into ArConfig later, if it specifies a map
    // file, then that map file will be loaded.
    ArMap map = new ArMap(Aria.getDirectory() + "examples/");
    map.setIgnoreEmptyFileName(true);
    // Make the localization task (using the laser device)
    ArLocalizationTask locTask = new ArLocalizationTask(robot, laser, map);

    // Make the path planning task (using both laser and sonar to avoid obstacles)
    ArPathPlanningTask pathTask = new ArPathPlanningTask(robot, laser, sonar, map);

    // Make logging system configurable by the user:
    ArLog.addToConfig(Aria.getConfig());

    // Open the server
    System.out.println( "Opening server on port 7272...");
    if (!server.open(7272))
    {
      System.out.println( "Could not open server on port 7272, exiting");
      System.exit(1);
    }
    System.out.println( "Server is open on port 7272.");


    // Create services for remote clients (e.g. MobileEyes):

    // Provides localization info:
    ArServerInfoLocalization serverInfoLocalization = new ArServerInfoLocalization(server, robot, locTask);

    // Allows client to manually trigger relocalization and a given point:
    ArServerHandlerLocalization serverLocHandler = new ArServerHandlerLocalization(server, robot, locTask);

    // Provides the map:
    ArServerHandlerMap serverMap = new ArServerHandlerMap(server, map);

    // Provides the planned path:
    ArServerInfoPath serverInfoPath = new ArServerInfoPath(server, robot, pathTask);

    // Information about the robot:
    ArServerInfoRobot serverInfoRobot = new ArServerInfoRobot(server, robot);

    // Info from range sensors:
    ArServerInfoSensor serverInfoSensor = new ArServerInfoSensor(server, robot);

    // Graphics to draw on the map:
    ArServerInfoDrawings drawings = new ArServerInfoDrawings(server);
    drawings.addRobotsRangeDevices(robot);

    // Modes for driving the robot:
    ArServerModeGoto modeGoto = new ArServerModeGoto(server, robot, pathTask, map, locTask.getHomePose());
    ArServerModeStop modeStop = new ArServerModeStop(server, robot, true);
    ArServerModeRatioDrive modeRatioDrive = new ArServerModeRatioDrive(server, robot);
    ArServerModeWander modeWander = new ArServerModeWander(server, robot);

    modeStop.addAsDefaultMode();
    modeStop.activate();

    // Simple text commands ("custom commands" in MobileEyes):
    ArServerHandlerCommands commands = new ArServerHandlerCommands(server);
    ArServerSimpleComUC uCCommands = new ArServerSimpleComUC(commands, robot);
    ArServerSimpleComMovementLogging loggingCommands = new ArServerSimpleComMovementLogging(commands, robot);
    ArServerSimpleComGyro gyroCommands = new ArServerSimpleComGyro(commands, robot, gyro);;
    ArServerSimpleComLogRobotConfig configCommands = new ArServerSimpleComLogRobotConfig(commands, robot);
    serverInfoPath.addControlCommands(commands);

    // Service that allows client to read and change ArConfig parameters (used 
    // throughout aria and arnl)
    String configFileName = Arnl.getTypicalParamFileName();
    System.out.println( "Will use config file: \"" + configFileName + "\"");
    ArServerHandlerConfig serverConfig = new ArServerHandlerConfig(server, Aria.getConfig(), configFileName, Aria.getDirectory());


    // Load the configuration file
    System.out.println( "Loading config file...");
    if (! Aria.getConfig().parseFile(configFileName)) 
    {
      System.out.println( "Warning: Error loading configuration file \""+configFileName+"\", exiting.");
    }

    // Run the robot and server threads in the background:
    System.out.println( "Running...");
    robot.runAsync(true);
    server.runAsync();
    
    robot.enableMotors();
    robot.waitForRunExit();
  }
}

