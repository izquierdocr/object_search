
  ----------------------------------------------------------------------
  Adept MobileRobots Advanced Robot Navigation and Localization Software
  ----------------------------------------------------------------------

Copyright (c) 2004-2005 ActivMedia Robotics, LLC. 
Copyright (c) 2006-2009 MobileRobots Inc.
Copyright (c) 2010-2014 Adept Technology, Inc.
All rights reserved.
The license for use of this package is described in the LICENSE.txt file.


******************************************************************************
  Important Note for release version 1.5:                                
******************************************************************************

As of version 1.5, ARNL now uses the general robot acceleration,
deceleration and rotational velocity parameters to perform more accurate
path following. However, this means that your existing configuration must
be updated.  If rotational acceleration or deceleration, translational 
acceleration or deceleration, or rotational velocity maximum
are too low, the robot will drive more slowly than neccesary, especially
when turning; Some recommended values to try are max. rot. vel of 250,
rot. accel. of 300, rot. decel. of 300, trans. accel. of 600, trans. decel.
of 600, then adjust as neccesary.  Either the "Driving..." parameters
in "Path Planning Settings", or the ARCOS firmware parameters may be set. 
See Changes.txt for more discussion of this issue. 

******************************************************************************


Contents
========
  Introduction
  Quick Start
  Windows Installation
  Using the ARNL libraries in Windows
  Linux Installation
    Debian, RedHat, and from the TGZ
    Using the Correct Compiler
    Installing GCC 3.4
  Using the ARNL libraries in Linux
  Java and Python

  

Introduction
============

ARNL is the Advanced Robotics Navigation and Localization extension
to ARIA, Adept MobileRobots' Advanced Interface for Applications software
development kit. All required components of ARIA and ArNetworking for
working with ARNL are included within this ARNL distribution (header
files, libraries, etc.)  Note: ARNL has been built with this version 
of ARIA; do not compile with a separate ARIA distribution.

This document provides essential information for using the ARNL software.  
More detailed reference manuals are available in the "docs" directories
of the software packages.  Manuals that provide an overview of using
the ARNL, MobileEyes, mapping software and robot together, including 
information on adding a SICK laser rangefinder to an existing robot if 
neccesary, also available as a printable PDF document from 
http://robots.mobilerobots.com/all_docs.

The ARNL robot navigation software is provided as several 
seperate libraries.  The exact set of libraries that you have will
depend on which navigation package option your purchased.

BaseArnl:
  * All packages include the BaseArnl library. This library includes
    the path planning and following task, and a general infrastructure
    for running one or more localization methods. It also includes 
    ARIA and ArNetworking libraries licensed for use with ARNL, and
    of the correct version. 
  * On Linux, it is installed in /usr/local/Arnl
  * On Windows, it is installed in Program Files\MobileRobots\Arnl

Arnl:
  * The indoor laser navigation package includes the "Arnl" library.
    This library provides a localization method that uses a laser 
    rangefinder to localize the robot in a map.  It must be used
    together with the BaseArnl, Aria and ArNetworking libraries.
  * On Linux, it is installed in /usr/local/Arnl
  * On Windows, it is installed in Program Files\MobileRobots\Arnl
  * Installation of BaseArnl in addition to Arnl is required

MOGS:
  * The outdoor navigation package (MOGS) includes the Mogs library.
    This library provides a localization method that uses data from
    the GPS to localize the robot.  It must be used together with
    with the BaseArnl, Aria and ArNetworking libraries.
  * On Linux, it is installed in /usr/local/Arnl
  * On Windows, it is installed in Program Files\MobileRobots\Arnl
  * Installation of BaseArnl in addition to MOGS is required

SonArnl:
  * If you do not have one of the above optional navigation packages,
    you can use SONARNL.  The SonArnl library provides a localization
    method that uses the sonar to do approximate localization in a
    mapped area.  SONARNL is also available with the other package
    options.  It must be used together with with the BaseArnl, 
    Aria and ArNetworking libraries.
  * On Linux, it is installed in /usr/local/Arnl
  * On Windows, it is installed in Program Files\MobileRobots\Arnl
  * Installation of BaseArnl in addition to SonArnl is required

In addition to the ARNL packages, you'll need Mapper3(TM) to
create maps for your robot for use with ARNL or Mapper3Basic(TM) to
create maps for your robot for use with SONARNL. And for a remote GUI,
use the MobileEyes(TM) application, also downloadable as a separate
package. Find links to all these components from the robots support
website at:

     http://robots.mobilerobots.com

In the docs directory of each library, you will find the reference
manual for the libraries.  Programming examples are in the examples/
directory (see the README in that directory for directions).  The
include/ directory contains files for ARNL as well as for ARIA and
ArNetworking.  The required ARNL, ARIA and ArNetworking libraries are
in the lib/ directory.  In src/ there is the ArServerClasses file that
shows ways of controlling the robot.

Specially note the params/arnl.p and params/sonarnl.p files (for ARNL
and SONARNL repsectively). One is loaded at startup and used by
ARNL/SONARNL for localization and path planning.  Modify the file to
tailor ARNL/SONARNL and your robot for its operating behaviors and
environment.  There also is a backup copy of these parameters in
params/default-arnl.p and params/default-sonarnl.p.  arnl.p and
sonarnl.pcan be edited from MobileEyes(TM) with a GUI that can also
restore sections from the default.

Sections describing ARNL installation on a Windows and Linux-based systems
are contained in this README.  Make sure you read and follow those 
directions or ARNL simply won't work or it will give you odd crashes.


Quick Start 
===========

This section provides instructions for running the example ARNL
server programs and MobileEyes so you can see it working right away.

To localize and plan navigation paths, ARNL and SONARNL need a map of
its operating environment. Although easy to do, it takes some time to
make and edit a map. Instead, you could also use MobileSim
with sample maps provided in Arnl/examples to quickly see how the
various ARNL, ArNetworking and MobileEyes components work.

MobileSim is in a seperate distribution.  Sample maps are in the
Arnl/examples directory. For example, with Linux (requires X-Windows):

% cd /usr/local/MobileSim/
% ./MobileSim -map /usr/local/Arnl/examples/columbia.map

Like ARNL with Windows, MobileSim typically gets installed in
C:\Program Files\MobileRobots\MobileSim.  At startup the
simulator will ask you which map to use or if you want to use no map
and offer you a dialog box to choose the map.  At startup you may also
choose a different robot other than the Pioneer DX.  There are also
command line arguments, see much more details in the MobileSim docs.

Detailed instructions for making maps with your real robot are in
docs/Mapping.txt for ARNL and docs/SonarMapping.txt for SONARNL.  

To operate your robot (or simulated one) through MobileEyes with ARNL,
start the example server, passing it, as an argument, the
map name.  For example:

% cd /usr/local/Arnl/examples
% ./arnlServer -map columbia.map

Or for SONARNL:

% cd /usr/local/Arnl/examples
% ./sonarnlServer -map columbia.map

Make the proper path and filename substitutions as needed for your own
maps, of course. The guiServer automatically connects with the
simulator, if it is running on the same machine, or will connect with
your robot's controller, if it is attached to the default serial port.

Now start MobileEyes from the same computer or elsewhere on your
LAN. MobileEyes communicates with your ARNL/ArNetworking-enabled robot
via TCP/IP networking even if they're all run on the same computer. So
when MobileEyes starts up, you need to tell it where the robot's
network server is located: Type in or click to select from the
adjacent drop-down menu, the Server Name for the network location of
the robot's computer. The Server Name is either an IP address, such as
"192.168.100.1", or a hostname ("pioneer" or "pioneer.local.net", for
example).  If the robot server is running on the same machine as your
MobileEyes, as might be the case with the simulated robot, simply
enter "localhost" as the hostname.  (Hostnames and IP addresses, once
entered, get stored automatically into MobileEye's drop-down list of
Server Names.)

Now click the Connect button to access the robot over the
network. With the real robot, MobileEyes won't connect until the
robot's laser is fully powered up.  So you might have to Wait a few
moments before trying to connect again.

The first thing the server does in response to a MobileEyes connection
is send out its map, which subsequently appears in the MobileEyes
display.  Thereafter, you may send your robot from its current
position to some goal that had been embedded in the map by
double-clicking the goal's name in the goal box at the left of the
MobileEyes display. Otherwise, click to highlight the goal name, and
then click the Go To button to have your robot plan a path and
automatically drive to that goal.

Further explore the MobileEyes application features to manually drive
your robot with the keyboard or from a joystick. Click in the map
directly to have your robot drive to that spot, if it can get there
from here, of course. MobileEyes displays the ARNL-planned path as
well as the localization points showing where ARNL thinks the robot
is.  ARNL will let you know if its robot gets lost. That should be
very rare, but happens, for instance, if you start your robot up
somewhere other than near its map HOME. Use the Tools: Robot Tools
Localizing features to reorient the robot.

For ARNL (ie with a laser) the easy way to make a map is to connect
with MobileEyes and then select 'Tools->Map Creation->Start Mapping'
and give it a file name, then drive the robot around, and then stop
mapping by selecting 'Tools->Map Creation->Stop Mapping'.  This map
won't be as good as other methods, see docs/Mapping.txt for more
information.  After stopping mapping you can start up Mapper3 and
'Open From Robot' and connect and then turn the .2d into a .map, edit
it, and 'Save to Robot' the file back onto your robot and then use
'Tools->Robot Config' to change your map file to your new map.

For SONARNL (ie just with sonar) you'll need to start up Mapper3Basic
and then select 'New Map' and then create a map yourself with a tape
measure.  Note that if you have a robot with a laser a map created
with the most recent Mapper3 will contain the line information needed
to use SONARNL and in a much easier fashion than making a map by hand.

NOTE: the default guiServer will allow you to make maps and put and
get files from the example directory, if you do not wish this to be
possible modify the guiServer code (search for ArServerFileLister).

You can also start up guiServer with an option for using user and
password information to allow connection.  To do this you should edit
examples/guiServer.userInfo and change the '.' for passwords to the
password you wish and then pass guiServer an additional '-userInfo
guiServer.userInfo' argument.


Windows Installation:
=====================

If you have a previous version of *either* ARNL or SONARNL, 
uninstall it now. 

Run the installer program to install BaseArnl. Then run 
the installer program to install ARNL, SONARNL and/or 
MOGS depending on what software option you purchased.

Development files, documentation, examples, etc. are
installed in C:\Program Files\MobileRobots\Arnl.  Find shortcuts
to useful files and directories in the Start menu as well.

ARNL will only work as distributed with Microsoft Visual C++
version 10 (Visual Studio 2010), or version 11 (Visual Studio 2012)


Using the ARNL libraries in Windows:
====================================

To use the libraries with your own programs in Windows, use either Microsoft Visual 
C++ 2010 (version 10), Visual C++ 2012 (version 11), or Visual
C++ 2013 (version 12).  Precompiled example programs in bin were 
built with Visual Studio 2012 and require the Visual C++ 2012
(VC11) ARNL, BaseArnl, Aria and ArNetworking DLLs.

Free "Express" versions of Visual C++ are available from Microsoft.  ARNL
requires only Visual C++, other options such as C#, Web Development, SQL
Server/database tools, or Silverlight are not required by ARNL or ARIA.

The library (.lib) files are installed in C:\Program Files\MobileRobots\Arnl\lib.
The DLL files (.dll) are installed in C:\Program Files\MobileRobots\Arnl\bin. This
is also where example program binaries are.  The names of the library and DLLs 
built with Visual Studio 2010 end with the tag "VC10".  The Visual Studio 2012
libraries and DLLs end with the tag "VC11". Visual C++ 2013 libraries
and DLLs end with the tag "VC12".  The names of the libraries and DLLs 
built with the "Debug DLL" runtime contain the word Debug. The "Release DLL" 
libraries and DLLs do not.  You must use the appropriate libraries and DLLs for 
your version of Visual Studio and runtime library selected.

1. Create a new project, either in a new solution or in one of the existing 
   solutions contained in the examples directory.

2. From the Project Menu, choose to edit your projects Properties. Or
  right-click on the project in the Solution Explorer and choose Properties.

To edit your new project's configuration properties, right click on the 
project in the solution explorer pane and choose Properties.

3. Change the "Configuration" (upper left of the dialog) menu to "All
  Configurations".

4. Click on the "General" section

  * Either change "Output Files" to "C:\Program Files\MobileRobots\Arnl\bin" 
    (Arnl’s bin directory, where the DLLs are), or, if you want your
    program saved in a different directory, you must put Arnl's bin directory
    in your system PATH environment variable, or copy the DLLs to your own
    project's output directory.

  * You can change Intermediate Files if desired. 

  * Set "Common Language Runtime Support" to "No Common
    Language Runtime Support"

5. Click on the "Link" or "Linker" section.

  * Click on the "Input" subsection.

    * To "Additional Library Path" add "C:\Program Files\MobileRobots\Arnl\lib".

    * Change the "Configuration" menu (upper left of the window) to "Debug"

    * (Visual Studio 2010) To "Additional Dependencies", add:
       BaseArnlDebug.lib; ArNetworkingDebugVC10.lib; AriaDebugVC10.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlDebugVC10.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlDebugVC10.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsDebugVC10.lib

    * (Visual Studio 2012) To "Additional Dependencies", add:
       BaseArnlDebugVC11.lib; ArNetworkingDebugVC11.lib; AriaDebugVC11.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlDebugVC11.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlDebugVC11.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsDebugVC11.lib
		
	* (Visual Studio 2013) To "Additional Dependencies", add:
       BaseArnlDebugVC12.lib; ArNetworkingDebugVC12.lib; AriaDebugVC12.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlDebugVC12.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlDebugVC12.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsDebugVC12.lib

    * Change the "Configuration" menu to "Release"

    * (Visual Studio 2010) To "Additional Dependencies", add:
       BaseArnlVC10.lib; ArNetworkingVC10.lib; AriaVC10.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlVC10.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlVC10.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsVC10.lib

    * (Visual Studio 2012) To "Additional Dependencies", add:
       BaseArnlVC11.lib; ArNetworkingVC11.lib; AriaVC11.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlVC11.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlVC11.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsVC11.lib
		
	* (Visual Studio 2013) To "Additional Dependencies", add:
       BaseArnlVC12.lib; ArNetworkingVC12.lib; AriaVC12.lib; winmm.lib; advapi32.lib; ws2_32.lib
      If you are using the Arnl  library for laser localization, also add:
        ArnlVC12.lib 
      If you are using the SonArnl library for sonar localization, also add:
        SonArnlVC12.lib
      If you are using the MOGS library for GPS positioning, also add:
        MogsVC12.lib
  
    * Change the "Configuration" menu back to "All Configurations"

6. Click on the "C++" section.

  * Click on the "General" subsection

    * To "Additional Include Directories" add 
      “C:\Program Files\MobileRobots\Arnl\include”;”C:\Program Files\MobileRobots\Arnl\include\Aria”;”C:\Program Files\MobileRobots\Arnl\include\ArNetworking”

   * Click on the "Code Generation" subsection

     * Change the "Configuration" menu (upper left of the window) to "Debug".

       * Under the "Use run-time library" pulldown select "Debug
         Multithreaded DLL".

     * Change the "Configuration" menu (upper left of the screen) to "Release".

       * Under the "Use run-time library" pulldown select "Multithreaded DLL".

     * Change the "Configuration" menu back to "All Configurations"

To get started using and writing programs with ARNL, see the introductory manuals
and BaseArnl, ARNL/SONARNL/MOGS, ARIA and ArNetworking Reference manuals contained
in the "docs" directory, as well as the example programs in the "examples" directory.



Linux Installation:
===================


-> On Debian or Ubuntu Linux, install the libarnl, libmogs and/or libsonarnl 
   package(s) along with or after installing arnl-base using the dpkg tool.
   Packages for Debian 5 "lenny" include the tag "+lenny" or "+debian5".  Packages
   for the older Debian 3.1 "sarge" end with the tag "+sarge+gcc34" (or do
   not end with such an identifier in older versions). Packages for Ubuntu 12.04 
   "precise" include the tag "+ubuntu12".  The package will also include a tag 
   indicating the version of GCC used; you should use a compatible GCC version 
   (major version number).   When installing on a 32-bit computer, use the _i386 
   architecture packages.  If installing on your own 64-bit laptop or other computer, 
   use the _amd64 architecture packages.

   For example, if installing on Ubuntu on a 32-bit computer:
   
      dpkg -i arnl-base_1.9.0+ubuntu12+gcc4_i386.deb
      dpkg -i libarnl_1.9.0+ubuntu12+gcc4_i386.deb
      dpkg -i libsonarnl_1.9.0+ubuntu12+gcc4_i386.deb
      dpkg -i libmogs_1.9.0+ubuntu12+gcc4_i386.deb
    
   The localization libraries (libarnl, libsonarnl, libmogs) depend on
   a matching version of arnl-base, so it must be installed before they
   can be installed.   

-> On other Linux systems, download the compressed TAR packages (.tar.gz file).
   Unpack with TAR (e.g. "tar -xzf ARNL-1.4-0.tar.gz"), enter the new directory
   created, become the root user with "su", then run "make install" to install
   Arnl in /usr/local/Arnl. Follow any additional instructions given by
   "make install".   Two sets of packages are provided, one set built with GCC 3.4
   ("+gcc34", or not specified in older versions), and one set built with GCC 4
   ("+gcc4").

To uninstall, use apt-get or synaptic (e.g. "apt-get remove libarnl"), or rename or remove /usr/local/Arnl if you installed it manually
with "make install".

Using ARNL Libraries in Linux
==============================

To use the libraries with your own programs in Linux, use the GNU C++
compiler "g++", and GNU Make "make".

The shared library (.so) files are installed in /usr/local/Arnl/lib. This includes
lebArnBase.so, any localization librarys you may have (e.g. libArnl.so), as well
as ArNetworking and ARIA libraries included with ARNL and which will be fully 
compatible with the ARNL libraries (libAriaForArnl.so and libArNetworkingForArnl.so).

To build a program using the ArnlBase library, add the Arnl include directory to the
include path and the Arnl lib directory to the linker path, and link to the libraries,
e.g.:

  g++ -fPIC -g -Wall -I/usr/local/Arnl/include -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking -o yourprogram yourprogram.cpp -L/usr/local/Arnl/lib -lArnlBase -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt

To use one or more of the localization implementations (ARNL, SonArnl, MOGS), you must
also link to that library.  To link to Arnl for laser localization, add -lArnl.  To link
to MOGS for GPS positioning, add -lMogs.  To link to SonArnl, add -lSonArnl.

See the g++ man page and other documentation for more information on using g++:

  man g++

http://gcc.gnu.org/onlinedocs/

You can also use the "make" tool by creating a Makefile:

  all: yourprogram

  CFLAGS=-fPIC -g -Wall
  ARNL_INCLUDE= -I/usr/local/Arnl/include -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking -o
  ARNL_BASE_LINK=-L/usr/local/Arnl/lib -lArnlBase -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt
  ARNL_LINK=-lArnl

  %: %.cpp
    $(CXX) $(CFLAGS) $(ARNL_INCLUDE) $< -o $@ $(ARNL_LINK) $(ARNL_BASE_LINK)

This will build any program, linking to the ARNL laser localization library as well as ArnlBase
and other required libraries.  libpthread, libdl and librt are standard Linux system
libraries used by ARIA.   See the GNU Make documentation for more information on 
using Makefiles to build programs.

For more information on using and writing programs with ARNL, see the introductory manuals
and BaseArnl, ARNL/SONARNL/MOGS, ARIA and ArNetworking Reference manuals contained
in the "docs" directory, as well as the example programs in the "examples" directory.



Java and Python
===============

ARNL also includes "wrapper" libraries, which provide an interface
layer between a Java or Python API and the native C++ libraries, allowing
you to access most parts of the libraries enirely from the Python or Java
language. The APIs in those languages are very similar to the native
C++ library.  The Java wrapper library can be found in the 'java' subdirectory,
and the Python wrapper library (module) can be found in the 'python' 
subdirectory. For example programs and details on setting up your Java or 
Python environment to use the wrapper libraries, see the 'javaExamples' or 
'pythonExamples' directory.





