This directory has various example pieces of code and Microsoft VC++ 
project files.

To compile these in Windows open arnlExamples.sln for laser navigation
with ARNL or sonarExamples.sln for sonar navigation with ARNL or
SONARNL.

To compile the primary example(s) in Linux type 'make'. With SONARNL this will
build sonarnlServer, with ARNL it'll compile arnlServer, with MOGS mogsServer.  

To compile any other individual example named <example>.cpp, run
'make <example>'. E.g. to build justPathPlanning.cpp, run 
'make justPathPlanning'.  This works for any program in this directory,
so you can copy examples to new names, modify them, and build them
using this method.

To compile all programs present, run 'make all'.

To remove all compiled examples, run 'make clean'.

Some examples are in the Base package, others come with a localization
package. The "guiServer" example provided with older versions of Arnl
and SonArnl has been split into three seperate servers for each
localization method, arnlServer, sonarnlServer, and mogsServer, plus
combinedServer which combines them all and whose behavior is modified
by preprocessor symbols.


Examples:

  arnlServer.cpp (from ARNL package)
                      Primary ARNL example with laser localization and  
                      networking (connect with MobileEyes)

  sonarnlServer.cpp (from SONARNL package)
                      Primary SONARNL example with sonar localization and 
                      networking (connect with MobileEyes)

  mogsServer.cpp (from MOGS package)
                      Primary MOGS example with GPS localization and networking
                      (connect with MobileEyes)

  justPathPlanning.cpp (Base)
                      Example server that only does path planning, and no
                      localization. (Add your own localization)

  arnlJustLocalization.cpp (ARNL package)
                      Version of arnlServer.cpp that omits path planning. (Add
                      your own path planning)

  sonarnlJustLocalization.cpp (SONARNL package)
                      Version of sonarnlServer.cpp that omits path planning.
                      (Add your own path planning)

  mogsJustLocalization.cpp (MOGS package)
                      Version of mogsServer.cpp that omits path planning. (Add
                      your own path planning)

  combinedServer.cpp   (Base)
                      More complex example that depends on which localization
                      libraries are available.

  combinedJustLocalization.cpp   (Base)
                      More complex example that depends on which localization
                      libraries are available.

  centralServerExample.cpp (Base)
                      Example of a "central server" that servers running on
                      multiple robots can connect to and provide a single
                      point of contact for clients such as MobileEyes, and 
                      also share some data to plan around each other.

  sickLogger.cpp  (Arnl)
                      Utility that makes a laser scan log (.2d) file.
                      (arnlServer can also do this through MobileEyes'
                      Start/Stop Mapping commands.)

  columbia.map    (Base)
                      Example map. Load it into both the simulator and a server.

