#ifndef ARTYPEDEFS_H
#define ARTYPEDEFS_H

#include <time.h>
#include <string>
#include <map>
#include <list>

#ifdef WIN32


#ifndef SWIG
#if !defined(ARIA_STATIC) && !defined(AREXPORT) && !defined(MINGW)
#define AREXPORT _declspec(dllimport)
#elif !defined(AREXPORT) // ARIA_STATIC
#define AREXPORT
#endif // ARIA_STATIC
#else
#define AREXPORT
#endif

#include <winsock2.h>
#include <windows.h>

#endif //WIN32L


#ifndef WIN32

#define AREXPORT
////
//// Linux
////

#endif // linux


typedef std::map<int, std::string> ArStrMap;

/// has enum for position in list
class ArListPos
{
public:
  typedef enum {
      FIRST = 1, ///< place item first in the list
      LAST = 2 ///< place item last in the list
  } Pos;
};

/// Contains platform independent sized variable types
class ArTypes
{
public:
  /// A single signed byte
  typedef char Byte;
  /// Two signed bytes
  typedef short Byte2;
  /// Four signed bytes
  typedef int Byte4;
  /// Eight signed bytes
  typedef long long Byte8;

  /// A single unsigned byte
  typedef unsigned char UByte;
  /// Two unsigned bytes
  typedef unsigned short UByte2;
  /// Four unsigned bytes
  typedef unsigned int UByte4;
  /// Eight unsigned bytes
  typedef unsigned long long UByte8;
};


#endif
