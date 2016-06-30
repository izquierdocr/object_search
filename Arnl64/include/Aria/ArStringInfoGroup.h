#ifndef ARSTRINGINFOGROUP_H
#define ARSTRINGINFOGROUP_H

#include "ariaUtil.h"
#include "ArMutex.h"
#include <string>
#include <set>
#include <list>

/**
   This class takes callbacks from different classes that want this
   string information and then lets you just add the information here
   instead of to each individual class.

  @ingroup OptionalClasses
 **/
class ArStringInfoGroup
{
public:
  /// Constructor
  AREXPORT ArStringInfoGroup();
  /// Destructor
  AREXPORT virtual ~ArStringInfoGroup();
  /// Adds a string to the list in the raw format
  AREXPORT bool addString(const char *name, ArTypes::UByte2 maxLen, 
			  ArFunctor2<char *, ArTypes::UByte2> *functor);

  /// Adds an int to the list in the helped way
  AREXPORT bool addStringInt(const char *name, ArTypes::UByte2 maxLen, 
			     ArRetFunctor<int> *functor, 
			     const char *format = "%d");

  /// Adds a double to the list in the helped way
  AREXPORT bool addStringDouble(const char *name, ArTypes::UByte2 maxLen, 
				ArRetFunctor<double> *functor, 
				const char *format = "%g");

  /// Adds a bool to the list in the helped way
  AREXPORT bool addStringBool(const char *name, ArTypes::UByte2 maxLen, 
			      ArRetFunctor<bool> *functor,
			      const char *format = "%s");

  /// Adds a string to the list in the helped way
  AREXPORT bool addStringString(const char *name, ArTypes::UByte2 maxLen, 
			      ArRetFunctor<const char *> *functor,
			      const char *format = "%s");

  /// Adds an int to the list in the helped way
  AREXPORT bool addStringUnsignedLong(const char *name, 
				      ArTypes::UByte2 maxLen, 
				      ArRetFunctor<unsigned long> *functor, 
				      const char *format = "%lu");

  /// Adds an int to the list in the helped way
  AREXPORT bool addStringLong(const char *name, 
			      ArTypes::UByte2 maxLen, 
			      ArRetFunctor<long> *functor, 
			      const char *format = "%ld");

  /// This is the function to add a callback to be called by addString
  AREXPORT void addAddStringCallback(
	  ArFunctor3<const char *, ArTypes::UByte2,
	  ArFunctor2<char *, ArTypes::UByte2> *> *functor,
	  ArListPos::Pos position = ArListPos::LAST);

protected:
  ArMutex myDataMutex;
  std::set<std::string, ArStrCaseCmpOp> myAddedStrings;
  std::list<ArFunctor3<const char *, ArTypes::UByte2,
	      ArFunctor2<char *, ArTypes::UByte2> *> *> myAddStringCBList;
};


#endif // ARSTRINGINFOHELPER_H
