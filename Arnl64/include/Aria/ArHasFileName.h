#ifndef ARHASFILENAME_H
#define ARHASFILENAME_H

#include "ariaTypedefs.h"

/// Interface to access an object's associated file name.
/**
 * ArHasFileName provides a single abstract method which should be overridden
 * to return the complete file path name of the associated file.  It is 
 * implemented by classes that have external persistent storage, such as 
 * ArConfig and ArMap.
 * 
 * Copyright (c) Adept Technology, Inc. All rights reserved.
**/
class ArHasFileName
{
public:

  /// Constructor
	AREXPORT ArHasFileName() 
  {}
	
  /// Copy constructor
  ArHasFileName(ArHasFileName const &) 
  {}

	/// Assignment operator.
  ArHasFileName &operator=(ArHasFileName const & )
  {
    return *this;
  }

  /// Destructor
	AREXPORT virtual ~ArHasFileName()
  {}

  /// Returns the complete file path name of the associated file
  AREXPORT virtual const char *getFileName() const  = 0;

}; // end class


#endif // ARHASFILENAME_H

