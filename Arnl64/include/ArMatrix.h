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
/* ****************************************************************************
 * 
 * File: ArMatrix.h
 * 
 * Function: Header for the matrix class.
 *
 * Created:  George V. Paul. gvp@activmedia.com. January 11 2006.
 *
 *****************************************************************************/
#ifndef ARMATRIX_H
#define ARMATRIX_H
#include <math.h>
#include "Aria.h"

class ArMatrix
{

public:
  /// Constructor.
  ArMatrix(void)
  {
    myRows = 0;
    myCols = 0;
    myElement = NULL;
    retval = 0.0;
  }
  /// Constructor.
  ArMatrix(int rows, int cols = 1) 
  {
    myRows = rows;
    myCols = cols;
    allocate(rows, cols);
    set(0.0);
    retval = 0.0;
  }
  /// Constructor.
  ArMatrix(int rows, int cols, double val) 
  {
    myRows = rows;
    myCols = cols;
    allocate(rows, cols);
    setDiagonal(val);
    retval = 0.0;
  }
  /// Copy constructor
  ArMatrix(const ArMatrix& mat) 
  { 
    allocate(mat.myRows, mat.myCols);
    copy(mat); 
  }
  // Assignment operator function. overloads the equal sign operator
  ArMatrix& operator=(const ArMatrix& mat) 
  {
    if(this == &mat) // If two sides equal, do nothing.
    {
      return *this;  
    }
    else if(myElement && myRows == mat.myRows && myCols == mat.myCols)    
    {    
      copy(mat);   // Copy right hand side to l.h.s.
      return *this;
    }
    else if(myElement)
    {
      deallocate();
    }
    allocate(mat.myRows, mat.myCols);
    copy(mat);       // Copy right hand side to l.h.s.
    return *this;
  }
  /// Destructor
  ~ArMatrix() 
  {
    if(myElement)
      deallocate();
  }
  /// Matrix access
  double& operator() (int i, int j = 0)
  {
    if(i < 0 || i >= myRows || j < 0 || j >= myCols)
    {
      ArLog::log(ArLog::Normal, "ArMatrix::Illegal index %d/%d  %d/%d", 
		 i, myRows, j, myCols);
      return retval;
    }
    return myElement[i][j];
  }
  const double& operator() (int i, int j = 0) const
  {
    if(i < 0 || i >= myRows || j < 0 || j >= myCols)
    {
      ArLog::log(ArLog::Normal, "ArMatrix::Illegal index %d/%d  %d/%d", 
		 i, myRows, j, myCols);
      return retval;
    }
    return myElement[i][j];
  }
  /// Matrix add
  ArMatrix operator+(ArMatrix mat)
  {
    if(mat.getRows() != myRows || mat.getCols() != myCols)
    {
      ArLog::log(ArLog::Normal, "ArMatrix::Illegal + %d, %d  %d, %d", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
    }
    ArMatrix sum = *this;
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	sum(i, j) += mat(i, j);
    return sum;
  }
  /// Matrix subtract
  ArMatrix operator-(ArMatrix mat)
  {
    if(mat.getRows() != myRows || mat.getCols() != myCols)
    {
      ArLog::log(ArLog::Normal, "ArMatrix::Illegal - %d, %d  %d, %d", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
    }
    ArMatrix sum = *this;
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	sum(i, j) -= mat(i, j);
    return sum;
  }
  /// Matrix multiply.
  ArMatrix operator*(ArMatrix mat)
  {
    if(mat.getRows() != myCols)
    {
      ArLog::log(ArLog::Normal, "ArMatrix::Illegal * %d, %d  %d, %d", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
    }
    return multiply(mat);
  }
  /// Matrix multiply.
  ArMatrix operator*(double m)
  {
    ArMatrix out(myRows, myCols);
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	out(i, j) = myElement[i][j] * m;
    return out;
  }
  // Set all elements to val.
  void set(double val)
  {
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = val;
  }
  /// Constructor for identity matrix.
  void setDiagonal(double val) 
  {
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = (i == j) ? val : 0.0;
  }
  /// Multiply with another matrix.
  ArMatrix multiply(ArMatrix& in)
  {
    int m = myRows;
    int n = myCols;
    int p = in.getRows();
    int q = in.getCols();
    ArMatrix out(m, q);
    if(n != p)
      return out;
    for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < q; j++)
      {
	double acc = 0.0;
	for(int k = 0; k < n; k++)
	{
	  acc += myElement[i][k]*in(k, j);
	}
	out(i, j) = acc;
      }
    }
    return out;
  }
  /// Transpose the matrix.
  ArMatrix transpose(void)
  {
    ArMatrix out(myCols, myRows);

    for(int i = 0; i < myRows; i++)
    {
      for(int j = 0; j < myCols; j++)
      {
	out(j, i) = myElement[i][j];
      }
    }
    return out;
  }
  /// Print matrix.
  void print(const char* header=NULL, FILE* fp=stdout)
  {
    char buffer[10000];
    buffer[0] = '\0';
    std::string line;
    if(header!=NULL)
    {
      if(myRows == 1)
      {
	sprintf(buffer, "%s [%d %d]\t", header, myRows, myCols);
      }
      else
      {
	sprintf(buffer, "%s [%d %d]\n", header, myRows, myCols);
	line += buffer;
	buffer[0] = '\0';
      }
    }
    for(int i = 0; i < myRows; i++)
    {
      for(int j = 0; j < myCols; j++)
      {
	sprintf(buffer, "%s%5.2f\t", buffer, myElement[i][j]);
      }
      if(i < myRows-1)
      {
	sprintf(buffer, "%s\n", buffer);
	line += buffer;
	buffer[0] = '\0';
      }
    }
    line += buffer;
    ArLog::log(ArLog::Normal, line.c_str());
  }
  /// Print matrix.
  void printDiagonal(char* header=NULL, FILE* fp=stdout)
  {
    char buffer[2064];
    if(header!=NULL)
      sprintf(buffer, "%s [%d %d]\t", header, myRows, myCols);
  
    if(myCols == 1)
    {
      for(int i = 0; i < myRows; i++)
	sprintf(buffer, "%s %5.2f\t", buffer, myElement[i][0]);
    }
    else if(myRows == 1)
    {
      for(int j = 0; j < myCols; j++)
	sprintf(buffer, "%s%5.2f\t", buffer, myElement[0][j]);
    }
    else
    {
      for(int i = 0; i < myRows; i++)
      {
	for(int j = 0; j < myCols; j++)
	  if(i == j)
	    sprintf(buffer, "%s%5.2f\t", buffer, myElement[i][j]);
      }
    }
    ArLog::log(ArLog::Normal, buffer);
  }
  /// Get the i,j element;
  double getElement(int i, int j) { return myElement[i][j];}
  /// Get the no of rows.
  int getRows(void) { return myRows;}
  /// Get the no of cols
  int getCols(void) { return myCols;}
  /// Set the i,j element;
  void setElement(int i, int j, double d) { myElement[i][j] = d;}
  /// Inverse of the matrix if square.
  AREXPORT double inverse(ArMatrix& Ainv);

protected:
  // Private copy function.
  // Copies values from one Matrix object to another.
  void copy(const ArMatrix& mat) 
  {
    myRows = mat.myRows;
    myCols = mat.myCols;
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = mat.myElement[i][j];
    retval = mat.retval;
  }
  void allocate(int rows, int cols)
  {
    if(!(myElement = new double*[rows]))
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Cannot allocate memory\n");
      return;
    }
    for(int i = 0; i < rows; i++)
      if(!(myElement[i] = new double[cols]))
      {
	ArLog::log(ArLog::Normal, "\nArMatrix::Cannot allocate memory\n");
	return;
      }
  }
  void deallocate(void)
  {
    for(int i = 0; i < myRows; i++)
      if(myElement[i])
      {
	delete [] myElement[i];
	myElement[i] = NULL;
      }
    if(myElement)
      delete [] myElement;
    myElement = NULL;
  }
protected:
  int myRows, myCols;
  double** myElement;
  double retval;
};

#endif // ARMATRIX_H
