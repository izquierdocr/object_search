
#ifndef ARIA_wrapper_Functors_h
#define ARIA_wrapper_Functors_h

/* For Python, define ArFunctor subclasses to hold Python C library
 * callable function objects.  These are used internally by the 
 * wrapper library, and typemaps convert target-language function 
 * objects to these ArFunctor subclasses-- you only need to pass
 * the function to Aria when in the C++ API you would pass a Functor.
 *
 * For Java, define subclasses of ArFunctor for various argument types,
 * since you can't access template classes in Java.  Then you can 
 * further subclass within Java and pass that object to Aria.
 */

#include "ArFunctor.h"

/* Functiors for Python: */

#ifdef SWIGPYTHON
class ArPyFunctor : public ArFunctor 
{
protected:
  PyObject* pyFunction;
public:
  ArPyFunctor(PyObject* _m) : pyFunction(_m) {
    Py_INCREF(pyFunction);
  }

  virtual void invoke() { 
    PyObject* r = PyObject_CallObject(pyFunction, NULL);
    if(!r) {
      fputs("** ArPyFunctor: Error calling Python function: ", stderr);
      PyErr_Print();
    }
  }

  virtual ~ArPyFunctor() {
    Py_DECREF(pyFunction);
  }

  virtual const char* getName() {
    return (const char*) PyString_AsString(PyObject_Str(pyFunction));
  }
};


class ArPyRetFunctor_Bool : 
  public ArRetFunctor<bool>,
  public ArPyFunctor
{
public:
  ArPyRetFunctor_Bool(PyObject* _m) : ArRetFunctor<bool>(), ArPyFunctor(_m) {
  }

  virtual bool invokeR() {
    PyObject* r = PyObject_CallObject(pyFunction, NULL);  
    if(!r) {
      fputs("** ArPyRetFunctor_Bool: Error calling Python function: ", stderr);
      PyErr_Print();
    }
    return(r == Py_True);
  }

  virtual const char* getName() {
    return (const char*) PyString_AsString(PyObject_Str(pyFunction));
  }
};

#endif // PYTHON




#endif // wrapperFunctors.h
