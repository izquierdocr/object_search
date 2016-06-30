#ifndef AREXPORT_H
#define AREXPORT_H

// Including this header file forces AREXPORT to be defined as the dllexport declaration in Windows DLL builds.
// It should only be included in source files in ARIA and other libraries. Header files should include ariaTypedefs instead.

#if (defined(_WIN32) || defined(WIN32)) && !defined(MINGW)

#ifndef SWIG
#ifndef ARIA_STATIC
#undef AREXPORT
#define AREXPORT _declspec(dllexport)
#else // ARIA_STATIC
#define AREXPORT
#endif // ARIA_STATIC
#endif // SWIG

#else // WIN32

#define AREXPORT

#endif // WIN32

#endif // AREXPORT_H


