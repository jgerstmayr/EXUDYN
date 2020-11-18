#ifndef FILE_NGSTD_CORE
#define FILE_NGSTD_CORE

#include <iostream>
#include <string>
#include <exception>
#include <complex>
#include <atomic>
#include <memory>
#include <list>
#include <tuple>
#include <mutex>
#include <iomanip>
#include <cstring>
#include <climits>
#include <thread>
#include <functional>

//add this to avoid definition of min / max in windows.h
#define NOMINMAX

#ifdef __INTEL_COMPILER
#ifdef WIN32
#define ALWAYS_INLINE __forceinline
#define INLINE __forceinline inline
#define LAMBDA_INLINE
#else
#define ALWAYS_INLINE __forceinline
#define INLINE __forceinline inline
#define LAMBDA_INLINE __attribute__ ((__always_inline__))
#endif
#else
#ifdef __GNUC__
#define ALWAYS_INLINE __attribute__ ((__always_inline__))
#define INLINE __attribute__ ((__always_inline__)) inline
#define LAMBDA_INLINE __attribute__ ((__always_inline__))
// #ifndef __clang__
#define VLA
// #endif
#else
#define ALWAYS_INLINE
#define INLINE inline
#define LAMBDA_INLINE
#endif
#endif

#ifndef __assume
#ifdef __GNUC__
#ifdef __clang__
#define __assume(cond) __builtin_assume(cond)
#else
#define __assume(cond) if (!(cond)) __builtin_unreachable(); else;
#endif
#else
#define __assume(cond)
#endif
#endif

// #ifdef __clang__
#if defined __GNUC__ and not defined __INTEL_COMPILER
namespace std
{
  // avoid expensive call to complex mult by using the grammar school implementation
  INLINE std::complex<double> operator* (std::complex<double> a, std::complex<double> b)
  {
    return std::complex<double> (a.real()*b.real()-a.imag()*b.imag(),
                                 a.real()*b.imag()+a.imag()*b.real());
  }
}
#endif

#ifdef PARALLEL
#include <unistd.h>  // for usleep (only for parallel)
#endif

#ifdef WIN32
   #ifdef NGINTERFACE_EXPORTS
      #define DLL_HEADER   __declspec(dllexport)
   #else
      #define DLL_HEADER   __declspec(dllimport)
   #endif

   #ifdef NGS_EXPORTS
      #define NGS_DLL_HEADER   __declspec(dllexport)
   #else
      #define NGS_DLL_HEADER   __declspec(dllimport)
   #endif
#else
   #define DLL_HEADER 
// #define NGS_DLL_HEADER 


/*
   #ifdef NGINTERFACE_EXPORTS
      #define DLL_HEADER   __declspec(dllexport)
   #else
      #define DLL_HEADER   __declspec(dllimport)
   #endif
*/

   #ifdef NGS_EXPORTS
      #define NGS_DLL_HEADER   __attribute__ ((visibility ("default")))
   #else
      #define NGS_DLL_HEADER
   #endif
#endif

namespace ngstd {
  using namespace std;
}

#include "ngs_utils.hpp"    
#include "templates.hpp"
#include "exception.hpp"
#include "localheap.hpp"
#include "profiler.hpp"

#include "simd.hpp"
#include "simd_complex.hpp"
#include "array.hpp"
#include "table.hpp"
#include "hashtable.hpp"
#include "bitarray.hpp"

#include "autodiff.hpp"
#include "taskmanager.hpp"

/// namespace for basic linear algebra
namespace ngbla
{
  using namespace std;
  using namespace ngstd;

  using ngstd::CArray;

#ifdef USE_MYCOMPLEX
  typedef MyComplex<double> Complex;
  inline double fabs (Complex v) { return ngstd::abs (v); }
#else
  // typedef std::complex<double> Complex;
  inline double fabs (Complex v) { return std::abs (v); }
#endif

  using ngstd::MyAtomicAdd;
  inline void MyAtomicAdd (Complex & x, Complex y)
  {
    auto real = y.real();
    ngstd::MyAtomicAdd (reinterpret_cast<double(&)[2]>(x)[0], real);
    auto imag = y.imag();
    ngstd::MyAtomicAdd (reinterpret_cast<double(&)[2]>(x)[1], imag);
  }

  inline bool IsComplex(double v) { return false; }
  inline bool IsComplex(Complex v) { return true; }
}

#ifdef PARALLEL
namespace ngstd
{
  template <>
  class MPI_Traits<ngbla::Complex>
  {
  public:
    /// gets the MPI datatype
    static MPI_Datatype MPIType () 
    { 
      // return MPI_C_DOUBLE_COMPLEX;   // no MPI_SUM defined ??
      return MPI_DOUBLE_COMPLEX;
    }
  };
}
#endif

using ngstd::Range;

#include "expr.hpp"
#include "vector.hpp"
#include "matrix.hpp"

#endif // FILE_NGSTD_CORE
