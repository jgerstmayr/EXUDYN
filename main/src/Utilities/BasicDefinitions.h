/** ***********************************************************************************************
* @file			BasicDefinitions.h
* @brief		This file contains definitions, not helper functions and not traits;
* @details		Details:
* 				- type definitions related to Utilities and Linalg
*
* @author		Gerstmayr Johannes
* @date			2018-04-30 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

#ifndef BASICDEFINITIONS__H
#define BASICDEFINITIONS__H

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//this part contains important definition of flags to setup the compiled module
#if defined(__linux) || defined(__linux__) || defined(__unix__)
#define __EXUDYN__LINUX__ //with any processor
#endif

#if defined(__GNUC__) //this is for the compiler - but most probably, all non-windows and non-apple compiler behave similar
#define __EXUDYN__GNUC__
#endif

//in case of _WIN32 we also assume _x86 ...; __i386__ not shown on VS2017:
#if defined(__x86_64) || defined(__x86_64__) || defined(__amd64__) || defined(__x86) || defined(__i386__) || defined(_WIN32) 
#define __EXUDYN__x86__ //Intel or AMD processor //tested also on gcc/linux/64bits!
#endif

//detect ARM for AppleM1 or raspberry Pi
//AppleM1 gcc has activated  __aarch64__ and __ARM_ARCH flags
#if defined(__arm__) || defined(__aarch64__) || defined(__ARM_ARCH) //ARM architecture: RaspberryPi on UbuntuMate 20.04 (64bits): gcc shows __aarch64__ but not __arm__
#define __EXUDYN__ARM__
#endif


//32bit/64 bit should be checked with length of void

//check some platform / architecture or compiler specific things at which is compiled and define globally used flags:
#if defined(__APPLE__)
#define __EXUDYN__APPLE__
#elif defined(_WIN32) || defined(_WIN64)
#define __EXUDYN__WINDOWS__
#elif defined(__arm__) || defined(__aarch64__) || defined(__ARM_ARCH) //ARM architecture: RaspberryPi on UbuntuMate 20.04 (64bits): gcc shows __aarch64__ but not __arm__
#define __EXUDYN__LINUX__ARM__
#elif defined(__EXUDYN__LINUX__) && defined(__EXUDYN__x86__)
#define __EXUDYN__LINUX__x86__ //linux with any x86 type CPU (AMD, Intel)
#else
#pragma message("WARNING: no architecture identified for compilation; check BasicDefinitions.h!!!")
#endif

//unused so far:
//#if defined(__GNUC__)
//#define __EXUDYN__GNUC__
//#endif

//#define EXUDYN_RELEASE			//!< defined in preprocessor flags, in setup.py (for all versions), set this flag to exclude experimental parts of the code
#define _USE_MATH_DEFINES		//!< this must be included very first before any cmath is included; needed for M_PI and other constants ==> but not used anymore

//#define __NOGLFW //passed from compiler
#ifndef __NOGLFW //passed from compiler
  #define USE_GLFW_GRAPHICS		//!< set this flag to enable OpenGL graphics with glfw
#endif
//#define FLIP_NORMALS //!< lets flip normals to point inside objects in some internal triangle drawing functions (sphere, ...) (old mode before 2022-06-27)
//#define FLIP_TRIANGLES  //!< lets flip triangle orientation in some internal triangle drawing functions (sphere, ...) (old mode before 2022-06-27)

//#define PERFORM_UNIT_TESTS	//!< defined in preprocessor flags, in setup.py (only for certain versions)
#define DoublePrecision

//use approach here: https://github.com/robbmcleod/cpufeature/tree/master/cpufeature
//  to automatically detect if AVX is available and show error messages (or try to automatically select according .pyd file in __init__.py)

#ifdef DoublePrecision
	typedef double Real;		//!< define datatype Real; use typedef for eigen lib
#else 
	typedef float Real;
	#define SinglePrecision // added from SH. needed to switch between double/float AVX instructions
#endif // DoublePrecision


#ifdef __AVX2__				//enabled by compiler; will also create many intrinsics automatically (e.g. for SlimVector<4>)
	#define use_AVX2		//!< this is used for specific vector operations, e.g., in Vector.AddLarge(...)
	//#define use_AVX512
	#ifdef use_AVX2
		#ifdef DoublePrecision
			#define exuMemoryAlignment 4		//alignment of Real times sizeof(Real) for vectors; for AVX	
			#define exuVectorLengthAlignment 8	//alignment in Real for vectors; for AVX	
		#else
			#define exuMemoryAlignment 8		//alignment of Real times sizeof(Real) for vectors; for AVX	
			#define exuVectorLengthAlignment 16 //alignment in Real for vectors; for AVX	
		#endif
	#endif
	#ifdef use_AVX512
		#ifdef DoublePrecision
			#define exuMemoryAlignment 8		//alignment of Real times sizeof(Real) for vectors; for AVX	
			#define exuVectorLengthAlignment 16	//alignment in Real for vectors; for AVX	
		#else
			#define exuMemoryAlignment 16		//alignment of Real times sizeof(Real) for vectors; for AVX	
			#define exuVectorLengthAlignment 32 //alignment in Real for vectors; for AVX	
		#endif
#endif
#else
	#define exuMemoryAlignment 1	   //alignment of Real times sizeof(Real) for vectors; for AVX	
	#define exuVectorLengthAlignment 1 //alignment in Real for vectors; for optimized operations
#endif

//Define empty argument (used for some macros)
#define EXU_NOARG 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//MULTITHREADED computation using ngsolve taskmanager; thanks to Joachim Schöberl
#if !defined(__APPLE__) //currently simd makes problems on different Apple platforms - needs sse2neon.h
#define USE_NGSOLVE_TASKMANAGER //!< for multithreaded computation
#endif
//#undef USE_NGSOLVE_TASKMANAGER //!< for multithreaded computation

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define __STDC_WANT_LIB_EXT1__ 1 //==> according to cppreference this shall enable strcpy_s, but does not work in gcc
#include <string> //std::string
#include <cstdint> //for uint32_t
#include <limits>  //not tested if works with MAC or linux

//#include "System/versionCpp.h" //exclude here, in order to avoid re-compile when version changes

typedef std::string STDstring;	//!< decouple std::string for future extensions, performance, etc.; all Exudyn strings must be STDstring; do not use std::string directly!


#define USE_INDEX_AS_INT
#ifdef USE_INDEX_AS_INT
	typedef int Index;
	typedef int SignedIndex;
	#if defined(__EXUDYN__APPLE__)
		typedef unsigned long UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
	#elif defined(__x86_64__) || defined(__ppc64__) || defined(_WIN64)
		typedef uint64_t UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
	#else
		typedef uint32_t UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
	#endif
#else
	//not that Index will become 'int' in future (but python interface Index will represent only positive indices ...)!
	#if defined(__EXUDYN__APPLE__)
		typedef unsigned long Index;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef unsigned long UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef long SignedIndex;	//!< for indices which need a sign (e.g. in linear solver); try to avoid!
	#elif defined(__x86_64__) || defined(__ppc64__) || defined(_WIN64)
		typedef uint64_t Index;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef uint64_t UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef int64_t SignedIndex;	//!< for indices which need a sign (e.g. in linear solver); try to avoid! 
	#else
		typedef uint32_t Index;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef uint32_t UnsignedIndex;			//!< all indices used in Exudyn must be declared with Index, not 'int' (64 bit portability)
		typedef int32_t SignedIndex;	//!< for indices which need a sign (e.g. in linear solver); try to avoid! 
	#endif
#endif

//not needed any more, as all structures are dynamic: constexpr Index MAX_NUMBER_OF_THREADS = 16;   //!< maximum number of threads, e.g., for predefined structures with fixed size


#include "Main/Stdoutput.h"		//for pout and error/warning messages

//DELETE: (openmp is much less performant than manual multithreading)
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
////#define USE_OPENMP //needs omp.h, not available on MacOS; currently only used in CSystem.cpp for some tests
//
//#ifdef USE_OPENMP
//const int maxThreads = 12; //adjust this for supercomputers, if necessary
//#endif
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




typedef uint32_t UInt;          //!< explicitly used for smaller indices

#define EXUINLINE inline // flag is compatible with netgen, see ngs_core.hpp

//define EXUstd constants
namespace EXUstd {

	const Real pi = 3.14159265358979323846; //as cmath does not work properly (#define _USE_MATH_DEFINES must be included everywhere), pi is defined here
	const float pi_f = (float)pi;
	const Index InvalidIndex = -1; //!< invalid index used e.g. in GetIndexOfItem, etc.
	const float _MINFLOAT = -1e38f; //!< largest negative value to be on the safe side; @TODO use std lib constants instead
	const float _MAXFLOAT =  1e38f; //!< largest positive value to be on the safe side

	constexpr Index dim3D = 3; //!< this shall make changes to other dimensionalities easier; avoid using 3 in code
	constexpr Index dim2D = 2; //!< this shall make changes to other dimensionalities easier; avoid using 2 in code

	//not tested if works with MAC or linux
	constexpr double LOWESTREAL = std::numeric_limits<Real>::lowest(); //lowest (neg) Real number
	constexpr double MAXREAL = std::numeric_limits<Real>::lowest();  //highest (pos) Real number

	//empty class for default initialization, cannot be converted from e.g. Real, Index, etc.
	class Dummy
	{
	};

	inline STDstring GetPlatformString()
	{
		STDstring s;
	#if defined(__EXUDYN__WINDOWS__)
			s += "Windows";
		#if !defined(_WIN64)
			s += "(32bit)";
		#endif
	#endif
	#if defined(__EXUDYN__APPLE__)
		s += "MacOS";
	#endif
		#if defined(__EXUDYN__LINUX__)
			s += "Linux";
		#if defined(__EXUDYN__LINUX__ARM__)
			s += "(ARM)";
		#endif
	#endif
	#if defined(use_AVX512)
		s += " AVX512";
	#elif defined(use_AVX2)
		s += " AVX2";
	#endif
	#ifdef DoublePrecision
		s += " FLOAT64";
	#else 
		s += " FLOAT32";
	#endif 

		return s;
	}


} //EXUstd

extern bool linalgPrintUsePythonFormat; //!< defined in Vector.cpp; true: use python format for output of vectors and matrices; false: use matlab format

/** ***********************************************************************************************
* @mainpage Getting started with Exudyn
* @section intro_sec Introduction
    This section contains important information about the basic idea, the general structure, copyright and FAQ.

* @subsection general_sec General information
    This section contains important information about the basic idea, the general structure, copyright and FAQ.

    Exudyn was developed for the efficient simulation of flexible multi-body systems. Exudyn was designed for rapid implementation and testing of new formulations and algorithms in multibody systems, whereby these algorithms can be easily implemented in efficient C++ code. Furthermore, an application in industry-related research projects and applications is planned.

    Four principles: developer-friendly, error minimization, efficiency, user-friendliness

    The focus is therefore on:
    - A developer-friendly basic structure regarding the C++ class library and the possibility to add new components.
    - The basic libraries are slim, but extensively tested; only the necessary components are available
    - Complete unit tests are added to new program parts during development; for more complex processes, tests are available in Python
    - In order to implement the sometimes difficult formulations and algorithms without errors, error avoidance is always prioritized.
    - To generate efficient code, classes for parallelization (vectorization and multithreading) are provided. We live the principle that parallelization takes place on multi-core processors with a central main memory, and thus an increase in efficiency through parallelization is only possible with small systems, as long as the program runs largely in the cache of the processor cores. Vectorization is tailored to SIMD commands as they have Intel processors, but could also be extended to GPGPUs in the future.
    - The user interface (Python) provides a 1:1 image of the system and the processes running in it, which can be controlled with the extensive possibilities of Python.

* @subsection core_structure_sec Core structure of Exudyn
	- CSystem / MainSystem: a multibody system which consists of nodes, objects, markers, loads, etc.
	- SystemContainer: holds a set of systems; connects to visualization (container)
	- node: used to hold coordinates (unknowns)
	- (computational) object: leads to equations, using nodes
	- marker: defines a consistent interface to objects (bodies) and nodes; write access ('AccessFunction') -- provides jacobian and read access ('OutputVariable')
	- load: acts on an object or node via a marker
	- computational objects: efficient objects for computation = bodies, connectors, connectors, loads, nodes, ...
	- visualization objects: interface between computational objects and 3D graphics
	- main (manager) objects: do all tasks (e.g. interface to visualization objects, GUI, python, ...) which are not needed during computation
	- static solver, kinematic solver, time integration
	- python interface via pybind11; items are accessed with a dictionary interface; system structures and settings read/written by direct access to the structure (e.g. SimulationSettings, VisualizationSettings)
	- interfaces to linear solvers; future: optimizer, eigenvalue solver, ... (mostly external or in python)

* @subsection structure_sec Internal structure of Exudyn
    Libraries:
	- Autogenerated: item (nodes, objects, markers and loads) classes split into main (management, python connection), visualization and computation
	- Graphics: a general data structure for 2D and 3D graphical objects and a tiny openGL visualization; linkage to GLFW
    - Linalg: Linear algebra with vectors and matrices; separate classes for small vectors (SlimVector), large vectors (Vector and ResizableVector), vectors without copying data (LinkedDataVector), and vectors with constant size (ConstVector)
	- Main: mainly contains SystemContainer, System and ObjectFactory
	- Objects: contains the implementation part of the autogenerated items
	- Pymodules: manually created libraries for linkage to python via pybind; remaining linking to python is located in autogenerated folder
	- pythonGenerator: contains python files for automatic generation of C++ interfaces and python interfaces of items;
	- Solver: contains all solvers for solving a CSystem
	- System: contains core item files (e.g., MainNode, CNode, MainObject, CObject, ...)
	- Tests: files for testing of internal linalg (vector/matrix), data structure libraries (array, etc.) and functions
    - Utilities: array structures for administrative/managing tasks (indices of objects ... bodies, forces, connectors, ...); basic classes with templates and definitions

* @subsection ext_structure_sec External structure of Exudyn
	External libraries:
	- LEST: for testing of internal functions (e.g. linalg)
	- GLFW: 3D graphics with openGL; cross-platform capabilities
	- Eigen: linear algebra for large matrices, linear solvers, sparse matrices and link to special solvers
	- pybind11: linking of C++ to python


* @subsection rules_sec General Coding rules and conventions
    Developers must comply with the following coding rules:
    - write simple code
    - write readable code
    - add complete unit test to every function (every file has link to LEST library)
    - put a header in every file, according to Doxygen format
    - put a comment to every (global) function, member function, data member, template parameter
    - ONE class ONE file rule
    - avoid large classes (>30 member functions; > 15 data members)
    - split up god classes (>60 member functions)
    - mark changed code with your name and date
    - USE 4-spaces-tab
    - REPLACE tabs by spaces: Extras->Options->C/C++->Tabstopps: tab stopp size = 4 (=standard) +  KEEP SPACES=YES
    - use C++11 standards when appropriate, but not exhaustively
    - ALWAYS USE curly brackets for single statements in 'if', 'for', etc.; example: if (i<n) {i += 1;}
    - use Doxygen-style comments (use '//!' Qt style and '@ date' with '@' instead of '\' for commands)
    - use Doxygen (with preceeding '@') 'test' for tests, 'todo' for todos and 'bug' for bugs

* @subsubsection abbreviations_sec Abbreviations in Exudyn
    The code uses a minimum set of abbreviations; however, the following abbreviation rules are used throughout:
    In general: DO NOT ABBREVIATE function, class or variable names: GetDataPointer() instead of GetPtr(); exception: cnt, i, j, k, x or v in cases where it is really clear (5-line member functions)
    ABBREVIATION ECEPTIONS:
    - ODE ... ordinary differential equations;
    - ODE2 ... marks parts related to second order differential equations (SOS2, EvalF2 in HOTINT1)
    - ODE1 ... marks parts related to first order differential equations (ES, EvalF in HOTINT1)
    - AE ... algebraic equations (IS, EvalG in HOTINT1)
    - 'C[...]' ... Computational, e.g. for ComputationalNode ==> use 'CNode'
    - min, max ... minimum and maximum
    - '[...]Init' ... in arguments, for initialization of variables; e.g. 'valueInit' for initialization of member variable 'value'
    - write time derivatives with underscore: _t, _tt; example: Position_t, Position_tt, ...
    - write space-wise derivatives ith underscore: _x, _xx, _y, ...
    - if a scalar, write coordinate derivative with underscore: _q, _v (derivative w.r.t. velocity coordinates)
    - for components, elements or entries of vectors, arrays, matrices: use 'item' throughout

* @subsubsection conventions_sec Conventions for notation in Exudyn
    The following notation conventions are applied:
    - use lowerCamelCase for names of variables (including class member variables), consts, c-define variables, ...; EXCEPTION: for algorithms following formulas, f = M*q_tt + K*q, GBar, ...
    - use UpperCamelCase for functions, classes, structs, ...
	- use American English troughout: Visualization, etc.
    - for (abbreviations) in captial letters, e.g. ODE, use a lower case letter afterwards:
	- EXAMPLES: write 'ODEsystem', BUT: 'ODE1Equations'
	- do not use consecutive capitalized words, e.g. DO NOT WRITE 'ODEAE'
	- for functions use ODEComputeCoords(), for variables avoid 'ODE' at beginning: use nODE or write odeCoords
    - do not use '_' within variable or function names; exception: derivatives, release_assert
    - use name which exactly describes the function/variable: 'numberOfItems' instead of 'size' or 'l'
    - Examples for variable names: secondOrderSize, massMatrix, mThetaTheta
    - Examples for function/class names: SecondOrderSize, EvaluateMassMatrix, Position(const Vector3D& localPosition)
    - Use the Get/Set...() convention if data is retrieved from a class (Get) or something is set in a class (Set); Use const T& Get()/T& Get if direct access to variables is needed; Use Get/Set for pybind11
    - Example Get/Set: Real* GetDataPointer(), Vector::SetAll(Real), GetTransposed(), SetRotationalParameters(...), SetColor(...), ...
    - use 'Real' instead of double or float: for compatibility, also for AVX with SP/DP
    - use 'Index' for array/vector size and index instead of size_t or int
	- Object: a computational object, which has nodes to define internal coordinates;
	- Connector: connects two or more objects; can be e.g. spring-damper or constraint
	- Item: object, node, marker, load: anything handled within the computational/visualization systems

* @subsubsection newobjects_sec How to add new objects in Exudyn
    - STEPS to add new objects:
    -#  add and edit new class in this file
    -#  run pythonAutoGenerateObjects.py
	-#  add autogenerated .h files to your C++ project (drag&drop))
	-#  add and edit according .cpp files for implementation (dir: Objects), usually only the computational class and visualization (e.g. CNodePoint.cpp VisuNodePoint.cpp); include new .h files
    -#  add .cpp files to your C++ project (drag&drop))
    -#  add new Item to MainObjectFactory.cpp into according group (node, object, marker, ...) and add it to the CreateMain...(...) function
    -#  add include files to MainObjectFactory.cpp, the
    -#  add according test and minimum example (DOCU)
    -#  possibly extend Assemble() and Solver functionality ONLY IF NECESSARY
	-#  re-run Latex documentation
	-#  re-run Doxygen

@todo use namespaces in .cpp files only

* @subsection copyright_sec Copyright
  Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.

* @subsection FAQ_sec FAQ

************************************************************************************************ */
#endif
