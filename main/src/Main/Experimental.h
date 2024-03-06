/** ***********************************************************************************************
* @class        PyExperimental
* @brief		class for experiments which can be accessed in Python
*
* @author		Gerstmayr Johannes
* @date			2023-06-11 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
*
************************************************************************************************ */
#ifndef PYEXPERIMENTAL__H
#define PYEXPERIMENTAL__H

#include "Tests/UnitTestBase.h" //for unit tests


//to use, do 
//#include "Main/Experimental.h"
//extern Experimental experimental; //!this class can be accessed from outside, but also from every other file where this is imported

//!class with according structure in PybindModule.cpp which can be accessed from Python and inside C++
//!as this is only used for experiments, all member variables are public
//!to add a NEW FEATURE, add variable, initialization, printing and def_readwrite line in PybindModule.cpp
class PyExperimental
{
public: 
    Index eigenFullPivotLUsolverDebugLevel; //!< debug: 0=off, 1=print rank and info, 2=print matrices

    PyExperimental()
    {
        Initialize();
    }

    void Initialize()
    {
        eigenFullPivotLUsolverDebugLevel = 0;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "  eigenFullPivotLUsolverDebugLevel = " << eigenFullPivotLUsolverDebugLevel << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PyExperimental& item)
    {
        item.Print(os);
        return os;
    }

};

//!class with according structure in PybindModule.cpp which can be accessed from Python and inside C++
//!used for special features and global settings
class PySpecialSolver
{
public:
    enum class MultiThreadingType {
        MicroThreading = 0,
        LoadBalancing = 1,
    };

    MultiThreadingType multiThreadingType;  //!< multithreadingType, according to given enum
    Real timeout;                     //!< specific timeout used if >= 0
    bool throwErrorWithCtrlC;               //!< switches behavior for CTRL-C; both works in console, but not in Spyder's iPython

    PySpecialSolver()
    {
        Initialize();
    }

    void Initialize()
    {
        multiThreadingType = MultiThreadingType::LoadBalancing;
        timeout = -1.; //means no timeout
        throwErrorWithCtrlC = false; //false works cleaner in Windows-console
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "  multiThreadingType = ";
        if (multiThreadingType == MultiThreadingType::LoadBalancing ) {os << "LoadBalancing";}
        else if (multiThreadingType == MultiThreadingType::MicroThreading) { os << "MicroThreading"; }
        else { os << "Undefined"; }
        os << "\n";
        os << "  timeout = " << timeout << "\n";
        os << "  throwErrorWithCtrlC = " << throwErrorWithCtrlC << "\n";

        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialSolver& item)
    {
        item.Print(os);
        return os;
    }

};

class PySpecialExceptions
{
public:
    bool dictionaryVersionMismatch;  //!< warn on version mismatch in SetDictionary(...)
    bool dictionaryNonCopyable;          //!< raise exception if things cannot be copied in GetDictionary(...)

    PySpecialExceptions()
    {
        Initialize();
    }

    void Initialize()
    {
        dictionaryVersionMismatch = true;
        dictionaryNonCopyable = true;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        os << "  dictionaryVersionMismatch = " << dictionaryVersionMismatch << "\n";
        os << "  dictionaryNonCopyable = " << dictionaryNonCopyable << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialExceptions& item)
    {
        item.Print(os);
        return os;
    }

};

//!class with according structure in PybindModule.cpp which can be accessed from Python and inside C++
//!used for special features and global settings
class PySpecial
{
public:
    PySpecialSolver solver;
    PySpecialExceptions exceptions;

    PySpecial()
    {
        Initialize();
    }

    void Initialize()
    {
        solver.Initialize();
        exceptions.Initialize();
    }

    //! put RunCppUnitTests into special class
    Index SpecialRunUnitTests(bool reportOnPass = false, bool printOutput = true)
    {
#ifdef PERFORM_UNIT_TESTS
        return RunUnitTests(reportOnPass, printOutput);
#else
        return 0;
#endif
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "solver:\n" << solver;
        os << "exceptions:\n" << exceptions;
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecial& item)
    {
        item.Print(os);
        return os;
    }

};


#endif
