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
#include <set>
#include <map>
#include "Main/UserTypes.h"


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
    Index markerSuperElementRigidTexpSO3; //!< True: use additional TexpSO3 for FFRF

    PyExperimental()
    {
        Initialize();
    }

    void Initialize()
    {
        eigenFullPivotLUsolverDebugLevel = 0;
        markerSuperElementRigidTexpSO3 = true;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "  eigenFullPivotLUsolverDebugLevel = " << eigenFullPivotLUsolverDebugLevel << "\n";
        os << "  markerSuperElementRigidTexpSO3 = " << markerSuperElementRigidTexpSO3 << "\n";
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

    Real timeout;                           //!< specific timeout used if >= 0
    bool throwErrorWithCtrlC;               //!< switches behavior for CTRL-C; both works in console, but not in Spyder's iPython
    bool multiThreadingLoadBalancing;       //!< true=use load balancing
    Index initialConstraintsViolations;     //!< number of time integrations whose initial conditions did not fulfill the constraints (#2234)
    bool suspendWorkersDuringFactorization; //!< the workers of the TaskManager sleep during a sparse factorization (#3428)
    Index suspendWorkersMinimumSize;        //!< ... of at least this number of unknowns (#3428)
    Index pardisoNumberOfThreads;           //!< threads of MKL PARDISO; 0: the threads of the TaskManager (#3431)
    Index pardisoCheckPeriod;               //!< the residual is checked after every n-th factorization of PARDISO; 0: never (#3431)
    Real pardisoResidualTolerance;          //!< relative residual above which a solve of PARDISO falls back to SparseLU (#3431)
    bool pardisoFallback;                   //!< false: an error instead of the fallback (#3431)
    std::map<Index, Index> pardisoParameters; //!< iparm entries written after pardisoinit and Exudyn's defaults (#3431)
    Index pardisoFallbacks;                 //!< number of fallbacks from PARDISO to SparseLU since the import (#3431)
    bool parallelSubstitution;              //!< EigenSparse with a nested-dissection ordering and the substitution on the threads (#3432)
    Real parallelSubstitutionPivotThreshold; //!< the diagonal pivot threshold of that factorization (#3432)
    Index parallelSubstitutionSerial;       //!< factorizations solved serially: a pivot coupled the subtrees, or the factorization failed (#3432)

    PySpecialSolver()
    {
        Initialize();
    }

    void Initialize()
    {
        multiThreadingLoadBalancing = true;
        initialConstraintsViolations = 0;
        timeout = -1.; //means no timeout
        throwErrorWithCtrlC = false; //false works cleaner in Windows-console
        suspendWorkersDuringFactorization = false;
        suspendWorkersMinimumSize = 1000;
        pardisoNumberOfThreads = 0;
        pardisoCheckPeriod = 1;
        pardisoResidualTolerance = 1e-6;
        pardisoFallback = true;
        pardisoParameters.clear();
        pardisoFallbacks = 0;
        parallelSubstitution = false;
        parallelSubstitutionPivotThreshold = 1e-3;
        parallelSubstitutionSerial = 0;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "\n";
        os << "  timeout = " << timeout << "\n";
        os << "  throwErrorWithCtrlC = " << throwErrorWithCtrlC << "\n";
        os << "  multiThreadingLoadBalancing = " << multiThreadingLoadBalancing << "\n";
        os << "  initialConstraintsViolations = " << initialConstraintsViolations << "\n";
        os << "  suspendWorkersDuringFactorization = " << suspendWorkersDuringFactorization << "\n";
        os << "  suspendWorkersMinimumSize = " << suspendWorkersMinimumSize << "\n";
        os << "  pardisoNumberOfThreads = " << pardisoNumberOfThreads << "\n";
        os << "  pardisoCheckPeriod = " << pardisoCheckPeriod << "\n";
        os << "  pardisoResidualTolerance = " << pardisoResidualTolerance << "\n";
        os << "  pardisoFallback = " << pardisoFallback << "\n";
        os << "  pardisoParameters = {";
        for (const auto& item : pardisoParameters) { os << item.first << ": " << item.second << ", "; }
        os << "}\n";
        os << "  pardisoFallbacks = " << pardisoFallbacks << "\n";
        os << "  parallelSubstitution = " << parallelSubstitution << "\n";
        os << "  parallelSubstitutionPivotThreshold = " << parallelSubstitutionPivotThreshold << "\n";
        os << "  parallelSubstitutionSerial = " << parallelSubstitutionSerial << "\n";

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
    bool parameterRangeChecks;           //!< raise if an item or settings parameter violates its range (UReal, PInt, ...); false accepts any value
    bool writeErrorsToLogFile;           //!< write the block of a user or system error into the log file of exudyn.SetWriteToFile (#2895)

    PySpecialExceptions()
    {
        Initialize();
    }

    void Initialize()
    {
        dictionaryVersionMismatch = true;
        dictionaryNonCopyable = true;
        parameterRangeChecks = true;
        writeErrorsToLogFile = true;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        os << "  dictionaryVersionMismatch = " << dictionaryVersionMismatch << "\n";
        os << "  dictionaryNonCopyable = " << dictionaryNonCopyable << "\n";
        os << "  parameterRangeChecks = " << parameterRangeChecks << "\n";
        os << "  writeErrorsToLogFile = " << writeErrorsToLogFile << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialExceptions& item)
    {
        item.Print(os);
        return os;
    }

};

//!flags that stop Exudyn from opening windows: the renderer, the solution viewer, plot windows and
//!dialogs. Meant for automated runs (test runners, CI, AI-assisted development), where a window
//!that waits for a human stops everything (#2477)
class PySpecialUserInterface
{
public:
    bool suppressRenderer;                  //!< Start() returns at once, IsActive() is false, DoIdleTasks() does nothing
    bool suppressSolutionViewer;            //!< SolutionViewer and AnimateModes return immediately
    bool suppressPlots;                     //!< plot windows are not shown; figures are still drawn and saved
    bool suppressDialogs;                   //!< tkinter dialogs return their defaults instead of opening

    //!a suppressed call is silent except for ONE notice per kind, so that a window-less session is
    //!never a mystery; these are not exposed to Python, they only remember what was said already
    mutable bool rendererNoticePrinted;

    PySpecialUserInterface()
    {
        Initialize();
    }

    void Initialize()
    {
        suppressRenderer = false;
        suppressSolutionViewer = false;
        suppressPlots = false;
        suppressDialogs = false;
        rendererNoticePrinted = false;
    }

    //! set all four flags at once; the usual case, as a run either wants windows or does not
    void SuppressAll(bool flag = true)
    {
        suppressRenderer = flag;
        suppressSolutionViewer = flag;
        suppressPlots = flag;
        suppressDialogs = flag;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        os << "  suppressRenderer = " << suppressRenderer << "\n";
        os << "  suppressSolutionViewer = " << suppressSolutionViewer << "\n";
        os << "  suppressPlots = " << suppressPlots << "\n";
        os << "  suppressDialogs = " << suppressDialogs << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialUserInterface& item)
    {
        item.Print(os);
        return os;
    }

};

//!switches of the beam elements that tests and comparisons use; a model keeps the defaults
class PySpecialBeams
{
public:
    bool geometricallyExactLumpedMass;  //!< ObjectBeamGeometricallyExact: the lumped mass per node instead of the element-consistent one (#1273, #2761)

    PySpecialBeams()
    {
        Initialize();
    }

    void Initialize()
    {
        geometricallyExactLumpedMass = false;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        os << "  geometricallyExactLumpedMass = " << geometricallyExactLumpedMass << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialBeams& item)
    {
        item.Print(os);
        return os;
    }

};

//!how a deprecated name is reported (#2804, #2806): every deprecation of Exudyn - settings, item parameters,
//!functions - goes through PyDeprecated(source, name, message), which reads these flags
class PySpecialDeprecations
{
public:
    bool warnOnce;      //!< true: each deprecated name warns once per session; false: every use warns, for debugging a script
    bool recordUse;     //!< true: every use is counted in exudyn.sys['deprecationUse'][source][name], also when no warning is shown
    bool userFunctionListArguments; //!< true: Python user functions get lists instead of numpy arrays, as before #3008
    bool userFunctionListArgumentsReported; //!< its use was reported in this session; not exposed to Python
    bool symbolicStarProduct; //!< true: * of two symbolic vectors or with a symbolic matrix is the scalar or matrix product, as before #3015
    bool symbolicStarProductReported; //!< its use was reported in this session; not exposed to Python
    bool symbolicVariablesWithoutScope; //!< true: a named esym.Real, Vector or Matrix is a variable of no scope, and the symbolic user functions read any named variable, as before #3208
    bool symbolicVariablesWithoutScopeReported; //!< its use was reported in this session; not exposed to Python

    std::set<std::string> warned; //!< the names warned about in this session, as "source:name"; not exposed to Python

    PySpecialDeprecations()
    {
        Initialize();
    }

    void Initialize()
    {
        warnOnce = true;
        recordUse = true;
        userFunctionListArguments = false;
        userFunctionListArgumentsReported = false;
        symbolicStarProduct = false;
        symbolicStarProductReported = false;
        symbolicVariablesWithoutScope = false;
        symbolicVariablesWithoutScopeReported = false;
        warned.clear();
    }

    //! forget which names were warned about, so that each warns once more
    void Reset() { warned.clear(); }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        os << "  warnOnce = " << warnOnce << "\n";
        os << "  recordUse = " << recordUse << "\n";
        os << "  userFunctionListArguments = " << userFunctionListArguments << "\n";
        os << "  symbolicStarProduct = " << symbolicStarProduct << "\n";
        os << "  symbolicVariablesWithoutScope = " << symbolicVariablesWithoutScope << "\n";
        os << "  names warned about = " << warned.size() << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialDeprecations& item)
    {
        item.Print(os);
        return os;
    }

};

//! the names of the user types, exudyn.special.userTypes (#2989); the names are kept in UserTypes::RegisteredName,
//! the functions SetName, GetName and Reset are bound in Pybind_manual_classes.cpp
class PySpecialUserTypes
{
public:
    void Initialize() {}

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    virtual void Print(std::ostream& os) const
    {
        const char* kinds[UserTypes::numberOfKinds] = { "OutputVariableType", "NodeType", "MarkerType" };
        for (Index kind = 0; kind < UserTypes::numberOfKinds; kind++)
        {
            for (Index index = 0; index < UserTypes::numberOfUserTypes; index++)
            {
                if (!UserTypes::RegisteredName(kind, index).empty())
                {
                    os << "  " << kinds[kind] << ".UserType" << index << " = " << UserTypes::RegisteredName(kind, index) << "\n";
                }
            }
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecialUserTypes& item)
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
    PySpecialUserInterface userInterface;
    PySpecialBeams beams;
    PySpecialDeprecations deprecations;
    PySpecialUserTypes userTypes;

    PySpecial()
    {
        Initialize();
    }

    void Initialize()
    {
        solver.Initialize();
        exceptions.Initialize();
        userInterface.Initialize();
        beams.Initialize();
        deprecations.Initialize();
        userTypes.Initialize();
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

    //py::list InfoStat(bool writeOutput = true) const { return PythonInfoStat(writeOutput); }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        os << "solver:\n" << solver;
        os << "exceptions:\n" << exceptions;
        os << "userInterface:\n" << userInterface;
        os << "beams:\n" << beams;
        os << "deprecations:\n" << deprecations;
        os << "userTypes:\n" << userTypes;
        //os << "  InfoStat() = " << InfoStat(false) << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const PySpecial& item)
    {
        item.Print(os);
        return os;
    }

};


#endif
