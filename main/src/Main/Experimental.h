/** ***********************************************************************************************
* @class        Experimental
* @brief		static class for experiments which can be accessed in Python
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
#ifndef EXPERIMENTAL__H
#define EXPERIMENTAL__H


//#include "Utilities/BasicFunctions.h"
//#include "Main/MainSystem.h"

//to use, do 
//#include "Main/Experimental.h"
//extern Experimental experimental; //!this class can be accessed from outside, but also from every other file where this is imported

//!class which can be accessed from Python and inside C++
//!as this is only used for experiments, all member variables are public
//!to add a NEW FEATURE, add variable, initialization, printing and def_readwrite line in PybindModule.cpp
class Experimental
{
public: 
    //bool useEigenFullPivotLUsolver;         //!< switch to experimental FullPivot LU solver, which should handle overdetermined systems!
    Index eigenFullPivotLUsolverDebugLevel; //!< debug: 0=off, 1=print rank and info, 2=print matrices

    Experimental() 
    {
        Initialize();
    }

    void Initialize()
    {
        //useEigenFullPivotLUsolver = false;
        eigenFullPivotLUsolverDebugLevel = 0;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! to get representation:
    virtual void Print(std::ostream& os) const
    {
        //os << "  useEigenFullPivotLUsolver = " << useEigenFullPivotLUsolver << "\n";
        os << "  eigenFullPivotLUsolverDebugLevel = " << eigenFullPivotLUsolverDebugLevel << "\n";
        os << "\n";
    }

    friend std::ostream& operator<<(std::ostream& os, const Experimental& item)
    {
        item.Print(os);
        return os;
    }

};

#endif
