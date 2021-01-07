/** ***********************************************************************************************
* @class	    CSystem
* @brief		
* @details		Details:
 				- a computational system, containing data and functions to be solved with a solver
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef CMATERIAL__H
#define CMATERIAL__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
//#include "Utilities/BasicDefinitions.h" //defines Real
//#include "Utilities/ObjectContainer.h" 
//#include "Linalg/Vector.h" 

class CMaterial
{
public:

	virtual ~CMaterial() {} //added for correct deletion of derived classes
	//! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual CMaterial* GetClone() const { return new CMaterial(*this); }

    //add a default material ...
    friend std::ostream& operator<<(std::ostream& os, const CMaterial& object) {
        os << "CMaterial";
        return os;
    }
};

#endif
