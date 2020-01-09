/** ***********************************************************************************************
* @file			BasicTraits.h
* @brief		Reduce amount of member functions and classes by means of traits
* @details		no details yet
*
* @author		Gerstmayr Johannes
* @date			2018-04-30 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
************************************************************************************************ */


// convert vector into string using preString before vector.
//STDstring& ToString(STDstring preString = STDstring(""))
//use traits to enable ToString via ostream:	std::ostringstream sstream;
//												sstream << myRealVar;
//												std::string varAsString = sstream.str();
//do this for Vector, SlimVector, Matrix, ...

//every object which has a void Print(std::ostream& os) command can stream to operator<<
//does not work in this general form! ==> reverse this and enable the print operator for the operator<<
//template<class T>
//std::ostream& operator<<(std::ostream& os, const T& object)
//{
//	T.Print(os);
//	return os;
//}

