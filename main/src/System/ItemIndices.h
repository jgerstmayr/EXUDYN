/** ***********************************************************************************************
* @brief		Special indices for safe handling of node, object, marker, ... indices
* @details		Details:
 				- mbs.AddNode(...), mbs.AddObject(...), ...
                - nodes can be of one category: ODE1coordinates, ODE2coordinates, AEvariables, DataVariables
*
* @author		Gerstmayr Johannes
* @date			2018-05-17 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: missing
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef ITEMINDICES__H
#define ITEMINDICES__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h" //defines Real

//! class which contains index which can only be used for node numbers (avoids mixing different indices ...)
class NodeIndex
{
private:
	Index index; //index of item
public:
	NodeIndex() { index = EXUstd::InvalidIndex; }
	NodeIndex(Index indexInit) { index = indexInit; }
	//! operator to convert (explicitly) to Index; how to convert with pybind11?
	explicit operator Index() const
	{
		return index;
	}
	Index GetIndex() const
	{
		return index;
	}
	void SetIndex(Index indexInit)
	{
		index = indexInit;
	}
	STDstring GetTypeString() const
	{
		return STDstring("NodeIndex");
	}
};

//! class which contains index which can only be used for object numbers (avoids mixing different indices ...)
class ObjectIndex
{
private:
	Index index; //index of item
public:
	ObjectIndex() { index = EXUstd::InvalidIndex; }
	ObjectIndex(Index indexInit) { index = indexInit; }
	//! operator to convert (explicitly) to Index; how to convert with pybind11?
	explicit operator Index() const //'explicit' this does not allow implicit conversion
	//operator Index() const
	{
		return index;
	}
	Index GetIndex() const
	{
		return index;
	}
	void SetIndex(Index indexInit)
	{
		index = indexInit;
	}
	STDstring GetTypeString() const
	{
		return STDstring("ObjectIndex");
	}
};

//! class which contains index which can only be used for marker numbers (avoids mixing different indices ...)
class MarkerIndex
{
private:
	Index index; //index of item
public:
	MarkerIndex() { index = EXUstd::InvalidIndex; }
	MarkerIndex(Index indexInit) { index = indexInit; }
	//! operator to convert (explicitly) to Index; how to convert with pybind11?
	explicit operator Index() const
	{
		return index;
	}
	Index GetIndex() const
	{
		return index;
	}
	void SetIndex(Index indexInit)
	{
		index = indexInit;
	}
	STDstring GetTypeString() const
	{
		return STDstring("MarkerIndex");
	}
};

//! class which contains index which can only be used for load numbers (avoids mixing different indices ...)
class LoadIndex
{
private:
	Index index; //index of item
public:
	LoadIndex() { index = EXUstd::InvalidIndex; }
	LoadIndex(Index indexInit) { index = indexInit; }
	//! operator to convert (explicitly) to Index; how to convert with pybind11?
	explicit operator Index() const
	{
		return index;
	}
	Index GetIndex() const
	{
		return index;
	}
	void SetIndex(Index indexInit)
	{
		index = indexInit;
	}
	STDstring GetTypeString() const
	{
		return STDstring("LoadIndex");
	}
};

//! class which contains index which can only be used for sensor numbers (avoids mixing different indices ...)
class SensorIndex
{
private:
	Index index; //index of item
public:
	SensorIndex() { index = EXUstd::InvalidIndex; }
	SensorIndex(Index indexInit) { index = indexInit; }
	//! operator to convert (explicitly) to Index; how to convert with pybind11?
	explicit operator Index() const
	{
		return index;
	}
	Index GetIndex() const
	{
		return index;
	}
	void SetIndex(Index indexInit)
	{
		index = indexInit;
	}
	STDstring GetTypeString() const
	{
		return STDstring("SensorIndex");
	}
};




#endif
