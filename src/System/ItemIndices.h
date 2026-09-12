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
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
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

	//! operators for computation with index, e.g., when used in FEM or other modules
	friend Index operator+(Index a, const NodeIndex& b)
	{
		return a + b.index;
	}
	friend Index operator+(const NodeIndex& a, Index b)
	{
		return a.index + b;
	}
	friend Index operator-(Index a, const NodeIndex& b)
	{
		return a - b.index;
	}
	friend Index operator-(const NodeIndex& a, Index b)
	{
		return a.index - b;
	}
	friend Index operator*(Index a, const NodeIndex& b)
	{
		return a*b.index;
	}
	friend Index operator*(const NodeIndex& a, Index b)
	{
		return a.index*b;
	}
	friend Index operator-(const NodeIndex& a)
	{
		return -a.index;
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
	//! operators for computation with index, e.g., when used in FEM or other modules
	friend Index operator+(Index a, const ObjectIndex& b)
	{
		return a + b.index;
	}
	friend Index operator+(const ObjectIndex& a, Index b)
	{
		return a.index + b;
	}
	friend Index operator-(Index a, const ObjectIndex& b)
	{
		return a - b.index;
	}
	friend Index operator-(const ObjectIndex& a, Index b)
	{
		return a.index - b;
	}
	friend Index operator*(Index a, const ObjectIndex& b)
	{
		return a * b.index;
	}
	friend Index operator*(const ObjectIndex& a, Index b)
	{
		return a.index*b;
	}
	friend Index operator-(const ObjectIndex& a)
	{
		return -a.index;
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
	//! operators for computation with index, e.g., when used in FEM or other modules
	friend Index operator+(Index a, const MarkerIndex& b)
	{
		return a + b.index;
	}
	friend Index operator+(const MarkerIndex& a, Index b)
	{
		return a.index + b;
	}
	friend Index operator-(Index a, const MarkerIndex& b)
	{
		return a - b.index;
	}
	friend Index operator-(const MarkerIndex& a, Index b)
	{
		return a.index - b;
	}
	friend Index operator*(Index a, const MarkerIndex& b)
	{
		return a * b.index;
	}
	friend Index operator*(const MarkerIndex& a, Index b)
	{
		return a.index*b;
	}
	friend Index operator-(const MarkerIndex& a)
	{
		return -a.index;
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
	//! operators for computation with index, e.g., when used in FEM or other modules
	friend Index operator+(Index a, const LoadIndex& b)
	{
		return a + b.index;
	}
	friend Index operator+(const LoadIndex& a, Index b)
	{
		return a.index + b;
	}
	friend Index operator-(Index a, const LoadIndex& b)
	{
		return a - b.index;
	}
	friend Index operator-(const LoadIndex& a, Index b)
	{
		return a.index - b;
	}
	friend Index operator*(Index a, const LoadIndex& b)
	{
		return a * b.index;
	}
	friend Index operator*(const LoadIndex& a, Index b)
	{
		return a.index*b;
	}
	friend Index operator-(const LoadIndex& a)
	{
		return -a.index;
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
	//! operators for computation with index, e.g., when used in FEM or other modules
	friend Index operator+(Index a, const SensorIndex& b)
	{
		return a + b.index;
	}
	friend Index operator+(const SensorIndex& a, Index b)
	{
		return a.index + b;
	}
	friend Index operator-(Index a, const SensorIndex& b)
	{
		return a - b.index;
	}
	friend Index operator-(const SensorIndex& a, Index b)
	{
		return a.index - b;
	}
	friend Index operator*(Index a, const SensorIndex& b)
	{
		return a * b.index;
	}
	friend Index operator*(const SensorIndex& a, Index b)
	{
		return a.index*b;
	}
	friend Index operator-(const SensorIndex& a)
	{
		return -a.index;
	}
};



#endif
