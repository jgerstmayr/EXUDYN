/** ***********************************************************************************************
* @file         BodyGraphicsDataPython.h
* @brief		the Python side of BodyGraphicsData
* @details		Details:
 				- the six functions that read a BodyGraphicsData from a Python dictionary or object
 				  and write it back; they are here and no longer in VisualizationSystemContainer.h,
 				  so that the item sources, which need BodyGraphicsData but no Python, do not have
 				  to include pybind11
*
* @author		Gerstmayr Johannes
* @date			2026-09-23
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
#ifndef BODYGRAPHICSDATAPYTHON__H
#define BODYGRAPHICSDATAPYTHON__H

#include "Graphics/VisualizationSystemContainer.h" //for BodyGraphicsData and BodyGraphicsDataList

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>
namespace py = pybind11;            //! namespace 'py' used throughout in code

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! python function to read BodyGraphicsData from dictionary, e.g. for body or ground graphics
bool PyWriteBodyGraphicsDataList(const py::dict& d, const char* item, BodyGraphicsData& data);

//! python function to read BodyGraphicsData from py::object, which must be a list of graphicsData dictionaries
bool PyWriteBodyGraphicsDataList(const py::object object, BodyGraphicsData& data, bool eraseData=true);

//! python function to write BodyGraphicsData to dictionary, e.g. for testing;
py::list PyGetBodyGraphicsDataList(const BodyGraphicsData& data, bool addGraphicsData);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//for BodyGraphicsData lists (KinematicTree)
//! python function to read BodyGraphicsDataList from dictionary, e.g. for body or ground graphics
bool PyWriteBodyGraphicsDataListOfLists(const py::dict& d, const char* item, BodyGraphicsDataList& data);

//! python function to read BodyGraphicsDataList from py::object, which must be a list of lists of graphicsData dictionaries
bool PyWriteBodyGraphicsDataListOfLists(const py::object object, BodyGraphicsDataList& data);

//! python function to write BodyGraphicsDataList to dictionary, e.g. for testing;
py::list PyGetBodyGraphicsDataListOfLists(const BodyGraphicsDataList& data, bool addGraphicsData);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif
