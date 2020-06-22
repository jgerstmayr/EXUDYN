/** ***********************************************************************************************
* @class        MainMarkerObjectODE2CoordinatesParameters
* @brief        Parameter class for MainMarkerObjectODE2Coordinates
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2020-06-01  20:10:12 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#include <pybind11/pybind11.h>      //! AUTO: include pybind for dictionary access
#include <pybind11/stl.h>           //! AUTO: needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!
namespace py = pybind11;            //! AUTO: "py" used throughout in code
#include "Autogenerated/CMarkerObjectODE2Coordinates.h"

#include "Autogenerated/VisuMarkerObjectODE2Coordinates.h"

//! AUTO: Parameters for class MainMarkerObjectODE2CoordinatesParameters
class MainMarkerObjectODE2CoordinatesParameters // AUTO: 
{
public: // AUTO: 
};


/** ***********************************************************************************************
* @class        MainMarkerObjectODE2Coordinates
* @brief        A Marker attached to all coordinates of an object (currently only body is possible), e.g. to apply special constraints or loads on all coordinates. The measured coordinates INCLUDE reference + current coordinates.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#pragma once

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

//! AUTO: MainMarkerObjectODE2Coordinates
class MainMarkerObjectODE2Coordinates: public MainMarker // AUTO: 
{
protected: // AUTO: 
    CMarkerObjectODE2Coordinates* cMarkerObjectODE2Coordinates; //pointer to computational object (initialized in object factory) AUTO:
    VisualizationMarkerObjectODE2Coordinates* visualizationMarkerObjectODE2Coordinates; //pointer to computational object (initialized in object factory) AUTO:

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    MainMarkerObjectODE2Coordinates()
    {
        name = "";
    };

    // AUTO: access functions
    //! AUTO: Get pointer to computational class
    CMarkerObjectODE2Coordinates* GetCMarkerObjectODE2Coordinates() { return cMarkerObjectODE2Coordinates; }
    //! AUTO: Get const pointer to computational class
    const CMarkerObjectODE2Coordinates* GetCMarkerObjectODE2Coordinates() const { return cMarkerObjectODE2Coordinates; }
    //! AUTO: Set pointer to computational class (do this only in object factory!!!)
    void SetCMarkerObjectODE2Coordinates(CMarkerObjectODE2Coordinates* pCMarkerObjectODE2Coordinates) { cMarkerObjectODE2Coordinates = pCMarkerObjectODE2Coordinates; }

    //! AUTO: Get pointer to visualization class
    VisualizationMarkerObjectODE2Coordinates* GetVisualizationMarkerObjectODE2Coordinates() { return visualizationMarkerObjectODE2Coordinates; }
    //! AUTO: Get const pointer to visualization class
    const VisualizationMarkerObjectODE2Coordinates* GetVisualizationMarkerObjectODE2Coordinates() const { return visualizationMarkerObjectODE2Coordinates; }
    //! AUTO: Set pointer to visualization class (do this only in object factory!!!)
    void SetVisualizationMarkerObjectODE2Coordinates(VisualizationMarkerObjectODE2Coordinates* pVisualizationMarkerObjectODE2Coordinates) { visualizationMarkerObjectODE2Coordinates = pVisualizationMarkerObjectODE2Coordinates; }

    //! AUTO: Get const pointer to computational base class object
    virtual CMarker* GetCMarker() const { return cMarkerObjectODE2Coordinates; }
    //! AUTO: Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetCMarker(CMarker* pCMarker) { cMarkerObjectODE2Coordinates = (CMarkerObjectODE2Coordinates*)pCMarker; }

    //! AUTO: Get const pointer to visualization base class object
    virtual VisualizationMarker* GetVisualizationMarker() const { return visualizationMarkerObjectODE2Coordinates; }
    //! AUTO: Set pointer to visualization base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetVisualizationMarker(VisualizationMarker* pVisualizationMarker) { visualizationMarkerObjectODE2Coordinates = (VisualizationMarkerObjectODE2Coordinates*)pVisualizationMarker; }

    //! AUTO:  Get type name of marker (without keyword 'Marker'...!); could also be realized via a string -> type conversion?
    virtual const char* GetTypeName() const override
    {
        return "ObjectODE2Coordinates";
    }

    //! AUTO:  Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail
    virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const override;


    //! AUTO:  dictionary write access
    virtual void SetWithDictionary(const py::dict& d) override
    {
        cMarkerObjectODE2Coordinates->GetParameters().objectNumber = py::cast<Index>(d["objectNumber"]); /* AUTO:  read out dictionary and cast to C++ type*/
        EPyUtils::SetStringSafely(d, "name", name); /*! AUTO:  safely cast to C++ type*/
        if (EPyUtils::DictItemExists(d, "Vshow")) { visualizationMarkerObjectODE2Coordinates->GetShow() = py::cast<bool>(d["Vshow"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
    }

    //! AUTO:  dictionary read access
    virtual py::dict GetDictionary() const override
    {
        auto d = py::dict();
        d["markerType"] = (std::string)GetTypeName();
        d["objectNumber"] = (Index)cMarkerObjectODE2Coordinates->GetParameters().objectNumber; //! AUTO: cast variables into python (not needed for standard types) 
        d["name"] = (std::string)name; //! AUTO: cast variables into python (not needed for standard types) 
        d["Vshow"] = (bool)visualizationMarkerObjectODE2Coordinates->GetShow(); //! AUTO: cast variables into python (not needed for standard types) 
        return d; 
    }

    //! AUTO:  parameter read access
    virtual py::object GetParameter(const STDstring& parameterName) const override 
    {
        if (parameterName.compare("name") == 0) { return py::cast((std::string)name);} //! AUTO: get parameter
        else if (parameterName.compare("objectNumber") == 0) { return py::cast((Index)cMarkerObjectODE2Coordinates->GetParameters().objectNumber);} //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { return py::cast((bool)visualizationMarkerObjectODE2Coordinates->GetShow());} //! AUTO: get parameter
        else  {PyError(STDstring("MarkerObjectODE2Coordinates::GetParameter(...): illegal parameter name ")+parameterName+" cannot be read");} // AUTO: add warning for user
        return py::object();
    }


    //! AUTO:  parameter write access
    virtual void SetParameter(const STDstring& parameterName, const py::object& value) override 
    {
        if (parameterName.compare("name") == 0) { EPyUtils::SetStringSafely(value, name); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("objectNumber") == 0) { cMarkerObjectODE2Coordinates->GetParameters().objectNumber = py::cast<Index>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { visualizationMarkerObjectODE2Coordinates->GetShow() = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else  {PyError(STDstring("MarkerObjectODE2Coordinates::SetParameter(...): illegal parameter name ")+parameterName+" cannot be modified");} // AUTO: add warning for user
    }

};

