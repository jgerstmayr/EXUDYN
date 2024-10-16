/** ***********************************************************************************************
* @class        MainSensorKinematicTreeParameters
* @brief        Parameter class for MainSensorKinematicTree
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
* @date         2024-02-03  15:35:24 (last modified)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#ifndef MAINSENSORKINEMATICTREEPARAMETERS__H
#define MAINSENSORKINEMATICTREEPARAMETERS__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

#include <pybind11/pybind11.h>      //! AUTO: include pybind for dictionary access
#include <pybind11/stl.h>           //! AUTO: needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!
namespace py = pybind11;            //! AUTO: "py" used throughout in code
#include "Autogenerated/CSensorKinematicTree.h"

#include "Autogenerated/VisuSensorKinematicTree.h"

//! AUTO: Parameters for class MainSensorKinematicTreeParameters
class MainSensorKinematicTreeParameters // AUTO: 
{
public: // AUTO: 
};


/** ***********************************************************************************************
* @class        MainSensorKinematicTree
* @brief        A sensor attached to a KinematicTree with local position \f$\pLocB\f$ and link number \f$n_l\f$. As a difference to SensorBody, the KinematicTree sensor needs a local position and a link number, which defines the sub-body at which the sensor values are evaluated. The local position is given in sub-body (link) local coordinates. The sensor measures OutputVariableKinematicTree and outputs values into a file, showing per line [time, sensorValue[0], sensorValue[1], ...]. Use SensorUserFunction to modify sensor results (e.g., transforming to other coordinates) and writing to file.
*
* @author       Gerstmayr Johannes
* @date         2019-07-01 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

//! AUTO: MainSensorKinematicTree
class MainSensorKinematicTree: public MainSensor // AUTO: 
{
protected: // AUTO: 
    CSensorKinematicTree* cSensorKinematicTree; //pointer to computational object (initialized in object factory) AUTO:
    VisualizationSensorKinematicTree* visualizationSensorKinematicTree; //pointer to computational object (initialized in object factory) AUTO:

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    MainSensorKinematicTree()
    {
        name = "";
    };

    // AUTO: access functions
    //! AUTO: Get pointer to computational class
    CSensorKinematicTree* GetCSensorKinematicTree() { return cSensorKinematicTree; }
    //! AUTO: Get const pointer to computational class
    const CSensorKinematicTree* GetCSensorKinematicTree() const { return cSensorKinematicTree; }
    //! AUTO: Set pointer to computational class (do this only in object factory!!!)
    void SetCSensorKinematicTree(CSensorKinematicTree* pCSensorKinematicTree) { cSensorKinematicTree = pCSensorKinematicTree; }

    //! AUTO: Get pointer to visualization class
    VisualizationSensorKinematicTree* GetVisualizationSensorKinematicTree() { return visualizationSensorKinematicTree; }
    //! AUTO: Get const pointer to visualization class
    const VisualizationSensorKinematicTree* GetVisualizationSensorKinematicTree() const { return visualizationSensorKinematicTree; }
    //! AUTO: Set pointer to visualization class (do this only in object factory!!!)
    void SetVisualizationSensorKinematicTree(VisualizationSensorKinematicTree* pVisualizationSensorKinematicTree) { visualizationSensorKinematicTree = pVisualizationSensorKinematicTree; }

    //! AUTO: Get const pointer to computational base class object
    virtual CSensor* GetCSensor() const { return cSensorKinematicTree; }
    //! AUTO: Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetCSensor(CSensor* pCSensor) { cSensorKinematicTree = (CSensorKinematicTree*)pCSensor; }

    //! AUTO: Get const pointer to visualization base class object
    virtual VisualizationSensor* GetVisualizationSensor() const { return visualizationSensorKinematicTree; }
    //! AUTO: Set pointer to visualization base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetVisualizationSensor(VisualizationSensor* pVisualizationSensor) { visualizationSensorKinematicTree = (VisualizationSensorKinematicTree*)pVisualizationSensor; }

    //! AUTO:  Get type name of sensor (without keyword 'Sensor'...!)
    virtual const char* GetTypeName() const override
    {
        return "KinematicTree";
    }

    //! AUTO:  Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail
    virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const override;


    //! AUTO:  dictionary write access
    virtual void SetWithDictionary(const py::dict& d) override
    {
        cSensorKinematicTree->GetParameters().objectNumber = EPyUtils::GetObjectIndexSafely(d["objectNumber"]); /* AUTO:  read out dictionary and cast to C++ type*/
        cSensorKinematicTree->GetParameters().linkNumber = py::cast<Index>(d["linkNumber"]); /* AUTO:  read out dictionary and cast to C++ type*/
        EPyUtils::SetSlimVectorTemplateSafely<Real, 3>(d, "localPosition", cSensorKinematicTree->GetParameters().localPosition); /*! AUTO:  safely cast to C++ type*/
        cSensorKinematicTree->GetParameters().writeToFile = py::cast<bool>(d["writeToFile"]); /* AUTO:  read out dictionary and cast to C++ type*/
        EPyUtils::SetStringSafely(d, "fileName", cSensorKinematicTree->GetParameters().fileName); /*! AUTO:  safely cast to C++ type*/
        cSensorKinematicTree->GetParameters().outputVariableType = (OutputVariableType)py::cast<Index>(d["outputVariableType"]); /* AUTO:  read out dictionary and cast to C++ type*/
        cSensorKinematicTree->GetParameters().storeInternal = py::cast<bool>(d["storeInternal"]); /* AUTO:  read out dictionary and cast to C++ type*/
        EPyUtils::SetStringSafely(d, "name", name); /*! AUTO:  safely cast to C++ type*/
        if (EPyUtils::DictItemExists(d, "Vshow")) { visualizationSensorKinematicTree->GetShow() = py::cast<bool>(d["Vshow"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
    }

    //! AUTO:  dictionary read access
    virtual py::dict GetDictionary() const override
    {
        auto d = py::dict();
        d["sensorType"] = (std::string)GetTypeName();
        d["objectNumber"] = (ObjectIndex)cSensorKinematicTree->GetParameters().objectNumber; //! AUTO: cast variables into python (not needed for standard types) 
        d["linkNumber"] = (Index)cSensorKinematicTree->GetParameters().linkNumber; //! AUTO: cast variables into python (not needed for standard types) 
        d["localPosition"] = EPyUtils::SlimVector2NumPy(cSensorKinematicTree->GetParameters().localPosition); //! AUTO: cast variables into python (not needed for standard types) 
        d["writeToFile"] = (bool)cSensorKinematicTree->GetParameters().writeToFile; //! AUTO: cast variables into python (not needed for standard types) 
        d["fileName"] = (std::string)cSensorKinematicTree->GetParameters().fileName; //! AUTO: cast variables into python (not needed for standard types) 
        d["outputVariableType"] = (OutputVariableType)cSensorKinematicTree->GetParameters().outputVariableType; //! AUTO: cast variables into python (not needed for standard types) 
        d["storeInternal"] = (bool)cSensorKinematicTree->GetParameters().storeInternal; //! AUTO: cast variables into python (not needed for standard types) 
        d["name"] = (std::string)name; //! AUTO: cast variables into python (not needed for standard types) 
        d["Vshow"] = (bool)visualizationSensorKinematicTree->GetShow(); //! AUTO: cast variables into python (not needed for standard types) 
        return d; 
    }

    //! AUTO:  parameter read access
    virtual py::object GetParameter(const STDstring& parameterName) const override 
    {
        if (parameterName.compare("name") == 0) { return py::cast((std::string)name);} //! AUTO: get parameter
        else if (parameterName.compare("objectNumber") == 0) { return py::cast((ObjectIndex)cSensorKinematicTree->GetParameters().objectNumber);} //! AUTO: get parameter
        else if (parameterName.compare("linkNumber") == 0) { return py::cast((Index)cSensorKinematicTree->GetParameters().linkNumber);} //! AUTO: get parameter
        else if (parameterName.compare("localPosition") == 0) { return EPyUtils::SlimVector2NumPy(cSensorKinematicTree->GetParameters().localPosition);} //! AUTO: get parameter
        else if (parameterName.compare("writeToFile") == 0) { return py::cast((bool)cSensorKinematicTree->GetParameters().writeToFile);} //! AUTO: get parameter
        else if (parameterName.compare("fileName") == 0) { return py::cast((std::string)cSensorKinematicTree->GetParameters().fileName);} //! AUTO: get parameter
        else if (parameterName.compare("outputVariableType") == 0) { return py::cast((OutputVariableType)cSensorKinematicTree->GetParameters().outputVariableType);} //! AUTO: get parameter
        else if (parameterName.compare("storeInternal") == 0) { return py::cast((bool)cSensorKinematicTree->GetParameters().storeInternal);} //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { return py::cast((bool)visualizationSensorKinematicTree->GetShow());} //! AUTO: get parameter
        else  {PyError(STDstring("SensorKinematicTree::GetParameter(...): illegal parameter name ")+parameterName+" cannot be read");} // AUTO: add warning for user
        return py::object();
    }


    //! AUTO:  parameter write access
    virtual void SetParameter(const STDstring& parameterName, const py::object& value) override 
    {
        if (parameterName.compare("name") == 0) { EPyUtils::SetStringSafely(value, name); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("objectNumber") == 0) { cSensorKinematicTree->GetParameters().objectNumber = EPyUtils::GetObjectIndexSafely(value); /* AUTO:  read out dictionary, check if correct index used and store (converted) Index to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("linkNumber") == 0) { cSensorKinematicTree->GetParameters().linkNumber = py::cast<Index>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("localPosition") == 0) { EPyUtils::SetSlimVectorTemplateSafely<Real, 3>(value, cSensorKinematicTree->GetParameters().localPosition); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("writeToFile") == 0) { cSensorKinematicTree->GetParameters().writeToFile = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("fileName") == 0) { EPyUtils::SetStringSafely(value, cSensorKinematicTree->GetParameters().fileName); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("outputVariableType") == 0) { cSensorKinematicTree->GetParameters().outputVariableType = py::cast<OutputVariableType>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("storeInternal") == 0) { cSensorKinematicTree->GetParameters().storeInternal = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { visualizationSensorKinematicTree->GetShow() = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else  {PyError(STDstring("SensorKinematicTree::SetParameter(...): illegal parameter name ")+parameterName+" cannot be modified");} // AUTO: add warning for user
    }

};



#endif //#ifdef include once...
