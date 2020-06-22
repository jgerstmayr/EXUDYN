/** ***********************************************************************************************
* @class        MainObjectFFRFreducedOrderParameters
* @brief        Parameter class for MainObjectFFRFreducedOrder
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
#include <pybind11/functional.h> //! AUTO: for function handling ... otherwise gives a python error (no compilation error in C++ !)
#include "Autogenerated/CObjectFFRFreducedOrder.h"

#include "Autogenerated/VisuObjectFFRFreducedOrder.h"

//! AUTO: Parameters for class MainObjectFFRFreducedOrderParameters
class MainObjectFFRFreducedOrderParameters // AUTO: 
{
public: // AUTO: 
};


/** ***********************************************************************************************
* @class        MainObjectFFRFreducedOrder
* @brief        This object is used to represent modally reduced flexible bodies using the floating frame of reference formulation (FFRF) and the component mode synthesis. It contains a RigidBodyNode (always node 0) and a NodeGenericODE2 representing the modal coordinates.
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

//! AUTO: MainObjectFFRFreducedOrder
class MainObjectFFRFreducedOrder: public MainObjectBody // AUTO: 
{
protected: // AUTO: 
    CObjectFFRFreducedOrder* cObjectFFRFreducedOrder; //pointer to computational object (initialized in object factory) AUTO:
    VisualizationObjectFFRFreducedOrder* visualizationObjectFFRFreducedOrder; //pointer to computational object (initialized in object factory) AUTO:

public: // AUTO: 
    //! AUTO: default constructor with parameter initialization
    MainObjectFFRFreducedOrder()
    {
        name = "";
    };

    // AUTO: access functions
    //! AUTO: Get pointer to computational class
    CObjectFFRFreducedOrder* GetCObjectFFRFreducedOrder() { return cObjectFFRFreducedOrder; }
    //! AUTO: Get const pointer to computational class
    const CObjectFFRFreducedOrder* GetCObjectFFRFreducedOrder() const { return cObjectFFRFreducedOrder; }
    //! AUTO: Set pointer to computational class (do this only in object factory!!!)
    void SetCObjectFFRFreducedOrder(CObjectFFRFreducedOrder* pCObjectFFRFreducedOrder) { cObjectFFRFreducedOrder = pCObjectFFRFreducedOrder; }

    //! AUTO: Get pointer to visualization class
    VisualizationObjectFFRFreducedOrder* GetVisualizationObjectFFRFreducedOrder() { return visualizationObjectFFRFreducedOrder; }
    //! AUTO: Get const pointer to visualization class
    const VisualizationObjectFFRFreducedOrder* GetVisualizationObjectFFRFreducedOrder() const { return visualizationObjectFFRFreducedOrder; }
    //! AUTO: Set pointer to visualization class (do this only in object factory!!!)
    void SetVisualizationObjectFFRFreducedOrder(VisualizationObjectFFRFreducedOrder* pVisualizationObjectFFRFreducedOrder) { visualizationObjectFFRFreducedOrder = pVisualizationObjectFFRFreducedOrder; }

    //! AUTO: Get const pointer to computational base class object
    virtual CObject* GetCObject() const { return cObjectFFRFreducedOrder; }
    //! AUTO: Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetCObject(CObject* pCObject) { cObjectFFRFreducedOrder = (CObjectFFRFreducedOrder*)pCObject; }

    //! AUTO: Get const pointer to visualization base class object
    virtual VisualizationObject* GetVisualizationObject() const { return visualizationObjectFFRFreducedOrder; }
    //! AUTO: Set pointer to visualization base class object (do this only in object factory; type is NOT CHECKED!!!)
    virtual void SetVisualizationObject(VisualizationObject* pVisualizationObject) { visualizationObjectFFRFreducedOrder = (VisualizationObjectFFRFreducedOrder*)pVisualizationObject; }

    //! AUTO:  Get type name of object; could also be realized via a string -> type conversion?
    virtual const char* GetTypeName() const override
    {
        return "GenericODE2";
    }

    //! AUTO:  provide requested nodeType for objects; used for automatic checks in CheckSystemIntegrity()
    virtual Node::Type GetRequestedNodeType() const override
    {
        return Node::_None;
    }

    //! AUTO:  Check consistency prior to CSystem::Assemble(); needs to find all possible violations such that Assemble() would fail
    virtual bool CheckPreAssembleConsistency(const MainSystem& mainSystem, STDstring& errorString) const override;


    //! AUTO:  dictionary write access
    virtual void SetWithDictionary(const py::dict& d) override
    {
        cObjectFFRFreducedOrder->GetParameters().nodeNumbers = py::cast<std::vector<Index>>(d["nodeNumbers"]); /* AUTO:  read out dictionary and cast to C++ type*/
        EPyUtils::SetPyMatrixContainerSafely(d, "massMatrixReduced", cObjectFFRFreducedOrder->GetParameters().massMatrixReduced); /*! AUTO:  safely cast to C++ type*/
        EPyUtils::SetPyMatrixContainerSafely(d, "stiffnessMatrixReduced", cObjectFFRFreducedOrder->GetParameters().stiffnessMatrixReduced); /*! AUTO:  safely cast to C++ type*/
        EPyUtils::SetPyMatrixContainerSafely(d, "dampingMatrixReduced", cObjectFFRFreducedOrder->GetParameters().dampingMatrixReduced); /*! AUTO:  safely cast to C++ type*/
        if (EPyUtils::DictItemExists(d, "forceUserFunction")) { if (EPyUtils::CheckForValidFunction(d["forceUserFunction"])) { cObjectFFRFreducedOrder->GetParameters().forceUserFunction = py::cast<std::function<StdVector(Real, StdVector,StdVector)>>((py::function)d["forceUserFunction"]); /* AUTO:  read out dictionary and cast to C++ type*/}} 
        if (EPyUtils::DictItemExists(d, "massMatrixUserFunction")) { if (EPyUtils::CheckForValidFunction(d["massMatrixUserFunction"])) { cObjectFFRFreducedOrder->GetParameters().massMatrixUserFunction = py::cast<std::function<NumpyMatrix(Real, StdVector,StdVector)>>((py::function)d["massMatrixUserFunction"]); /* AUTO:  read out dictionary and cast to C++ type*/}} 
        if (EPyUtils::DictItemExists(d, "computeFFRFterms")) { cObjectFFRFreducedOrder->GetParameters().computeFFRFterms = py::cast<bool>(d["computeFFRFterms"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
        EPyUtils::SetNumpyMatrixSafely(d, "modeBasis", cObjectFFRFreducedOrder->GetParameters().modeBasis); /*! AUTO:  safely cast to C++ type*/
        EPyUtils::SetNumpyVectorSafely(d, "referencePositions", cObjectFFRFreducedOrder->GetParameters().referencePositions); /*! AUTO:  safely cast to C++ type*/
        EPyUtils::SetStringSafely(d, "name", name); /*! AUTO:  safely cast to C++ type*/
        if (EPyUtils::DictItemExists(d, "Vshow")) { visualizationObjectFFRFreducedOrder->GetShow() = py::cast<bool>(d["Vshow"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
        if (EPyUtils::DictItemExists(d, "Vcolor")) { visualizationObjectFFRFreducedOrder->GetColor() = py::cast<std::vector<float>>(d["Vcolor"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
        if (EPyUtils::DictItemExists(d, "VtriangleMesh")) { EPyUtils::SetNumpyMatrixISafely(d, "VtriangleMesh", visualizationObjectFFRFreducedOrder->GetTriangleMesh()); /*! AUTO:  safely cast to C++ type*/} 
        if (EPyUtils::DictItemExists(d, "VshowNodes")) { visualizationObjectFFRFreducedOrder->GetShowNodes() = py::cast<bool>(d["VshowNodes"]); /* AUTO:  read out dictionary and cast to C++ type*/} 
        GetCObject()->ParametersHaveChanged();
    }

    //! AUTO:  dictionary read access
    virtual py::dict GetDictionary() const override
    {
        auto d = py::dict();
        d["objectType"] = (std::string)GetTypeName();
        d["nodeNumbers"] = (std::vector<Index>)cObjectFFRFreducedOrder->GetParameters().nodeNumbers; //! AUTO: cast variables into python (not needed for standard types) 
        d["massMatrixReduced"] = (PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().massMatrixReduced; //! AUTO: cast variables into python (not needed for standard types) 
        d["stiffnessMatrixReduced"] = (PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().stiffnessMatrixReduced; //! AUTO: cast variables into python (not needed for standard types) 
        d["dampingMatrixReduced"] = (PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().dampingMatrixReduced; //! AUTO: cast variables into python (not needed for standard types) 
        d["forceUserFunction"] = (std::function<StdVector(Real, StdVector,StdVector)>)cObjectFFRFreducedOrder->GetParameters().forceUserFunction; //! AUTO: cast variables into python (not needed for standard types) 
        d["massMatrixUserFunction"] = (std::function<NumpyMatrix(Real, StdVector,StdVector)>)cObjectFFRFreducedOrder->GetParameters().massMatrixUserFunction; //! AUTO: cast variables into python (not needed for standard types) 
        d["computeFFRFterms"] = (bool)cObjectFFRFreducedOrder->GetParameters().computeFFRFterms; //! AUTO: cast variables into python (not needed for standard types) 
        d["modeBasis"] = EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetParameters().modeBasis); //! AUTO: cast variables into python (not needed for standard types) 
        d["referencePositions"] = EPyUtils::Vector2NumPy(cObjectFFRFreducedOrder->GetParameters().referencePositions); //! AUTO: cast variables into python (not needed for standard types) 
        d["physicsMass"] = (Real)cObjectFFRFreducedOrder->GetPhysicsMass(); //! AUTO: cast variables into python (not needed for standard types) 
        d["physicsInertia"] = EXUmath::Matrix3DToStdArray33(cObjectFFRFreducedOrder->GetPhysicsInertia()); //! AUTO: cast variables into python (not needed for standard types) 
        d["physicsCenterOfMass"] = (std::vector<Real>)cObjectFFRFreducedOrder->GetPhysicsCenterOfMass(); //! AUTO: cast variables into python (not needed for standard types) 
        d["PHItTM"] = EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetPHItTM()); //! AUTO: cast variables into python (not needed for standard types) 
        d["tempUserFunctionForce"] = EPyUtils::Vector2NumPy(cObjectFFRFreducedOrder->GetTempUserFunctionForce()); //! AUTO: cast variables into python (not needed for standard types) 
        d["tempRefPosSkew"] = EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetTempRefPosSkew()); //! AUTO: cast variables into python (not needed for standard types) 
        d["tempVelSkew"] = EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetTempVelSkew()); //! AUTO: cast variables into python (not needed for standard types) 
        d["name"] = (std::string)name; //! AUTO: cast variables into python (not needed for standard types) 
        d["Vshow"] = (bool)visualizationObjectFFRFreducedOrder->GetShow(); //! AUTO: cast variables into python (not needed for standard types) 
        d["Vcolor"] = (std::vector<float>)visualizationObjectFFRFreducedOrder->GetColor(); //! AUTO: cast variables into python (not needed for standard types) 
        d["VtriangleMesh"] = EPyUtils::MatrixI2NumPy(visualizationObjectFFRFreducedOrder->GetTriangleMesh()); //! AUTO: cast variables into python (not needed for standard types) 
        d["VshowNodes"] = (bool)visualizationObjectFFRFreducedOrder->GetShowNodes(); //! AUTO: cast variables into python (not needed for standard types) 
        return d; 
    }

    //! AUTO:  parameter read access
    virtual py::object GetParameter(const STDstring& parameterName) const override 
    {
        if (parameterName.compare("name") == 0) { return py::cast((std::string)name);} //! AUTO: get parameter
        else if (parameterName.compare("nodeNumbers") == 0) { return py::cast((std::vector<Index>)cObjectFFRFreducedOrder->GetParameters().nodeNumbers);} //! AUTO: get parameter
        else if (parameterName.compare("massMatrixReduced") == 0) { return py::cast((PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().massMatrixReduced);} //! AUTO: get parameter
        else if (parameterName.compare("stiffnessMatrixReduced") == 0) { return py::cast((PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().stiffnessMatrixReduced);} //! AUTO: get parameter
        else if (parameterName.compare("dampingMatrixReduced") == 0) { return py::cast((PyMatrixContainer)cObjectFFRFreducedOrder->GetParameters().dampingMatrixReduced);} //! AUTO: get parameter
        else if (parameterName.compare("forceUserFunction") == 0) { return py::cast((std::function<StdVector(Real, StdVector,StdVector)>)cObjectFFRFreducedOrder->GetParameters().forceUserFunction);} //! AUTO: get parameter
        else if (parameterName.compare("massMatrixUserFunction") == 0) { return py::cast((std::function<NumpyMatrix(Real, StdVector,StdVector)>)cObjectFFRFreducedOrder->GetParameters().massMatrixUserFunction);} //! AUTO: get parameter
        else if (parameterName.compare("computeFFRFterms") == 0) { return py::cast((bool)cObjectFFRFreducedOrder->GetParameters().computeFFRFterms);} //! AUTO: get parameter
        else if (parameterName.compare("modeBasis") == 0) { return EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetParameters().modeBasis);} //! AUTO: get parameter
        else if (parameterName.compare("referencePositions") == 0) { return EPyUtils::Vector2NumPy(cObjectFFRFreducedOrder->GetParameters().referencePositions);} //! AUTO: get parameter
        else if (parameterName.compare("physicsMass") == 0) { return py::cast((Real)cObjectFFRFreducedOrder->GetPhysicsMass());} //! AUTO: get parameter
        else if (parameterName.compare("physicsInertia") == 0) { return py::cast(EXUmath::Matrix3DToStdArray33(cObjectFFRFreducedOrder->GetPhysicsInertia()));} //! AUTO: get parameter
        else if (parameterName.compare("physicsCenterOfMass") == 0) { return py::cast((std::vector<Real>)cObjectFFRFreducedOrder->GetPhysicsCenterOfMass());} //! AUTO: get parameter
        else if (parameterName.compare("PHItTM") == 0) { return EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetPHItTM());} //! AUTO: get parameter
        else if (parameterName.compare("tempUserFunctionForce") == 0) { return EPyUtils::Vector2NumPy(cObjectFFRFreducedOrder->GetTempUserFunctionForce());} //! AUTO: get parameter
        else if (parameterName.compare("tempRefPosSkew") == 0) { return EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetTempRefPosSkew());} //! AUTO: get parameter
        else if (parameterName.compare("tempVelSkew") == 0) { return EPyUtils::Matrix2NumPy(cObjectFFRFreducedOrder->GetTempVelSkew());} //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { return py::cast((bool)visualizationObjectFFRFreducedOrder->GetShow());} //! AUTO: get parameter
        else if (parameterName.compare("Vcolor") == 0) { return py::cast((std::vector<float>)visualizationObjectFFRFreducedOrder->GetColor());} //! AUTO: get parameter
        else if (parameterName.compare("VtriangleMesh") == 0) { return EPyUtils::MatrixI2NumPy(visualizationObjectFFRFreducedOrder->GetTriangleMesh());} //! AUTO: get parameter
        else if (parameterName.compare("VshowNodes") == 0) { return py::cast((bool)visualizationObjectFFRFreducedOrder->GetShowNodes());} //! AUTO: get parameter
        else  {PyError(STDstring("ObjectFFRFreducedOrder::GetParameter(...): illegal parameter name ")+parameterName+" cannot be read");} // AUTO: add warning for user
        return py::object();
    }


    //! AUTO:  parameter write access
    virtual void SetParameter(const STDstring& parameterName, const py::object& value) override 
    {
        if (parameterName.compare("name") == 0) { EPyUtils::SetStringSafely(value, name); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("nodeNumbers") == 0) { cObjectFFRFreducedOrder->GetParameters().nodeNumbers = py::cast<std::vector<Index>>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("massMatrixReduced") == 0) { EPyUtils::SetPyMatrixContainerSafely(value, cObjectFFRFreducedOrder->GetParameters().massMatrixReduced); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("stiffnessMatrixReduced") == 0) { EPyUtils::SetPyMatrixContainerSafely(value, cObjectFFRFreducedOrder->GetParameters().stiffnessMatrixReduced); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("dampingMatrixReduced") == 0) { EPyUtils::SetPyMatrixContainerSafely(value, cObjectFFRFreducedOrder->GetParameters().dampingMatrixReduced); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("forceUserFunction") == 0) { cObjectFFRFreducedOrder->GetParameters().forceUserFunction = py::cast<std::function<StdVector(Real, StdVector,StdVector)>>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("massMatrixUserFunction") == 0) { cObjectFFRFreducedOrder->GetParameters().massMatrixUserFunction = py::cast<std::function<NumpyMatrix(Real, StdVector,StdVector)>>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("computeFFRFterms") == 0) { cObjectFFRFreducedOrder->GetParameters().computeFFRFterms = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("modeBasis") == 0) { EPyUtils::SetNumpyMatrixSafely(value, cObjectFFRFreducedOrder->GetParameters().modeBasis); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("referencePositions") == 0) { EPyUtils::SetNumpyVectorSafely(value, cObjectFFRFreducedOrder->GetParameters().referencePositions); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("Vshow") == 0) { visualizationObjectFFRFreducedOrder->GetShow() = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("Vcolor") == 0) { visualizationObjectFFRFreducedOrder->GetColor() = py::cast<std::vector<float>>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("VtriangleMesh") == 0) { EPyUtils::SetNumpyMatrixISafely(value, visualizationObjectFFRFreducedOrder->GetTriangleMesh()); /*! AUTO:  safely cast to C++ type*/; } //! AUTO: get parameter
        else if (parameterName.compare("VshowNodes") == 0) { visualizationObjectFFRFreducedOrder->GetShowNodes() = py::cast<bool>(value); /* AUTO:  read out dictionary and cast to C++ type*/; } //! AUTO: get parameter
        else  {PyError(STDstring("ObjectFFRFreducedOrder::SetParameter(...): illegal parameter name ")+parameterName+" cannot be modified");} // AUTO: add warning for user
        GetCObject()->ParametersHaveChanged();
    }

};

