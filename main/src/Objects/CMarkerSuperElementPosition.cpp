/** ***********************************************************************************************
* @brief        implementation for MarkerSuperElementPosition
*
* @author       Gerstmayr Johannes
* @date         2019-05-02 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include "Main/CSystemData.h"
#include "Linalg/RigidBodyMath.h"

//#include <pybind11/pybind11.h>      //! AUTO: include pybind for dictionary access
//#include <pybind11/stl.h>           //! AUTO: needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!
//namespace py = pybind11;            //! AUTO: "py" used throughout in code
#include "Autogenerated/CMarkerSuperElementPosition.h"

void CMarkerSuperElementPosition::GetPosition(const CSystemData& cSystemData, Vector3D& position, ConfigurationType configuration) const
{
	const ArrayIndex& nodeNumbers = parameters.meshNodeNumbers;
	const CObjectSuperElement& cObject = (const CObjectSuperElement&)(cSystemData.GetCObjectBody(GetObjectNumber())); //always possible

	position.SetAll(0);
	for (Index i = 0; i < nodeNumbers.NumberOfItems(); i++)
	{
		position += parameters.weightingFactors[i] * cObject.GetMeshNodePosition(nodeNumbers[i], configuration);
	}

	////alternative:
	//Matrix3D A=cObject.GetRotationMatrix(Vector3D(0));
	//Vector3D p0 = cObject.GetPosition(Vector3D(0)); //reference frame position
	//for (Index i = 0; i < nodeNumbers.NumberOfItems(); i++)
	//{
	//	position += parameters.weightingFactors[i] * cObject.GetMeshNodeLocalPosition(nodeNumbers[i], configuration);
	//}
	//position = p0 + A * position;

}

void CMarkerSuperElementPosition::GetVelocity(const CSystemData& cSystemData, Vector3D& velocity, ConfigurationType configuration) const
{
	const ArrayIndex& nodeNumbers = parameters.meshNodeNumbers;
	const CObjectSuperElement& cObject = (const CObjectSuperElement&)(cSystemData.GetCObjectBody(GetObjectNumber())); //always possible

	velocity.SetAll(0);
	for (Index i = 0; i < nodeNumbers.NumberOfItems(); i++)
	{
		velocity += parameters.weightingFactors[i] * cObject.GetMeshNodeVelocity(nodeNumbers[i],configuration);
	}
}

void CMarkerSuperElementPosition::ComputeMarkerData(const CSystemData& cSystemData, bool computeJacobian, MarkerData& markerData) const
{
	GetPosition(cSystemData, markerData.position, ConfigurationType::Current);
	GetVelocity(cSystemData, markerData.velocity, ConfigurationType::Current);
	markerData.velocityAvailable = true;

	if (computeJacobian)
	{
		//const ArrayIndex& nodeNumbers = parameters.meshNodeNumbers;
		const CObjectSuperElement& cObject = (const CObjectSuperElement&)(cSystemData.GetCObjectBody(GetObjectNumber())); //always possible

		markerData.positionJacobian.SetNumberOfRowsAndColumns(3, cObject.GetODE2Size());
		markerData.positionJacobian.SetAll(0.);

		Index nw = parameters.weightingFactors.NumberOfItems();
		LinkedDataMatrix weightingMatrix(parameters.weightingFactors.GetDataPointer(), nw, 1);

		cObject.GetAccessFunctionSuperElement((AccessFunctionType)((Index)AccessFunctionType::TranslationalVelocity_qt + (Index)AccessFunctionType::SuperElement),
			weightingMatrix, parameters.meshNodeNumbers, Vector3D(0.), markerData.positionJacobian);
	}
}
