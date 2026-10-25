/** ***********************************************************************************************
* @class        PyConnectorMarker
* @brief        The access to one marker of ObjectConnectorUser in its forceUserFunction (#2975): reads the marker - its
*               marker data, and positions and velocities anywhere on its body - and applies forces and torques through
*               the Jacobians of the marker into the block of the marker in the generalized forces of the connector.
* @details      Details:
*               - one object per marker and call, created by CObjectConnectorUser::EvaluateUserFunctionForce and passed
*                 to Python by reference; invalidated after the call, as the marker data and the forces it points to
*                 belong to that call
*               - the forces are added to the left-hand side with a negative sign: a force acts on the marker as a load
*
* @author       Gerstmayr Johannes
* @date         2026-10-23 (created)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYCONNECTORMARKER__H
#define PYCONNECTORMARKER__H

#include "Pymodules/PybindUtilities.h"
#include "Main/CSystemData.h"
#include "Main/MarkerData.h"
#include "System/CMarker.h"
#include "System/CObjectBody.h"

class PyConnectorMarker
{
private:
	const CSystemData* cSystemData;
	Index markerNumber;
	const MarkerData* markerData;	//!< the marker data of the call, with Jacobians
	Vector* ode2Lhs;				//!< the generalized forces of the connector (left-hand side)
	Index blockStart;				//!< the first coordinate of this marker in ode2Lhs
	Index blockSize;				//!< the number of coordinates of this marker
	bool valid;						//!< false after the call

	const CMarker& GetCMarker() const { return *(cSystemData->GetCMarkers()[markerNumber]); }
	bool HasType(Marker::Type type) const { return EXUstd::IsOfType(GetCMarker().GetType(), type); }

	void Check(Marker::Type type, const char* function) const
	{
		if (!valid)
		{
			PyError(STDstring("ConnectorMarker.") + function + ": the ConnectorMarker of marker " + EXUstd::ToString(markerNumber)
				+ " is used after the user function call it was passed to; it is valid during that call only", PyErrorType::valueError);
		}
		if (type != Marker::_None && !HasType(type))
		{
			PyError(STDstring("ConnectorMarker.") + function + ": marker " + EXUstd::ToString(markerNumber) + " (type "
				+ Marker::GetTypeString(GetCMarker().GetType()) + ") is not of type " + Marker::GetTypeString(type), PyErrorType::valueError);
		}
	}

	const CObjectBody& Body(const char* function) const
	{
		Check(Marker::Body, function);
		const CObject* object = cSystemData->GetCObjects()[GetCMarker().GetObjectNumber()];
		if (!EXUstd::IsOfType(object->GetType(), CObjectType::Body))
		{
			PyError(STDstring("ConnectorMarker.") + function + ": the object of marker " + EXUstd::ToString(markerNumber) + " is not a body",
				PyErrorType::valueError);
		}
		return *((const CObjectBody*)object);
	}

	//! -J^T force added to the block of this marker
	template<class TMatrix, class TVector>
	void AddForce(const TMatrix& jacobian, const TVector& force, const char* function)
	{
		if (ode2Lhs == nullptr) //the postNewtonStepUserFunction reads the markers only (#3102)
		{
			PyError(STDstring("ConnectorMarker.") + function + ": forces are applied in the forceUserFunction; in the postNewtonStepUserFunction "
				"the markers are for reading", PyErrorType::valueError);
		}
		if (jacobian.NumberOfColumns() == 0) { return; } //ground
		if (jacobian.NumberOfColumns() != blockSize || jacobian.NumberOfRows() != force.NumberOfItems())
		{
			PyError(STDstring("ConnectorMarker.") + function + ": marker " + EXUstd::ToString(markerNumber) + " has a Jacobian of size ("
				+ EXUstd::ToString(jacobian.NumberOfRows()) + " x " + EXUstd::ToString(jacobian.NumberOfColumns()) + "), which does not fit "
				+ EXUstd::ToString(force.NumberOfItems()) + " force components and its " + EXUstd::ToString(blockSize) + " coordinates",
				PyErrorType::valueError);
		}
		LinkedDataVector block(*ode2Lhs, blockStart, blockSize);
		for (Index j = 0; j < blockSize; j++)
		{
			Real value = 0.;
			for (Index i = 0; i < jacobian.NumberOfRows(); i++) { value += jacobian(i, j)*force[i]; }
			block[j] -= value;
		}
	}

	static py::array_t<Real> NumPy(const Vector3D& v) { return py::array_t<Real>(3, v.GetDataPointer()); }

public:
	PyConnectorMarker(const CSystemData* cSystemDataInit, Index markerNumberInit, const MarkerData* markerDataInit,
		Vector* ode2LhsInit, Index blockStartInit, Index blockSizeInit) :
		cSystemData(cSystemDataInit), markerNumber(markerNumberInit), markerData(markerDataInit), ode2Lhs(ode2LhsInit),
		blockStart(blockStartInit), blockSize(blockSizeInit), valid(true) {}

	//! a ConnectorMarker exists only in the forceUserFunction of ObjectConnectorUser
	static PyConnectorMarker* ForbidConstructor()
	{
		PyError("exudyn.ConnectorMarker() may not be called: ObjectConnectorUser passes one per marker to its forceUserFunction", PyErrorType::typeError);
		return nullptr; //never reached
	}

	void Invalidate() { valid = false; }
	//! for the representation, also after the call
	Index GetMarkerNumberUnchecked() const { return markerNumber; }

	Index GetMarkerNumber() const { Check(Marker::_None, "GetMarkerNumber"); return markerNumber; }
	Index GetObjectNumber() const
	{
		Check(Marker::_None, "GetObjectNumber");
		return HasType(Marker::Object) ? GetCMarker().GetObjectNumber() : -1;
	}
	Index GetNodeNumber() const
	{
		Check(Marker::_None, "GetNodeNumber");
		return HasType(Marker::Node) ? GetCMarker().GetNodeNumber() : -1;
	}

	py::array_t<Real> GetPosition() const { Check(Marker::Position, "GetPosition"); return NumPy(markerData->position); }
	py::array_t<Real> GetVelocity() const
	{
		Check(Marker::Position, "GetVelocity");
		if (!markerData->velocityAvailable) { PyError("ConnectorMarker.GetVelocity: marker " + EXUstd::ToString(markerNumber) + " provides no velocity", PyErrorType::valueError); }
		return NumPy(markerData->velocity);
	}
	py::array_t<Real> GetRotationMatrix() const
	{
		Check(Marker::Orientation, "GetRotationMatrix");
		return py::array_t<Real>(std::vector<ptrdiff_t>{ 3, 3 }, markerData->orientation.GetDataPointer());
	}
	py::array_t<Real> GetAngularVelocityLocal() const { Check(Marker::Orientation, "GetAngularVelocityLocal"); return NumPy(markerData->angularVelocityLocal); }
	py::array_t<Real> GetAngularVelocity() const
	{
		Check(Marker::Orientation, "GetAngularVelocity");
		return NumPy(markerData->orientation*markerData->angularVelocityLocal);
	}
	py::array_t<Real> GetCoordinates() const
	{
		if (!(HasType(Marker::Coordinate) || HasType(Marker::Coordinates))) { Check(Marker::Coordinate, "GetCoordinates"); }
		Check(Marker::_None, "GetCoordinates");
		return EPyUtils::VectorRef2NumPy(markerData->vectorValue, false);
	}
	py::array_t<Real> GetCoordinates_t() const
	{
		if (!(HasType(Marker::Coordinate) || HasType(Marker::Coordinates))) { Check(Marker::Coordinate, "GetCoordinates_t"); }
		Check(Marker::_None, "GetCoordinates_t");
		return EPyUtils::VectorRef2NumPy(markerData->vectorValue_t, false);
	}
	py::array_t<Real> GetBodyPosition(const std::array<Real, 3>& localPosition) const
	{
		return NumPy(Body("GetBodyPosition").GetPosition(Vector3D(localPosition), ConfigurationType::Current));
	}
	py::array_t<Real> GetBodyVelocity(const std::array<Real, 3>& localPosition) const
	{
		return NumPy(Body("GetBodyVelocity").GetVelocity(Vector3D(localPosition), ConfigurationType::Current));
	}

	void ApplyForce(const std::array<Real, 3>& force)
	{
		Check(Marker::Position, "ApplyForce");
		AddForce(markerData->positionJacobian, Vector3D(force), "ApplyForce");
	}
	void ApplyTorque(const std::array<Real, 3>& torque)
	{
		Check(Marker::Orientation, "ApplyTorque");
		AddForce(markerData->rotationJacobian, Vector3D(torque), "ApplyTorque");
	}
	void ApplyBodyForce(const std::array<Real, 3>& force, const std::array<Real, 3>& localPosition)
	{
		const CObjectBody& body = Body("ApplyBodyForce");
		Matrix jacobian;
		body.GetPositionJacobian(Vector3D(localPosition), jacobian);
		AddForce(jacobian, Vector3D(force), "ApplyBodyForce");
	}
	void ApplyCoordinateForces(const std::vector<Real>& forces)
	{
		if (!(HasType(Marker::Coordinate) || HasType(Marker::Coordinates))) { Check(Marker::Coordinate, "ApplyCoordinateForces"); }
		Check(Marker::_None, "ApplyCoordinateForces");
		ResizableVector f;
		f.SetNumberOfItems((Index)forces.size());
		for (Index i = 0; i < f.NumberOfItems(); i++) { f[i] = forces[i]; }
		AddForce(markerData->jacobian, f, "ApplyCoordinateForces");
	}
};

#endif
