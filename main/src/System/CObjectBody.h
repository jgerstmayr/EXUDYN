/** ***********************************************************************************************
* @class	    CObjectBody
* @brief		
* @details		Details:
 				- base class for computational body
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
#ifndef COBJECTBODY__H
#define COBJECTBODY__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include "Utilities/BasicDefinitions.h" //defines Real
#include "Utilities/ResizableArray.h" 
#include "Linalg/MatrixContainer.h" 

#include "Main/OutputVariable.h" 
#include "System/CObject.h" 

#include "Main/MarkerData.h"

////! flags that are transferred during mass matrix computation
//namespace MassMatrixFlags {
//	//! used mainly to show which jacobians are available analytically in objects; can be combined binary to see, which jacobian is available
//	enum Type {
//		_None = 0,						//marks that no flag is added
//		PREFER_DENSE_MATRIX = 1 << 1,	//dense matrix computation preferred; otherwise sparse matrix is preferred
//	};
//}
//

class CNode;

class CObjectBody: public CObject 
{
protected:
	//ResizableArray<Index> nodes; //done in every object; body has single or several nodes

public:
    //! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
    virtual CObjectBody* GetClone() const { return new CObjectBody(*this); }

    virtual CObjectType GetType() const override { return CObjectType::Body; }

	virtual void Print(std::ostream& os) const;

	//! local to global coordinates are available in CSystemData
	virtual void GetODE2LocalToGlobalCoordinates(ArrayIndex& ltg) const;

	//!compute local coordinate index (within body coordinates) for a certain local node number
	//!this is inefficient for objects with many nodes and needs to be reimplemented, e.g., in ObjectGenericODE2
	virtual Index GetLocalODE2CoordinateIndexPerNode(Index localNode) const
	{
		Index nn = GetNumberOfNodes();
		Index localCoordinate = 0;
		for (Index i = 0; i < nn; i++)
		{
			if (localNode == i) { return localCoordinate; }
			localCoordinate += GetCNode(i)->GetNumberOfODE2Coordinates();
		}
		CHECKandTHROWstring("CObjectBody::GetLocalCoordinateIndexPerNode: invalid localNode number");
		return 0;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ACCESS FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    virtual void GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const { CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetAccessFunctionBody"); }
    virtual void GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, 
									   ConfigurationType configuration, Vector& value, Index objectNumber) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetOutputVariableBody");
	}

    //! basic access function position, which are available in every body; used for connectors
    virtual Vector3D GetPosition(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetPosition");
        return Vector3D({ 0., 0., 0. });
    }

	//! basic access function velocity, which are available in every body; used for connectors
	virtual Vector3D GetVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetVelocity");
		return Vector3D({ 0., 0., 0. });
	}

	//! basic access function displacement, which are available in every body; used for connectors
	virtual Vector3D GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetDisplacement");
		return Vector3D({ 0., 0., 0. });
    }

	//! basic access function for rotation of node; returns always a 3D Vector; for rigid bodies, the argument localPosition has no effect
	virtual Matrix3D GetRotationMatrix(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetRotationMatrix");
		return Matrix3D();
	}

	//! basic access function for angular velocity of node; returns always a 3D Vector; for rigid bodies, the argument localPosition has no effect
	virtual Vector3D GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetAngularVelocity");
		return Vector3D();
	}

	//! basic access function for angular velocity of node; returns always a 3D Vector; for rigid bodies, the argument localPosition has no effect
	virtual Vector3D GetAngularVelocityLocal(const Vector3D& localPosition, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetAngularVelocityLocal");
		return Vector3D();
	}

	//return center of mass --> necessary for RigidBody
	virtual Vector3D GetLocalCenterOfMass() const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectBody::GetLocalCenterOfMass");
		return Vector3D({ 0., 0., 0. });
	}

	//! speedup function for rigid body marker access
	virtual void ComputeRigidBodyMarkerData(const Vector3D& localPosition, bool computeJacobian, MarkerData& markerData) const {
		//CHECKandTHROWstring("ERROR: illegal call to CObjectBody::ComputeRigidBodyMarkerData");
		
		markerData.position = GetPosition(localPosition, ConfigurationType::Current);
		markerData.velocity = GetVelocity(localPosition, ConfigurationType::Current);

		markerData.orientation = GetRotationMatrix(localPosition, ConfigurationType::Current);
		markerData.angularVelocityLocal = GetAngularVelocityLocal(localPosition, ConfigurationType::Current);
		markerData.velocityAvailable = true;

		if (computeJacobian)
		{
			GetAccessFunctionBody(AccessFunctionType::TranslationalVelocity_qt, localPosition, markerData.positionJacobian);
			GetAccessFunctionBody(AccessFunctionType::AngularVelocity_qt, localPosition, markerData.rotationJacobian);
		}

	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Computation FUNCTIONS

	//! compute object massmatrix to massMatrix; offers interface to dense and sparse mass matrix computation; standard is dense mode; ltg only used in sparse mode; only possible for bodies
	virtual void ComputeMassMatrix(EXUmath::MatrixContainer& massMatrix, const ArrayIndex& ltg, Index objectNumber) const { CHECKandTHROWstring("ERROR: illegal call to CObjectBody::ComputeMassMatrix"); }
	//old: virtual void ComputeMassMatrix(Matrix& massMatrix, Index objectNumber) const { CHECKandTHROWstring("ERROR: illegal call to CObjectBody::ComputeMassMatrix"); }

	//! return true if object has time and coordinate independent (=constant) mass matrix; used by solver
	virtual bool HasConstantMassMatrix() const { CHECKandTHROWstring("ERROR: illegal call to CObjectBody::HasConstantMassMatrix"); return false; }

}; //CObjectBody



//! an element with more access functions than CObjectBody; may be slower
class CObjectSuperElement : public CObjectBody
{
//protected:

public:
	static const Index nDim3D = 3;
	//! get an exact clone of *this, must be implemented in all derived classes! Necessary for better handling in ObjectContainer
	virtual CObjectSuperElement* GetClone() const { return new CObjectSuperElement(*this); }

	virtual CObjectType GetType() const override { return (CObjectType)((Index)CObjectType::Body + (Index)CObjectType::SuperElement); }

	//virtual void Print(std::ostream& os) const;

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ACCESS FUNCTIONS
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++     MESH NODE FUNCTIONS      ++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
	//! special access function types for multi-nodal objects with mesh nodes
	//==>integrated into GetAccessFunctionTypes()
	//virtual AccessFunctionType GetAccessFunctionTypesSuperElement() const
	//{ CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetAccessFunctionTypesSuperElement"); }

	//! special access function for multi - nodal objects with mesh nodes; compute Jacobian with weightingMatrix(WM), which defines how flexible coordinates(w / wo modeBasis) are 
	//! CASE 1:
	//!   if meshNodeNumbers (out of n total mesh nodes in the SuperElement) are provided, the nodes are weighted by weightingMatrix (can be n x 1 or n x 3) 
	//!   and the position (or velocity) of every node is added to a weighted sum. In case (n x 3), the single components are weighted.
	//!   If the marker is defined for the global position, the transformations read: pLoc = WM * q, pGlob = pRef + A * pLoc == > Jac = d(pGlob) / d([q0, q])
	//! CASE 2:
	//!   For reducedOrder elements, meshNodeNumbers.NumberOfItems()==0, the weightingMatrix acts on reduced coordinates (WM = WMfull * modalBasis), 
	//!   and must have size (nReducedCoordinates x 3): pLoc = WM * qReduced, pGlob = pRef + A * pLoc == > Jac = d(pGlob) / d([q0, q])
	virtual void GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, const ArrayIndex& meshNodeNumbers, Matrix& value) const;
	//virtual void GetAccessFunctionSuperElement(AccessFunctionType accessType, const Matrix& weightingMatrix, Matrix& value) const
	//{ CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetAccessFunctionSuperElement"); }

	//! get extended output variable types for multi-nodal objects with mesh nodes
	virtual OutputVariableType GetOutputVariableTypesSuperElement(Index meshNodeNumber) const
	{ CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetOutputVariableTypesSuperElement"); return OutputVariableType::_None;}

	//! get extended output variables for multi-nodal objects with mesh nodes
	virtual void GetOutputVariableSuperElement(OutputVariableType variableType, Index meshNodeNumber,
		ConfigurationType configuration, Vector& value) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetOutputVariableSuperElement");
	}

	//! return true, if SuperElement has reference frame node; node number returned in referenceFrameNode, if available
	virtual bool HasReferenceFrame(Index& referenceFrameNode) const
	{
		return false;
	}

	//! return the number of mesh nodes, which may be different from number of nodes
	virtual Index GetNumberOfMeshNodes() const
	{
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetNumberOfMeshNodes");
		return 0;
	}

	//does not make sense, because inconsistent with node functions (only position, but no displacements)
	//basic access function to reference (body-fixed) mesh position of a node, which is available in every superelement; meshNodeNumber is the local node number of the (underlying) mesh
	//virtual Vector3D GetMeshNodeReferencePosition(Index meshNodeNumber) const {
	//	CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeLocalPosition");
	//	return Vector3D({ 0., 0., 0. });
	//}

	//! return the (local) position of a mesh node according to configuration type; use Configuration.Reference to access the mesh reference position; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodeLocalPosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeLocalPosition");
		return Vector3D({ 0., 0., 0. });
	}

	//! return the (local) velocity of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodeLocalVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeLocalVelocity");
		return Vector3D({ 0., 0., 0. });
	}

	//! return the (local) acceleration of a mesh node according to configuration type; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodeLocalAcceleration(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeLocalAcceleration");
		return Vector3D({ 0., 0., 0. });
	}

	//! return the (global) position of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodePosition(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodePosition");
		return Vector3D({ 0., 0., 0. });
	}

	//! return the (global) velocity of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodeVelocity(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeVelocity");
		return Vector3D({ 0., 0., 0. });
	}

	//! return the (global) acceleration of a mesh node according to configuration type; this is the node position transformed by the motion of the reference frame; meshNodeNumber is the local node number of the (underlying) mesh
	virtual Vector3D GetMeshNodeAcceleration(Index meshNodeNumber, ConfigurationType configuration = ConfigurationType::Current) const {
		CHECKandTHROWstring("ERROR: illegal call to CObjectSuperElement::GetMeshNodeAcceleration");
		return Vector3D({ 0., 0., 0. });
	}

	//@TODO ObjectSuperElement:
	// - AddMassMatrixSparse(GeneralMatrix& systemMassMatrix, const ArrayIndex& ltgRows, const ArrayIndex& ltgColumns, Real factor)
	// //! Add jacobians and jacobians_t to system jacobian; useful for large problems; Jac and Jac_t only computed if factors!=0
	// - AddJacobianSparse(GeneralMatrix& systemJacobian, const ArrayIndex& ltgRows, const ArrayIndex& ltgColumns, 
	//                     Real factorJac, Real factorJac_t)
	// - 
	// - 
};

#endif
