/** ***********************************************************************************************
* @brief		CObjectConnector implementation
* @details		Details:
 				- implementation for connectors and constraints
*
* @author		Gerstmayr Johannes
* @date			2021-12-23 (generated)
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

#include "Main/CSystemData.h"
//#include "Linalg/RigidBodyMath.h"
#include "System/CObjectConnector.h"

//! function to compute jacobian for connectors having a simple structure with a local jacobian 
//! using K=d(F)/(dq), D=d(F)/(dq_t) ==> localJac must be: localJac = factorODE2*K + factorODE_t*D
//! jacobianODE2 is computed using the marker jacobians and the jacobianDerivative stored in markerData
//! dense mode is used here; if activeConnector=false, jacobian becomes a zeros matrix
void CObjectConnector::ComputeJacobianODE2_ODE2generic(ResizableMatrix& localJac, EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp,
	Real factorODE2, Real factorODE2_t, Index objectNumber, const MarkerDataStructure& markerData, 
	bool activeConnector, bool isCoordinateConnector, bool hasRotationJacobian) const
{
	const ResizableMatrix& jac0 = (isCoordinateConnector ? markerData.GetMarkerData(0).jacobian : markerData.GetMarkerData(0).positionJacobian);
	const ResizableMatrix& jac1 = (isCoordinateConnector ? markerData.GetMarkerData(1).jacobian : markerData.GetMarkerData(1).positionJacobian);

	//CHECKandTHROWstring("ERROR: illegal call to CObjectConnectorCartesianSpringDamper::ComputeJacobianODE2_ODE2");
	Index n0 = jac0.NumberOfColumns();
	Index n1 = jac1.NumberOfColumns();

	CHECKandTHROW(hasRotationJacobian == false, "CObjectConnector::ComputeJacobianODE2_ODE2generic: not implemented for rotationJacobian");

	jacobianODE2.SetUseDenseMatrix();
	jacobianODE2.GetInternalDenseMatrix().SetNumberOfRowsAndColumns(n0 + n1, n0 + n1);
	//jacobianODE2.GetInternalDenseMatrix().SetAll(0.); //not needed, everything is filled
	if (activeConnector) //this function is only called manually, but CSystem checks already earlier if IsActive() = false
	{
		//compute jacobian:
		//jacobian part 1:
		//[-Jpos0.T*F,pos0*Jpos0 , -Jpos0.T*F,pos1*Jpos1]
		//[+Jpos1.T*F,pos0*Jpos0 , +Jpos1.T*F,pos1*Jpos1]
		//F,pos0 = -K, F,pos1=K
		//[ Jpos0.T*K*Jpos0 , -Jpos0.T*K*Jpos1]
		//[-Jpos1.T*K*Jpos0 , +Jpos1.T*K*Jpos1]

		if (n0)
		{
			//J_pos0.T*K 
			//pout << "1: " << jac0 << ",\n" << localJac << "\n";
			EXUmath::MultMatrixTransposedMatrixTemplate(jac0, localJac, temp.matrix0);
			//J_pos0.T*K*Jpos0
			//EXUmath::MultMatrixMatrixTemplate<ResizableMatrix, ResizableMatrix, ResizableMatrix>(temp.matrix0, jac0, temp.matrix1);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac0, jacobianODE2.GetInternalDenseMatrix(), 0, 0);

			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac0.GetTransposed()*localJac*jac0, 0, 0, 1.);
		}
		if (n1)
		{
			//pout << "2: " << jac1 << ",\n" << localJac << "\n";
			//J_pos1.T*K*Jpos1
			EXUmath::MultMatrixTransposedMatrixTemplate(jac1, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac1, jacobianODE2.GetInternalDenseMatrix(), n0, n0);
			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac1.GetTransposed()*localJac*jac1, n0, n0, 1.);
		}
		if (n0 != 0 && n1 != 0)
		{
			localJac *= -1.;
			//-J_pos0.T*K*Jpos1
			EXUmath::MultMatrixTransposedMatrixTemplate(jac0, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac1, jacobianODE2.GetInternalDenseMatrix(), 0, n0);

			//-J_pos1.T*K*Jpos0
			EXUmath::MultMatrixTransposedMatrixTemplate(jac1, localJac, temp.matrix0);
			EXUmath::MultMatrixMatrix2SubmatrixTemplate(temp.matrix0,
				jac0, jacobianODE2.GetInternalDenseMatrix(), n0, 0);

			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac0.GetTransposed()*localJac*jac1, 0, n0, 1.);
			//jacobianODE2.GetInternalDenseMatrix().SetSubmatrix(jac1.GetTransposed()*localJac*jac0, n0, 0, 1.);

		}

		////add jacobian derivative:
		if (true)
		{
			if (n0 != 0 && markerData.GetMarkerData(0).jacobianDerivative.NumberOfRows() != 0)
			{
				jacobianODE2.GetInternalDenseMatrix().AddSubmatrixWithFactor(markerData.GetMarkerData(0).jacobianDerivative, -factorODE2, 0, 0); //force on marker0 acts with negative sign!
			}
			if (n1 != 0 && markerData.GetMarkerData(1).jacobianDerivative.NumberOfRows() != 0)
			{
				jacobianODE2.GetInternalDenseMatrix().AddSubmatrixWithFactor(markerData.GetMarkerData(1).jacobianDerivative, factorODE2, n0, n0);
			}
		}
	}
	else
	{
		jacobianODE2.GetInternalDenseMatrix().SetAll(0.);
	}
}

