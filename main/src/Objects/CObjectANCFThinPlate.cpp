/** ***********************************************************************************************
* @brief        CObjectANCFThinPlate implementation; following SimpleThinPlate3D in HOTINT
*
* @author       Gerstmayr Johannes
* @date         2019-06-17 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

//#define USE_COBJECTANCFTHINPLATE
#ifdef USE_COBJECTANCFTHINPLATE

#include "Main/CSystemData.h"
#include "Autogenerated/CObjectANCFThinPlate.h"

#include "Utilities/AutomaticDifferentiation.h"
//typedef EXUmath::AutoDiff<24, Real> DReal24;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ANCFThinPlate BASE class
const Real size1 = 2; //? check if this is normalized?
const Real size2 = 2; //? check if this is normalized?

//! get compressed shape function vector, using unit coordinates xi and eta
void CObjectANCFThinPlate::ComputeShapeFunctions(Real xi, Real eta, Vector12D& sf)
{
	Real xieta = xi * eta;
	Real xi2 = xi * xi;
	Real xi3 = xi * xi2;
	Real eta2 = eta * eta;
	Real eta3 = eta * eta2;

	sf.SetVector({ xieta * 0.5 - xi3 * eta * 0.125 - xi * eta3 * 0.125 + 0.25 + xi3 * 0.125 + eta2
			* eta * 0.125 - 0.375 * eta - 0.375 * xi,
		size1 * xi2 * eta * 0.0625 + size1 * 0.0625 + size1 * xieta * 0.0625 - size1 * xi * 0.0625 - size1 * xi2 * 0.0625 - size1 *
			xi3 * eta * 0.0625 + size1 * xi3 * 0.0625 - size1 * eta * 0.0625,
		size2 * 0.0625 - size2 * xi * eta3 * 0.0625 - size2 * eta * 0.0625 - size2 * eta2 * 0.0625 + size2 * eta3 /
			16. - size2 * xi * 0.0625 + size2 * xieta * 0.0625 + size2 * xi * eta2 * 0.0625,
		-xieta * 0.5 + xi3 * eta * 0.125 + xi * eta3 * 0.125 + 0.25 - xi3 * 0.125 + eta *
			eta2 * 0.125 - 0.375 * eta + 0.375 * xi,
		-size1 * 0.0625 - size1 * xi * 0.0625 + size1 * xi2 * 0.0625 + size1 * xi3 * 0.0625 + size1 * eta * 0.0625 + size1 * xi *
			eta * 0.0625 - size1 * xi2 * eta * 0.0625 - size1 * xi3 * eta * 0.0625,
		size2 * 0.0625 - size2 * eta * 0.0625 + size2 * xi * 0.0625 - size2 * xieta * 0.0625 - size2 * eta2 * 0.0625 - size2 * xieta *
			eta * 0.0625 + size2 * eta3 * 0.0625 + size2 * xi * eta3 * 0.0625,
		xieta * 0.5 - xi3 * eta * 0.125 - xi * eta3 * 0.125 + 0.25 - xi3 * 0.125 - eta2
			* eta * 0.125 + 0.375 * eta + 0.375 * xi,
		-size1 * 0.0625 - size1 * eta * 0.0625 - size1 * xi * 0.0625 - size1 * xieta * 0.0625 + size1 * xi2 * 0.0625 + size1 * xi2 *
			eta * 0.0625 + size1 * xi3 * 0.0625 + size1 * xi3 * eta * 0.0625,
		-size2 * 0.0625 - size2 * eta * 0.0625 + size2 * eta2 * 0.0625 - size2 * xi * 0.0625 - size2 * xieta * 0.0625 + size2 * xieta *
			eta * 0.0625 + size2 * eta3 * 0.0625 + size2 * xi * eta3 * 0.0625,
		-xieta * 0.5 + xi3 * eta * 0.125 + xi * eta3 * 0.125 + 0.25 + xi3 * 0.125 - eta *
			eta2 * 0.125 + 0.375 * eta - 0.375 * xi,
		size1 * 0.0625 + size1 * eta * 0.0625 - size1 * xi * 0.0625 - size1 * xieta * 0.0625 - size1 * xi2 * 0.0625 - size1 * xi2 *
			eta * 0.0625 + size1 * xi3 * 0.0625 + size1 * xi3 * eta * 0.0625,
		-size2 * 0.0625 - size2 * eta * 0.0625 + size2 * eta2 * 0.0625 + size2 * eta3 * 0.0625 + size2 * xi * 0.0625 + size2 * xi *
			eta * 0.0625 - size2 * xi * eta2 * 0.0625 - size2 * xi * eta3 * 0.0625 } );
}

//! get derivative of compressed shape function vector for slopes
void CObjectANCFThinPlate::ComputeShapeFunctions_xy(Real xi, Real eta, Vector12D& sf_x, Vector12D& sf_y)
{
	Real xieta = xi * eta;
	Real xi2 = xi * xi;
	Real xi3 = xi * xi2;
	Real eta2 = eta * eta;
	Real eta3 = eta * eta2;

	sf_x.SetVector({
			eta / 2.0 - 0.375 * xi2 * eta - eta3 * 0.125 + 0.375 * xi2 - 0.375,
			size1 * xi * eta * 0.125 + size1 * eta * 0.0625 - size1 * 0.0625 - size1 * xi * 0.125 - 3.0 * 0.0625 * size1 * xi2 * eta + 3.0 * 0.0625 * size1 * xi2,
			-size2 * eta3 * 0.0625 - size2 * 0.0625 + size2 * eta * 0.0625 + size2 * eta2 * 0.0625,
			-eta / 2.0 + 0.375 * xi2 * eta + eta3 * 0.125 - 0.375 * xi2 + 0.375,
			-size1 * 0.0625 + size1 * xi * 0.125 + 3.0 * 0.0625 * size1 * xi2 + size1 * eta * 0.0625 - size1 * xi * eta * 0.125 - 3.0 * 0.0625 * size1 * xi2 * eta,
			size2 * 0.0625 - size2 * eta * 0.0625 - size2 * eta2 * 0.0625 + size2 * eta3 * 0.0625,
			eta / 2.0 - 0.375 * xi2 * eta - eta3 * 0.125 - 0.375 * xi2 + 0.375,
			-size1 * 0.0625 - size1 * eta * 0.0625 + size1 * xi * 0.125 + size1 * xi * eta * 0.125 + 3.0 * 0.0625 * size1 * xi2 + 3.0 * 0.0625 * size1 * xi2 * eta,
			-size2 * 0.0625 - size2 * eta * 0.0625 + size2 * eta2 * 0.0625 + size2 * eta3 * 0.0625,
			-eta / 2.0 + 0.375 * xi2 * eta + eta3 * 0.125 + 0.375 * xi2 - 0.375,
			-size1 * 0.0625 - size1 * eta * 0.0625 - size1 * xi * 0.125 - size1 * xi * eta * 0.125 + 3.0 * 0.0625 * size1 * xi2 + 3.0 * 0.0625 * size1 * xi2 * eta,
			size2 * 0.0625 + size2 * eta * 0.0625 - size2 * eta2 * 0.0625 - size2 * eta3 * 0.0625,
		}),

	sf_y.SetVector({
			xi / 2.0 - xi3 * 0.125 - 0.375 * xi * eta2 + 0.375 * eta2 - 0.375,
			size1 * xi2 * 0.0625 + size1 * xi * 0.0625 - size1 * xi3 * 0.0625 - size1 * 0.0625,
			-3.0 * 0.0625 * size2 * xi * eta2 - size2 * 0.0625 - size2 * eta * 0.125 + 3.0 * 0.0625 * size2 * eta2 + size2 * xi * 0.0625 + size2 * xi * eta * 0.125,
			-xi / 2.0 + xi3 * 0.125 + 0.375 * xi * eta2 + 0.375 * eta2 - 0.375,
			size1 * 0.0625 + size1 * xi * 0.0625 - size1 * xi2 * 0.0625 - size1 * xi3 * 0.0625,
			-size2 * 0.0625 - size2 * xi * 0.0625 - size2 * eta * 0.125 - size2 * xi * eta * 0.125 + 3.0 * 0.0625 * size2 * eta2 + 3.0 * 0.0625 * size2 * xi * eta2,
			xi / 2.0 - xi3 * 0.125 - 0.375 * xi * eta2 - 0.375 * eta2 + 0.375,
			-size1 * 0.0625 - size1 * xi * 0.0625 + size1 * xi2 * 0.0625 + size1 * xi3 * 0.0625,
			-size2 * 0.0625 + size2 * eta * 0.125 - size2 * xi * 0.0625 + size2 * xi * eta * 0.125 + 3.0 * 0.0625 * size2 * eta2 + 3.0 * 0.0625 * size2 * xi * eta2,
			-xi / 2.0 + xi3 * 0.125 + 0.375 * xi * eta2 - 0.375 * eta2 + 0.375,
			size1 * 0.0625 - size1 * xi * 0.0625 - size1 * xi2 * 0.0625 + size1 * xi3 * 0.0625,
			-size2 * 0.0625 + size2 * eta * 0.125 + 3.0 * 0.0625 * size2 * eta2 + size2 * xi * 0.0625 - size2 * xi * eta * 0.125 - 3.0 * 0.0625 * size2 * xi * eta2,
		});
}

//! get second derivative of compressed shape function vector for slopes
void CObjectANCFThinPlate::ComputeShapeFunctions_xxyy(Real xi, Real eta, 
	Vector12D& sf_xx, Vector12D& sf_yy, Vector12D& sf_xy)
{
	Real xieta = xi * eta;
	Real xi2 = xi * xi;
	Real xi3 = xi * xi2;
	Real eta2 = eta * eta;
	Real eta3 = eta * eta2;

	sf_xx.SetVector({
		 -0.75 * xi * eta + 0.75 * xi,
		 -0.375 * size1 * xi * eta + size1 * eta / 8.0 + 0.375 * size1 * xi - size1 / 8.0,
		 0.0,
		 0.75 * xi * eta - 0.75 * xi,
		 size1 / 8.0 + 0.375 * size1 * xi - size1 * eta / 8.0 - 0.375 * size1 * xi * eta,
		 0.0,
		 -0.75 * xi * eta - 0.75 * xi,
		 size1 / 8.0 + size1 * eta / 8.0 + 0.375 * size1 * xi + 0.375 * size1 * xi * eta,
		 0.0,
		 0.75 * xi * eta + 0.75 * xi,
		 -size1 / 8.0 - size1 * eta / 8.0 + 0.375 * size1 * xi + 0.375 * size1 * xi * eta,
		 0.0,
		});
	sf_yy.SetVector({
		0.5 - 0.375 * xi * xi - 0.375 * eta * eta,
		-0.1875 * size1 * xi * xi + size1 * xi / 8.0 + size1 / 16.0,
		size2 / 16.0 - 0.1875 * size2 * eta * eta + size2 * eta / 8.0,
		-0.5 + 0.375 * xi * xi + 0.375 * eta * eta,
		size1 / 16.0 - size1 * xi / 8.0 - 0.1875 * size1 * xi * xi,
		-size2 / 16.0 - size2 * eta / 8.0 + 0.1875 * size2 * eta * eta,
		0.5 - 0.375 * xi * xi - 0.375 * eta * eta,
		-size1 / 16.0 + size1 * xi / 8.0 + 0.1875 * size1 * xi * xi,
		-size2 / 16.0 + size2 * eta / 8.0 + 0.1875 * size2 * eta * eta,
		-0.5 + 0.375 * xi * xi + 0.375 * eta * eta,
		-size1 / 16.0 - size1 * xi / 8.0 + 0.1875 * size1 * xi * xi,
		size2 / 16.0 - size2 * eta / 8.0 - 0.1875 * size2 * eta * eta,
		});
	sf_xy.SetVector({
		-0.75 * xi * eta + 0.75 * eta,
		0.0,
		-0.375 * size2 * xi * eta + size2 * xi / 8.0 - size2 / 8.0 + 0.375 * size2 * eta,
		0.75 * xi * eta + 0.75 * eta,
		0.0,
		-size2 / 8.0 - size2 * xi / 8.0 + 0.375 * size2 * eta + 0.375 * size2 * xi * eta,
		-0.75 * xi * eta - 0.75 * eta,
		0.0,
		size2 / 8.0 + size2 * xi / 8.0 + 0.375 * size2 * eta + 0.375 * size2 * xi * eta,
		0.75 * xi * eta - 0.75 * eta,
		0.0,
		size2 / 8.0 + 0.375 * size2 * eta - size2 * xi / 8.0 - 0.375 * size2 * xi * eta,
		});

}


//! map element coordinates (position or veloctiy level) given by nodal vectors q0 ... q3 onto compressed shape function vector to compute position, etc.
Vector3D CObjectANCFThinPlate::MapCoordinates(const Vector12D& SV, 
	const LinkedDataVector& q0, const LinkedDataVector& q1, const LinkedDataVector& q2, const LinkedDataVector& q3)
{
	const Index dim = 3;		//3D finite element
	const Index nnv = 3; //number of vectors per node
	Vector3D v(0.);
	for (Index i = 0; i < dim; i++)
	{
		for (Index j = 0; j < nnv; j++)
		{
			v[i] += SV[j] * q0[i + j * dim];
			v[i] += SV[j + 1 * nnv] * q1[i + j * dim];
			v[i] += SV[j + 2 * nnv] * q2[i + j * dim];
			v[i] += SV[j + 3 * nnv] * q3[i + j * dim];

		}
	}
	return v;
}

//! locally defined template: map element coordinates (position or veloctiy level) given by nodal vectors q0 and q1 onto compressed shape function vector to compute position, etc.
template<class TReal, Index ancfSize>
SlimVectorBase<TReal, 3> MapCoordinatesElement(const Vector12D& SV, const ConstSizeVectorBase<TReal, ancfSize>& qANCF)
{
	const Index dim = 3;		//3D finite element
	SlimVectorBase<TReal, dim> v;
	v[0] = 0;
	v[1] = 0;
	v[2] = 0;
	for (Index i = 0; i < SV.NumberOfItems(); i++)
	{
		v[0] += SV[i] * qANCF[dim * i];
		v[1] += SV[i] * qANCF[dim * i + 1];
		v[2] += SV[i] * qANCF[dim * i + 2];
	}
	return v;
}

void CObjectANCFThinPlate::ComputeCurrentObjectCoordinates(ConstSizeVector<nODE2coordinates>& qANCF) const
{
	const Index dim = 3;		//3D finite element
	LinkedDataVector qNode[nNodes];		//link node values to element vector

	for (Index i = 0; i < nNodes; i++)
	{
		qNode[i].LinkDataTo(qANCF, nnc*i, nnc);
		qNode[i] = ((CNodeODE2*)GetCNode(0))->GetCurrentCoordinateVector(); //displacement coordinates node 0
		qNode[i] += ((CNodeODE2*)GetCNode(0))->GetReferenceCoordinateVector(); //reference coordinates + displacements
	}
}

void CObjectANCFThinPlate::ComputeCurrentObjectVelocities(ConstSizeVector<nODE2coordinates>& qANCF_t) const
{
	const Index dim = 3;		//3D finite element
	LinkedDataVector qNode[nNodes];		//link node values to element vector

	for (Index i = 0; i < nNodes; i++)
	{
		qNode[i].LinkDataTo(qANCF_t, nnc * i, nnc);
		qNode[i] = ((CNodeODE2*)GetCNode(0))->GetCurrentCoordinateVector_t(); //velocity coordinates node 0
	}
}

//! Computational function: compute mass matrix
void CObjectANCFThinPlate::PreComputeMassTerms() const
{
	if (!massMatrixComputed)
	{
		precomputedMassMatrix.SetScalarMatrix(nODE2coordinates, 0.); //set 8x8 matrix
		Real t = parameters.physicsThickness;
		Real rhoT = t * parameters.physicsDensity;
		const Index dim = 3;		//3D finite element
		const Index ns = 12;			//number of shape functions

		Vector12D SV;
		//integration rules in HOTINT:
		// mass matrix: order 4

		Index cnt = 0;
		for (Index iXi=0; iXi < EXUmath::gaussRuleOrder5Points.NumberOfItems(); iXi++)
		{
			Real xi = EXUmath::gaussRuleOrder5Points[iXi];
			for (Index iEta = 0; iEta < EXUmath::gaussRuleOrder5Points.NumberOfItems(); iEta++)
			{
				Real eta = EXUmath::gaussRuleOrder5Points[iEta];
				ComputeShapeFunctions(xi, eta, SV);

				Vector12D SVint = SV; //copy
				Real factor = rhoT * EXUmath::gaussRuleOrder5Weights[iXi] * EXUmath::gaussRuleOrder5Weights[iEta];

				Real jacDet = GetJacobianDeterminant(); //for non-rectangular element

				for (Index i = 0; i < ns; i++)
				{
					for (Index j = 0; j < ns; j++)
					{
						Real value = SV[i] * SV[j] * factor;
						precomputedMassMatrix(i * dim, j * dim) += value;
						precomputedMassMatrix(i * dim + 1, j * dim + 1) += value;
						precomputedMassMatrix(i * dim + 2, j * dim + 2) += value;
					}
				}
			}
		}
		massMatrixComputed = true;
	}
}

//! Computational function: compute mass matrix
void CObjectANCFThinPlate::ComputeMassMatrix(EXUmath::MatrixContainer& massMatrixC, const ArrayIndex& ltg, Index objectNumber, bool computeInverse) const
{
	CHECKandTHROW(!computeInverse, "CObjectANCFThinPlate::ComputeMassMatrix: computeMassMatrixInversePerBody=True is not implemented; change solver settings");

	Matrix& massMatrix = massMatrixC.GetInternalDenseMatrix();
	PreComputeMassTerms();
	massMatrix.CopyFrom(precomputedMassMatrix); //copy
}

//! Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to "ode2Lhs"
void CObjectANCFThinPlate::ComputeODE2LHS(Vector& ode2Lhs, Index objectNumber) const
{
	const Index dim = 3;		//3D finite element
	const Index ns = 12;			//number of shape functions

	ConstSizeVector<dim * ns> qANCF;
	ConstSizeVector<dim * ns> qANCF_t;
	ComputeCurrentObjectCoordinates(qANCF);
	ComputeCurrentObjectVelocities(qANCF_t);
	//ComputeODE2LHStemplate<Real>(ode2Lhs, qANCF, qANCF_t);
	ComputeODE2LHStemplate(ode2Lhs, qANCF, qANCF_t);
}

//! Computational function: compute left-hand-side (LHS) of second order ordinary differential equations (ODE) to "ode2Lhs"
//template<class TReal, Index ancfSize>
//constexpr Index ancfSize = 36;
//typedef Real TReal;
void CObjectANCFThinPlate::ComputeODE2LHStemplate(VectorBase<TReal>& ode2Lhs, 
	const ConstSizeVectorBase<TReal, ancfSize>& qANCF, const ConstSizeVectorBase<TReal, ancfSize>& qANCF_t) const
{
	ode2Lhs.SetNumberOfItems(ancfSize); //works both for ANCF and ALE-ANCF
	ode2Lhs.SetAll(0.);

	//compute work of elastic forces:
	const Index dim = 3;		//3D finite element
	const Index ns = 12;		//number of shape functions
	const Index nnc = dim*3;	//number of node coordinates
	const Index nn = 4;			//number of nodes


	Real t = parameters.physicsThickness;
	const Matrix3D& Dkappa = parameters.physicsCurvatureCoefficients;
	const Matrix3D& Deps = parameters.physicsStrainCoefficients;
	
	ConstSizeVector<nn * nnc> qANCFref;
	if (parameters.strainIsRelativeToReference != 0.)
	{
		for (Index i = 0; i < nn; i++)
		{
			LinkedDataVector qNodeRef(qANCFref, nnc*i, nnc);		//link node values to element vector
			qNodeRef = ((CNodeODE2*)GetCNode(i))->GetReferenceCoordinateVector();
		}
	}

	ConstSizeVectorBase<TReal, ancfSize> elasticForces;

	const Index maxIntegrationPoints = 5;
	ConstSizeVector<maxIntegrationPoints> integrationPoints;
	ConstSizeVector<maxIntegrationPoints> integrationWeights;

	//integration rules in HOTINT:
	// stiffness: order 8 (large deformation), 6 (moderately large)
	if (parameters.useReducedOrderIntegration == 0)
	{
		integrationPoints.CopyFrom(EXUmath::gaussRuleOrder9Points); 
		integrationWeights.CopyFrom(EXUmath::gaussRuleOrder9Weights);
	}
	else if (parameters.useReducedOrderIntegration == 1)
	{
		integrationPoints.CopyFrom(EXUmath::gaussRuleOrder7Points); 
		integrationWeights.CopyFrom(EXUmath::gaussRuleOrder7Weights);
	}
	else { CHECKandTHROWstring("ObjectANCFThinPlate::ComputeODE2LHS: useReducedOrderIntegration must be between 0 and 2"); }

	//integrate to compute elastic forces:
	for (Index iXi = 0; iXi < integrationPoints.NumberOfItems(); iXi++)
	{
		Real xi = integrationPoints[iXi];
		for (Index iEta = 0; iEta < integrationPoints.NumberOfItems(); iEta++)
		{
			Real eta = integrationPoints[iEta];
			Real integrationFactor = integrationWeights[iXi] * integrationWeights[iEta];

			Vector12D SV, SV_x, SV_y, SV_xx, SV_yy, SV_xy;
			ComputeShapeFunctions(xi, eta, SV);
			ComputeShapeFunctions_xy(xi, eta, SV_x, SV_y);
			ComputeShapeFunctions_xxyy(xi, eta, SV_xx, SV_yy, SV_xy);

			SlimVectorBase<TReal, dim> r_x = MapCoordinatesElement<TReal, dim* ns>(SV_x, qANCF);
			SlimVectorBase<TReal, dim> r_y = MapCoordinatesElement<TReal, dim* ns>(SV_y, qANCF);

			SlimVectorBase<TReal, dim> r_xx = MapCoordinatesElement<TReal, dim* ns>(SV_xx, qANCF);
			SlimVectorBase<TReal, dim> r_yy = MapCoordinatesElement<TReal, dim* ns>(SV_yy, qANCF);
			SlimVectorBase<TReal, dim> r_xy = MapCoordinatesElement<TReal, dim* ns>(SV_xy, qANCF);

			SlimVectorBase<TReal, dim> nNorm3 = r_x.CrossProduct(r_y);
			TReal nNorm = nNorm3.GetL2Norm();
			//TReal nNormCubic = nNorm * nNorm * nNorm;
			nNorm3 *= 1. / (nNorm * nNorm * nNorm);

			SlimVectorBase<TReal, dim> kappa({ nNorm3 * r_xx, nNorm3 * r_yy, nNorm3 * r_xy });

			SlimVectorBase<TReal, dim> epsMidplane({ 0.5 * (r_x * r_x - 1), 0.5 * (r_y * r_y - 1), (r_x * r_y) });

			Vector12D nNorm3_q = ...

			...
				deltaKappa d(kappa)/d(qANCF) = { 
				d(nNorm3 * r_xx) / d(qANCF) => ,
				d(nNorm3 * r_yy) / d(qANCF),
				d(nNorm3 * r_xy) / d(qANCF) }

				deltaEpsMidplane
				//int_A [(eps * Deps * delta eps + kappa * Dkappa * delta kappa) * |det(jac)| ]



			//elasticForces *= integrationFactor * GetParameters().physicsAxialStiffness * (axialStrain - GetParameters().physicsReferenceAxialStrain);
			//elasticForces *= integrationFactor * (EA * (axialStrain - axialStrainRef) + axialDamping * axialStrain_t);

			ode2Lhs += elasticForces;  //add to element elastic forces
		}
	}


}

void GetJacobianDeterminant() const
{
    ...
}

//! jacobian of LHS, w.r.t. position AND velocity level coordinates
//void CObjectANCFThinPlate::ComputeJacobianODE2_ODE2(ResizableMatrix& jacobian, ResizableMatrix& jacobian_ODE2_t) const
void CObjectANCFThinPlate::ComputeJacobianODE2_ODE2(EXUmath::MatrixContainer& jacobianODE2, JacobianTemp& temp, 
	Real factorODE2, Real factorODE2_t,
	Index objectNumber, const ArrayIndex& ltg) const
{
	==> johannes

	const Index dim = 3;		//3D finite element
	const Index ns = 4;			//number of shape functions
	//const Index nnc = dim * 2;  //number of node coordinates

	ConstSizeVector<dim * ns> qANCF0;
	ConstSizeVector<dim * ns> qANCF0_t;
	ConstSizeVectorBase<DReal24, dim * ns> qANCF;
	ConstSizeVectorBase<DReal24, dim * ns> qANCF_t;
	ComputeCurrentObjectCoordinates(qANCF0);
	ComputeCurrentObjectVelocities(qANCF0_t);
	for (Index i = 0; i < dim * ns; i++)
	{
		qANCF[i] = qANCF0[i];
		qANCF_t[i] = qANCF0_t[i];
		qANCF[i].DValue((int)i) = 1; //mark that this is the corresponding derivative
		qANCF_t[i].DValue((int)(i + dim * ns)) = 1; //mark that this is the corresponding derivative; velocity derivatives are in second block
	}
	ConstSizeVectorBase<DReal24, dim * ns> ode2Lhs;
	LinkedDataVectorBase<DReal24> linkedOde2Lhs(ode2Lhs); //added because of decoupling of ConstSizeVectorBase

	ComputeODE2LHStemplate<DReal24>(linkedOde2Lhs, qANCF, qANCF_t);

	jacobianODE2.SetUseDenseMatrix(true);
	ResizableMatrix& jac = jacobianODE2.GetInternalDenseMatrix();
	jac.SetNumberOfRowsAndColumns(dim * ns, dim * ns);

	//now copy autodifferentiated result:
	for (Index i = 0; i < dim * ns; i++)
	{
		for (Index j = 0; j < dim * ns; j++)
		{
			jac(i, j) = factorODE2*ode2Lhs[i].DValue((int)j) + factorODE2_t*ode2Lhs[i].DValue((int)(j + dim * ns));
		}
	}
}


//! Flags to determine, which access (forces, moments, connectors, ...) to object are possible
AccessFunctionType CObjectANCFThinPlate::GetAccessFunctionTypes() const
{
	return (AccessFunctionType)((Index)AccessFunctionType::TranslationalVelocity_qt + 
		(Index)AccessFunctionType::AngularVelocity_qt +
		//TODO: (Index)AccessFunctionType::JacobianTtimesVector_q +
		(Index)AccessFunctionType::DisplacementMassIntegral_q);
}

//! provide Jacobian at localPosition in "value" according to object access
void CObjectANCFThinPlate::GetAccessFunctionBody(AccessFunctionType accessType, const Vector3D& localPosition, Matrix& value) const
{
	Real L = GetLength();

	switch (accessType)
	{
	case AccessFunctionType::TranslationalVelocity_qt:
	{
		const Index dim = 3;		//3D finite element
		const Index ns = 4;			//number of shape functions

		Real x = localPosition[0]; //only x-coordinate
		Vector4D SV = ComputeShapeFunctions(x, L);
		value.SetNumberOfRowsAndColumns(dim, dim * ns); //3D velocity, 12 coordinates qt
		//pout << "inside ..." << localPosition << "\n";

		CHECKandTHROW(localPosition[1] == 0 && localPosition[2] == 0,
			"CObjectANCFThinPlate: markers, forces and constraints can only act at the beam centerline at Y=Z=0; check your code");

		value.SetAll(0.);
		value(0, 0) = SV[0];
		value(1, 1) = SV[0];
		value(2, 2) = SV[0];
		value(0, 3) = SV[1];
		value(1, 4) = SV[1];
		value(2, 5) = SV[1];
		value(0, 6) = SV[2];
		value(1, 7) = SV[2];
		value(2, 8) = SV[2];
		value(0, 9) = SV[3];
		value(1,10) = SV[3];
		value(2,11) = SV[3];

		break;
	}
	case AccessFunctionType::DisplacementMassIntegral_q:
	{
		const Index dim = 3;		//3D finite element
		const Index ns = 4;			//number of shape functions

		value.SetNumberOfRowsAndColumns(dim, dim * ns); //3D velocity, 12 coordinates qt
		value.SetAll(0.);

		Real L = parameters.physicsLength;
		Real rhoA = parameters.physicsMassPerLength;

		Index cnt = 0;
		Real a = 0; //integration interval [a,b]
		Real b = L;

		Vector4D SV({0.,0.,0.,0.});

		for (auto item : EXUmath::gaussRuleOrder3Points)
		{
			Real x = 0.5*(b - a)*item + 0.5*(b + a);
			Vector4D SVloc = ComputeShapeFunctions(x, L);
			SVloc *= rhoA * (0.5*(b - a)*EXUmath::gaussRuleOrder3Weights[cnt++]);
			SV += SVloc;
		}

		value(0, 0) = SV[0];
		value(1, 1) = SV[0];
		value(2, 2) = SV[0];
		value(0, 3) = SV[1];
		value(1, 4) = SV[1];
		value(2, 5) = SV[1];
		value(0, 6) = SV[2];
		value(1, 7) = SV[2];
		value(2, 8) = SV[2];
		value(0, 9) = SV[3];
		value(1,10) = SV[3];
		value(2,11) = SV[3];
		break;
	}
	default:
		SysError("CObjectANCFThinPlate:GetAccessFunctionBody illegal accessType");
	}
}

//! provide according output variable in "value"
void CObjectANCFThinPlate::GetOutputVariableBody(OutputVariableType variableType, const Vector3D& localPosition, ConfigurationType configuration, Vector& value, Index objectNumber) const
{
	//outputVariables = "{
	//'Position':'global position vector of local axis (1) and cross section (2) position', 
	//'Velocity':'global velocity vector of local axis (1) and cross section (2) position', 
	//'Director1':'(axial) slope vector of local axis position', 
	//'Strain':'axial strain (scalar)', 
	//'Curvature':'axial strain (scalar)', 
	//'Force':'(local) section normal force (scalar)', 
	//'Torque':'(local) bending moment (scalar)'}"
	Real x = localPosition[0];

	switch (variableType)
	{
	case OutputVariableType::Position:	
	{
		value.CopyFrom(GetPosition(localPosition, configuration)); break;
	}
	case OutputVariableType::Displacement:	
	{
		value.CopyFrom(GetPosition(localPosition, configuration) - GetPosition(localPosition, ConfigurationType::Reference)); break;
	}
	case OutputVariableType::Velocity:
	{
		value.CopyFrom(GetVelocity(localPosition, configuration)); break;
	}
	case OutputVariableType::Acceleration:
	{
		//only for ANCFThinPlate, but not ALEANCF ==> not included in GetOutputVariableTypes(...)
		value.CopyFrom(GetAcceleration(localPosition, configuration)); break;
	}
	//case OutputVariableType::AngularVelocity:
	//{
	//	//independent of y, but correct
	//	value.CopyFrom(GetAngularVelocity(localPosition, configuration)); break;
	//}
	case OutputVariableType::Director1: {
		//CHECKandTHROW(y == 0., "CObjectANCFThinPlate::GetOutputVariableBody: Y-component of localPosition must be zero for Director1");
		Vector3D rx = ComputeSlopeVector(localPosition[0], configuration);
		value.SetVector({rx[0], rx[1] , rx[2]});
		break; }
	//case OutputVariableType::StrainLocal:	
	//{
	//	//CHECKandTHROW(y == 0., "CObjectANCFThinPlate::GetOutputVariableBody: Y-component of localPosition must be zero for StrainLocal");
	//	Real strain = ComputeAxialStrain(x, configuration);

	//	value.SetVector({ strain }); 
	//	break;
	//}
	//case OutputVariableType::CurvatureLocal:	
	//{
	//	//CHECKandTHROW(y == 0., "CObjectANCFThinPlate::GetOutputVariableBody: Y-component of localPosition must be zero for CurvatureLocal");
	//	value.CopyFrom( ComputeCurvature(x, configuration) );
	//	break;
	//}
	//case OutputVariableType::ForceLocal: {
	//	//do not add this due to drawing function: CHECKandTHROW(y == 0., "CObjectANCFThinPlate::GetOutputVariableBody: Y-component of localPosition must be zero for ForceLocal");

	//	Real axialStrainRef = parameters.physicsReferenceAxialStrain;
	//	if (parameters.strainIsRelativeToReference != 0.)
	//	{
	//		Vector3D rxRef = ComputeSlopeVector(x, ConfigurationType::Reference);
	//		axialStrainRef += parameters.strainIsRelativeToReference*(rxRef.GetL2Norm() - 1.);
	//	}

	//	Real force = parameters.physicsAxialStiffness * (ComputeAxialStrain(x, configuration) - axialStrainRef);
	//	if (parameters.physicsAxialDamping != 0) { force += parameters.physicsAxialDamping * ComputeAxialStrain_t(x, configuration); }

	//	value.SetVector({ force }); break;
	//}
	//case OutputVariableType::TorqueLocal: {
	//	//do not add this due to drawing function: CHECKandTHROW(y == 0., "CObjectANCFThinPlate::GetOutputVariableBody: Y-component of localPosition must be zero for TorqueLocal");

	//	Vector3D curvatureRef(0.);
	//	if (parameters.strainIsRelativeToReference != 0.)
	//	{
	//		Vector3D rxRef = ComputeSlopeVector(x, ConfigurationType::Reference);
	//		Vector3D rxxRef = ComputeSlopeVector_x(x, ConfigurationType::Reference);

	//		Real rxNorm2ref = rxRef.GetL2NormSquared();
	//		Vector3D rxCrossRxxRef = rxRef.CrossProduct(rxxRef);
	//		curvatureRef += parameters.strainIsRelativeToReference*(rxCrossRxxRef * (1. / rxNorm2ref) );
	//	}

	//	Vector3D torque = parameters.physicsBendingStiffness * (ComputeCurvature(x, configuration) - curvatureRef);
	//	if (parameters.physicsBendingDamping != 0) 
	//	{ 
	//		torque += parameters.physicsBendingDamping * ComputeCurvature_t(x, configuration); 
	//	}
	//	value.CopyFrom(torque); 
	//	break;
	//}
	default:
		SysError("CObjectANCFThinPlate::GetOutputVariableBody failed"); //error should not occur, because types are checked!
	}
}

//  return the (global) position of "localPosition" according to configuration type
Vector3D CObjectANCFThinPlate::GetPosition(const Vector3D& localPosition, ConfigurationType configuration) const
{
	Vector12D SV = ComputeShapeFunctions(localPosition[0], localPosition[1], GetLength());
	
	Vector3D v = MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector(configuration));
	if (configuration != ConfigurationType::Reference)
	{
		v += MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector(ConfigurationType::Reference), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector(ConfigurationType::Reference));
	}

	return v;
}

//  return the (global) velocity of "localPosition" according to configuration type
Vector3D CObjectANCFThinPlate::GetVelocity(const Vector3D& localPosition, ConfigurationType configuration) const
{
	Real x = localPosition[0]; 
	Vector4D SV = ComputeShapeFunctions(x, GetLength());

	Vector3D v = MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector_t(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector_t(configuration));

	return v;
}

//  return the (global) acceleration of "localPosition" according to configuration type
Vector3D CObjectANCFThinPlate::GetAcceleration(const Vector3D& localPosition, ConfigurationType configuration) const
{
	Real x = localPosition[0];
	Vector4D SV = ComputeShapeFunctions(x, GetLength());

	//Vector3D v = MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector_t(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector_t(configuration));
	Vector3D a = MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector_tt(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector_tt(configuration));

	return a;
}

//! return the (global) position of "localPosition" according to configuration type
Vector3D CObjectANCFThinPlate::GetDisplacement(const Vector3D& localPosition, ConfigurationType configuration) const
{
	if (localPosition[1] != 0.)
	{
		Real x = localPosition[0]; //only x-coordinate
		Vector4D SV = ComputeShapeFunctions(x, GetLength());

		Vector3D v(0.);
		if (configuration != ConfigurationType::Reference)
		{
			v = MapCoordinates(SV, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector(configuration));
		}
		return v;
	}
	else
	{
		//slower, but includes off-axis part
		return GetPosition(localPosition, configuration) - GetPosition(localPosition, ConfigurationType::Reference);
	}
}

//! return configuration dependent angular velocity of node; returns always a 3D Vector
Vector3D CObjectANCFThinPlate::GetAngularVelocity(const Vector3D& localPosition, ConfigurationType configuration) const
{
	CHECKandTHROWstring("CObjectANCFThinPlate::GetAngularVelocity: not implemented!!!");
	//for details see GetAngularVelocity in Point2DSlope1

	//Real xLoc = localPosition[0]; //only x-coordinate
	//Vector3D slope = ComputeSlopeVector(xLoc, configuration);
	//Real x = slope[0]; //x-slopex
	//Real y = slope[1]; //y-slopex
	////REQUIRES SOME 3D FORMULA!!!!!!!!!!!!!!!!!!

	//Vector4D SVx = ComputeShapeFunctions_x(xLoc, GetLength());
	//Vector3D slope_t = MapCoordinates(SVx, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector_t(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector_t(configuration));
	////Vector3D slope_t = MapCoordinates(SVx, ((CNodeODE2*)GetCNode(0))->GetCurrentCoordinateVector_t(), ((CNodeODE2*)GetCNode(1))->GetCurrentCoordinateVector_t());

	////compare this function to GetRotationMatrix(...)
	//return Vector3D({ 0., 0., (-y * slope_t[0] + x * slope_t[1]) / (x*x + y * y) }); //!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void CObjectANCFThinPlate::ComputeSlopeVectors(Real x, ConfigurationType configuration, Vector3D& slopeX, Vector3D& slopeY) const
{
	Vector4D SVx = ComputeShapeFunctions_x(x, GetLength());

	Vector3D slope = MapCoordinates(SVx, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector(configuration), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector(configuration));
	if (configuration != ConfigurationType::Reference) //add reference configuration to any current, initial, visualization coordinates (except reference configuration!)
	{
		slope += MapCoordinates(SVx, ((CNodeODE2*)GetCNode(0))->GetCoordinateVector(ConfigurationType::Reference), ((CNodeODE2*)GetCNode(1))->GetCoordinateVector(ConfigurationType::Reference));
	}

	return slope;

}

//!  compute inplane strains xx, yy, xy
Vector3D CObjectANCFThinPlate::ComputeInplaneStrains(Real x, ConfigurationType configuration) const
{
	Vector3D rx = ComputeSlopeVector(x, configuration);

	Real rxNorm2 = rx.GetL2NormSquared();
	Real rxNorm = sqrt(rxNorm2);
	return rxNorm - 1.; // axial strain
}


//!  compute the (bending) curvature at a certain axial position, for given configuration
Vector3D CObjectANCFThinPlate::ComputeCurvatures(Real x, ConfigurationType configuration) const
{
	//Vector3D rx = ComputeSlopeVector(x, configuration);
	//Vector3D rxx = ComputeSlopeVector_x(x, configuration);

	//Real rxNorm2 = rx.GetL2NormSquared(); //computation see ComputeODE2LHS(...)

	//Vector3D rxCrossRxx = rx.CrossProduct(rxx);
	//return rxCrossRxx * (1. / rxNorm2); //curvature
}



#endif //USE_COBJECTANCFTHINPLATE
