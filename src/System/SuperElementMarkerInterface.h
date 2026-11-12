/** ***********************************************************************************************
* @class        SuperElementMarkerInterface
* @brief        What MarkerSuperElementPosition and MarkerSuperElementRigid need of the mesh nodes of a superelement
*               that depends only on the reference configuration and on the parameters of the marker: computed by
*               CObjectSuperElement::InitializeMarkerInterface once at Assemble (#3450, #2184)
*
* @author       Gerstmayr Johannes
* @date         2026-11-12 (created)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */
#ifndef SUPERELEMENTMARKERINTERFACE__H
#define SUPERELEMENTMARKERINTERFACE__H

#include "Linalg/BasicLinalg.h"

//! the marker has mesh nodes i with weights w_i, reference positions p_i^ref and displacements u_i in the body frame;
//! p_0 = sum_i w_i p_i^ref, and the weighted rotation is W^{-1} sum_i w_i (p_i^ref - p_0) x (u_i - sum_j w_j u_j)
class SuperElementMarkerInterface
{
public:
	bool isInitialized = false;       //!< false: not computed, or a parameter of the marker changed since; then the marker computes it on every evaluation
	bool hasRotation = false;         //!< the rotation part is computed (MarkerSuperElementRigid)
	bool alternativeApproach = false; //!< useAlternativeApproach of MarkerSuperElementRigid: W is a matrix, not a scalar
	Vector3D referencePosition;       //!< p_0, in the body frame
	Real rotationFactor = 0.;         //!< W = sum_i w_i |p_i^ref - p_0|^2, if not alternativeApproach
	Matrix3D rotationMatrixInverse;   //!< W^{-1} = (-sum_i w_i tilde(p_i^ref - p_0)^2)^{-1}, if alternativeApproach
	Matrix positionModes;             //!< ObjectFFRFreducedOrder: d(sum_i w_i u_i) / d(modal coordinates), 3 x nModes; empty: computed from the mesh nodes on every evaluation
	Matrix rotationModes;             //!< ObjectFFRFreducedOrder: d(weighted rotation) / d(modal coordinates), 3 x nModes, if hasRotation

	//! true if the sums are computed from positionModes and rotationModes
	bool HasModes() const { return positionModes.NumberOfColumns() != 0; }

	//! W^{-1} v, for the weighted rotation and angular velocity
	Vector3D ApplyRotationWeighting(const Vector3D& v) const
	{
		if (alternativeApproach) { return rotationMatrixInverse * v; }
		return (1. / rotationFactor) * v;
	}
};

#endif
