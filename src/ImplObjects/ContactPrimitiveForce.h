/** ***********************************************************************************************
* @brief		The contact law of the primitive contacts: the normal force of the penalty and impact models and the
*				regularized dry friction (#3320)
* @details		Details:
*				- the law of ObjectContactSphereTriangle, SphereCylinder and the cylinder contacts (per contact point),
*				  the force on marker 0 for the gap, the normal n0 from marker 0 to marker 1, the normal and the
*				  relative velocity
*				- parameters: an item parameter structure with contactStiffness, contactDamping,
*				  contactStiffnessExponent, impactModel, minimumImpactVelocity, restitutionCoefficient and
*				  frictionProportionalZone
*
* @author		Gerstmayr Johannes
* @date			2026-11-07
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef CONTACTPRIMITIVEFORCE__H
#define CONTACTPRIMITIVEFORCE__H

//! the parameters of the Edinburgh adhesive elasto-plastic model of ObjectContactSphereSphere and the plastic overlap
//! of the contact point, from its data block (#3336)
struct ContactPlasticity
{
	Real plasticityRatio = 0.;		//!< lambda_P = 1 - k_c/K_2; 0: the elastic law
	Real adhesionCoefficient = 0.;
	Real adhesionExponent = 1.;
	Real pullOffForce = 0.;
	Real deltaPlastic = 0.;			//!< the plastic overlap of the last post Newton step
};

//! the contact force fVec on marker 0 (with the friction force fFriction in it) and the normal force; impactVelocity
//! the one of the data node, deltaVnormal the separation velocity, deltaVji the velocity of marker 1 relative to
//! marker 0 at the contact point; with forceFrictionMode the friction regime is frictionRegularizedRegion (from the
//! data node), else from the tangential velocity
template<typename TReal, class TParameters>
TReal ContactPrimitiveForce(const TParameters& parameters, Real impactVelocity, const char* itemName, TReal gap,
	const SlimVectorBase<TReal, 3>& n0, TReal deltaVnormal, const SlimVectorBase<TReal, 3>& deltaVji, TReal dryFriction,
	bool frictionRegularizedRegion, SlimVectorBase<TReal, 3>& fVec, SlimVectorBase<TReal, 3>& fFriction, bool forceFrictionMode,
	const ContactPlasticity* plasticity = nullptr)
{
	Real k1 = parameters.contactStiffness;
	Real d1 = parameters.contactDamping;
	Real n = parameters.contactStiffnessExponent;
	TReal deltaExp = -gap; // overwrite this in case that n != 1 holds (see following line)
	if (n != 1) { deltaExp = EXUstd::SignReal(deltaExp) * EM::Pow(fabs(deltaExp), n); } // also respect sign
	// at first, just use the easierst spring damper model (damping acts against n0 for sphere 1 if deltaVnormal is positive)
	TReal fDamp = -d1 * deltaVnormal;
	TReal contactForce = k1 * deltaExp + fDamp;

	if (plasticity && plasticity->plasticityRatio != 0. && parameters.impactModel == ContactImpactModel::Linear)
	{
		//the adhesive elasto-plastic model of ObjectContactSphereSphere: loading with k1, unloading and reloading with
		//k2 from the plastic overlap, adhesion down to -kAdh*delta^nAdh, a constant pull-off force (#3336)
		Real k2 = k1 / (1. - plasticity->plasticityRatio);
		TReal deltaPlasticExp = plasticity->deltaPlastic;
		if (n != 1) { deltaPlasticExp = EXUstd::SignReal(deltaPlasticExp) * EM::Pow(fabs(deltaPlasticExp), n); }
		TReal deltaAdhExp = -gap;
		if (plasticity->adhesionExponent != 1) { deltaAdhExp = EXUstd::SignReal(deltaAdhExp) * EM::Pow(fabs(deltaAdhExp), plasticity->adhesionExponent); }
		TReal unloading = k2 * (deltaExp - deltaPlasticExp);
		TReal adhesion = -plasticity->adhesionCoefficient * deltaAdhExp;
		if (unloading >= k1 * deltaExp) { contactForce = k1 * deltaExp; } //loading
		else if (unloading > adhesion) { contactForce = unloading; } //unloading
		else { contactForce = adhesion; } //unloading with adhesion
		contactForce += -plasticity->pullOffForce + fDamp;
	}
	else if (parameters.impactModel != ContactImpactModel::Linear)
	{
		// if impactModel != 0  treat the contact force with restitution model
		// for resititution coefficient < 1, lambda must be negative as component acting in direction of damping is added
		TReal minImpactVel = EXUstd::Maximum(parameters.minimumImpactVelocity, impactVelocity);
		if (minImpactVel > 0)
		{
			TReal lambda = k1 / minImpactVel;
			const Real& ee = parameters.restitutionCoefficient;
			if (parameters.impactModel == ContactImpactModel::HuntCrossley) // Hunt-Crossley
			{
				lambda *= 3. / 2. * (ee - 1.);
			}
			else if (parameters.impactModel == ContactImpactModel::GonthierCarvalhoMartins) // mixed Gonthier/EtAl-Carvalho/Martins
			{
				if (ee > 1. / 3.)
				{
					lambda *= 3. / 2. * (ee - 1.) * (11. - ee) / (1. + 9. * ee);
				}
				else if (ee > 0)
				{
					lambda *= (ee * ee - 1.) / ee;
				}
				else {
					CHECKandTHROWstring(STDstring(itemName) + ": coefficient of restitution must be > 0", ExudynValueError);
				}
			}
			contactForce += lambda * deltaExp * deltaVnormal;
		}
	}
	contactForce *= (-1); // this is the contact force acting on sphere (and moved to the LHS of the ODE2)
	fVec = contactForce * n0; //fVec = f_m1
	////////////////////////// add tangential component for the force vector if friction should be added
	// note that for coulomb friction we need fR acting against the velocity
	// fR = - fN * deltaVtangent
	// since fN is already moved to LHS of ODE2, the negative sign is not necessary: fR = fN * deltaVtangent
	// since we then must also move fR to LHS of ODE2, the computation again gets a negative sign
	if (dryFriction != 0.)
	{
		// global tangent velocity of spehere 1 relative to spehere 0
		SlimVectorBase<TReal, 3> deltaVtangent = deltaVji - deltaVnormal * n0;	// tangent velocity
		TReal relVel = deltaVtangent.GetL2Norm();
		if (!forceFrictionMode) // if mode is not enforced, then we have to compute it (this is wrong in GeneralContact!)
		{
			frictionRegularizedRegion = (relVel < parameters.frictionProportionalZone);
		}
		if (frictionRegularizedRegion && parameters.frictionProportionalZone != 0)
		{
			//Warning: in the adhesive model, contactForce could become positive (=tension) and cause unphysical friction behavior
			// as long as relVel < frictionProportionalZone, friction force is computed linearily with deltaVtangent
			fFriction = (-dryFriction * contactForce / parameters.frictionProportionalZone) * deltaVtangent;
		}
		else
		{
			if (relVel != 0.) //in case that frictionProportionalZone=0, this could happen!
			{
				// this is to scale the friction force with the relative velocity
				// the friction force is decreased for high relative velocities
				fFriction = (-dryFriction * contactForce / relVel) * deltaVtangent; //this should be put into the nonlinear iteration for better Newton convergence ...
			}
			else { fFriction.SetAll(0.); }
		}
		fVec += fFriction;
	}
	return contactForce;
}

#endif
