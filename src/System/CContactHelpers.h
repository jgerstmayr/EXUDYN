/** ***********************************************************************************************
* @brief		Helper functions for class CContact
* @details		Details:
*               Helper functions for special contact models
*
* @author		Gerstmayr Johannes
* @date			2021-12-12 (generated)
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
#ifndef CCONTACTHELPERS__H
#define CCONTACTHELPERS__H


#define USE_GENERAL_CONTACT
#ifdef USE_GENERAL_CONTACT

//#include "Linalg/BasicLinalg.h"		//includes basic classes, all basic arrays and vectors
//#include "Linalg/SearchTree.h"
//#include "Main/TemporaryComputationData.h"
//
//#include "Objects/CObjectANCFCable2DBase.h"
#include "Linalg/Geometry.h"

extern bool warnedComputeEigenValuesANCFcableCircleContact; //low-level warning for ANCF contact; may be eliminated in future





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace ContactHelper
{
	//! regularized dry friction model, based on frictionProportionalZone (m/s), setting forceVector to friction force
	//! return true, if friction is in frictionProportionalZone; otherwise false
	//! if forceFrictionMode==true, regularized friction is always used if frictionRegularizedRegion=true, otherwise dry friction is always used 
	//! also returns scalar tangent velocity in relVel
	template<class TVector, class TReal, bool forceFrictionMode = false>
	bool ComputeFrictionForce(TVector& forceVector, const TVector& deltaVtangent, TReal contactPressure,
		TReal dryFriction, TReal frictionProportionalZone, TReal& relVel, bool frictionRegularizedRegion=true)
	{
		relVel = deltaVtangent.GetL2Norm();

		//BUG? in case that forceFrictionMode = True and frictionRegularizedRegion =False, it MUST always go to else branch!
		if (relVel < frictionProportionalZone || (frictionRegularizedRegion && forceFrictionMode)) //would also work for frictionProportionalZone=0 ==> no proportional zone!
		{
			//as long as vVel < frictionProportionalZone, friction force shall linearly increase
			forceVector = (dryFriction * contactPressure / frictionProportionalZone) * deltaVtangent;
			return true;
		}
		else
		{
			if (relVel != 0.) //in case that frictionProportionalZone=0, this could happen!
			{
				forceVector = (dryFriction*contactPressure / relVel) * deltaVtangent; //this should be put into the nonlinear iteration for better Newton convergence ...
			}
			else { forceVector.SetAll(0.); }
		}
		return false;
	}

	//! regularized dry friction model based on frictionVelocityPenalty factor (N/(m^2*m/s); relative to unit area), setting forceVector to friction force
	//! return true, if friction is in frictionProportionalZone; otherwise false
	//! if forceFrictionMode==true, regularized friction is always used if frictionRegularizedRegion=true, otherwise dry friction is always used 
	//! also returns scalar tangent velocity in relVel
	template<class TVector, bool forceFrictionMode = false>
	bool ComputeFrictionForcePenalty(TVector& forceVector, const TVector& deltaVtangent, Real contactPressure,
		Real dryFriction, Real frictionVelocityPenalty, Real& relVel, bool frictionRegularizedRegion = true)
	{
		relVel = deltaVtangent.GetL2Norm();
		Real force = relVel * frictionVelocityPenalty;

		if (force < fabs(contactPressure)*dryFriction || (frictionRegularizedRegion && forceFrictionMode)) 
		{
			//as long as friction force < mu*contact force, friction force shall linearly increase
			forceVector = frictionVelocityPenalty * deltaVtangent;
			return true;
		}
		else
		{
			if (relVel != 0.) 
			{
				forceVector = (dryFriction*contactPressure / relVel) * deltaVtangent; //this should be put into the nonlinear iteration for better Newton convergence ...
			}
			else { forceVector.SetAll(0.); }
		}
		return false;
	}

	//! compute approximate contact segments for ANCF cable element
	void ComputeContactSegmentsANCFcableCircleContactApprox(const ConstSizeVector<DANCFmaxCoordinates>& q, Real L, Real halfHeight,
		const Vector2D& circlePos, Real r, 
		ConstSizeVectorBase<Vector2D, DANCFselectedSegmentsLength>& selectedSegments, Index nSegments)
	{
		selectedSegments.SetNumberOfItems(0);
		ConstSizeVector<4> c4x;
		ConstSizeVector<4> c4y;
		CObjectANCFCable2DBase::ComputePolynomialCoeffs<ConstSizeVector<DANCFmaxCoordinates>>(q, L, c4x, c4y);

		Real lSeg = L / (Real)nSegments;
		for (Index ix = 0; ix < nSegments; ix++)
		{
			Real x0 = ix * lSeg;
			Real x1 = (ix+1) * lSeg;
			Vector2D p0({ EXUmath::EvaluatePolynomial(c4x, x0), EXUmath::EvaluatePolynomial(c4y, x0) });
			Vector2D p1({ EXUmath::EvaluatePolynomial(c4x, x1), EXUmath::EvaluatePolynomial(c4y, x1) });
			//pout << "p0, p1=" << p0 << p1 << "\n";
			//Vector2D p0({ q[0], q[1] });
			//Vector2D p1({ q[4], q[5] });

			ConstSizeVector<2> relPos;
			
			EGeometry::LineCircleIntersectionPoints(p0, p1, circlePos, r+halfHeight, relPos);
			//pout << "p0=" << p0 << "p1=" << p1 << "circlePos=" << circlePos << "r=" << r + hANCF << "relPos=" << relPos << "\n";
			//Real xNearest;
			//Vector2D vv;
			//Real distance = HGeometry::ShortestDistanceEndPointsRelativePosition(p0, p1, circlePos, xNearest, vv);
			//if (distance - r - hANCF < 0)
/*
			if (relPos.NumberOfItems() > 0)
			{
				pout << "relPos=" << relPos << ", lSeg=" << lSeg << "\n";
			}*/
			if (relPos.NumberOfItems() == 2)
			{
				selectedSegments.AppendItem(Vector2D({ relPos[0]*lSeg+x0, relPos[1]*lSeg+x0}));
			}
		}
	}

	//! compute max/min values for 3rd order polynomial in range [0, L]
	void ComputePoly3rdOrderMinMax(ConstSizeVector<4>& p, Real L, Real& fMin, Real& fMax)
	{
//#define ComputePoly3rdOrderMinMaxDebug
		ConstSizeVector<3> c; //derivative polynomial
		c[0] = p[1];
		c[1] = 2*p[2];
		c[2] = 3*p[3];

#ifdef ComputePoly3rdOrderMinMaxDebug
		pout << "ComputePoly3rdOrderMinMax: p=" << p << ", L=" << L << "\n";
#endif
		Real v0 = EXUmath::EvaluatePolynomial(p, 0);
		Real vL = EXUmath::EvaluatePolynomial(p, L);
		fMin = EXUstd::Minimum(v0, vL);
		fMax = EXUstd::Maximum(v0, vL);

		if (c[2] != 0)
		{
			Real detX = c[1] * c[1] - 4. * c[2] * c[0];
#ifdef ComputePoly3rdOrderMinMaxDebug
			pout << "  detX=" << detX << "\n";
#endif

			if (detX >= 0.)
			{
				Real sDetX = sqrt(detX);
				Real x, v;
				x = (-c[1] + sDetX) / (2. * c[2]);
#ifdef ComputePoly3rdOrderMinMaxDebug
				pout << "   x0=" << x << ", v0=" << EXUmath::EvaluatePolynomial(p, x) << "\n";
#endif
				if (x > 0 && x < L)
				{
					v = EXUmath::EvaluatePolynomial(p, x);
					fMin = EXUstd::Minimum(fMin, v);
					fMax = EXUstd::Maximum(fMax, v);
				}

				x = (-c[1] - sDetX) / (2. * c[2]);
#ifdef ComputePoly3rdOrderMinMaxDebug
				pout << "   x1=" << x << ", v1=" << EXUmath::EvaluatePolynomial(p, x) << "\n";
#endif
				if (x > 0 && x < L)
				{
					v = EXUmath::EvaluatePolynomial(p, x);
					fMin = EXUstd::Minimum(fMin, v);
					fMax = EXUstd::Maximum(fMax, v);
				}
			} //otherwise, no min/max with real numbers! ==> min/max at boundary!
		}
		else
		{
			if (c[1] != 0)
			{
				Real x = -c[0] / c[1]; //zero
#ifdef ComputePoly3rdOrderMinMaxDebug
				pout << "   x2=" << x << ", v2=" << EXUmath::EvaluatePolynomial(p, x) << "\n";
#endif
				if (x > 0 && x < L)
				{
					Real v = EXUmath::EvaluatePolynomial(p, x);
					fMin = EXUstd::Minimum(fMin, v);
					fMax = EXUstd::Maximum(fMax, v);
				}
			}
		}
	}

	//! compute polynomial coefficients for 6th order polynomial coeffs (coeffs[0] is constant coefficient) in contact with circle,
	//! using ANCF element (total) coordinates q, element length, circle center point circlePos and radius r 
	void ANCFCable2DcontactCircleCoeffs(const ConstSizeVector<DANCFmaxCoordinates>& q, Real L, const Vector2D& circlePos, Real r,
		ConstSizeVector<DANCFcirclePolynomialDegree+1>& coeffs)
	{
		//6th order polynom
		Real L2 = EXUstd::Square(L);
		//Real L3 = L * EXUstd::Square(L);
		Real divL = 1 / L;
		Real divL2 = EXUstd::Square(divL);
		Real divL3 = divL * divL2;
		coeffs[0] = EXUstd::Square(circlePos[0]) - EXUstd::Square(r) - 2 * circlePos[0] * q[0] + EXUstd::Square(q[0]) + EXUstd::Square(circlePos[1] - q[1]);

		coeffs[1] = -2 * circlePos[0] * q[2] + 2 * q[0] * q[2] - 2 * circlePos[1] * q[3] + 2 * q[1] * q[3];

		coeffs[2] = (divL2)*(-6 * EXUstd::Square(q[0]) + 6 * q[0] * q[4] - 2 * L* q[0] * (2 * q[2] + q[6]) +
			2 * circlePos[0] * (3 * q[0] - 3 * q[4] + 2 * L* q[2] + L * q[6]) + 6 * (circlePos[1] - q[1]) *(q[1] - q[5]) +
			L2 * (EXUstd::Square(q[2]) + EXUstd::Square(q[3])) + 2 * L* (circlePos[1] - q[1])* (2 * q[3] + q[7]));

		coeffs[3] = (divL3)*(-4 * circlePos[0] * q[0] + 4 * EXUstd::Square(q[0]) + 4 * circlePos[0] * q[4] - 4 * q[0] * (q[4] + L * q[2]) +
			2 * L* q[0] * q[6] - 2 * circlePos[0] * L* (q[2] + q[6]) - 4 * (circlePos[1] - q[1]) *(q[1] - q[5]) -
			2 * L* (-3 * q[4] * q[2] +
				L * q[2] * (2 * q[2] + q[6]) + (circlePos[1] + 2 * q[1] - 3 * q[5]) *q[3] + (circlePos[1] - q[1])* q[7] +
				L * q[3] * (2 * q[3] + q[7])));

		coeffs[4] = (divL2*divL2)*(9 * EXUstd::Square(q[0]) + 9 * EXUstd::Square(q[4]) - 2 * L* q[4] * (8 * q[2] + 3 * q[6]) +
			2 * q[0] * (-9 * q[4] + 8 * L *q[2] + 3 * L *q[6]) + 9 * EXUstd::Square(q[1] - q[5]) +
			L * (2 * (q[1] - q[5])* (8 * q[3] + 3 * q[7]) +
				L * (6 * EXUstd::Square(q[2]) + 6 * q[2] * q[6] + EXUstd::Square(q[6]) + 6 * EXUstd::Square(q[3]) + 6 * q[3] * q[7] + EXUstd::Square(q[7]))));

		coeffs[5] = -(divL3*divL2) * 2 * (6 * EXUstd::Square(q[0]) + 6 * EXUstd::Square(q[4]) - L * q[4] * (7 * q[2] + 5 * q[6]) +
			q[0] * (-12 * q[4] + 7 * L *q[2] + 5 * L *q[6]) + 6 * EXUstd::Square(q[1] - q[5]) +
			L * (L *(q[2] + q[6]) *(2 * q[2] + q[6]) +
				L * (q[3] + q[7]) *(2 * q[3] + q[7]) + (q[1] - q[5]) *(7 * q[3] + 5 * q[7])));

		coeffs[6] = (divL3*divL3)*(4 * EXUstd::Square(q[0]) - 8 * q[0] * q[4] + 4 * EXUstd::Square(q[4]) + 4 * L *q[0] * (q[2] + q[6]) -
			4 * L* q[4] * (q[2] + q[6]) + 4 * EXUstd::Square(q[1] - q[5]) + 4 * L *(q[1] - q[5]) *(q[3] + q[7]) +
			L2 * (EXUstd::Square(q[2] + q[6]) + EXUstd::Square(q[3] + q[7])));
	}

	//! helper function to compute roots of 6th order polynomial - contact of circle with ANCF cable 
	void ComputeRootsANCFcableCircleContact(const ConstSizeVector<DANCFcirclePolynomialDegree+1>& coeffs, 
		ConstSizeVectorBase<RealC, DANCFcirclePolynomialDegree>& complexRoots)
	{
		//as our polynomial is obtained from quadratic circle equation,
		complexRoots.SetNumberOfItems(0); //if no roots found
		//there are only 6, 4, or 2 roots!
		if (coeffs[6] != 0.)
		{
			complexRoots.SetNumberOfItems(6); //this is the standard case, very costly!
			EXUmath::PolynomialRoots<6>(coeffs, complexRoots);
		}
		else if (coeffs[4] != 0.)
		{
			complexRoots.SetNumberOfItems(4);
			//copy coefficients, but solution is much faster than with 6 coeffs!
			ConstSizeVector<5> coeffs2;
			ConstSizeVectorBase<RealC, 4> complexRoots2;
			for (Index i = 0; i < coeffs2.NumberOfItems(); i++)
			{
				coeffs2[i] = coeffs[i];
			}

			EXUmath::PolynomialRoots<4>(coeffs2, complexRoots2);

			complexRoots.SetNumberOfItems(4); //this is the standard case, very costly!
			for (Index i = 0; i < complexRoots2.NumberOfItems(); i++)
			{
				complexRoots[i] = complexRoots2[i];
			}
		}
		else if (coeffs[2] != 0.)
		{ //use quadratic equation
			//x1,2 = (-c1 +- sqrt(c1^2-4*c0*c2) ) / (2*c2)
			complexRoots.SetNumberOfItems(2);
			Real det = EXUstd::Square(coeffs[1]) - 4.*coeffs[0] * coeffs[2];
			if (det >= 0)
			{
				//2 real solutions:
				complexRoots[0] = RealC((-coeffs[1] - sqrt(det)) / (2 * coeffs[2]), 0.);
				complexRoots[1] = RealC((-coeffs[1] + sqrt(det)) / (2 * coeffs[2]), 0.);
			}
			else
			{
				//2 complex solutions:
				complexRoots[0] = RealC(-coeffs[1] / (2 * coeffs[2]), -sqrt(-det) / (2 * coeffs[2]));
				complexRoots[1] = RealC(-coeffs[1] / (2 * coeffs[2]), sqrt(-det) / (2 * coeffs[2]));
			}
		}
		else if (coeffs[1] != 0.)
		{
			//this should not happen!
			if (fabs(coeffs[1] > 1e-12))
			{
				if (!warnedComputeEigenValuesANCFcableCircleContact)
				{
					PyWarning("GeneralContact::ComputeEigenValuesANCFcableCircleContact: polynomial has less than 2 non-zero coefficients; this should not happen, probably your ANCF element or circle have zero size? Please report to Exudyn developers!");
					warnedComputeEigenValuesANCFcableCircleContact = true;
				}
			}
		}
		//pout << "  ANCF eigv=" << complexRoots << "\n";

		//returns unsorted complex roots; further processing done later on
	}

	//! helper function to compute contact sections based on roots of 6th order polynomial - contact of circle with ANCF cable 
	//! only selects sections with midpoint being inside circle, using range [0, L]
	void ComputeContactSegmentsANCFcableCircleContact(const ConstSizeVector<DANCFcirclePolynomialDegree+1>& coeffs, const ConstSizeVectorBase<RealC, DANCFcirclePolynomialDegree>& complexRoots,
		Real L, ConstSizeVectorBase<Vector2D, DANCFselectedSegmentsLength>& selectedSegments)
	{
		//select roots:
		ConstSizeVector<DANCFcirclePolynomialDegree> selectedRoots(0);
		const Real epsImag = 1e-9; //treashold can be very small, as it is a quadratic equation
		const Real tolReal = 1e-14; //tolerance avoiding double end points

		bool has0 = false;
		bool hasL = false;
		for (const RealC& value : complexRoots)
		{
			if (fabs(value.imag()) <= epsImag)
			{
				if (value.real() < tolReal)
				{
					if (!has0)
					{
						selectedRoots.AppendItem(0);
						has0 = true;
					}
				}
				else if (value.real() > L - tolReal)
				{
					if (!hasL)
					{
						selectedRoots.AppendItem(L);
						hasL = true;
					}
				}
				else
				{
					selectedRoots.AppendItem(value.real());
				}
			}
		}

		selectedSegments.SetNumberOfItems(0);

		if (selectedRoots.NumberOfItems() != 0)
		{
			EXUstd::QuickSort(selectedRoots);

			//now compute segments
			for (Index i = 0; i < selectedRoots.NumberOfItems() - 1; i++)
			{
				Real xa = selectedRoots[i];
				Real xb = selectedRoots[i + 1];
				if (EXUmath::EvaluatePolynomial(coeffs, 0.5*(xa + xb)) < 0)
				{
					selectedSegments.AppendItem(Vector2D({ xa,xb }));
				}
			}
		}
		//pout << "  ANCF segs=" << selectedSegments << "\n";
	}

	//! helper function to compute contact forces for given segments on ANCF cable, using numerical integration
	//! computes gereralized forces fANCF acting on ANCF, force fCircle acting on circle and torque tCircle acting on circle
	//! if forceFrictionMode=true, the switching between static friction force and regularized is selected by frictionRegularizedRegion
	void ComputeContactForcesANCFcableCircleContact(const ConstSizeVector<DANCFmaxCoordinates>& q, const ConstSizeVector<DANCFmaxCoordinates>& q_t,
		Real L, Real halfHeight, const Vector2D& circlePos, const ContactSpheresMarkerBased& sphere, 
		const ConstSizeVectorBase<Vector2D, DANCFselectedSegmentsLength>& selectedSegments,
		Real stiffness, Real damping, Real dryFriction, 
		ConstSizeVector<DANCFmaxCoordinates>& fANCF, Vector2D& fCircle, Real& tCircle, 
		ConstSizeVectorBase<Real, DANCFselectedSegmentsLength>& contactForces,
		ConstSizeVectorBase<Real, DANCFselectedSegmentsLength>& maxRelFrictionVels,
		const GeneralContactSettings& settings, bool forceFrictionMode=false)
	{
		//size must be set from function caller: could be 8 or 9! fANCF.SetNumberOfItems(8 or 9);
		fANCF.SetAll(0.);
		fCircle.SetAll(0.);
		tCircle = 0.;
		if (!forceFrictionMode) { 
			maxRelFrictionVels.SetNumberOfItems(selectedSegments.NumberOfItems());
			maxRelFrictionVels.SetAll(0.);
		}
		contactForces.SetNumberOfItems(0);

		Real r = sphere.radius;

		ConstSizeVector<4> cx;
		ConstSizeVector<4> cy;
		CObjectANCFCable2DBase::ComputePolynomialCoeffs(q, L, cx, cy);

		ConstSizeVector<4> cx_t;
		ConstSizeVector<4> cy_t;
		CObjectANCFCable2DBase::ComputePolynomialCoeffs(q_t, L, cx_t, cy_t);

		const Index dim = 2;
		const Index ns = 4;

		ConstSizeVector<4> integrationPoints;
		ConstSizeVector<4> integrationWeights;
		if (settings.ancfCableUseExactMethod)
		{
			integrationPoints.CopyFrom(EXUmath::lobattoRuleOrder3Points);
			integrationWeights.CopyFrom(EXUmath::lobattoRuleOrder3Weights);
		}
		else
		{
			integrationPoints.CopyFrom(EXUmath::lobattoRuleOrder3Points);
			integrationWeights.CopyFrom(EXUmath::lobattoRuleOrder3Weights);
			//integrationPoints.CopyFrom(EXUmath::gaussRuleOrder1Points);
			//integrationWeights.CopyFrom(EXUmath::gaussRuleOrder1Weights);
		}

		Vector2D p0, p1;
		Vector2D v0, v1;

		for (Index segCnt=0; segCnt < selectedSegments.NumberOfItems(); segCnt++)
		{
			const Vector2D& seg = selectedSegments[segCnt];

			if (!settings.ancfCableUseExactMethod)
			{
				Real x = 0.5*(seg[0] + seg[1]);
				Index segI = (Index)((x*settings.ancfCableNumberOfContactSegments) / L);
				Real lSeg = L / (Real)settings.ancfCableNumberOfContactSegments;
				Real seg0 = segI * lSeg;
				Real seg1 = (segI+1) * lSeg;
				//if (seg0 > seg[0] || seg1 < seg[1]) //due to roundoff, this warnings get called!
				//{
				//	pout << "WARNING: seg0,seg[0]=" << seg0 << "," << seg[0] << ", seg1,seg[1]" << seg1 << "," << seg[1] << "\n";
				//}
				Vector2D p0s, p1s; //position at segment boundaries
				Vector2D v0s, v1s;
				p0s = Vector2D({ EXUmath::EvaluatePolynomial(cx, seg0), EXUmath::EvaluatePolynomial(cy, seg0) });
				p1s = Vector2D({ EXUmath::EvaluatePolynomial(cx, seg1), EXUmath::EvaluatePolynomial(cy, seg1) });
				v0s = Vector2D({ EXUmath::EvaluatePolynomial(cx_t, seg0), EXUmath::EvaluatePolynomial(cy_t, seg0) });
				v1s = Vector2D({ EXUmath::EvaluatePolynomial(cx_t, seg1), EXUmath::EvaluatePolynomial(cy_t, seg1) });
				Real divlSeg = 1. / lSeg;
				p0 = divlSeg * ((seg[0] - seg0)*p1s + (seg1 - seg[0])*p0s);
				p1 = divlSeg * ((seg[1] - seg0)*p1s + (seg1 - seg[1])*p0s);
				v0 = divlSeg * ((seg[0] - seg0)*v1s + (seg1 - seg[0])*v0s);
				v1 = divlSeg * ((seg[1] - seg0)*v1s + (seg1 - seg[1])*v0s);
			}
			Real averageContactForce = 0.;

			for (Index i = 0; i < integrationPoints.NumberOfItems(); i++)
			{
				Real intFact = 0.5*(seg[1] - seg[0])*integrationWeights[i];

				Real x = 0.5*(seg[1] - seg[0])*integrationPoints[i] + 0.5*(seg[1] + seg[0]);
				Vector4D SV = CObjectANCFCable2DBase::ComputeShapeFunctions(x, L);
				Vector2D rANCF;
				Vector2D rANCF_t;
				if (settings.ancfCableUseExactMethod)
				{
					rANCF = Vector2D({ EXUmath::EvaluatePolynomial(cx, x), EXUmath::EvaluatePolynomial(cy, x) });
					rANCF_t = Vector2D({ EXUmath::EvaluatePolynomial(cx_t, x), EXUmath::EvaluatePolynomial(cy_t, x) });
				}
				else
				{
					rANCF = 0.5*((1 - integrationPoints[i])*p0 + (1 + integrationPoints[i])*p1);
					rANCF_t = 0.5*((1 - integrationPoints[i])*v0 + (1 + integrationPoints[i])*v1);
				}
				Vector2D dirForce = rANCF - circlePos;
				Vector2D vSphereJ({ sphere.velocity[0], sphere.velocity[1] });
				if (dryFriction != 0.)
				{
					vSphereJ += sphere.angularVelocity[2] * Vector2D({ -dirForce[1], dirForce[0] }); //here, dirForce is still the relative vector; local angular velocity, but in 2D does not matter
				}
				Real dist = dirForce.GetL2Norm();
				Real gap = dist - (r + halfHeight);
				if (dist != 0.) { dirForce *= 1. / dist; } //otherwise, no action ... because dirForce=0

				Real deltaVnormal = dirForce * (rANCF_t - vSphereJ ); //penetration velocity, = -(v.J-v.I) !

				Real contactForce = (stiffness * gap + damping * deltaVnormal);//includes intFact ==> friction forces then also include it!
				Vector2D fContact = contactForce * dirForce;

				averageContactForce += (contactForce / integrationPoints.NumberOfItems());

				//pout << "fContact = " << fContact << "\n";
				//add friction: just on/off per segment?
				//Vector2D fContact0 = fContact;
				if (dryFriction != 0.)
				{
					//global sphere velocity at contact point:

					Vector2D deltaVtangent = (rANCF_t - vSphereJ) - deltaVnormal * dirForce;	//tangent velocity
					Real relVel;
					//pout << "deltaVtang=" << deltaVtangent << ", deltaVnormal=" << deltaVnormal << ", dirForce=" << dirForce << "\n";
					Vector2D frictionAdd;
					//negative contactForce, because it is a pressure
					if (forceFrictionMode)
					{
						ComputeFrictionForce<Vector2D, Real, true>(frictionAdd, deltaVtangent, -contactForce, dryFriction,
						settings.frictionProportionalZone, relVel, maxRelFrictionVels[segCnt] < 1.);
					}
					else //compute friction from relative velocity
					{
						ComputeFrictionForce<Vector2D, Real, false>(frictionAdd, deltaVtangent, -contactForce, dryFriction,
							settings.frictionProportionalZone, relVel, maxRelFrictionVels[segCnt] < 1.);

						//store maximum; if it exceeds the dry friction, this is the way to go
						Real relativeFrictionVel = 0;
						if (settings.frictionProportionalZone != 0.) { relativeFrictionVel = relVel / settings.frictionProportionalZone; }
						if (fabs(relativeFrictionVel) > fabs(maxRelFrictionVels[segCnt])) { maxRelFrictionVels[segCnt] = relativeFrictionVel; }
					}
					fContact += frictionAdd;

					//pout << "I=" << i << ", fContact=" << fVec << ", Ffric=" << fVec - f0 << ", dP0=" << deltaP0 << ", vN=" << deltaVnormal << ", dV(I-J)=" << (vSphereI - vSphereJ) << ", deltaVtang=" << deltaVtangent << "\n";
				}

				//++++++++++++++++++++++++++++++++++
				//now add forces to circle and ANCF:
				tCircle += intFact * (r*dirForce).CrossProduct2D(fContact);
				fCircle += intFact * fContact; //-= in Python!

				for (Index i = 0; i < dim; i++)
				{
					for (Index j = 0; j < ns; j++)
					{
						fANCF[j*dim + i] += intFact * SV[j] * fContact[i];
					}
				}
			} //integration points loop
			contactForces.AppendItem(averageContactForce);
		} //selectedSeg loop
	}





};




#endif //USE_GENERAL_CONTACT

#endif
