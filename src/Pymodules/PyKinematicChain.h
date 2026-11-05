/** ***********************************************************************************************
* @brief		class PyKinematicChain: exudyn.cUtilities.KinematicChain, the kinematics of a serial or tree-like robot in
*				C++ for Python (#3259)
* @details		Details:
*				- the links of exudyn.robotics.Robot: per link a joint type (Rx, Ry, Rz, Px, Py, Pz), the parent link,
*				  preHT (from the parent's link frame to the joint frame, before the joint motion), localHT (from the
*				  joint frame after the joint motion to the link frame), mass and center of mass in the link frame; the
*				  base HT, the tool HT relative to the frame of the last link, and gravity
*				- the joint frame of link i: JointHT_i = LinkHT_parent * preHT_i * HTjoint(q_i), its link frame
*				  LinkHT_i = JointHT_i * localHT_i, the base HT instead of LinkHT_parent for a link without parent
*				- the analytic inverse kinematics of 3R robots and of 6R robots with a spherical wrist or the UR geometry
*				  (#3260), by the subproblems of Paden and Kahan on the product of exponentials
*				- the Python versions of the same algorithms are Robot.JointHT, Robot.LinkHT, Robot.Jacobian,
*				  Robot.StaticTorques and Robot.InverseKinematicsAnalytic of exudyn.robotics
*
* @author		Gerstmayr Johannes
* @date			2026-11-05
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYKINEMATICCHAIN__H
#define PYKINEMATICCHAIN__H

#include "Pymodules/PyCUtilities.h"

class PyKinematicChain
{
	Index numberOfLinks;
	ArrayIndex jointTypes;			//!< 0, 1, 2: revolute about x, y, z; 3, 4, 5: prismatic along x, y, z
	ArrayIndex parents;				//!< the parent link, -1 for the base
	ResizableArray<HomogeneousTransformation> preHTs;
	ResizableArray<HomogeneousTransformation> localHTs;
	Vector masses;
	Vector3DList COMs;				//!< the centers of mass in the link frames
	HomogeneousTransformation baseHT;
	HomogeneousTransformation toolHT;
	Vector3D gravity;

	//! the HT of the joint motion q about or along the joint axis
	static HomogeneousTransformation JointMotion(Index jointType, Real q)
	{
		HomogeneousTransformation H;
		if (jointType == 0) { H.SetRotationX(q); }
		else if (jointType == 1) { H.SetRotationY(q); }
		else if (jointType == 2) { H.SetRotationZ(q); }
		else
		{
			Vector3D p({ 0., 0., 0. });
			p[jointType - 3] = q;
			H.SetTranslation(p);
		}
		return H;
	}

	Vector GetCoordinates(const py::object& q, const char* functionName) const
	{
		PyCUtilities::ArrayReal array = PyCUtilities::ToArray(q, functionName, "q");
		if (array.ndim() != 1 || array.size() != numberOfLinks)
		{
			PyError(STDstring(functionName) + ": q must have " + EXUstd::ToString(numberOfLinks) + " components, one per link; received shape "
				+ PyCUtilities::ShapeString(array), PyErrorType::valueError);
		}
		Vector v(numberOfLinks);
		for (Index i = 0; i < numberOfLinks; i++) { v[i] = array.data()[i]; }
		return v;
	}

	static void HTListFromPython(const py::object& value, Index size, ResizableArray<HomogeneousTransformation>& list, const char* name)
	{
		list.SetNumberOfItems(size);
		if (value.is_none()) { for (Index i = 0; i < size; i++) { list[i] = HomogeneousTransformation(); } return; }
		py::list values = py::cast<py::list>(value);
		if ((Index)values.size() != size)
		{
			PyError(STDstring("KinematicChain: ") + name + " must have one HT per link (" + EXUstd::ToString(size) + "), received "
				+ EXUstd::ToString((Index)values.size()), PyErrorType::valueError);
		}
		for (Index i = 0; i < size; i++) { EPyUtils::HTFromPython(values[i], list[i], name); }
	}

	static py::list ToPythonList(const ResizableArray<HomogeneousTransformation>& list)
	{
		py::list result;
		for (const HomogeneousTransformation& H : list) { result.append(py::cast(PyHT(H))); }
		return result;
	}

public:
	//! the chain of jointTypes (a list of 'Rx', 'Ry', 'Rz', 'Px', 'Py', 'Pz'), parents (None: a serial chain), preHTs and
	//! localHTs (None: identities), masses and COMs (None: zero), the base and tool HT (None: identity) and gravity
	PyKinematicChain(const py::object& jointTypesPy, const py::object& preHTsPy, const py::object& localHTsPy,
		const py::object& parentsPy, const py::object& massesPy, const py::object& COMsPy, const py::object& baseHTPy,
		const py::object& toolHTPy, const py::object& gravityPy)
	{
		py::list types = py::cast<py::list>(jointTypesPy);
		numberOfLinks = (Index)types.size();
		const STDstring names[6] = { "Rx", "Ry", "Rz", "Px", "Py", "Pz" };
		jointTypes.SetNumberOfItems(numberOfLinks);
		parents.SetNumberOfItems(numberOfLinks);
		for (Index i = 0; i < numberOfLinks; i++)
		{
			STDstring type = py::cast<STDstring>(types[i]);
			jointTypes[i] = -1;
			for (Index j = 0; j < 6; j++) { if (type == names[j]) { jointTypes[i] = j; } }
			if (jointTypes[i] == -1)
			{
				PyError("KinematicChain: jointTypes[" + EXUstd::ToString(i) + "] is '" + type + "'; it must be one of Rx, Ry, Rz, Px, Py, Pz",
					PyErrorType::valueError);
			}
			parents[i] = i - 1;
		}
		if (!parentsPy.is_none())
		{
			py::list list = py::cast<py::list>(parentsPy);
			if ((Index)list.size() != numberOfLinks) { PyError("KinematicChain: parents must have one entry per link", PyErrorType::valueError); }
			for (Index i = 0; i < numberOfLinks; i++)
			{
				parents[i] = py::cast<Index>(list[i]);
				if (parents[i] < -1 || parents[i] >= i)
				{
					PyError("KinematicChain: parents[" + EXUstd::ToString(i) + "] must be -1 (the base) or a link before it", PyErrorType::valueError);
				}
			}
		}
		HTListFromPython(preHTsPy, numberOfLinks, preHTs, "preHTs");
		HTListFromPython(localHTsPy, numberOfLinks, localHTs, "localHTs");
		masses.SetNumberOfItems(numberOfLinks);
		masses.SetAll(0.);
		if (!massesPy.is_none())
		{
			py::list list = py::cast<py::list>(massesPy);
			if ((Index)list.size() != numberOfLinks) { PyError("KinematicChain: masses must have one entry per link", PyErrorType::valueError); }
			for (Index i = 0; i < numberOfLinks; i++) { masses[i] = py::cast<Real>(list[i]); }
		}
		COMs.SetNumberOfItems(numberOfLinks);
		for (Index i = 0; i < numberOfLinks; i++) { COMs[i] = Vector3D({ 0., 0., 0. }); }
		if (!COMsPy.is_none())
		{
			py::list list = py::cast<py::list>(COMsPy);
			if ((Index)list.size() != numberOfLinks) { PyError("KinematicChain: COMs must have one entry per link", PyErrorType::valueError); }
			for (Index i = 0; i < numberOfLinks; i++) { COMs[i] = PyCUtilities::GetVector<3>(list[i], "KinematicChain", "COMs"); }
		}
		if (!baseHTPy.is_none()) { EPyUtils::HTFromPython(baseHTPy, baseHT, "KinematicChain: baseHT"); }
		if (!toolHTPy.is_none()) { EPyUtils::HTFromPython(toolHTPy, toolHT, "KinematicChain: toolHT"); }
		gravity = gravityPy.is_none() ? Vector3D({ 0., 0., 0. }) : PyCUtilities::GetVector<3>(gravityPy, "KinematicChain", "gravity");
	}

	//! KinematicChain() without arguments is refused: a chain needs its links
	static PyKinematicChain* ForbidConstructor()
	{
		PyError("exudyn.cUtilities.KinematicChain() needs jointTypes, one per link; Robot.GetKinematicChain() creates it from a Robot", PyErrorType::typeError);
		return nullptr; //never reached
	}

	Index NumberOfLinks() const { return numberOfLinks; }

	//! the joint frames (after the joint motion) and the link frames of all links
	void ComputeFrames(const Vector& q, ResizableArray<HomogeneousTransformation>& jointFrames,
		ResizableArray<HomogeneousTransformation>& linkFrames) const
	{
		jointFrames.SetNumberOfItems(numberOfLinks);
		linkFrames.SetNumberOfItems(numberOfLinks);
		for (Index i = 0; i < numberOfLinks; i++)
		{
			const HomogeneousTransformation& parentFrame = parents[i] == -1 ? baseHT : linkFrames[parents[i]];
			jointFrames[i] = parentFrame * preHTs[i] * JointMotion(jointTypes[i], q[i]);
			linkFrames[i] = jointFrames[i] * localHTs[i];
		}
	}

	//! the global axis of joint i from its joint frame
	Vector3D JointAxis(Index i, const HomogeneousTransformation& jointFrame) const
	{
		Vector3D axis({ 0., 0., 0. });
		axis[jointTypes[i] % 3] = 1.;
		return jointFrame.RotateVector(axis);
	}

	py::list JointHT(const py::object& q) const
	{
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(GetCoordinates(q, "KinematicChain.JointHT"), jointFrames, linkFrames);
		return ToPythonList(jointFrames);
	}

	py::list LinkHT(const py::object& q) const
	{
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(GetCoordinates(q, "KinematicChain.LinkHT"), jointFrames, linkFrames);
		return ToPythonList(linkFrames);
	}

	PyHT ToolHT(const py::object& q) const
	{
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(GetCoordinates(q, "KinematicChain.ToolHT"), jointFrames, linkFrames);
		return PyHT(linkFrames[numberOfLinks - 1] * toolHT);
	}

	//! the 6 x n Jacobian of the point localPosition (None: the origin) of link linkIndex (-1: the last link), in its link frame:
	//! rows 0-2 the velocity of the point, rows 3-5 the angular velocity, per joint velocity; zero for the joints not
	//! in the chain from the link to the base
	py::array_t<Real> Jacobian(const py::object& q, Index linkIndex, const py::object& localPosition) const
	{
		if (linkIndex == -1) { linkIndex = numberOfLinks - 1; }
		if (linkIndex < 0 || linkIndex >= numberOfLinks)
		{
			PyError("KinematicChain.Jacobian: linkIndex must be -1 or in 0 .. " + EXUstd::ToString(numberOfLinks - 1), PyErrorType::valueError);
		}
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(GetCoordinates(q, "KinematicChain.Jacobian"), jointFrames, linkFrames);
		Vector3D local({ 0., 0., 0. });
		if (!localPosition.is_none()) { local = PyCUtilities::GetVector<3>(localPosition, "KinematicChain.Jacobian", "localPosition"); }
		Vector3D p = linkFrames[linkIndex] * local;
		py::array_t<Real> J(std::vector<py::ssize_t>({ 6, numberOfLinks }));
		Real* data = J.mutable_data();
		for (Index k = 0; k < 6 * numberOfLinks; k++) { data[k] = 0.; }
		for (Index i = linkIndex; i != -1; i = parents[i])
		{
			Vector3D axis = JointAxis(i, jointFrames[i]);
			Vector3D velocity = axis;
			if (jointTypes[i] < 3)
			{
				velocity = axis.CrossProduct(p - jointFrames[i].GetTranslation());
				for (Index k = 0; k < 3; k++) { data[(3 + k) * numberOfLinks + i] = axis[k]; }
			}
			for (Index k = 0; k < 3; k++) { data[k * numberOfLinks + i] = velocity[k]; }
		}
		return J;
	}

	//! the generalized forces of gravity on the joints, sum over the links j of J_j^T (m_j g) with the Jacobian of the
	//! center of mass of link j; in one backward pass: the mass and its first moment summed over the links after each
	//! joint, then tau_i = axis . ((S_i - M_i o_i) x g) for a revolute joint at o_i, axis . (M_i g) for a prismatic one
	py::array_t<Real> StaticTorques(const py::object& q) const
	{
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(GetCoordinates(q, "KinematicChain.StaticTorques"), jointFrames, linkFrames);
		Vector massSum(numberOfLinks);
		Vector3DList momentSum;
		momentSum.SetNumberOfItems(numberOfLinks);
		for (Index i = 0; i < numberOfLinks; i++)
		{
			massSum[i] = masses[i];
			momentSum[i] = masses[i] * (linkFrames[i] * COMs[i]);
		}
		for (Index i = numberOfLinks - 1; i >= 0; i--) //the links of the subtree come after their parent
		{
			if (parents[i] != -1)
			{
				massSum[parents[i]] += massSum[i];
				momentSum[parents[i]] += momentSum[i];
			}
		}
		py::array_t<Real> tau(numberOfLinks);
		Real* data = tau.mutable_data();
		for (Index i = 0; i < numberOfLinks; i++)
		{
			Vector3D axis = JointAxis(i, jointFrames[i]);
			if (jointTypes[i] < 3)
			{
				Vector3D lever = momentSum[i] - massSum[i] * jointFrames[i].GetTranslation();
				data[i] = axis * lever.CrossProduct(gravity);
			}
			else { data[i] = massSum[i] * (axis * gravity); }
		}
		return tau;
	}

private:
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//the analytic inverse kinematics (#3260): the robot as a product of exponentials, T(q) = E_1(q_1) ... E_n(q_n) T(0),
	//E_i the rotation about the axis of joint i at q = 0 (direction omega_i through the point p_i); the subproblems of
	//Paden and Kahan solve it; Robot.InverseKinematicsAnalytic is the Python version of the same algorithm

	//! a rotation matrix, row by row
	struct IKRotation
	{
		Real a[9];
		Vector3D operator*(const Vector3D& v) const
		{
			return Vector3D({ a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2], a[6] * v[0] + a[7] * v[1] + a[8] * v[2] });
		}
		Vector3D MultTransposed(const Vector3D& v) const
		{
			return Vector3D({ a[0] * v[0] + a[3] * v[1] + a[6] * v[2], a[1] * v[0] + a[4] * v[1] + a[7] * v[2], a[2] * v[0] + a[5] * v[1] + a[8] * v[2] });
		}
		IKRotation operator*(const IKRotation& B) const
		{
			IKRotation C;
			for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 3; j++) { C.a[3 * i + j] = a[3 * i] * B.a[j] + a[3 * i + 1] * B.a[3 + j] + a[3 * i + 2] * B.a[6 + j]; } }
			return C;
		}
		IKRotation Transposed() const
		{
			IKRotation C;
			for (Index i = 0; i < 3; i++) { for (Index j = 0; j < 3; j++) { C.a[3 * i + j] = a[3 * j + i]; } }
			return C;
		}
	};

	//! v rotated about the unit vector axis by angle (Rodrigues)
	static Vector3D IKRotate(const Vector3D& axis, Real angle, const Vector3D& v)
	{
		Real c = EM::Cos(angle);
		return c * v + EM::Sin(angle) * axis.CrossProduct(v) + ((1. - c) * (axis * v)) * axis;
	}

	static IKRotation IKRotationMatrix(const Vector3D& axis, Real angle)
	{
		IKRotation R;
		for (Index j = 0; j < 3; j++)
		{
			Vector3D e({ 0., 0., 0. });
			e[j] = 1.;
			Vector3D column = IKRotate(axis, angle, e);
			for (Index i = 0; i < 3; i++) { R.a[3 * i + j] = column[i]; }
		}
		return R;
	}

	static IKRotation IKRotationOf(const HomogeneousTransformation& H)
	{
		IKRotation R;
		for (Index i = 0; i < 9; i++) { R.a[i] = H.GetRotationDataPointer()[i]; }
		return R;
	}

	//! subproblem 1: the angle about axis that rotates u into v, both projected onto the plane normal to axis; fallback
	//! if a projection vanishes (any angle does it, a singular configuration)
	static Real IKAngle(const Vector3D& axis, const Vector3D& u, const Vector3D& v, Real fallback)
	{
		Vector3D up = u - (axis * u) * axis;
		Vector3D vp = v - (axis * v) * axis;
		if (up.GetL2Norm() < 1e-12 * (1. + u.GetL2Norm()) || vp.GetL2Norm() < 1e-12 * (1. + v.GetL2Norm())) { return fallback; }
		return EM::Atan2(axis * up.CrossProduct(vp), up * vp);
	}

	//! the angles x of a EM::Cos(x) + b EM::Sin(x) = e: two, or none if |e| > |(a, b)|, or fallback if a = b = e = 0
	static void IKAnglesCosSin(Real a, Real b, Real e, Real fallback, Real tolerance, std::vector<Real>& angles)
	{
		angles.clear();
		Real r = sqrt(a * a + b * b);
		if (r < tolerance)
		{
			if (fabs(e) < tolerance) { angles.push_back(fallback); }
			return;
		}
		Real x = e / r;
		if (fabs(x) > 1. + 1e-10) { return; }
		Real phi = EM::Atan2(b, a);
		Real d = EM::Acos(EXUstd::Minimum(1., EXUstd::Maximum(-1., x)));
		angles.push_back(phi + d);
		angles.push_back(phi - d);
	}

	//! the angles q_2, q_3 about the parallel axes 2 and 3 that move the point W0 to Y: subproblem 3 for q_3, the distance
	//! of the point from axis 2, then subproblem 1 for q_2; up to 2 solutions, appended as pairs to q23
	static void IKPlanar2R(const Vector3D& omega2, const Vector3D& omega3, const Vector3D& p2, const Vector3D& p3,
		const Vector3D& W0, const Vector3D& Y, Real fallback2, Real tolerance, std::vector<Real>& q23)
	{
		q23.clear();
		auto Projected = [&omega2](const Vector3D& x) { return x - (omega2 * x) * omega2; };
		Vector3D u = Projected(W0 - p3);
		Vector3D v = Projected(p2 - p3);
		Real delta = Projected(Y - p2).GetL2Norm();
		Real nu = u.GetL2Norm();
		Real nv = v.GetL2Norm();
		if (nu < tolerance || nv < tolerance) { return; }
		Real kappa = (nu * nu + nv * nv - delta * delta) / (2. * nu * nv);
		if (fabs(kappa) > 1. + 1e-10) { return; }
		Real alpha = EM::Atan2(omega3 * v.CrossProduct(u), v * u);
		Real d = EM::Acos(EXUstd::Minimum(1., EXUstd::Maximum(-1., kappa)));
		for (Real q3 : { -alpha + d, -alpha - d })
		{
			Vector3D Z = p3 + IKRotate(omega3, q3, W0 - p3);
			q23.push_back(IKAngle(omega2, Z - p2, Y - p2, fallback2));
			q23.push_back(q3);
		}
	}

	//! the joint angles q_1, q_2, q_3 that move the point W0 (fixed in link 3, at q = 0) to X, for the axes 2 and 3
	//! parallel and axis 1 not parallel to them: q_1 puts X into the plane normal to axis 2 in which links 2 and 3 move W0,
	//! then IKPlanar2R; up to 4 solutions, appended as triples to q123
	static void IKArm(const Vector3D* omega, const Vector3D* p, const Vector3D& W0, const Vector3D& X, const Vector& fallback,
		Real tolerance, std::vector<Real>& q123)
	{
		q123.clear();
		const Vector3D& n = omega[1];
		const Vector3D& w = omega[0];
		Vector3D d = X - p[0];
		Real wd = w * d;
		std::vector<Real> q1List, q23;
		IKAnglesCosSin(n * d - wd * (w * n), -(n * w.CrossProduct(d)), n * (W0 - p[0]) - wd * (w * n), fallback[0], tolerance, q1List);
		for (Real q1 : q1List)
		{
			IKPlanar2R(omega[1], omega[2], p[1], p[2], W0, p[0] + IKRotate(w, -q1, d), fallback[1], tolerance, q23);
			for (size_t k = 0; k < q23.size(); k += 2) { q123.push_back(q1); q123.push_back(q23[k]); q123.push_back(q23[k + 1]); }
		}
	}

	//! the angles q_4, q_5, q_6 of R_4 R_5 R_6 = R for three axes through one point: subproblem 2 for q_4, q_5, which turn
	//! omega_6 into R omega_6, then subproblem 1 for q_6; up to 2 solutions, appended as triples to q456
	static void IKWrist(const Vector3D& omega4, const Vector3D& omega5, const Vector3D& omega6, const IKRotation& R,
		const Vector& fallback, std::vector<Real>& q456)
	{
		q456.clear();
		const Vector3D& u = omega6;
		Vector3D v = R * omega6;
		Real c = omega4 * omega5;
		Vector3D cross = omega4.CrossProduct(omega5);
		if (cross.GetL2Norm() < 1e-12) { return; }
		Real alpha = (omega4 * v - c * (omega5 * u)) / (1. - c * c);
		Real beta = (omega5 * u - c * (omega4 * v)) / (1. - c * c);
		Real gamma2 = (u * u - alpha * alpha - beta * beta - 2. * alpha * beta * c) / (cross * cross);
		if (gamma2 < -1e-10) { return; }
		Real gamma = sqrt(EXUstd::Maximum(0., gamma2));
		Vector3D x = omega6.CrossProduct(cross); //a vector normal to omega6 for q_6
		if (x.GetL2Norm() <= 1e-6) { x = omega6.CrossProduct(omega4); }
		for (Real g : { gamma, -gamma })
		{
			Vector3D z = alpha * omega4 + beta * omega5 + g * cross;
			Real q5 = IKAngle(omega5, u, z, fallback[4]);
			Real q4 = IKAngle(omega4, z, v, fallback[3]);
			Vector3D y = IKRotationMatrix(omega5, q5).MultTransposed(IKRotationMatrix(omega4, q4).MultTransposed(R * x));
			q456.push_back(q4);
			q456.push_back(q5);
			q456.push_back(IKAngle(omega6, x, y, fallback[5]));
		}
	}

	//! the point nearest to the lines p0 + s a0 and p1 + t a1 (unit vectors) and their distance; false for parallel lines
	static bool IKLineIntersection(const Vector3D& p0, const Vector3D& a0, const Vector3D& p1, const Vector3D& a1, Vector3D& point, Real& distance)
	{
		Vector3D w = p0 - p1;
		Real b = a0 * a1;
		if (1. - b * b < 1e-12) { return false; }
		Real s = (b * (a1 * w) - a0 * w) / (1. - b * b);
		Real t = (a1 * w - b * (a0 * w)) / (1. - b * b);
		Vector3D x0 = p0 + s * a0;
		Vector3D x1 = p1 + t * a1;
		point = 0.5 * (x0 + x1);
		distance = (x0 - x1).GetL2Norm();
		return true;
	}

public:
	//! the analytic inverse kinematics: all joint coordinates q with ToolHT(q) equal to HT, for 3 revolute joints with
	//! the axes 2 and 3 parallel (the position only), 6 with the axes 2 and 3 parallel and a spherical wrist, or 6 with the
	//! axes 2, 3, 4 parallel and the axes 5 and 6 crossing (UR); one solution per row, checked by the forward kinematics;
	//! with qReference the angles nearest to it and sorted by the distance from it, else in [-pi, pi)
	py::array_t<Real> InverseKinematicsAnalytic(const py::object& HTpy, const py::object& qReferencePy) const
	{
		const char* functionName = "KinematicChain.InverseKinematicsAnalytic";
		const Index n = numberOfLinks;
		bool revoluteSerial = (n == 3 || n == 6);
		for (Index i = 0; i < n; i++) { if (jointTypes[i] >= 3 || parents[i] != i - 1) { revoluteSerial = false; } }
		if (!revoluteSerial) { PyError(STDstring(functionName) + ": the analytic inverse kinematics needs 3 or 6 revolute joints in series", PyErrorType::valueError); }
		Vector fallback(n);
		fallback.SetAll(0.);
		bool hasReference = !qReferencePy.is_none();
		if (hasReference) { fallback = GetCoordinates(qReferencePy, functionName); }
		HomogeneousTransformation T;
		EPyUtils::HTFromPython(HTpy, T, "KinematicChain.InverseKinematicsAnalytic: HT");

		//the axes at q = 0
		Vector zero(n);
		zero.SetAll(0.);
		ResizableArray<HomogeneousTransformation> jointFrames, linkFrames;
		ComputeFrames(zero, jointFrames, linkFrames);
		Vector3D omega[6], p[6];
		for (Index i = 0; i < n; i++)
		{
			omega[i] = JointAxis(i, jointFrames[i]);
			p[i] = jointFrames[i].GetTranslation();
		}
		HomogeneousTransformation T0 = linkFrames[n - 1] * toolHT;
		Real scale = (T0.GetTranslation() - p[0]).GetL2Norm();
		for (Index i = 0; i < n; i++) { scale = EXUstd::Maximum(scale, (p[i] - p[0]).GetL2Norm()); }
		scale += 1.;
		Real tolerance = 1e-10 * scale;
		auto Parallel = [&omega](Index i, Index j) { return omega[i].CrossProduct(omega[j]).GetL2Norm() < 1e-10; };
		if (!Parallel(1, 2) || Parallel(0, 1))
		{
			PyError(STDstring(functionName) + ": the axes of joints 2 and 3 must be parallel and that of joint 1 not parallel to them", PyErrorType::valueError);
		}

		HomogeneousTransformation G = T * T0.GetInverse(); //T(q) T(0)^-1 = E_1 ... E_n
		IKRotation RG = IKRotationOf(G);
		std::vector<Real> solutions; //n per solution
		std::vector<Real> q123, q456, q5List, q23;
		if (n == 3)
		{
			IKArm(omega, p, T0.GetTranslation(), T.GetTranslation(), fallback, tolerance, solutions);
		}
		else
		{
			Vector3D W0;
			Real distance;
			if (IKLineIntersection(p[3], omega[3], p[4], omega[4], W0, distance) && distance < tolerance
				&& (W0 - p[5]).CrossProduct(omega[5]).GetL2Norm() < tolerance)
			{
				//spherical wrist: the wrist center from q_1, q_2, q_3, then the wrist angles
				IKArm(omega, p, W0, G * W0, fallback, tolerance, q123);
				for (size_t k = 0; k < q123.size(); k += 3)
				{
					IKRotation R123 = IKRotationMatrix(omega[0], q123[k]) * IKRotationMatrix(omega[1], q123[k + 1]) * IKRotationMatrix(omega[2], q123[k + 2]);
					IKWrist(omega[3], omega[4], omega[5], R123.Transposed() * RG, fallback, q456);
					for (size_t j = 0; j < q456.size(); j += 3)
					{
						solutions.insert(solutions.end(), { q123[k], q123[k + 1], q123[k + 2], q456[j], q456[j + 1], q456[j + 2] });
					}
				}
			}
			else
			{
				if (!Parallel(1, 3) || !IKLineIntersection(p[4], omega[4], p[5], omega[5], W0, distance) || distance > tolerance || Parallel(3, 4))
				{
					PyError(STDstring(functionName) + ": a robot of 6 joints needs a spherical wrist (the axes 4, 5, 6 through one point) or the axes 2, 3, 4 parallel and the axes 5 and 6 crossing (UR)",
						PyErrorType::valueError);
				}
				//UR: q_1 from the point W0 of the axes 5 and 6, q_5 from the axis 6, q_6, the sum of q_2, q_3, q_4, then
				//q_2, q_3 for a point of axis 4
				const Vector3D& nAxis = omega[1];
				const Vector3D& w = omega[0];
				Vector3D X = G * W0;
				Vector3D d = X - p[0];
				Real wd = w * d;
				std::vector<Real> q1List;
				IKAnglesCosSin(nAxis * d - wd * (w * nAxis), -(nAxis * w.CrossProduct(d)), nAxis * (W0 - p[0]) - wd * (w * nAxis), fallback[0], tolerance, q1List);
				for (Real q1 : q1List)
				{
					IKRotation Rrest = IKRotationMatrix(w, q1).Transposed() * RG; //R(n, q_2+q_3+q_4) R_5 R_6
					Vector3D b = Rrest * omega[5];
					Real c56 = omega[4] * omega[5];
					IKAnglesCosSin(nAxis * omega[5] - c56 * (nAxis * omega[4]), nAxis * omega[4].CrossProduct(omega[5]),
						nAxis * b - c56 * (nAxis * omega[4]), fallback[4], 1e-10, q5List);
					for (Real q5 : q5List)
					{
						IKRotation R5 = IKRotationMatrix(omega[4], q5);
						Real q6 = IKAngle(omega[5], Rrest.MultTransposed(nAxis), R5.MultTransposed(nAxis), fallback[5]);
						IKRotation Rn = Rrest * IKRotationMatrix(omega[5], q6).Transposed() * R5.Transposed();
						Vector3D x = nAxis.CrossProduct(omega[4]); //normal to the parallel axes
						Real q234 = IKAngle(nAxis, x, Rn * x, 0.);
						Vector3D Y = p[0] + IKRotate(w, -q1, d);
						Vector3D target4 = Y - IKRotate(nAxis, q234, W0 - p[3]);
						IKPlanar2R(omega[1], omega[2], p[1], p[2], p[3], target4, fallback[1], tolerance, q23);
						for (size_t k = 0; k < q23.size(); k += 2)
						{
							Real q4 = (q234 - (omega[1] * nAxis) * q23[k] - (omega[2] * nAxis) * q23[k + 1]) / (omega[3] * nAxis);
							solutions.insert(solutions.end(), { q1, q23[k], q23[k + 1], q4, q5, q6 });
						}
					}
				}
			}
		}

		//the angles in [-pi, pi) or nearest to qReference, checked by the forward kinematics, without repetitions
		std::vector<Vector> result;
		Vector q(n);
		for (size_t k = 0; k < solutions.size(); k += n)
		{
			for (Index i = 0; i < n; i++)
			{
				Real angle = solutions[k + i];
				q[i] = hasReference ? angle - 2. * EXUstd::pi * std::round((angle - fallback[i]) / (2. * EXUstd::pi))
					: angle - 2. * EXUstd::pi * std::floor((angle + EXUstd::pi) / (2. * EXUstd::pi));
			}
			ComputeFrames(q, jointFrames, linkFrames);
			HomogeneousTransformation Tq = linkFrames[n - 1] * toolHT;
			Real error = (Tq.GetTranslation() - T.GetTranslation()).GetL2Norm() / scale;
			if (n == 6)
			{
				Real sum = 0.;
				for (Index i = 0; i < 9; i++) { sum += EXUstd::Square(Tq.GetRotationDataPointer()[i] - T.GetRotationDataPointer()[i]); }
				error += sqrt(sum);
			}
			bool isNew = true;
			for (const Vector& r : result) { if ((q - r).GetL2Norm() <= 1e-8) { isNew = false; } }
			if (error < 1e-8 && isNew) { result.push_back(q); }
		}
		if (hasReference)
		{
			std::stable_sort(result.begin(), result.end(), [&fallback](const Vector& a, const Vector& b) { return (a - fallback).GetL2Norm() < (b - fallback).GetL2Norm(); });
		}
		py::array_t<Real> array(std::vector<py::ssize_t>({ (py::ssize_t)result.size(), n }));
		Real* data = array.mutable_data();
		for (size_t k = 0; k < result.size(); k++) { for (Index i = 0; i < n; i++) { data[k * n + i] = result[k][i]; } }
		return array;
	}

	STDstring ToString() const
	{
		return "KinematicChain(" + EXUstd::ToString(numberOfLinks) + " links)";
	}
};

#endif
