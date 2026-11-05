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
*				- the Python versions of the same algorithms are Robot.JointHT, Robot.LinkHT, Robot.Jacobian and
*				  Robot.StaticTorques of exudyn.robotics
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

	STDstring ToString() const
	{
		return "KinematicChain(" + EXUstd::ToString(numberOfLinks) + " links)";
	}
};

#endif
