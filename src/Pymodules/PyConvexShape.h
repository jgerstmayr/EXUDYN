/** ***********************************************************************************************
* @brief		class PyConvexShape and PyCUtilities::Distance: the convex shapes of EGeometry and their distance for
*				exudyn.cUtilities (#3317)
* @details		Details:
*				- a ConvexShape is created by its type ('Sphere', 'Capsule', 'Cylinder', 'Box', 'Triangle', 'Points')
*				  and the parameters the type uses; a wrong or missing parameter raises a ValueError
*				- Distance(shape0, HT0, shape1, HT1) by EGeometry::ConvexDistance (GJK and EPA, the sphere and the
*				  cylinder analytic)
*				- bound in definitions/pybindCUtilities.py
*
* @author		Gerstmayr Johannes
* @date			2026-11-07
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
#ifndef PYCONVEXSHAPE__H
#define PYCONVEXSHAPE__H

#include "Linalg/GeometryConvex.h"
#include "Pymodules/PyCUtilities.h"

class PyConvexShape : public EGeometry::ConvexShape
{
public:
	//! the shape of a type with its parameters: radius (Sphere, Capsule, Cylinder), length (Capsule, Cylinder),
	//! size (Box: the edge lengths), points (Triangle: 3 points, Points: n >= 1 points, rows of 3)
	PyConvexShape(const STDstring& shapeType, Real radiusInit, Real lengthInit, const py::object& size, const py::object& pointsInit,
		Real radius1Init)
	{
		const STDstring name = "ConvexShape";
		auto Refuse = [&](const STDstring& message) { PyError(name + "('" + shapeType + "'): " + message, PyErrorType::valueError); };
		if (shapeType == "Sphere") { type = EGeometry::ConvexShapeType::Sphere; }
		else if (shapeType == "Capsule") { type = EGeometry::ConvexShapeType::Capsule; }
		else if (shapeType == "Cylinder") { type = EGeometry::ConvexShapeType::Cylinder; }
		else if (shapeType == "Box") { type = EGeometry::ConvexShapeType::Box; }
		else if (shapeType == "Triangle") { type = EGeometry::ConvexShapeType::Triangle; }
		else if (shapeType == "Points") { type = EGeometry::ConvexShapeType::Points; }
		else if (shapeType == "Cone") { type = EGeometry::ConvexShapeType::Cone; }
		else { PyError(name + ": shapeType must be 'Sphere', 'Capsule', 'Cylinder', 'Box', 'Triangle', 'Points' or 'Cone', not '" + shapeType + "'", PyErrorType::valueError); }
		radius = radiusInit;
		length = lengthInit;
		bool isCone = type == EGeometry::ConvexShapeType::Cone;
		if (isCone)
		{
			radius1 = radius1Init;
			if (!(radius >= 0. && radius1 >= 0. && radius + radius1 > 0.)) { Refuse("radius and radius1 must be >= 0, not both 0"); }
			if (!(length > 0.)) { Refuse("length must be positive"); }
		}
		else if (radius1Init != 0.) { Refuse("has no radius1"); }
		bool usesRadius = type == EGeometry::ConvexShapeType::Sphere || type == EGeometry::ConvexShapeType::Capsule
			|| type == EGeometry::ConvexShapeType::Cylinder;
		bool usesLength = type == EGeometry::ConvexShapeType::Capsule || type == EGeometry::ConvexShapeType::Cylinder;
		bool usesPoints = type == EGeometry::ConvexShapeType::Triangle || type == EGeometry::ConvexShapeType::Points;
		if (usesRadius && !(radius > 0.)) { Refuse("radius must be positive"); }
		if (!usesRadius && !isCone && radius != 0.) { Refuse("has no radius"); }
		if (usesLength && (type == EGeometry::ConvexShapeType::Cylinder ? !(length > 0.) : !(length >= 0.))) { Refuse("length must be positive"); }
		if (!usesLength && !isCone && length != 0.) { Refuse("has no length"); }
		if (type == EGeometry::ConvexShapeType::Box)
		{
			if (size.is_none()) { Refuse("needs size, the three edge lengths"); }
			halfSize = 0.5 * PyCUtilities::GetVector<3>(size, "ConvexShape", "size");
			if (!(halfSize[0] > 0. && halfSize[1] > 0. && halfSize[2] > 0.)) { Refuse("the edge lengths of size must be positive"); }
		}
		else if (!size.is_none()) { Refuse("has no size"); }
		if (usesPoints)
		{
			if (pointsInit.is_none()) { Refuse("needs points, as rows of 3 coordinates"); }
			PyCUtilities::ArrayReal array = PyCUtilities::ToArray(pointsInit, "ConvexShape", "points");
			if (array.ndim() != 2 || array.shape(1) != 3 || array.shape(0) < 1
				|| (type == EGeometry::ConvexShapeType::Triangle && array.shape(0) != 3))
			{
				Refuse(STDstring("points must be ") + (type == EGeometry::ConvexShapeType::Triangle ? "3 rows" : "rows") + " of 3 coordinates; received shape "
					+ PyCUtilities::ShapeString(array));
			}
			for (py::ssize_t i = 0; i < array.shape(0); i++)
			{
				const Real* p = array.data() + 3 * i;
				if (type == EGeometry::ConvexShapeType::Triangle) { triangle[i] = Vector3D({ p[0], p[1], p[2] }); }
				else { points.push_back(Vector3D({ p[0], p[1], p[2] })); }
			}
		}
		else if (!pointsInit.is_none()) { Refuse("has no points"); }
	}

	//! ConvexShape() without arguments is refused: a shape needs its type
	static PyConvexShape* ForbidConstructor()
	{
		PyError("exudyn.cUtilities.ConvexShape() needs shapeType, 'Sphere', 'Capsule', 'Cylinder', 'Box', 'Triangle', 'Points' or 'Cone'", PyErrorType::typeError);
		return nullptr; //never reached
	}

	STDstring ShapeType() const
	{
		const char* names[] = { "Sphere", "Capsule", "Cylinder", "Box", "Triangle", "Points", "Cone" };
		return names[(Index)type];
	}

	STDstring ToString() const
	{
		STDstring s = "ConvexShape('" + ShapeType() + "'";
		if (Margin() != 0. || type == EGeometry::ConvexShapeType::Cylinder) { s += ", radius=" + EXUstd::ToString(radius); }
		if (type == EGeometry::ConvexShapeType::Cone) { s += ", radius=" + EXUstd::ToString(radius) + ", radius1=" + EXUstd::ToString(radius1); }
		if (type == EGeometry::ConvexShapeType::Capsule || type == EGeometry::ConvexShapeType::Cylinder || type == EGeometry::ConvexShapeType::Cone) { s += ", length=" + EXUstd::ToString(length); }
		if (type == EGeometry::ConvexShapeType::Box) { s += ", size=" + EXUstd::ToString(2. * halfSize); }
		if (type == EGeometry::ConvexShapeType::Triangle) { s += ", 3 points"; }
		if (!points.empty()) { s += ", " + EXUstd::ToString((Index)points.size()) + " points"; }
		return s + ")";
	}
};

namespace PyCUtilities {

	//! the signed distance of two convex shapes in the frames HT0 and HT1, the closest points and the normal from
	//! shape 0 to shape 1, as a tuple
	inline py::tuple Distance(const PyConvexShape& shape0, const py::object& HT0py, const PyConvexShape& shape1, const py::object& HT1py,
		bool analytic)
	{
		HomogeneousTransformation HT0, HT1;
		EPyUtils::HTFromPython(HT0py, HT0, "Distance: HT0");
		EPyUtils::HTFromPython(HT1py, HT1, "Distance: HT1");
		EGeometry::ConvexDistanceResult result;
		EGeometry::ConvexDistance(shape0, HT0, shape1, HT1, result, analytic);
		return py::make_tuple(result.distance, ToNumPy(result.point0), ToNumPy(result.point1), ToNumPy(result.normal));
	}
}

#endif
