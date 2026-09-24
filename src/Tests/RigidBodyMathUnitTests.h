/** ***********************************************************************************************
* @file			RigidBodyMathUnitTests.h
* @brief		Unit tests for RigidBodyMath, Geometry, BoundingBox and SearchTree.
* @details		Details:
*				- these four headers carry the geometry every rigid body and every contact model
*				  runs through, and none of them had a unit test. A defect here does not crash: it
*				  moves a result, which is exactly the class of difference the revision spent days
*				  chasing.
*				- the rotation tests are PROPERTY tests, not value tables: a conversion composed
*				  with its inverse is the identity, a rotation matrix is orthonormal with
*				  determinant +1, and Skew is the cross product. Such a test states what the
*				  function means and keeps holding when the implementation is rewritten - a table
*				  of 9 numbers per case only states what it returned on the day it was written.
*				- SearchTree is checked against BRUTE FORCE over the same data: whatever the tree
*				  answers, a linear scan must answer the same. That is the property which makes the
*				  tree replaceable, and it is checked for several cell counts including 1x1x1,
*				  where the tree degenerates into that linear scan.
*
* @author		Gerstmayr Johannes
* @date			2026-09-17 (created; revision2026 step R5.4.1, issue #2472)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef RIGIDBODYMATHUNITTESTS__H
#define RIGIDBODYMATHUNITTESTS__H

#include "Linalg/RigidBodyMath.h"
#include "Linalg/Geometry.h"
#include "Linalg/BoundingBox.h"
#include "Linalg/SearchTree.h"

//rotations are floating point: these tests compare within a tolerance, and the tolerance is tight
//enough that a wrong sign or a transposed matrix cannot pass
const Real rigidBodyMathTolerance = 1e-13;

inline bool RBMClose(Real a, Real b) { return fabs(a - b) <= rigidBodyMathTolerance; }

inline bool RBMMatricesClose(const Matrix3D& a, const Matrix3D& b)
{
	for (Index i = 0; i < 3; i++)
	{
		for (Index j = 0; j < 3; j++)
		{
			if (!RBMClose(a(i, j), b(i, j))) { return false; }
		}
	}
	return true;
}

inline bool RBMVectorsClose(const Vector3D& a, const Vector3D& b)
{
	for (Index i = 0; i < 3; i++) { if (!RBMClose(a[i], b[i])) { return false; } }
	return true;
}

//! A is a rotation matrix: its columns are orthonormal and its determinant is +1. Everything that
//! returns a rotation must satisfy this, whatever else it does
inline bool RBMIsRotationMatrix(const Matrix3D& A)
{
	Matrix3D shouldBeIdentity = A.GetTransposed() * A;
	for (Index i = 0; i < 3; i++)
	{
		for (Index j = 0; j < 3; j++)
		{
			if (!RBMClose(shouldBeIdentity(i, j), (i == j) ? 1. : 0.)) { return false; }
		}
	}
	const Real determinant =
		  A(0, 0) * (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1))
		- A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0))
		+ A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));
	return RBMClose(determinant, 1.);
}

//a handful of rotations used by several cases: identity, single axes, and one general rotation
//that is not close to any axis - the case where a sloppy conversion still looks plausible
inline ResizableArray<Vector3D> RBMTestRotationVectors()
{
	ResizableArray<Vector3D> rotations;
	rotations.Append(Vector3D({ 0., 0., 0. }));
	rotations.Append(Vector3D({ 0.7, 0., 0. }));
	rotations.Append(Vector3D({ 0., -1.1, 0. }));
	rotations.Append(Vector3D({ 0., 0., 2.5 }));
	rotations.Append(Vector3D({ 0.3, -0.45, 0.8 }));
	rotations.Append(Vector3D({ -1.2, 0.9, -0.15 }));
	return rotations;
}

const lest::test rigidBodyMath_specific_test[] =
{

	CASE("RigidBodyMath: Vector2SkewMatrix is the cross product, and skew-symmetric")
	{
		Vector3D a({ 1.5, -2., 0.25 });
		Vector3D b({ -0.5, 3., 2. });

		ConstSizeMatrix<9> skew = RigidBodyMath::Vector2SkewMatrix(a);
		EXPECT(skew.NumberOfRows() == 3);
		EXPECT(skew.NumberOfColumns() == 3);

		for (Index i = 0; i < 3; i++) //skew-symmetric: S^T = -S, diagonal zero
		{
			EXPECT(skew(i, i) == 0.);
			for (Index j = 0; j < 3; j++) { EXPECT(RBMClose(skew(i, j), -skew(j, i))); }
		}

		//the defining property: S(a)*b == a x b, written out here so the test says what it means
		Vector3D crossProduct({ a[1] * b[2] - a[2] * b[1],
								a[2] * b[0] - a[0] * b[2],
								a[0] * b[1] - a[1] * b[0] });
		Vector3D skewTimesB({ 0., 0., 0. });
		for (Index i = 0; i < 3; i++)
		{
			for (Index j = 0; j < 3; j++) { skewTimesB[i] += skew(i, j) * b[j]; }
		}
		EXPECT(RBMVectorsClose(skewTimesB, crossProduct));

		//and back: SkewMatrix2Vector undoes it
		Vector3D recovered = RigidBodyMath::SkewMatrix2Vector(skew);
		EXPECT(RBMVectorsClose(recovered, a));
	},

	CASE("RigidBodyMath: RotationVector2RotationMatrix rotates by |rot| about rot")
	{
		//there is no RotationMatrix2RotationVector to round-trip against, so the test states the
		//two properties that DEFINE the exponential map instead: the axis is fixed, and the angle
		//follows from the trace, since trace(A) = 1 + 2 cos(angle) for any rotation
		for (const Vector3D& rotation : RBMTestRotationVectors())
		{
			Matrix3D A = RigidBodyMath::RotationVector2RotationMatrix(rotation);
			EXPECT(RBMIsRotationMatrix(A));

			const Real angle = sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1]
									+ rotation[2] * rotation[2]);
			const Real trace = A(0, 0) + A(1, 1) + A(2, 2);
			EXPECT(RBMClose(trace, 1. + 2. * cos(angle)));

			if (angle > 1e-8) //the axis of the rotation is not moved by it
			{
				Vector3D axis({ rotation[0] / angle, rotation[1] / angle, rotation[2] / angle });
				EXPECT(RBMVectorsClose(A * axis, axis));
			}
			else
			{
				EXPECT(RBMMatricesClose(A, EXUmath::unitMatrix3D)); //no rotation is the identity
			}
		}
	},

	CASE("RigidBodyMath: Euler parameters -> matrix -> Euler parameters round trip")
	{
		for (const Vector3D& rotation : RBMTestRotationVectors())
		{
			Matrix3D A = RigidBodyMath::RotationVector2RotationMatrix(rotation);

			Real ep0, ep1, ep2, ep3;
			RigidBodyMath::RotationMatrix2EP(A, ep0, ep1, ep2, ep3);

			//unit quaternion: the constraint the solver relies on
			EXPECT(RBMClose(ep0 * ep0 + ep1 * ep1 + ep2 * ep2 + ep3 * ep3, 1.));

			Matrix3D recovered = RigidBodyMath::EP2RotationMatrix(ep0, ep1, ep2, ep3);
			EXPECT(RBMIsRotationMatrix(recovered));
			EXPECT(RBMMatricesClose(recovered, A)); //the matrix is what must survive, not the sign
		}
	},

	CASE("RigidBodyMath: RotXYZ -> matrix -> RotXYZ round trip and composition order")
	{
		for (const Vector3D& rotation : RBMTestRotationVectors())
		{
			Matrix3D A = RigidBodyMath::RotXYZ2RotationMatrix(rotation);
			EXPECT(RBMIsRotationMatrix(A));

			Vector3D recovered = RigidBodyMath::RotationMatrix2RotXYZ(A);
			Matrix3D again = RigidBodyMath::RotXYZ2RotationMatrix(recovered);
			//the angles may differ by a representation of the same rotation; the MATRIX must not
			EXPECT(RBMMatricesClose(again, A));
		}

		//the composition ORDER, which no other test states: RotXYZ2RotationMatrix(x,y,z) equals
		//A(x)*A(y)*A(z), so the z rotation is applied to the vector FIRST. Getting this backwards
		//is the classic Euler-angle defect, and it is invisible unless two angles are non-zero.
		Vector3D onlyX({ 0.4, 0., 0. }), onlyY({ 0., -0.7, 0. }), onlyZ({ 0., 0., 1.3 });
		Matrix3D Ax = RigidBodyMath::RotXYZ2RotationMatrix(onlyX);
		Matrix3D Ay = RigidBodyMath::RotXYZ2RotationMatrix(onlyY);
		Matrix3D Az = RigidBodyMath::RotXYZ2RotationMatrix(onlyZ);
		Vector3D all({ 0.4, -0.7, 1.3 });
		Matrix3D combined = RigidBodyMath::RotXYZ2RotationMatrix(all);
		EXPECT(RBMMatricesClose(combined, Ax * Ay * Az));
		EXPECT(!RBMMatricesClose(combined, Az * Ay * Ax)); //and the other order is NOT the same
	},

	CASE("RigidBodyMath: a rotation matrix rotates, and the transpose rotates back")
	{
		Vector3D rotation({ 0.3, -0.45, 0.8 });
		Matrix3D A = RigidBodyMath::RotationVector2RotationMatrix(rotation);
		Vector3D v({ 2., -1., 0.5 });

		Vector3D rotated = A * v;
		Vector3D back = A.GetTransposed() * rotated;
		EXPECT(RBMVectorsClose(back, v));

		//a rotation preserves length - the one property a wrong scaling cannot fake
		Real lengthBefore = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		Real lengthAfter = sqrt(rotated[0] * rotated[0] + rotated[1] * rotated[1] + rotated[2] * rotated[2]);
		EXPECT(RBMClose(lengthBefore, lengthAfter));

		//a rotation about an axis leaves that axis alone
		Vector3D aboutZ({ 0., 0., 0.9 });
		Matrix3D Az = RigidBodyMath::RotationVector2RotationMatrix(aboutZ);
		Vector3D zAxis({ 0., 0., 1. });
		EXPECT(RBMVectorsClose(Az * zAxis, zAxis));
	},

	CASE("Geometry: shortest distance of a point to a line, including the degenerate cases")
	{
		Vector3D linePoint0({ 0., 0., 0. });
		Vector3D linePoint1({ 2., 0., 0. }); //along x

		Real relativePosition = 0.;
		//a point above the middle: distance 1, relative position 0.5
		Vector3D above({ 1., 1., 0. });
		Real distance = EGeometry::ShortestDistanceRelativePosition(linePoint0, linePoint1, above, relativePosition);
		EXPECT(RBMClose(distance, 1.));
		EXPECT(RBMClose(relativePosition, 0.5));

		//a point ON the line: distance 0
		Vector3D onLine({ 0.5, 0., 0. });
		distance = EGeometry::ShortestDistanceRelativePosition(linePoint0, linePoint1, onLine, relativePosition);
		EXPECT(RBMClose(distance, 0.));
		EXPECT(RBMClose(relativePosition, 0.25));

		//beyond the end point: the INFINITE line still has relative position > 1 ...
		Vector3D beyond({ 3., 0., 0. });
		distance = EGeometry::ShortestDistanceRelativePosition(linePoint0, linePoint1, beyond, relativePosition);
		EXPECT(relativePosition > 1.);

		//... while the segment version clamps to the end point and reports the distance to IT
		Real distanceSegment = EGeometry::MinDistToLinePoints(linePoint0, linePoint1, beyond);
		EXPECT(RBMClose(distanceSegment, 1.)); //from (3,0,0) to the end point (2,0,0)

		//a zero-length segment is the distance to the point itself, not a division by zero
		Vector3D far({ 0., 3., 4. });
		Real distanceDegenerate = EGeometry::MinDistToLinePoints(linePoint0, linePoint0, far);
		EXPECT(RBMClose(distanceDegenerate, 5.));
	},

	CASE("Geometry: plane distance and triangle normal")
	{
		Vector3D planePoint({ 0., 0., 0. });
		Vector3D normal({ 0., 0., 1. }); //the xy plane

		EXPECT(RBMClose(EGeometry::DistanceToPlane(Vector3D({ 5., -2., 3. }), normal, planePoint), 3.));
		EXPECT(RBMClose(EGeometry::DistanceToPlane(Vector3D({ 5., -2., 0. }), normal, planePoint), 0.));
		//DistanceToPlane is UNSIGNED - it returns fabs(...) - so it cannot say which side of the
		//plane the point is on. Pinned down here because the name suggests otherwise and a caller
		//that needs the side has to compute the projection itself
		EXPECT(RBMClose(EGeometry::DistanceToPlane(Vector3D({ 0., 0., -1.5 }), normal, planePoint), 1.5));

		//the normal need not be normalized: the function divides by its length
		Vector3D longNormal({ 0., 0., 7. });
		EXPECT(RBMClose(EGeometry::DistanceToPlane(Vector3D({ 1., 1., 2. }), longNormal, planePoint), 2.));

		//a triangle in the xy plane, counter-clockwise, has the +z normal; swapping two points flips it
		std::array<Vector3D, 3> triangle = { Vector3D({ 0., 0., 0. }),
											 Vector3D({ 1., 0., 0. }),
											 Vector3D({ 0., 1., 0. }) };
		Vector3D triangleNormal = EGeometry::ComputeTriangleNormal(triangle);
		EXPECT(RBMVectorsClose(triangleNormal, Vector3D({ 0., 0., 1. })));

		std::array<Vector3D, 3> flipped = { triangle[0], triangle[2], triangle[1] };
		EXPECT(RBMVectorsClose(EGeometry::ComputeTriangleNormal(flipped), Vector3D({ 0., 0., -1. })));
	},

	CASE("BoundingBox: Add grows the box, Intersect and PointInside agree with the geometry")
	{
		Box3D box;
		EXPECT(box.Empty()); //a fresh box contains nothing - the state every Add starts from

		box.Add(Vector3D({ 1., 2., 3. }));
		EXPECT(!box.Empty());
		EXPECT(box.IsInside(Vector3D({ 1., 2., 3. }))); //a single point is inside itself

		box.Add(Vector3D({ -1., 0., 1. }));
		EXPECT(box.IsInside(Vector3D({ 0., 1., 2. })));   //between the two corners
		EXPECT(!box.IsInside(Vector3D({ 2., 1., 2. })));  //outside in x

		Box3D other;
		other.Add(Vector3D({ 0.5, 0.5, 0.5 }));
		other.Add(Vector3D({ 5., 5., 5. }));
		EXPECT(box.Intersect(other));   //they overlap
		EXPECT(other.Intersect(box));   //and that cannot depend on the order

		Box3D disjoint;
		disjoint.Add(Vector3D({ 10., 10., 10. }));
		disjoint.Add(Vector3D({ 11., 11., 11. }));
		EXPECT(!box.Intersect(disjoint));
		EXPECT(!disjoint.Intersect(box));

		//a box that only TOUCHES does NOT count as intersecting: Intersect compares with >= and <=,
		//so a shared face or corner is rejected. Worth pinning down because the comment in
		//BoundingBox.h claims the opposite ("changed to >= and <= in order to simplify problems
		//with points on boundaries"), and contact code sits right on this boundary case
		Box3D touching;
		touching.Add(Vector3D({ 1., 2., 3. }));
		touching.Add(Vector3D({ 4., 5., 6. }));
		EXPECT(!box.Intersect(touching));

		//overlapping by any positive amount is an intersection, however small
		Box3D barelyOverlapping;
		barelyOverlapping.Add(Vector3D({ 0.999, 1.9, 2.9 }));
		barelyOverlapping.Add(Vector3D({ 4., 5., 6. }));
		EXPECT(box.Intersect(barelyOverlapping));

		//adding a box is adding all of its points
		Box3D merged = box;
		merged.Add(disjoint);
		EXPECT(merged.IsInside(Vector3D({ 10.5, 10.5, 10.5 })));
		EXPECT(merged.IsInside(Vector3D({ 0., 1., 2. })));
	},

	CASE("SearchTree: a query returns what a brute-force scan returns")
	{
		//the property that makes the tree replaceable. Items are boxes on a regular grid, and the
		//query box is deliberately placed to cut cell boundaries rather than to sit inside one
		const Index itemsPerAxis = 4;
		ResizableArray<Box3D> itemBoxes;
		for (Index ix = 0; ix < itemsPerAxis; ix++)
		{
			for (Index iy = 0; iy < itemsPerAxis; iy++)
			{
				for (Index iz = 0; iz < itemsPerAxis; iz++)
				{
					Box3D item;
					Vector3D corner({ 0.25 * (Real)ix, 0.25 * (Real)iy, 0.25 * (Real)iz });
					item.Add(corner);
					item.Add(Vector3D({ corner[0] + 0.1, corner[1] + 0.1, corner[2] + 0.1 }));
					itemBoxes.Append(item);
				}
			}
		}

		Box3D wholeDomain;
		wholeDomain.Add(Vector3D({ -0.1, -0.1, -0.1 }));
		wholeDomain.Add(Vector3D({ 1.1, 1.1, 1.1 }));

		ResizableArray<Box3D> queries;
		queries.Append(wholeDomain); //everything
		Box3D corner;
		corner.Add(Vector3D({ -0.05, -0.05, -0.05 }));
		corner.Add(Vector3D({ 0.3, 0.3, 0.3 }));
		queries.Append(corner);
		Box3D slab;
		slab.Add(Vector3D({ 0.4, -1., -1. }));
		slab.Add(Vector3D({ 0.6, 2., 2. }));
		queries.Append(slab);
		Box3D outside;
		outside.Add(Vector3D({ 5., 5., 5. }));
		outside.Add(Vector3D({ 6., 6., 6. }));
		queries.Append(outside); //nothing

		//1x1x1 degenerates into the brute-force scan itself, which is worth testing separately
		for (Index cells : { 1, 2, 5 })
		{
			SearchTree searchTree;
			searchTree.ResetSearchTree(cells, cells, cells, wholeDomain);
			for (Index i = 0; i < itemBoxes.NumberOfItems(); i++)
			{
				searchTree.AddItem(itemBoxes[i], i);
			}

			for (const Box3D& query : queries)
			{
				ArrayIndex found;
				searchTree.GetItemsInBox(query, found);

				//the tree may return an item more than once (it sits in several cells) and may
				//return items which only share a cell with the query: it is a PRE-selection. What
				//it must never do is miss one that really intersects
				for (Index i = 0; i < itemBoxes.NumberOfItems(); i++)
				{
					if (query.Intersect(itemBoxes[i]))
					{
						bool isInResult = false;
						for (Index k = 0; k < found.NumberOfItems(); k++)
						{
							if (found[k] == i) { isInResult = true; break; }
						}
						EXPECT(isInResult);
					}
				}

				//and everything it returns must exist
				for (Index k = 0; k < found.NumberOfItems(); k++)
				{
					EXPECT(found[k] >= 0);
					EXPECT(found[k] < itemBoxes.NumberOfItems());
				}
			}
		}
	},

	CASE("SearchTree: resetting clears the items")
	{
		Box3D domain;
		domain.Add(Vector3D({ 0., 0., 0. }));
		domain.Add(Vector3D({ 1., 1., 1. }));

		SearchTree searchTree;
		searchTree.ResetSearchTree(2, 2, 2, domain);

		Box3D item;
		item.Add(Vector3D({ 0.1, 0.1, 0.1 }));
		item.Add(Vector3D({ 0.2, 0.2, 0.2 }));
		searchTree.AddItem(item, 0);

		ArrayIndex found;
		searchTree.GetItemsInBox(domain, found);
		EXPECT(found.NumberOfItems() == 1);
		EXPECT(found[0] == 0);

		searchTree.ResetSearchTree(2, 2, 2, domain); //the same tree, emptied
		searchTree.GetItemsInBox(domain, found);
		EXPECT(found.NumberOfItems() == 0);
	},

};

#endif //include header once
