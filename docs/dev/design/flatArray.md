# FlatArray: a transport class for the marker kinematics and forces of connectors (revision2026c step RG11.14)

A design note of an open step: the analysis and the recommendation of RG11.14.1, for the maintainer to decide. It leaves
the repository when the step closes (WORKFLOW.md §8.4).

## 1. The request (maintainer 2026-11-08)

`CObjectContactSphereSphere::ComputeConnectorForceRigid(const MarkerRigid<Real>* markers, Real t, Vector3D* forces,
Vector3D* torques)` passes raw pointers, which is uncommon in Exudyn and unsafe: the number of markers is not passed and
loops are hardcoded as `for (Index k = 0; k < 2; k++)`. Exudyn's way is a transporting class: it stores pointer and size
and offers only `[]`, `NumberOfItems()`, the iterators and the pointer to the first element (or null). `ResizableArray`
owns its data, so a new `FlatArray`: no resize, no math. `std::vector` or `std::span` would do it, but without the member
functions that templates of Exudyn reuse.

## 2. What exists

All counted on 2026-11-08 at V1.13.469.

| what | where | count |
|---|---|---|
| the virtual functions of the interface with raw pointers | `src/System/CObjectConnector.h`: `ComputeConnectorForce{Position,Rigid,Coordinate}` and their `Diff` versions, `ComputeConnectorForcesCoordinate`, `ComputeConstraintEquations{Position,Rigid,Coordinate}` and their `Diff` versions | 13 |
| the declarations of the items, generated | the `args=` strings of `definitions/itemDefsObjects.py`, `ComputeConnectorForce*` only | 33 (15 Rigid, 4 Position, ...), plus those of the constraints |
| the implementations | `src/ImplObjects/CObject{Connector,Contact,Joint}*.cpp` | 18 files with the force functions, 17 with `MarkerRigid<Real>*` |
| the callers | `src/System/CObjectConnector.cpp`: local arrays `MarkerRigid<Real> kinematics[2]`, `perturbed[2]`, `forces[2]`, ... | about 15 places |
| a hardcoded 2 in a loop | in the files above, for example `CObjectContactSphereSphere.cpp:211` | 29 lines, not all over these arrays |

- The problem applies to all connectors on this interface, not only to the contacts: it came with the connector forces of
  #2745.
- In the class family: `LinkedDataVector` is the same idea for `Real`, with math; `ResizableArray` and `SlimArray` own
  their data; `ArrayIndex` is a `ResizableArray<Index>`. Nothing transports `T` without math or ownership.
- `SlimArray::operator[]` and the others check by `CHECKandTHROW`, which the fast module (`EXUDYN_MODULE=fast`) compiles
  out. The new class gets the same checks and the same cost in the fast module: none.

## 3. Options

| | how | for | against |
|---|---|---|---|
| A | `SlimArray<MarkerRigid<Real>, 2>&` | no new class, the size is in the type | copies when a caller has other storage; fixes 2 markers in the interface, which a connector on more markers cannot use; forces and torques are output arrays of the same fixed size |
| B | `std::span` (C++20) | standard | no `NumberOfItems()`; the C++ standard of the build is to be checked on all platforms; no range check in release |
| C | **`FlatArray<T>`** | the maintainer's idea, the names of the other classes, reuse in templates, range check as everywhere | a new small class |

**Recommended: C.**

## 4. The class

`src/Utilities/FlatArray.h`, about 40 lines, no dependencies beyond `ReleaseAssert.h`:

```cpp
template <class T>
class FlatArray
{
	T* data; Index numberOfItems;
public:
	FlatArray() : data(nullptr), numberOfItems(0) {}
	FlatArray(T* dataInit, Index numberOfItemsInit);
	template <Index n> FlatArray(T (&array)[n]);                  //local arrays: kinematics[2]
	template <class U, Index n> FlatArray(SlimArray<U, n>& array);
	template <class U> FlatArray(ResizableArray<U>& array);

	Index NumberOfItems() const;
	T& operator[](Index item) const;                              //CHECKandTHROW of the range
	T* GetDataPointer() const;                                    //nullptr if empty
	T* begin() const; T* end() const;
};
```

- **Passed by value** (a pointer and an Index), so that it is as cheap as the raw pointer and there is one way to write it.
- **Const correctness in the type**: `FlatArray<const MarkerRigid<Real>>` for what is read, `FlatArray<Vector3D>` for
  what is written. `operator[]` is `const` and returns `T&`, as the class does not own the elements; the constness of the
  elements is that of `T`.
- `operator[]` returns a reference in both cases, unlike `SlimArray`, whose const access copies: a `MarkerRigid` is too
  large to copy.
- Not provided: resize, append, math, comparison, output.

The interface then reads

```cpp
virtual void ComputeConnectorForceRigid(FlatArray<const MarkerRigid<Real>> markers, Real t,
	FlatArray<Vector3D> forces, FlatArray<Vector3D> torques) const;
```

and the loop `for (Index k = 0; k < 2; k++) { forces[k].SetAll(0.); torques[k].SetAll(0.); }` becomes
`for (Vector3D& force : forces) { force.SetAll(0.); }` or a loop to `forces.NumberOfItems()`. Where the formula is for
exactly two markers, as `markers[1].position - markers[0].position`, the index stays; the item checks
`markers.NumberOfItems() == 2` nowhere, as the range check of `[]` catches it.

## 5. Steps proposed

1. RG11.14.2: `FlatArray.h` with a unit test in `src/Tests` (the C++ unit tests), a template function that takes both
   `FlatArray` and `SlimArray` through `NumberOfItems()`.
2. RG11.14.3: the interface of `CObjectConnector` and all its implementations in one commit, since the signature is
   virtual and generated: `args=` in `definitions/itemDefsObjects.py` (a regular expression, reviewed by the diff),
   `CObjectConnector.h/.cpp`, the items. The hardcoded 2 over these arrays replaced.
3. Measured before and after: the speed of a contact model and of a model with many joints, in the module with range
   checks and in the fast module; the fast module must not change. The test suite gives identical results.

Other raw pointers of the same kind are not part of this step: `CSystem.h` and the marker data have some, each to be
looked at separately once the class exists.
