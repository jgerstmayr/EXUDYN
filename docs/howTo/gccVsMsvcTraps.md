# What MSVC accepts and GCC does not

Exudyn is developed on Windows with MSVC and built on Linux with GCC, and the Linux build is where
the sloppiness shows. These are the traps that actually cost time, kept because each of them was
found the hard way; the code has been fixed in every case, so this page is about **not writing them
again**.

## `#pragma once` is MSVC

GCC accepts it, but the portable form is the include guard, and Exudyn uses both:

```cpp
#ifdef _MSC_VER
#pragma once
#endif

#ifndef RELEASEASSERT__H
#define RELEASEASSERT__H
...
#endif
```

## `std::exception` cannot be constructed with a message

```cpp
throw std::exception("unexpected EXUDYN internal error");   //MSVC only
```

GCC's `std::exception` has no such constructor. Exudyn therefore throws `std::runtime_error`, which
is what `EXUexception` is:

```cpp
#define EXUexception std::runtime_error
```

(Since revision2026 step R6.3 there are typed exception classes; see `docs/dev/CODING_STYLE.md`
§10 for which one a new check should raise.)

## Reserved names

`MINFLOAT` and `MAXFLOAT` are reserved in GCC; Exudyn's own constants are `_MINFLOAT` and
`_MAXFLOAT`.

`NDEBUG` is already defined by GCC, so define it only if it is not there:

```cpp
#ifndef NDEBUG
    #define NDEBUG      //avoids range checks, e.g. in Eigen
#endif
```

## Headers MSVC pulls in for you

MSVC's headers include each other generously; GCC's do not. Every standard template used must be
included by name:

```cpp
//GCC does not get these from <stdlib.h>:
#include <vector>
#include <array>
#include <exception>
```

The same holds inside the project: `Vector.h` needs
`#include "Utilities/BasicFunctions.h"` for `EXUstd::Minimum`, even though MSVC finds it anyway.

## Templates need `<T>` where MSVC guesses

```
Linalg/LinkedDataMatrix.h:45:27: error: class 'LinkedDataMatrixBase<T>' does not have any field named 'MatrixBase'
```

The base class in the constructor initializer list must be written with its template argument:
`MatrixBase<T>`.

## The Microsoft "safe" functions do not exist

`strcpy_s` and friends are MSVC extensions — use `strcpy` (and be careful, which is what `_s` was
for).

## Virtual destructors

```
warning: deleting object of polymorphic class type 'CLoad' which has non-virtual destructor
         might cause undefined behavior [-Wdelete-non-virtual-dtor]
```

GCC is right and MSVC is silent: every base class with virtual functions needs a virtual
destructor. `CLoad`, `CMarker`, `MainLoad`, `VisualizationLoad` and `ResizableArray` all got one.

The other direction is worth doing at the same time: a class with **no** derived class does not
need `virtual` at all — it was removed from `MarkerDataStructure` and from the destructor of
`SlimVectorBase`.

## Initialisation order in constructors

```
warning: 'VectorBase<double>::numberOfItems' will be initialized after [-Wreorder]
```

Members are initialised in the order they are **declared**, not in the order of the initializer
list. Write the list in declaration order:

```cpp
VectorBase(): numberOfItems(0), data(nullptr) {};     //warns
VectorBase(): data(nullptr), numberOfItems(0) {};     //correct
```

## A `static const` used as a template argument

```
ImportError: .../exudyn.cpython-36m-x86_64-linux-gnu.so: undefined symbol:
    _ZN27CObjectContactCircleCable2D19maxNumberOfSegmentsE
```

An import error rather than a compile error, which is what makes this one memorable. The cause is a
`static const Index` used as a template argument — `ConstSizeVector<maxNumberOfSegments>` — which
needs the value at compile time and a definition at link time. The fix is one word:

```cpp
static constexpr Index maxNumberOfSegments = 12;
```

## See also

- [buildFromSource.md](buildFromSource.md) — building on Linux
- [buildQuirks.md](buildQuirks.md) — the Windows side
- `docs/dev/CODING_STYLE.md` — the conventions these traps live under
