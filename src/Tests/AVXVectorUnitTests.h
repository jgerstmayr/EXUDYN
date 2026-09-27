/** ***********************************************************************************************
* @file			AVXVectorUnitTests.h
* @brief		Unit tests for the two vectorized vector classes, ResizableVectorParallel and
*				LinkedDataVectorParallel.
* @details		Details:
*				- every operation of these classes runs an AVX loop over floor(n/AVXRealSize)
*				  packets and a scalar loop over the remainder, and picks a multithreaded variant
*				  above ResizableVectorParallelThreadingLimit. A defect in any one of those three
*				  branches is invisible for most lengths, which is why each case here runs a whole
*				  range of lengths AROUND the packet boundary rather than one convenient size.
*				- LinkedDataVectorParallel is additionally tested on OFFSET sub-ranges: issue #2394
*				  was a misaligned __m256d load on a sub-range of a LinkedDataVector, found by a
*				  sanitizer on a whole test model instead of by a unit test.
*				- every result is compared against the same operation computed in a plain scalar
*				  loop, so the test states what the operation means, not what it currently returns.
*				- the file compiles with and without USE_RESIZABLE_VECTOR_PARALLEL: without it the
*				  two classes are aliases of their scalar base classes and these cases then check
*				  the base classes, which is correct and still worth running.
*
* @author		Gerstmayr Johannes
* @date			2026-09-16 (#2465)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef AVXVECTORUNITTESTS__H
#define AVXVECTORUNITTESTS__H

//BasicLinalg.h does NOT include these two - they are pulled in by the solver, not by the general
//linear algebra header, which is part of why they had no unit tests at all until now
#include "Linalg/ResizableVectorParallel.h"
#include "Linalg/LinkedDataVectorParallel.h"

//the number of Reals in one AVX packet, which is what every boundary below is built from: 4 for
//AVX2 with doubles, 8 for AVX-512 or for floats. Without AVX the classes are their scalar base
//classes and there is no packet; 4 then just gives a reasonable set of lengths to test.
#ifdef AVXRealSize
	#define AVXTestPacketSize AVXRealSize
#else
	#define AVXTestPacketSize 4
#endif

//the lengths every case runs through: empty, shorter than one AVX packet, exactly one packet,
//one item more, and two further sizes that leave a remainder. AVXTestPacketSize is 4 for AVX2 and 8 for
//AVX-512, so the list is built from it rather than written out - a test that only ever ran n=10
//would pass on AVX2 and miss the boundary on AVX-512.
inline ResizableArray<Index> AVXTestLengths()
{
	ResizableArray<Index> lengths;
	for (Index n : { 0, 1, 2, 3, 5, 7, 9, 13, 17, 33 })
	{
		lengths.Append(n);
	}
	for (Index k : { 1, 2, 3 }) //around the packet boundary, whatever AVXTestPacketSize is
	{
		lengths.Append(k * AVXTestPacketSize - 1);
		lengths.Append(k * AVXTestPacketSize);
		lengths.Append(k * AVXTestPacketSize + 1);
	}
	return lengths;
}

//a reproducible value; not 1,2,3,... because a wrong index in an AVX packet stays visible here
inline Real AVXTestValue(Index i) { return 1. + 0.25 * (Real)i + 0.125 * (Real)((i * 7) % 11); }

//! the two vectors agree item by item; ToString would only say that for equal lengths
inline bool AVXVectorsEqual(const Vector& computed, const Vector& reference)
{
	if (computed.NumberOfItems() != reference.NumberOfItems()) { return false; }
	for (Index i = 0; i < reference.NumberOfItems(); i++)
	{
		if (computed[i] != reference[i]) { return false; }
	}
	return true;
}

const lest::test avxVector_specific_test[] =
{

	CASE("ResizableVectorParallel: CopyFrom and operator= over the packet boundary")
	{
		for (Index n : AVXTestLengths())
		{
			Vector source(n);
			for (Index i = 0; i < n; i++) { source[i] = AVXTestValue(i); }

			ResizableVectorParallel v;
			v.SetNumberOfItems(n);
			v.CopyFrom(source);

			Vector result(n);
			for (Index i = 0; i < n; i++) { result[i] = v[i]; }
			EXPECT(AVXVectorsEqual(result, source));

			//operator= runs the same loop; a length that ends exactly on a packet boundary must
			//not leave the scalar remainder loop writing past the data
			ResizableVectorParallel w;
			w = source;
			EXPECT(w.NumberOfItems() == n);
			for (Index i = 0; i < n; i++) { EXPECT(w[i] == source[i]); }
		}
	},

	CASE("ResizableVectorParallel: operator+= and operator-= over the packet boundary")
	{
		for (Index n : AVXTestLengths())
		{
			Vector a(n), b(n), reference(n);
			for (Index i = 0; i < n; i++)
			{
				a[i] = AVXTestValue(i);
				b[i] = -0.5 * AVXTestValue(n - i);
				reference[i] = a[i] + b[i];
			}

			ResizableVectorParallel v;
			v = a;
			v += b;
			Vector result(n);
			for (Index i = 0; i < n; i++) { result[i] = v[i]; }
			EXPECT(AVXVectorsEqual(result, reference));

			for (Index i = 0; i < n; i++) { reference[i] = a[i] - b[i]; }
			ResizableVectorParallel w;
			w = a;
			w -= b;
			for (Index i = 0; i < n; i++) { result[i] = w[i]; }
			EXPECT(AVXVectorsEqual(result, reference));
		}
	},

	CASE("ResizableVectorParallel: scalar multiply and divide over the packet boundary")
	{
		for (Index n : AVXTestLengths())
		{
			const Real scalar = 2.5;
			Vector a(n), reference(n);
			for (Index i = 0; i < n; i++)
			{
				a[i] = AVXTestValue(i);
				reference[i] = a[i] * scalar;
			}

			ResizableVectorParallel v;
			v = a;
			v *= scalar;
			Vector result(n);
			for (Index i = 0; i < n; i++) { result[i] = v[i]; }
			EXPECT(AVXVectorsEqual(result, reference));

			//dividing by the same scalar must return the original values EXACTLY: 2.5 and these
			//values are representable, so this also says that the AVX and the scalar branch use
			//the same operation and not, say, a reciprocal multiplication in one of them
			v /= scalar;
			for (Index i = 0; i < n; i++) { result[i] = v[i]; }
			EXPECT(AVXVectorsEqual(result, a));
		}
	},

	CASE("ResizableVectorParallel: MultAdd over the packet boundary")
	{
		for (Index n : AVXTestLengths())
		{
			const Real scalar = -1.75;
			Vector a(n), b(n), reference(n);
			for (Index i = 0; i < n; i++)
			{
				a[i] = AVXTestValue(i);
				b[i] = 0.75 * AVXTestValue(2 * i + 1);
				reference[i] = a[i] + scalar * b[i];
			}

			ResizableVectorParallel v;
			v = a;
			v.MultAdd(scalar, b);

			//NOTE: the AVX branch uses a fused multiply-add and the remainder loop does not, so
			//the two branches can differ in the last bit. The values above are chosen so that
			//a + scalar*b is exact in both, which is what makes an exact comparison legitimate
			//here on FMA contraction.
			Vector result(n);
			for (Index i = 0; i < n; i++) { result[i] = v[i]; }
			EXPECT(AVXVectorsEqual(result, reference));
		}
	},

	CASE("LinkedDataVectorParallel: OFFSET sub-ranges - the #2394 case")
	{
		//a sub-range starting at 1, 2 or 3 doubles is NOT 32-byte aligned, which is what #2394 was
		//about: an aligned AVX load on such a range reads the wrong memory or faults. Every offset
		//from 0 to AVXTestPacketSize is exercised, at every length around the packet boundary.
		for (Index offset = 0; offset <= AVXTestPacketSize; offset++)
		{
			for (Index n : AVXTestLengths())
			{
				Vector buffer(n + offset + AVXTestPacketSize); //padding behind, so an overrun is visible
				for (Index i = 0; i < buffer.NumberOfItems(); i++) { buffer[i] = AVXTestValue(i); }
				Vector untouched = buffer;

				Vector other(n);
				for (Index i = 0; i < n; i++) { other[i] = 0.5 * AVXTestValue(i + 3); }

				LinkedDataVectorParallel linked;
				linked.LinkDataTo(buffer, offset, n);
				EXPECT(linked.NumberOfItems() == n);

				linked += other;

				for (Index i = 0; i < n; i++) //the sub-range holds the sum
				{
					EXPECT(linked[i] == untouched[i + offset] + other[i]);
					EXPECT(buffer[i + offset] == untouched[i + offset] + other[i]);
				}
				for (Index i = 0; i < offset; i++) //nothing before the sub-range was touched
				{
					EXPECT(buffer[i] == untouched[i]);
				}
				for (Index i = n + offset; i < buffer.NumberOfItems(); i++) //and nothing behind it
				{
					EXPECT(buffer[i] == untouched[i]);
				}
			}
		}
	},

	CASE("LinkedDataVectorParallel: CopyFrom, -=, scalar and MultAdd on an offset sub-range")
	{
		for (Index offset = 0; offset <= AVXTestPacketSize; offset++)
		{
			for (Index n : AVXTestLengths())
			{
				const Real scalar = 0.5;
				Vector buffer(n + offset + AVXTestPacketSize);
				for (Index i = 0; i < buffer.NumberOfItems(); i++) { buffer[i] = AVXTestValue(i); }
				Vector untouched = buffer;

				Vector source(n);
				for (Index i = 0; i < n; i++) { source[i] = AVXTestValue(3 * i + 2); }

				LinkedDataVectorParallel linked;
				linked.LinkDataTo(buffer, offset, n);

				linked.CopyFrom(source);
				for (Index i = 0; i < n; i++) { EXPECT(buffer[i + offset] == source[i]); }

				linked -= source; //back to zero, exactly
				for (Index i = 0; i < n; i++) { EXPECT(buffer[i + offset] == 0.); }

				linked.CopyFrom(source);
				linked *= scalar;
				for (Index i = 0; i < n; i++) { EXPECT(buffer[i + offset] == source[i] * scalar); }

				linked.CopyFrom(source);
				linked.MultAdd(scalar, source); //(1+scalar)*source, exact for scalar=0.5
				for (Index i = 0; i < n; i++)
				{
					EXPECT(buffer[i + offset] == source[i] + scalar * source[i]);
				}

				//the padding behind the sub-range survived all of it
				for (Index i = n + offset; i < buffer.NumberOfItems(); i++)
				{
					EXPECT(buffer[i] == untouched[i]);
				}
			}
		}
	},

	CASE("AVX vectors: above the multithreading limit")
	{
		//every operation switches to a multithreaded variant above
		//ResizableVectorParallelThreadingLimit; below that limit the loop above never reaches it.
		//The length is the limit plus a remainder, so the packet loop, the remainder loop and the
		//threading decision are all exercised at once.
		//NOTE: the multithreaded BRANCH is only taken when the task manager actually has more than
		//one thread; with one thread this runs the serial branch and still checks the boundary.
		const Index n = ResizableVectorParallelThreadingLimit + AVXTestPacketSize + 1;
		const Real scalar = 0.25;

		Vector a(n), b(n);
		for (Index i = 0; i < n; i++)
		{
			a[i] = AVXTestValue(i % 97);
			b[i] = 0.5 * AVXTestValue(i % 89);
		}

		ResizableVectorParallel v;
		v = a;
		v += b;
		v -= b;
		v *= scalar;
		v /= scalar;
		v.MultAdd(scalar, b);

		bool allEqual = true;
		for (Index i = 0; i < n; i++)
		{
			if (v[i] != a[i] + scalar * b[i]) { allEqual = false; break; }
		}
		EXPECT(v.NumberOfItems() == n);
		EXPECT(allEqual);
	},

};

#endif //include header once
