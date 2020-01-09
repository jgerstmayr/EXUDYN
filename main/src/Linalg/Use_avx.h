#pragma once
#include <immintrin.h> // AVX/AVX2 intrinsic header


// boundary for memory alignment
#define alignmentBytes 32
#include "Utilities/BasicDefinitions.h"


#define use_AVX2		
//#define use_AVX512

#ifdef use_AVX512
#include <zmmintrin.h> // AVX512 header for VS2017 compiler
#endif // use_AVX512


#ifdef use_AVX2
#ifdef DoublePrecision
#define AVXRealSize 4 // number of doubles in a AVXvector
#define PReal __m256d // packed double, 256-bit vector containing 4 doubles
#define _mm_add_ _mm256_add_pd 
#define _mm_sub_ _mm256_sub_pd
#define _mm_mul_ _mm256_mul_pd 
#define _mm_div_ _mm256_div_pd  
#define	_mm_setr_ _mm256_setr_pd
#define _mm_setr_zero_ _mm256_setzero_pd
#define _mm_store_ _mm256_store_pd
#define _mm_fmadd_ _mm256_fmadd_pd
#define _mm_load_ _mm256_load_pd
#define _mm_load_u _mm256_loadu_pd
#define _mm_set1_ _mm256_set1_pd
#define _mm_fmadd_ _mm256_fmadd_pd
#define _mm_xor_ _mm256_xor_pd
#endif

#ifdef SinglePrecision
#define AVXRealSize 8 // number of floats in a AVXvector
#define PReal __m256  // packed float, 256-bit vector containing 8 floats
#define _mm_add_ _mm256_add_ps
#define _mm_sub_ _mm256_sub_ps
#define _mm_mul_ _mm256_mul_ps
#define _mm_div_ _mm256_div_ps
#define	_mm_setr_ _mm256_setr_ps
#define _mm_setr_zero_ _mm256_setzero_ps
#define _mm_store_ _mm256_store_ps
#define _mm_load_ _mm256_load_ps
#define _mm_load_u _mm256_loadu_ps
#define _mm_fmadd_ _mm256_fmadd_ps
#define _mm_set1_ _mm256_set1_ps
#define _mm_fmadd_ _mm256_fmadd_ps
#define _mm_xor_ _mm256_xor_ps
#endif
#endif


#ifdef use_AVX512
#ifdef DoublePrecision
#define AVXRealSize 8 // number of doubles in a AVXvector
#define PReal __m512d // packed double, 256-bit vector containing 8 double
#define _mm_add_ _mm512_add_pd
#define _mm_sub_ _mm512_sub_pd
#define _mm_mul_ _mm512_mul_pd
#define _mm_div_ _mm512_div_pd
#define	_mm_setr_ _mm512_setr_pd
#define _mm_setr_zero_ _mm512_setzero_pd
#define _mm_store_ _mm512_store_pd
#define _mm_load_ _mm512_load_pd
#define _mm_load_u _mm512_loadu_pd
#define _mm_fmadd_ _mm512_fmadd_pd
#define _mm_set1_ _mm512_set1_pd
#define _mm_fmadd_ _mm512_fmadd_pd
#define _mm_xor_ _mm512_xor_pd
#endif

#ifdef SinglePrecision
#define AVXRealSize 16 // number of floats in a AVXvector
#define PReal __m512   // packed float, 256-bit vector containing 16 floats
#define _mm_add_ _mm512_add_ps
#define _mm_sub_ _mm512_sub_ps
#define _mm_mul_ _mm512_mul_ps
#define _mm_div_ _mm512_div_ps
#define	_mm_setr_ _mm512_setr_ps
#define _mm_setr_zero_ _mm512_setzero_ps
#define _mm_store_ _mm512_store_ps
#define _mm_load_ _mm512_load_ps
#define _mm_load_u _mm512_loadu_ps
#define _mm_fmadd_ _mm512_fmadd_ps
#define _mm_set1_ _mm512_set1_ps
#define _mm_fmadd_ _mm512_fmadd_ps
#define _mm_xor_ _mm512_xor_ps
#endif
#endif


// operators for AVX2 & AVX512 
// note: in case of AVX2-DoublePrecision, AVX2 intrinsics in simd.hpp is deactivated and the instructions below will be used by ngs
// note: in case of AVX2-SinglePrecision and AVX512, the AVX2-DoublePrecision Intrinsics will be activated in simd.hpp for ngs
INLINE PReal operator- (PReal a) { return _mm_xor_(a, _mm_set1_(-0.0)); }
INLINE PReal operator+ (PReal a, PReal b) { return _mm_add_(a, b); }
INLINE PReal operator- (PReal a, PReal b) { return _mm_sub_(a, b); }
INLINE PReal operator* (PReal a, PReal b) { return _mm_mul_(a, b); }
INLINE PReal operator/ (PReal a, PReal b) { return _mm_div_(a, b); }
INLINE PReal operator* (Real a, PReal b) { return _mm_set1_(a)*b; }
INLINE PReal operator* (PReal b, double a) { return _mm_set1_(a)*b; }
INLINE PReal operator+= (PReal &a, PReal b) { return a = a + b; }
INLINE PReal operator-= (PReal &a, PReal b) { return a = a - b; }
INLINE PReal operator*= (PReal &a, PReal b) { return a = a*b; }
INLINE PReal operator/= (PReal &a, PReal b) { return a = a / b; }



