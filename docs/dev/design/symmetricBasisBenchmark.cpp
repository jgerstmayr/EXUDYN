//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This is an EXUDYN design benchmark (#2228, docs/dev/design/symmetricBasis.md)
//
// Details:  the time per frame of the variants of an orthonormal frame from two slope vectors a, b: Gram-Schmidt,
//           the symmetric basis exact (4 square roots), the polar rotation of [a, b] in closed form (2 square
//           roots), and iterative schemes without square roots (Newton-Schulz for the orthogonality, Newton for the
//           inverse square root); with the largest deviation from the exact symmetric basis
//
// Build:    Windows: cl /O2 /EHsc /std:c++17 /fp:precise symmetricBasisBenchmark.cpp
//           Linux:   g++ -O3 -ffp-contract=off -std=c++17 symmetricBasisBenchmark.cpp
//
// Author:   Johannes Gerstmayr
// Date:     2026-10-13
//
// Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

typedef std::array<double, 3> V3;
struct Frame { V3 e1, e2, e3; };

inline V3 operator+(const V3& a, const V3& b) { return { a[0] + b[0], a[1] + b[1], a[2] + b[2] }; }
inline V3 operator-(const V3& a, const V3& b) { return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }
inline V3 operator*(double s, const V3& a) { return { s*a[0], s*a[1], s*a[2] }; }
inline double Dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline V3 Cross(const V3& a, const V3& b) { return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] }; }

const double sqrt1_2 = 0.70710678118654752440;

//! Gram-Schmidt, as EXUmath::OrthogonalBasisFromVectorsXY: 2 square roots
inline Frame GramSchmidt(const V3& a, const V3& b)
{
	V3 e1 = (1. / std::sqrt(Dot(a, a)))*a;
	V3 e2 = b - Dot(b, e1)*e1;
	e2 = (1. / std::sqrt(Dot(e2, e2)))*e2;
	return { e1, e2, Cross(e1, e2) };
}

//! the symmetric basis as in symmetricBasis.py: normalize a, b, u, v: 4 square roots
inline Frame SymmetricDirect(const V3& a, const V3& b)
{
	V3 ah = (1. / std::sqrt(Dot(a, a)))*a;
	V3 bh = (1. / std::sqrt(Dot(b, b)))*b;
	V3 u = ah + bh;
	V3 v = ah - bh;
	u = (1. / std::sqrt(Dot(u, u)))*u;
	v = (1. / std::sqrt(Dot(v, v)))*v;
	V3 e1 = sqrt1_2*(u + v);
	V3 e2 = sqrt1_2*(u - v);
	return { e1, e2, Cross(e1, e2) };
}

//! the symmetric basis with c = ah.bh: |u| = sqrt(2+2c), |v| = sqrt(2-2c), e1, e2 as combinations of ah, bh, and
//! e3 = (ah x bh)/sin = 2 ku kv (ah x bh): 4 square roots, no further normalization
inline Frame SymmetricCombined(const V3& a, const V3& b)
{
	double ra = 1. / std::sqrt(Dot(a, a));
	double rb = 1. / std::sqrt(Dot(b, b));
	double c = Dot(a, b)*ra*rb;
	double ku = 1. / std::sqrt(2. + 2.*c);
	double kv = 1. / std::sqrt(2. - 2.*c);
	double p = sqrt1_2*(ku + kv);
	double m = sqrt1_2*(ku - kv);
	V3 ah = ra * a;
	V3 bh = rb * b;
	return { p*ah + m * bh, m*ah + p * bh, (2.*ku*kv)*Cross(ah, bh) };
}

//! the polar rotation of [a, b] (not normalized): X S^(-1/2), S = X^T X (2x2), sqrt(S) = (S + d I)/t with
//! d = sqrt(det S), t = sqrt(tr S + 2d): 2 square roots, 1 division; e3 = e1 x e2
inline Frame PolarAB(const V3& a, const V3& b)
{
	double saa = Dot(a, a), sab = Dot(a, b), sbb = Dot(b, b);
	double d = std::sqrt(saa*sbb - sab * sab);
	double t = std::sqrt(saa + sbb + 2.*d);
	//S^(-1/2) = t (S + d I)^(-1) = t/det(S+dI) [[sbb+d, -sab], [-sab, saa+d]]
	double k = t / ((saa + d)*(sbb + d) - sab * sab);
	double m11 = k * (sbb + d), m12 = -k * sab, m22 = k * (saa + d);
	V3 e1 = m11 * a + m12 * b;
	V3 e2 = m12 * a + m22 * b;
	return { e1, e2, Cross(e1, e2) };
}

//! the polar rotation of [ah, bh] by Newton-Schulz, X <- X (3I - X^T X)/2, after an exact normalization: 2 square
//! roots and 'iterations' steps of 3 dot products and 2 combinations
inline Frame NewtonSchulz(const V3& a, const V3& b, int iterations)
{
	V3 e1 = (1. / std::sqrt(Dot(a, a)))*a;
	V3 e2 = (1. / std::sqrt(Dot(b, b)))*b;
	for (int i = 0; i < iterations; i++)
	{
		double n1 = Dot(e1, e1), n2 = Dot(e2, e2), c = Dot(e1, e2);
		V3 f1 = (0.5*(3. - n1))*e1 - (0.5*c)*e2;
		V3 f2 = (0.5*(3. - n2))*e2 - (0.5*c)*e1;
		e1 = f1; e2 = f2;
	}
	return { e1, e2, Cross(e1, e2) };
}

//! no square root: the inverse lengths by Newton steps r <- r (3 - x r^2)/2 from r = 1 (slopes of about unit length),
//! then Newton-Schulz
inline Frame NewtonSchulzNoSqrt(const V3& a, const V3& b, int normalizeSteps, int iterations)
{
	double xa = Dot(a, a), xb = Dot(b, b), ra = 1., rb = 1.;
	for (int i = 0; i < normalizeSteps; i++)
	{
		ra = ra * (1.5 - 0.5*xa*ra*ra);
		rb = rb * (1.5 - 0.5*xb*rb*rb);
	}
	V3 e1 = ra * a;
	V3 e2 = rb * b;
	for (int i = 0; i < iterations; i++)
	{
		double n1 = Dot(e1, e1), n2 = Dot(e2, e2), c = Dot(e1, e2);
		V3 f1 = (0.5*(3. - n1))*e1 - (0.5*c)*e2;
		V3 f2 = (0.5*(3. - n2))*e2 - (0.5*c)*e1;
		e1 = f1; e2 = f2;
	}
	return { e1, e2, Cross(e1, e2) };
}

double Deviation(const Frame& f, const Frame& g)
{
	double d = 0.;
	for (int i = 0; i < 3; i++)
	{
		d = std::fmax(d, std::fabs(f.e1[i] - g.e1[i]));
		d = std::fmax(d, std::fabs(f.e2[i] - g.e2[i]));
		d = std::fmax(d, std::fabs(f.e3[i] - g.e3[i]));
	}
	return d;
}

template<class TFunction>
void Measure(const char* name, TFunction function, const std::vector<V3>& as, const std::vector<V3>& bs)
{
	const int repetitions = 2000;
	double best = 1e30;
	double checksum = 0.;
	double deviation = 0.;
	for (int run = 0; run < 5; run++)
	{
		auto start = std::chrono::steady_clock::now();
		for (int r = 0; r < repetitions; r++)
		{
			for (size_t i = 0; i < as.size(); i++)
			{
				Frame f = function(as[i], bs[i]);
				checksum += f.e1[0] + f.e2[1] + f.e3[2];
			}
		}
		auto end = std::chrono::steady_clock::now();
		best = std::fmin(best, std::chrono::duration<double>(end - start).count() / ((double)repetitions*as.size()));
	}
	for (size_t i = 0; i < as.size(); i++)
	{
		deviation = std::fmax(deviation, Deviation(function(as[i], bs[i]), SymmetricDirect(as[i], bs[i])));
	}
	std::printf("  %-36s %6.2f ns   deviation from symmetric %.1e   (checksum %.3f)\n", name, best*1e9, deviation, checksum*1e-12);
}

//! slope pairs: a rotated orthonormal frame, stretched by 'strain' and sheared by 'shear' (radians), at random
void Pairs(double strain, double shear, std::vector<V3>& as, std::vector<V3>& bs)
{
	std::mt19937 generator(1);
	std::uniform_real_distribution<double> uniform(-1., 1.);
	as.clear(); bs.clear();
	for (int i = 0; i < 1024; i++)
	{
		V3 x = { uniform(generator), uniform(generator), uniform(generator) };
		V3 y = { uniform(generator), uniform(generator), uniform(generator) };
		Frame f = GramSchmidt(x, y);
		double angle = shear * uniform(generator);
		V3 a = (1. + strain * uniform(generator))*f.e1;
		V3 b = (1. + strain * uniform(generator))*(std::cos(angle)*f.e2 + std::sin(angle)*f.e1);
		as.push_back(a); bs.push_back(b);
	}
}

int main()
{
	std::vector<V3> as, bs;
	const double cases[3][2] = { { 0.001, 0.01 }, { 0.01, 0.1 }, { 0.1, 0.5 } };
	for (auto& c : cases)
	{
		Pairs(c[0], c[1], as, bs);
		std::printf("strain up to %.3f, shear up to %.2f rad:\n", c[0], c[1]);
		Measure("Gram-Schmidt (2 sqrt)", GramSchmidt, as, bs);
		Measure("symmetric, direct (4 sqrt)", SymmetricDirect, as, bs);
		Measure("symmetric, combined (4 sqrt)", SymmetricCombined, as, bs);
		Measure("polar of [a,b] (2 sqrt, 1 div)", PolarAB, as, bs);
		Measure("normalize + Newton-Schulz 2", [](const V3& a, const V3& b) { return NewtonSchulz(a, b, 2); }, as, bs);
		Measure("normalize + Newton-Schulz 3", [](const V3& a, const V3& b) { return NewtonSchulz(a, b, 3); }, as, bs);
		Measure("normalize + Newton-Schulz 4", [](const V3& a, const V3& b) { return NewtonSchulz(a, b, 4); }, as, bs);
		Measure("no sqrt: Newton 3 + Newton-Schulz 3", [](const V3& a, const V3& b) { return NewtonSchulzNoSqrt(a, b, 3, 3); }, as, bs);
		Measure("no sqrt: Newton 4 + Newton-Schulz 4", [](const V3& a, const V3& b) { return NewtonSchulzNoSqrt(a, b, 4, 4); }, as, bs);
	}
	return 0;
}
