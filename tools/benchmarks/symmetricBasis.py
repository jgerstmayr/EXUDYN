#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN design script (#2228, docs/dev/internals/symmetricBasis.md)
#
# Details:  The symmetric basis of two non-parallel vectors a, b - the slope vectors of NodePointSlope12 and of the
#           other 3D nodes with two slopes - and the rotation Jacobian G with omega = G [a_t; b_t], in Python as the
#           reference for the C++ implementation:
#             SymmetricBasis(a, b)                 the rotation matrix [e1, e2, e3]
#             SymmetricBasisRotationJacobian(a, b) G (3 x 6), omega global, from dot products only
#           and the checks the concept relies on: an orthonormal right-handed basis, symmetric in a and b, independent
#           of the lengths of a and b, equal to a and b where they are orthogonal, equal to the rotation of the polar
#           decomposition, G equal to the numerical derivative of the basis, and the comparison with what
#           NodePointSlope12 computes now (Gram-Schmidt for the rotation, a least squares fit for omega).
#
# Usage:    python tools/benchmarks/symmetricBasis.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np

sqrt2 = np.sqrt(2.)

def Skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the basis
def SymmetricBasis(a, b):
    """the rotation matrix [e1, e2, e3] (columns) of the symmetric basis of the non-parallel vectors a, b

    With the unit vectors ah = a/|a|, bh = b/|b|, the bisector uh = (ah+bh)/|ah+bh| and vh = (ah-bh)/|ah-bh|, which
    are orthogonal: e1 = (uh+vh)/sqrt(2), e2 = (uh-vh)/sqrt(2) lie in the plane of a and b, at 45 degrees to the
    bisector, and e3 = e1 x e2 is the normal. e1 is as close to a as e2 is to b; for orthogonal a, b they are equal.
    """
    ah = a/np.linalg.norm(a)
    bh = b/np.linalg.norm(b)
    u = ah + bh
    v = ah - bh
    uh = u/np.linalg.norm(u)
    vh = v/np.linalg.norm(v)
    e1 = (uh + vh)/sqrt2
    e2 = (uh - vh)/sqrt2
    return np.column_stack([e1, e2, np.cross(e1, e2)])

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the rotation Jacobian, from dot products
def SymmetricBasisRotationJacobian(a, b):
    """G (3 x 6) with omega = G @ [a_t; b_t], omega the global angular velocity of SymmetricBasis(a, b)

    With ei_t = omega x ei: omega = (e3.e2_t) e1 - (e3.e1_t) e2 - (e1.e2_t) e3. The three rows follow from d(x/|x|) = (I - xh xh^T) dx/|x| and the relations e3.uh = e3.vh = e3.ah =
    e3.bh = 0, e1.uh = e1.vh = 1/sqrt(2): only dot products with e3, uh, vh and the projections of uh, vh on the
    planes normal to ah and bh remain - no 3 x 3 matrix is formed.
    """
    la = np.linalg.norm(a)
    lb = np.linalg.norm(b)
    ah = a/la
    bh = b/lb
    u = ah + bh
    v = ah - bh
    lu = np.linalg.norm(u)
    lv = np.linalg.norm(v)
    uh = u/lu
    vh = v/lv
    e1 = (uh + vh)/sqrt2
    e2 = (uh - vh)/sqrt2
    e3 = np.cross(e1, e2)

    #out of the plane: e3.ah_t = e3.a_t/|a|, e3.bh_t = e3.b_t/|b|
    #e3.uh_t = (e3.ah_t + e3.bh_t)/|u|, e3.vh_t = (e3.ah_t - e3.bh_t)/|v|
    #e3.e1_t = (e3.uh_t + e3.vh_t)/sqrt(2), e3.e2_t = (e3.uh_t - e3.vh_t)/sqrt(2)
    ra = e3/la                              #e3.ah_t = ra.a_t
    rb = e3/lb                              #e3.bh_t = rb.b_t
    e3uh = np.concatenate([ra, rb])/lu      #row of e3.uh_t
    e3vh = np.concatenate([ra, -rb])/lv     #row of e3.vh_t
    e3e1 = (e3uh + e3vh)/sqrt2
    e3e2 = (e3uh - e3vh)/sqrt2

    #in the plane: e1.e2_t = (e1.uh_t - e1.vh_t)/sqrt(2), with e1.uh_t = vh.(ah_t + bh_t)/(sqrt(2)|u|) and
    #e1.vh_t = uh.(ah_t - bh_t)/(sqrt(2)|v|); x.ah_t = (x - (x.ah) ah).a_t/|a|, and the same for b
    def Tangential(x, h, l):
        return (x - (x @ h)*h)/l
    vha = Tangential(vh, ah, la)
    vhb = Tangential(vh, bh, lb)
    uha = Tangential(uh, ah, la)
    uhb = Tangential(uh, bh, lb)
    e1uh = np.concatenate([vha, vhb])/(sqrt2*lu)
    e1vh = np.concatenate([uha, -uhb])/(sqrt2*lv)
    e1e2 = (e1uh - e1vh)/sqrt2

    return np.outer(e1, e3e2) - np.outer(e2, e3e1) - np.outer(e3, e1e2)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what NodePointSlope12 computes now (src/ImplNodes/CNodePointSlope12.cpp)
def GramSchmidtBasis(a, b):
    """EXUmath::OrthogonalBasisFromVectorsXY: e1 along a, e2 from b by Gram-Schmidt"""
    e1 = a/np.linalg.norm(a)
    e2 = b - (b @ e1)*e1
    e2 /= np.linalg.norm(e2)
    return np.column_stack([e1, e2, np.cross(e1, e2)])

def LeastSquaresOmega(a, b, a_t, b_t):
    """CNodePointSlope12::GetAngularVelocity: omega with omega x a ~ a_t and omega x b ~ b_t in the least squares sense"""
    W = -(Skew(a) @ Skew(a) + Skew(b) @ Skew(b))
    return np.linalg.solve(W, Skew(a) @ a_t + Skew(b) @ b_t)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def OmegaOfBasis(Basis, a, b, a_t, b_t, epsilon=1e-7):
    """omega from the numerical time derivative of the basis: A_t A^T = skew(omega)"""
    A = Basis(a, b)
    A_t = (Basis(a + epsilon*a_t, b + epsilon*b_t) - Basis(a - epsilon*a_t, b - epsilon*b_t))/(2*epsilon)
    W = A_t @ A.T
    return np.array([W[2, 1], W[0, 2], W[1, 0]])

def PolarRotationAB(a, b):
    """the polar rotation of [a, b] itself, not normalized: [a, b] S^(-1/2) with S = [a, b]^T [a, b] (2 x 2) and
    sqrt(S) = (S + d I)/t, d = sqrt(det S), t = sqrt(tr S + 2d) - 2 square roots; the rotation R of F = R U for the
    in-plane deformation gradient F = [a, b] of slopes that are orthonormal in the reference configuration"""
    saa, sab, sbb = a @ a, a @ b, b @ b
    d = np.sqrt(saa*sbb - sab**2)
    t = np.sqrt(saa + sbb + 2*d)
    k = t/((saa + d)*(sbb + d) - sab**2)
    e1 = k*((sbb + d)*a - sab*b)
    e2 = k*(-sab*a + (saa + d)*b)
    return np.column_stack([e1, e2, np.cross(e1, e2)])

def PolarRotation(a, b):
    """the rotation of the polar decomposition of [ah, bh, n], n the unit normal"""
    ah = a/np.linalg.norm(a)
    bh = b/np.linalg.norm(b)
    n = np.cross(ah, bh)
    n /= np.linalg.norm(n)
    U, s, Vt = np.linalg.svd(np.column_stack([ah, bh, n]))
    return U @ Vt

if __name__ == '__main__':
    rng = np.random.default_rng(1)
    worst = {}
    def Record(name, value):
        worst[name] = max(worst.get(name, 0.), value)

    for i in range(1000):
        a = rng.normal(size=3)
        b = rng.normal(size=3)
        if np.linalg.norm(np.cross(a, b)) < 0.1*np.linalg.norm(a)*np.linalg.norm(b):
            continue #nearly parallel: the basis is not defined there
        a_t = rng.normal(size=3)
        b_t = rng.normal(size=3)
        A = SymmetricBasis(a, b)
        Record('orthonormal |A^T A - I|', np.abs(A.T @ A - np.eye(3)).max())
        Record('right-handed |det A - 1|', abs(np.linalg.det(A) - 1))
        B = SymmetricBasis(b, a)
        Record('symmetric: e1(b,a) = e2(a,b)', np.abs(B[:, 0] - A[:, 1]).max())
        Record('independent of |a|, |b|', np.abs(SymmetricBasis(3.7*a, 0.2*b) - A).max())
        Record('equal to the polar rotation', np.abs(PolarRotation(a, b) - A).max())
        P = PolarRotationAB(a, b)
        Record('polar of [a,b]: orthonormal', np.abs(P.T @ P - np.eye(3)).max())
        Record('polar of [a,b]: symmetric, e1(b,a) = e2(a,b)', np.abs(PolarRotationAB(b, a)[:, 0] - P[:, 1]).max())
        Record('polar of [a,b]: equal to the symmetric basis for |a| = |b|',
               np.abs(PolarRotationAB(a/np.linalg.norm(a), b/np.linalg.norm(b)) - A).max())
        G = SymmetricBasisRotationJacobian(a, b)
        omega = G @ np.concatenate([a_t, b_t])
        Record('G against the numerical derivative', np.abs(omega - OmegaOfBasis(SymmetricBasis, a, b, a_t, b_t)).max()
               / max(1, np.abs(omega).max()))
        #the present NodePointSlope12: its omega is not the derivative of its rotation matrix
        omegaGS = OmegaOfBasis(GramSchmidtBasis, a, b, a_t, b_t)
        Record('NodePointSlope12 now: least squares omega - omega of Gram-Schmidt',
               np.abs(LeastSquaresOmega(a, b, a_t, b_t) - omegaGS).max()/max(1, np.abs(omegaGS).max()))
        Record('NodePointSlope12 now: least squares omega - omega of the symmetric basis',
               np.abs(LeastSquaresOmega(a, b, a_t, b_t) - omega).max()/max(1, np.abs(omega).max()))
        #a rigid motion of a and b: all three agree
        w = rng.normal(size=3)
        Record('rigid motion: least squares, Gram-Schmidt, symmetric omega - w',
               max(np.abs(LeastSquaresOmega(a, b, np.cross(w, a), np.cross(w, b)) - w).max(),
                   np.abs(OmegaOfBasis(GramSchmidtBasis, a, b, np.cross(w, a), np.cross(w, b)) - w).max(),
                   np.abs(G @ np.concatenate([np.cross(w, a), np.cross(w, b)]) - w).max()))
        Record('rotation angle between symmetric and Gram-Schmidt basis [deg]',
               np.degrees(np.arccos(np.clip((np.trace(GramSchmidtBasis(a, b).T @ A) - 1)/2, -1, 1))))

    #orthogonal a, b: the basis is a and b themselves
    a = np.array([2., 0, 0])
    b = np.array([0, 0.5, 0])
    Record('orthogonal: [ah, bh, n]', np.abs(SymmetricBasis(a, b) - np.eye(3)).max())

    for (name, value) in worst.items():
        print(f'{name:70s} {value:.2e}')
