/** ***********************************************************************************************
* @brief		Eigen's DenseBase plugin (EIGEN_DENSEBASE_PLUGIN) on arm64, set in LinearSolver.h: it adds nothing to
*				DenseBase, but is included by Eigen/Core after the NEON packet math and before the product kernels,
*				which is where EIGEN_HAS_SINGLE_INSTRUCTION_MADD is undefined (#2990). Eigen 3.3.7 defines the macro
*				on NEON unconditionally, on SSE only with FMA; it selects the block height of the general matrix
*				product (mr = 3 packets instead of 2) and a fused multiply-add in its kernel, and with them the order
*				of the sums in SparseLU. Without it, arm64 gives the bits of x86-64, without a change of the vendored
*				Eigen sources.
*
* @author		Gerstmayr Johannes
* @date			2026-10-15
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
*
************************************************************************************************ */
//no include guard: Eigen includes it once, inside the class DenseBase, and only preprocessor lines may be here
#undef EIGEN_HAS_SINGLE_INSTRUCTION_MADD
