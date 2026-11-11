/** ***********************************************************************************************
* @brief		The forward and backward substitution of a sparse LU factorization on Exudyn's threads (#3432):
*               Eigen's SparseLU with a nested-dissection ordering, whose tree makes the subtrees independent; the
*               factors are copied into rows after each factorization and solved by the levels of the tree
* @details		Details:
*               - the ordering: nested dissection by breadth-first level separators on the pattern of A + A^T joined
*                 with the cliques of the structural groups (the coordinates of each object, as exact zeros are
*                 dropped from the Jacobian), to a depth that gives about two subtrees per thread; unknowns with a
*                 zero diagonal (the multipliers of constraints) are lifted into the highest tree node of their
*                 neighbours with a nonzero diagonal and ordered last in it, so that their diagonal is nonzero when
*                 they are eliminated and a small pivot threshold keeps the diagonal pivots
*               - the ordering is kept while the entries of the matrix join related tree nodes; a factor that
*                 couples two subtrees after pivoting is detected, solved serially, and ordered anew next time
*               - design: docs/dev/design/sparseSolvers.md section 4
*
* @author		Gerstmayr Johannes
* @date			2026-11-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */
#ifndef PARALLELSUBSTITUTION__H
#define PARALLELSUBSTITUTION__H

#include <vector>

//! the nested-dissection ordering and its tree (#3432)
class NestedDissection
{
public:
	//! the column permutation (position of each column) and the tree node of each column (heap numbering, root 1),
	//! from the pattern of a column-major matrix of size n
	static void Order(int n, const int* outerIndex, const int* innerIndex, const int* innerNonZeros, const double* values,
		int maxDepth, std::vector<int>& position, std::vector<int>& node);
	//! true if node a is node b or one of its ancestors
	static bool IsAncestorOrSelf(int a, int b) { while (b > a) { b /= 2; } return a == b; }

	static thread_local int maxDepth;			//!< set by SparseLUParallel before the analysis
	static thread_local std::vector<int>* nodes; //!< receives the tree node of each column
	static thread_local std::vector<int>* positions; //!< receives the position of each column; if reuse, gives it
	static thread_local bool reuse;				//!< true: positions and nodes are kept from the last ordering
	static thread_local const Eigen::SparseMatrix<double>* orderingMatrix; //!< if set, the pattern ordered instead of the analyzed one
};

//! the ordering functor of Eigen::SparseLU (#3432)
template <typename StorageIndex>
class NestedDissectionOrdering
{
public:
	typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, StorageIndex> PermutationType;
	template <typename MatrixType>
	void operator() (const MatrixType& mat, PermutationType& perm)
	{
		int n = (int)mat.cols();
		std::vector<int> ownPosition;
		std::vector<int>& position = NestedDissection::positions ? *NestedDissection::positions : ownPosition;
		std::vector<int> dummyNodes;
		if (!NestedDissection::reuse || (int)position.size() != n)
		{
			const Eigen::SparseMatrix<double>* other = NestedDissection::orderingMatrix;
			if (other && other->cols() == n && other->isCompressed())
			{
				NestedDissection::Order(n, other->outerIndexPtr(), other->innerIndexPtr(), nullptr, other->valuePtr(),
					NestedDissection::maxDepth, position, NestedDissection::nodes ? *NestedDissection::nodes : dummyNodes);
			}
			else
			{
				NestedDissection::Order(n, mat.outerIndexPtr(), mat.innerIndexPtr(), mat.innerNonZeroPtr(), mat.valuePtr(),
					NestedDissection::maxDepth, position, NestedDissection::nodes ? *NestedDissection::nodes : dummyNodes);
			}
		}
		perm.resize(n);
		for (int i = 0; i < n; i++) { perm.indices()(i) = (StorageIndex)position[i]; }
	}
};

//! SparseLU with the nested-dissection ordering and a parallel substitution (#3432)
class SparseLUParallel : public Eigen::SparseLU<Eigen::SparseMatrix<double>, NestedDissectionOrdering<int> >
{
	typedef Eigen::SparseLU<Eigen::SparseMatrix<double>, NestedDissectionOrdering<int> > Base;

	int maxDepth = 0;
	std::vector<int> columnNodes;		//!< the tree node of each column of the matrix
	std::vector<int> columnPositions;	//!< the position of each column in the nested-dissection ordering
	bool prepared = false;				//!< the rows of the factors are ready and the tree holds
	//the factors by rows, in the positions of the factorization:
	std::vector<int> lRowStart, lColumns; std::vector<double> lValues; //!< L without its unit diagonal
	std::vector<int> uRowStart, uColumns; std::vector<double> uValues; //!< U without its diagonal
	std::vector<double> uDiagonal;
	//the schedule: the tree nodes per depth, the positions per tree node
	std::vector<std::vector<int>> depthNodes;	//!< node ids of each depth
	std::vector<int> nodeStart, nodePositions;	//!< nodePositions[nodeStart[id] ... nodeStart[id+1]-1], increasing
	std::vector<double> depthWork;				//!< the nonzeros of the rows of each depth
	std::vector<double> y;						//!< work vector
	std::vector<std::vector<int>> groups;		//!< from SetStructuralGroups
	Eigen::SparseMatrix<double> unionPattern;	//!< the union of the patterns seen, which is ordered
	std::vector<int> analyzedOuter, analyzedInner; //!< the pattern of the last analysis
	int analyzedDepth = -1;						//!< the tree depth of the last analysis; -1: none
public:
	int numberOfOrderings = 0;					//!< nested-dissection orderings computed

public:
	//! the groups of unknowns that may be coupled (the coordinates of an object); their cliques join the pattern ordered
	void SetStructuralGroups(const std::vector<std::vector<int>>& groupsInit) { groups = groupsInit; analyzedDepth = -1; unionPattern.resize(0, 0); }
	//! analyze (ordering for numberOfThreads) and factorize; returns Eigen's info(), 0 = success
	int AnalyzeAndFactorize(const Eigen::SparseMatrix<double>& matrix, int numberOfThreads, double pivotThreshold);
	//! true if the last factorization is solved in parallel; false: its factors couple subtrees (serial solve)
	bool Prepared() const { return prepared; }
	//! solve with the factors; in parallel on the threads of the TaskManager if Prepared()
	void SolveParallel(const double* rhs, double* solution, int n);

private:
	//! copy the factors into rows, check the tree and build the schedule; false if a dependency crosses the tree
	bool Prepare();
};

#endif
