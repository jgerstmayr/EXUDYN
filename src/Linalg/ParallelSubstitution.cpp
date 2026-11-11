/** ***********************************************************************************************
* @brief		The forward and backward substitution of a sparse LU factorization on Exudyn's threads (#3432)
* @details		Details: see ParallelSubstitution.h and docs/dev/design/sparseSolvers.md section 4
*
* @author		Gerstmayr Johannes
* @date			2026-11-11 (created)
* @copyright	This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
************************************************************************************************ */

#include "Linalg/LinearSolver.h"	//Eigen with Exudyn's settings, and ParallelSubstitution.h
#include "Utilities/Parallel.h"		//ExuThreading::ParallelFor
#include <algorithm>
#include <deque>

thread_local int NestedDissection::maxDepth = 4;
thread_local std::vector<int>* NestedDissection::nodes = nullptr;
thread_local std::vector<int>* NestedDissection::positions = nullptr;
thread_local bool NestedDissection::reuse = false;
thread_local const Eigen::SparseMatrix<double>* NestedDissection::orderingMatrix = nullptr;

namespace {
	const int leafSize = 64; //a part of at most this many unknowns is not dissected further

	//the graph of A + A^T without the diagonal, and the work arrays of the dissection
	struct Graph
	{
		std::vector<int> start, adjacent;
		std::vector<int> mark;	//!< stamp of the set a vertex is in
		std::vector<int> level;
		int stamp = 0;
		std::vector<int>* node;

		void Mark(const std::vector<int>& vertices) { stamp++; for (int v : vertices) { mark[v] = stamp; } }

		//breadth-first levels within the marked set from start; returns the vertices in the order visited
		void Levels(const std::vector<int>& vertices, int startVertex, std::vector<int>& order)
		{
			Mark(vertices);
			int visited = ++stamp; //vertices of the set carry stamp-1, the visited ones stamp
			order.clear();
			order.push_back(startVertex); mark[startVertex] = visited; level[startVertex] = 0;
			for (size_t q = 0; q < order.size(); q++)
			{
				int v = order[q];
				for (int k = start[v]; k < start[v + 1]; k++)
				{
					int w = adjacent[k];
					if (mark[w] == visited - 1) { mark[w] = visited; level[w] = level[v] + 1; order.push_back(w); }
				}
			}
		}

		void Assign(const std::vector<int>& vertices, int id) { for (int v : vertices) { (*node)[v] = id; } }

		void Dissect(const std::vector<int>& vertices, int depth, int maxDepth, int id)
		{
			if (depth >= maxDepth || (int)vertices.size() <= leafSize) { Assign(vertices, id); return; }
			std::vector<int> order;
			Levels(vertices, vertices[0], order);
			if (order.size() < vertices.size()) //several components: two halves of whole components
			{
				std::vector<std::vector<int>> components;
				Mark(vertices);
				int visited = ++stamp;
				for (int s : vertices)
				{
					if (mark[s] == visited) { continue; }
					std::vector<int> component(1, s); mark[s] = visited;
					for (size_t q = 0; q < component.size(); q++)
					{
						int v = component[q];
						for (int k = start[v]; k < start[v + 1]; k++)
						{
							int w = adjacent[k];
							if (mark[w] == visited - 1) { mark[w] = visited; component.push_back(w); }
						}
					}
					components.push_back(std::move(component));
				}
				std::sort(components.begin(), components.end(), [](const std::vector<int>& a, const std::vector<int>& b) { return a.size() > b.size(); });
				std::vector<int> half[2];
				for (const auto& c : components)
				{
					std::vector<int>& h = half[half[0].size() <= half[1].size() ? 0 : 1];
					h.insert(h.end(), c.begin(), c.end());
				}
				Dissect(half[0], depth + 1, maxDepth, 2 * id);
				Dissect(half[1], depth + 1, maxDepth, 2 * id + 1);
				return;
			}
			Levels(vertices, order.back(), order); //from a pseudo-peripheral vertex
			int numberOfLevels = level[order.back()] + 1;
			if (numberOfLevels < 3) { Assign(vertices, id); return; }
			std::vector<int> counts(numberOfLevels, 0);
			for (int v : order) { counts[level[v]]++; }
			int m = 0, sum = 0;
			while (m < numberOfLevels && 2 * (sum + counts[m]) < (int)order.size()) { sum += counts[m]; m++; }
			m = std::min(std::max(m, 1), numberOfLevels - 2);
			std::vector<int> partA, partB, separator;
			for (int v : order)
			{
				if (level[v] < m) { partA.push_back(v); }
				else if (level[v] > m) { partB.push_back(v); }
				else { separator.push_back(v); }
			}
			Assign(separator, id);
			Dissect(partA, depth + 1, maxDepth, 2 * id);
			Dissect(partB, depth + 1, maxDepth, 2 * id + 1);
		}
	};
}

void NestedDissection::Order(int n, const int* outerIndex, const int* innerIndex, const int* innerNonZeros, const double* values,
	int maxDepthInit, std::vector<int>& position, std::vector<int>& node)
{
	//the pattern of A + A^T without the diagonal; zero diagonals
	std::vector<std::pair<int, int>> edges;
	std::vector<char> zeroDiagonal(n, 1);
	for (int j = 0; j < n; j++)
	{
		int end = innerNonZeros ? outerIndex[j] + innerNonZeros[j] : outerIndex[j + 1];
		for (int k = outerIndex[j]; k < end; k++)
		{
			int i = innerIndex[k];
			if (i == j) { if (values[k] != 0.) { zeroDiagonal[j] = 0; } }
			else { edges.push_back({ i, j }); edges.push_back({ j, i }); }
		}
	}
	std::sort(edges.begin(), edges.end());
	edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
	Graph graph;
	graph.start.assign(n + 1, 0);
	for (const auto& e : edges) { graph.start[e.first + 1]++; }
	for (int i = 0; i < n; i++) { graph.start[i + 1] += graph.start[i]; }
	graph.adjacent.resize(edges.size());
	for (size_t k = 0; k < edges.size(); k++) { graph.adjacent[k] = edges[k].second; } //edges are sorted by the first vertex
	graph.mark.assign(n, 0);
	graph.level.assign(n, 0);
	node.assign(n, 1);
	graph.node = &node;

	std::vector<int> all(n);
	for (int i = 0; i < n; i++) { all[i] = i; }
	if (n > 0) { graph.Dissect(all, 0, maxDepthInit, 1); }

	//a zero diagonal goes into the highest tree node of its neighbours with a nonzero diagonal (all of them are in its
	//own node, its ancestors or its descendants), so that they are eliminated before it; the nodes of the dissection
	//decide, not those of other lifted unknowns, which would carry a lift along a chain of zero diagonals
	std::vector<int> dissectionNode(node);
	for (int v = 0; v < n; v++)
	{
		if (!zeroDiagonal[v]) { continue; }
		int best = dissectionNode[v];
		for (int k = graph.start[v]; k < graph.start[v + 1]; k++)
		{
			int w = graph.adjacent[k];
			if (!zeroDiagonal[w] && IsAncestorOrSelf(dissectionNode[w], best)) { best = dissectionNode[w]; }
		}
		node[v] = best;
	}

	//the elimination order: the tree in postorder (children before parents), in a node the zero diagonals last
	int maxId = 1;
	for (int v = 0; v < n; v++) { maxId = std::max(maxId, node[v]); }
	std::vector<int> rank(maxId + 1, 0);
	{
		int counter = 0;
		std::vector<std::pair<int, bool>> stack(1, { 1, false });
		while (!stack.empty())
		{
			auto item = stack.back(); stack.pop_back();
			if (item.second) { rank[item.first] = counter++; continue; }
			stack.push_back({ item.first, true });
			for (int child : { 2 * item.first + 1, 2 * item.first }) //the left child is visited first
			{
				if (child <= maxId) { stack.push_back({ child, false }); }
			}
		}
	}
	std::vector<int> order(n);
	for (int i = 0; i < n; i++) { order[i] = i; }
	std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
		if (rank[node[a]] != rank[node[b]]) { return rank[node[a]] < rank[node[b]]; }
		return zeroDiagonal[a] < zeroDiagonal[b]; });
	position.assign(n, 0);
	for (int k = 0; k < n; k++) { position[order[k]] = k; }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int SparseLUParallel::AnalyzeAndFactorize(const Eigen::SparseMatrix<double>& matrix, int numberOfThreads, double pivotThreshold)
{
	//a depth with about two subtrees per thread (1, 2 or 4 gave the same times on chains)
	maxDepth = 2;
	while ((1 << maxDepth) < 2 * numberOfThreads && maxDepth < 10) { maxDepth++; }
	setPivotThreshold(pivotThreshold);

	int n = (int)matrix.cols();
	const int* outer = matrix.outerIndexPtr();
	const int* inner = matrix.innerIndexPtr();
	bool samePattern = analyzedDepth == maxDepth && (int)analyzedOuter.size() == n + 1 &&
		std::equal(outer, outer + n + 1, analyzedOuter.begin()) && (int)analyzedInner.size() == outer[n] &&
		std::equal(inner, inner + outer[n], analyzedInner.begin());
	if (!samePattern)
	{
		//Exudyn drops exact zeros, so the pattern changes as the system moves; the ordering is kept while every entry
		//joins related tree nodes, otherwise it is computed anew on the union of the patterns seen (#3432)
		bool keepOrdering = analyzedDepth == maxDepth && (int)columnNodes.size() == n && (int)columnPositions.size() == n;
		for (int j = 0; j < n && keepOrdering; j++)
		{
			for (int k = outer[j]; k < outer[j + 1]; k++)
			{
				int a = columnNodes[inner[k]], b = columnNodes[j];
				if (!NestedDissection::IsAncestorOrSelf(a, b) && !NestedDissection::IsAncestorOrSelf(b, a)) { keepOrdering = false; break; }
			}
		}
		if (!keepOrdering)
		{
			std::vector<Eigen::Triplet<double>> triplets;
			bool keepUnion = analyzedDepth == maxDepth && unionPattern.cols() == n;
			triplets.reserve(matrix.nonZeros() + (keepUnion ? unionPattern.nonZeros() : 0));
			if (!keepUnion) //the first pattern: the cliques of the structural groups with it
			{
				for (const auto& group : groups)
				{
					for (int a : group) { for (int b : group) { if (a != b && a < n && b < n) { triplets.push_back(Eigen::Triplet<double>(a, b, 0.)); } } }
				}
			}
			if (keepUnion)
			{
				for (int j = 0; j < n; j++)
				{
					for (int k = unionPattern.outerIndexPtr()[j]; k < unionPattern.outerIndexPtr()[j + 1]; k++)
					{
						triplets.push_back(Eigen::Triplet<double>(unionPattern.innerIndexPtr()[k], j, 0.));
					}
				}
			}
			for (int j = 0; j < n; j++)
			{
				for (int k = outer[j]; k < outer[j + 1]; k++) { triplets.push_back(Eigen::Triplet<double>(inner[k], j, matrix.valuePtr()[k])); }
			}
			unionPattern.resize(n, n);
			unionPattern.setFromTriplets(triplets.begin(), triplets.end()); //the values of this matrix: zero diagonals are found
			unionPattern.makeCompressed();
			numberOfOrderings++;
		}
		NestedDissection::maxDepth = maxDepth;
		NestedDissection::nodes = &columnNodes;
		NestedDissection::positions = &columnPositions;
		NestedDissection::reuse = keepOrdering;
		NestedDissection::orderingMatrix = &unionPattern;
		analyzePattern(matrix);
		NestedDissection::nodes = nullptr;
		NestedDissection::positions = nullptr;
		NestedDissection::reuse = false;
		NestedDissection::orderingMatrix = nullptr;
		analyzedOuter.assign(outer, outer + n + 1);
		analyzedInner.assign(inner, inner + outer[n]);
		analyzedDepth = maxDepth;
	}
	factorize(matrix);
	prepared = false;
	if (info() != Eigen::Success) { analyzedDepth = -1; return (int)info(); }
	prepared = Prepare();
	if (!prepared) { analyzedDepth = -1; } //a pivot coupled the subtrees: a new ordering next time
	return 0;
}

bool SparseLUParallel::Prepare()
{
	int n = (int)rows();
	if ((int)columnNodes.size() != n) { return false; }
	//the tree node of each position: column i of the matrix is at position m_perm_c(i)
	std::vector<int> positionNode(n);
	for (int i = 0; i < n; i++) { positionNode[m_perm_c.indices()(i)] = columnNodes[i]; }

	//the factors by rows: L strictly lower (unit diagonal), U strictly upper and its diagonal
	std::vector<int> lCount(n + 1, 0), uCount(n + 1, 0);
	uDiagonal.assign(n, 0.);
	for (int j = 0; j < n; j++)
	{
		for (SCMatrix::InnerIterator it(m_Lstore, j); it; ++it)
		{
			int i = (int)it.index();
			if (i > j) { lCount[i + 1]++; }
			else if (i < j) { uCount[i + 1]++; }
		}
		for (Eigen::MappedSparseMatrix<double, Eigen::ColMajor, int>::InnerIterator it(m_Ustore, j); it; ++it) { uCount[it.index() + 1]++; }
	}
	for (int i = 0; i < n; i++) { lCount[i + 1] += lCount[i]; uCount[i + 1] += uCount[i]; }
	lRowStart = lCount; uRowStart = uCount;
	lColumns.resize(lCount[n]); lValues.resize(lCount[n]);
	uColumns.resize(uCount[n]); uValues.resize(uCount[n]);
	std::vector<int> lFill(lRowStart.begin(), lRowStart.end() - 1), uFill(uRowStart.begin(), uRowStart.end() - 1);
	bool treeHolds = true;
	for (int j = 0; j < n; j++)
	{
		for (SCMatrix::InnerIterator it(m_Lstore, j); it; ++it)
		{
			int i = (int)it.index();
			if (i > j) //row i of the forward solve needs position j: its node must be below the node of i
			{
				lColumns[lFill[i]] = j; lValues[lFill[i]++] = it.value();
				if (!NestedDissection::IsAncestorOrSelf(positionNode[i], positionNode[j])) { treeHolds = false; }
			}
			else if (i < j) //row i of the backward solve needs position j: its node must be above the node of i
			{
				uColumns[uFill[i]] = j; uValues[uFill[i]++] = it.value();
				if (!NestedDissection::IsAncestorOrSelf(positionNode[j], positionNode[i])) { treeHolds = false; }
			}
			else { uDiagonal[j] = it.value(); }
		}
		for (Eigen::MappedSparseMatrix<double, Eigen::ColMajor, int>::InnerIterator it(m_Ustore, j); it; ++it)
		{
			int i = (int)it.index();
			uColumns[uFill[i]] = j; uValues[uFill[i]++] = it.value();
			if (!NestedDissection::IsAncestorOrSelf(positionNode[j], positionNode[i])) { treeHolds = false; }
		}
	}
	if (!treeHolds) { return false; }

	//the schedule: the positions of each node in increasing order, the nodes of each depth
	int maxId = 1;
	for (int p = 0; p < n; p++) { maxId = std::max(maxId, positionNode[p]); }
	nodeStart.assign(maxId + 2, 0);
	for (int p = 0; p < n; p++) { nodeStart[positionNode[p] + 1]++; }
	for (int id = 0; id <= maxId; id++) { nodeStart[id + 1] += nodeStart[id]; }
	nodePositions.resize(n);
	std::vector<int> fill(nodeStart.begin(), nodeStart.end() - 1);
	for (int p = 0; p < n; p++) { nodePositions[fill[positionNode[p]]++] = p; }
	int depths = 0;
	while ((1 << depths) <= maxId) { depths++; }
	depthNodes.assign(depths, std::vector<int>());
	depthWork.assign(depths, 0.);
	for (int id = 1; id <= maxId; id++)
	{
		if (nodeStart[id + 1] == nodeStart[id]) { continue; }
		int d = 0;
		while ((2 << d) <= id) { d++; }
		depthNodes[d].push_back(id);
		for (int k = nodeStart[id]; k < nodeStart[id + 1]; k++)
		{
			int p = nodePositions[k];
			depthWork[d] += (lRowStart[p + 1] - lRowStart[p]) + (uRowStart[p + 1] - uRowStart[p]) + 1;
		}
	}
	y.resize(n);
	return true;
}

void SparseLUParallel::SolveParallel(const double* rhs, double* solution, int n)
{
	//y = Pr * b
	const auto& rowPermutation = m_perm_r.indices();
	for (int i = 0; i < n; i++) { y[rowPermutation(i)] = rhs[i]; }

	const int* lStart = lRowStart.data(); const int* lCol = lColumns.data(); const double* lVal = lValues.data();
	const int* uStart = uRowStart.data(); const int* uCol = uColumns.data(); const double* uVal = uValues.data();
	const double* diag = uDiagonal.data();
	const int* positions = nodePositions.data(); const int* start = nodeStart.data();
	double* x = y.data();
	const double minimumParallelWork = 2000.; //below, a level is solved serially

	auto Forward = [=](int id)
	{
		for (int k = start[id]; k < start[id + 1]; k++)
		{
			int i = positions[k];
			double sum = x[i];
			for (int m = lStart[i]; m < lStart[i + 1]; m++) { sum -= lVal[m] * x[lCol[m]]; }
			x[i] = sum;
		}
	};
	auto Backward = [=](int id)
	{
		for (int k = start[id + 1] - 1; k >= start[id]; k--)
		{
			int i = positions[k];
			double sum = x[i];
			for (int m = uStart[i]; m < uStart[i + 1]; m++) { sum -= uVal[m] * x[uCol[m]]; }
			x[i] = sum / diag[i];
		}
	};
	bool parallel = ExuThreading::TaskManager::GetNumThreads() > 1 && ExuThreading::task_manager;
	int depths = (int)depthNodes.size();
	for (int d = depths - 1; d >= 0; d--) //forward: the deepest nodes first
	{
		const std::vector<int>& ids = depthNodes[d];
		if (parallel && ids.size() > 1 && depthWork[d] >= minimumParallelWork)
		{
			ExuThreading::ParallelFor(ids.size(), [&](size_t k) { Forward(ids[k]); }, (int)ids.size());
		}
		else { for (int id : ids) { Forward(id); } }
	}
	for (int d = 0; d < depths; d++) //backward: the root first
	{
		const std::vector<int>& ids = depthNodes[d];
		if (parallel && ids.size() > 1 && depthWork[d] >= minimumParallelWork)
		{
			ExuThreading::ParallelFor(ids.size(), [&](size_t k) { Backward(ids[k]); }, (int)ids.size());
		}
		else { for (int id : ids) { Backward(id); } }
	}
	//solution = Pc^-1 * x: column i of the matrix is at position m_perm_c(i)
	const auto& columnPermutation = m_perm_c.indices();
	for (int i = 0; i < n; i++) { solution[i] = x[columnPermutation(i)]; }
}

