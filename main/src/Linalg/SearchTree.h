/** ***********************************************************************************************
* @class	    SearchTree
* @brief		Class for boxed search
* @details		Details:
*               allows to find objects in space efficiently
*
* @author		Gerstmayr Johannes
* @date			2021-10-23 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
************************************************************************************************ */
#ifndef SEARCHTREE__H
#define SEARCHTREE__H

#include "Linalg/BasicLinalg.h"		//includes Vector.h


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++    BOX3D     +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//_Quad a = 4;
//_float128 a = 1.23;

class Box3D
{
public:
	Box3D()
	{
		Clear();
		////set empty box:
		//pmin = Vector3D({ EXUstd::MAXREAL, EXUstd::MAXREAL, EXUstd::MAXREAL });
		//pmax = Vector3D({EXUstd::LOWESTREAL, EXUstd::LOWESTREAL, EXUstd::LOWESTREAL});
	}
	Box3D(const Vector3D& p1, const Vector3D& p2)
	{
		pmin.X() = EXUstd::Minimum(p1.X(),p2.X());
		pmin.Y() = EXUstd::Minimum(p1.Y(),p2.Y());
		pmin.Z() = EXUstd::Minimum(p1.Z(),p2.Z());

		pmax.X() = EXUstd::Maximum(p1.X(),p2.X());
		pmax.Y() = EXUstd::Maximum(p1.Y(),p2.Y());
		pmax.Z() = EXUstd::Maximum(p1.Z(),p2.Z());
	}
	Box3D(const Box3D& b)
	{
		pmin = b.pmin;
		pmax = b.pmax;
	}
	Box3D(const Vector3D& c, Real r)
	{
		pmin = c;
		pmax = c;
		Increase(r);
	}

	//! check if box is empty, only based on x-value!
	bool Empty() const 
	{
		if (pmin.X() == EXUstd::MAXREAL) {return true;}
		return false;
	}
	void Clear()
	{
		//set empty box:
		pmin = Vector3D({ EXUstd::MAXREAL, EXUstd::MAXREAL, EXUstd::MAXREAL });
		pmax = Vector3D({ EXUstd::LOWESTREAL, EXUstd::LOWESTREAL, EXUstd::LOWESTREAL });
	}

	void Add(const Vector3D & p)
	{
		if (Empty())
		{
			pmin = p;
			pmax = p;
		}
		else
		{
			pmin.X() = EXUstd::Minimum(pmin.X(),p.X());
			pmin.Y() = EXUstd::Minimum(pmin.Y(),p.Y());
			pmin.Z() = EXUstd::Minimum(pmin.Z(),p.Z());

			pmax.X() = EXUstd::Maximum(pmax.X(),p.X());
			pmax.Y() = EXUstd::Maximum(pmax.Y(),p.Y());
			pmax.Z() = EXUstd::Maximum(pmax.Z(),p.Z());
		}
	}
	void Add(const Box3D& b)
	{
		if (b.Empty()) return;

		if (Empty())
		{
			pmin = b.PMin();
			pmax = b.PMax();
		}
		else
		{
			pmin.X() = EXUstd::Minimum(pmin.X(),b.pmin.X());
			pmin.Y() = EXUstd::Minimum(pmin.Y(),b.pmin.Y());
			pmin.Z() = EXUstd::Minimum(pmin.Z(),b.pmin.Z());

			pmax.X() = EXUstd::Maximum(pmax.X(),b.pmax.X());
			pmax.Y() = EXUstd::Maximum(pmax.Y(),b.pmax.Y());
			pmax.Z() = EXUstd::Maximum(pmax.Z(),b.pmax.Z());
		}
	}
	const Vector3D& PMin() const { return pmin; }
	const Vector3D& PMax() const { return pmax; }
	Vector3D& PMin() { return pmin; }
	Vector3D& PMax() { return pmax; }
	const Real SizeX() const {return pmax[0]-pmin[0];}
	const Real SizeY() const {return pmax[1]-pmin[1];}
	const Real SizeZ() const {return pmax[2]-pmin[2];}
	Vector3D Center() const {return 0.5*(pmin+pmax);}
	Real Radius() const {return 0.5*(pmax-pmin).GetL2Norm();}

	void Increase(Real x, Real y, Real z)
	{
		pmin.X() -= x;
		pmin.Y() -= y;
		pmin.Z() -= z;
		pmax.X() += x;
		pmax.Y() += y;
		pmax.Z() += z;
	}
	void Increase(Real x) 
	{
		Increase(x,x,x);
	}
  void InflateFactor(Real x)
	{
		Vector3D pmi = PMin();
		Vector3D pma = PMax();
		Vector3D pc = Center();
		pmin = pc + ( pmi-pc ) * x;
		pmax = pc + ( pma-pc ) * x;
	}

	//! check if box b intersects with this (boundary included)
	bool Intersect(const Box3D& b) const 
	{
		//> and < changed to >= and <= in order to simplify problems with points on boundaries
		if ( pmin[0] >= b.pmax[0] || pmax[0] <= b.pmin[0]
			|| pmin[1] >= b.pmax[1] || pmax[1] <= b.pmin[1]
			|| pmin[2] >= b.pmax[2] || pmax[2] <= b.pmin[2])
			return false;

		return true;
	}

	//! return true if point p in closure or on boundary
	bool IsInside (const Vector3D & p) const
	{
		if ( pmin[0] <= p[0] && pmax[0] >= p[0]
			&& pmin[1] <= p[1] && pmax[1] >= p[1]
			&& pmin[2] <= p[2] && pmax[2] >= p[2])
			return true;

		return false;
	}

	//! @brief Output operator for Box3D
	friend std::ostream& operator<<(std::ostream& os, const Box3D& v)
	{
		os << "{" << v.PMin() << ", " << v.PMax() << "}";
		return os;
	}

private:
	Vector3D pmin, pmax;
};



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Zero-based!!!!
//generate a searchtree with a physical size of Box3D b
//all items must fit into this box and should be equally distributed
//then use AddItems to add items with a bounding box and an identifier
//GetItemsInBox() gives you all identifiers which have a bounding box within the specified box
class SearchTree
{
private:
	Index sx, sy, sz;
	ArrayIndex* data;
	Box3D box;

public:
	SearchTree(): sx(0), sy(0), sz(0), data(nullptr) {};

	//SearchTree& operator=(const SearchTree& tree)
	//{
	//	if (&tree == this) return *this;

	//	if (data)
	//	{
	//		for (Index i = 0; i < TotalSize(); i++)
	//		{
	//			data[i].Flush();
	//		}
	//		delete [] data;
	//	}

	//	box = tree.box;

	//	sx = tree.sx;
	//	sy = tree.sy;
	//	sz = tree.sz;

	//	data = new ArrayIndex[TotalSize()]();
	//	for (Index i = 0; i < TotalSize(); i++)
	//	{
	//		data[i] = tree.data[i];
	//	}
	//	return *this;
	//}
	
	bool isEmpty()
	{
		return TotalSize() == 0;
	}

	~SearchTree()
	{
		Flush();
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//! empty all data and reset cells
	void Flush()
	{
		if (data)
		{
			for (Index i = 0; i < TotalSize(); i++)
			{
				data[i].Flush();
			}
			delete[] data;
			data = nullptr;
		}
	}

	const Box3D& GetBox() const { return box; }
	Box3D& GetBox() { return box; }

	Index SizeX() const { return sx; }
	Index SizeY() const { return sy; }
	Index SizeZ() const { return sz; }
	Index TotalSize() const { return sx*sy*sz; }

	void ResetSearchTree(Index sizex, Index sizey, Index sizez, Box3D b)
	{
		ClearItems(); //empty all items
		box = b;
		CHECKandTHROW(box.SizeX()*box.SizeY()*box.SizeZ() > 0, "SearchTree: size of box must be not equal 0");

		if (sx != sizex || sy != sizey || sz != sizez)
		{
			if (data)
			{
				for (Index i = 0; i < TotalSize(); i++)
				{
					data[i].Flush();
				}
				delete [] data;
			}

			sx = sizex;
			sy = sizey;
			sz = sizez;

			data = new ArrayIndex[TotalSize()]();
		}
	}

	//return 6 indices for box: minx, maxx, miny, maxy, minz, maxz
	void GetBoxIndizes(const Box3D& b, Index6& ind) const
	{
		ind[0] = IndX(b.PMin().X());
		ind[1] = IndX(b.PMax().X());
		ind[2] = IndY(b.PMin().Y());
		ind[3] = IndY(b.PMax().Y());
		ind[4] = IndZ(b.PMin().Z());
		ind[5] = IndZ(b.PMax().Z());
	}

	//return items in box defined by 6 indices: minx, maxx, miny, maxy, minz, maxz
	void GetItemsInBox(Index6& ind, ArrayIndex& items) const
	{
		CHECKandTHROW(data != 0, "GetItemsInBox: data=0");

		items.SetNumberOfItems(0);
		Index ld;
		for (Index ix = ind[0]; ix <= ind[1]; ix++) //<= ind is correct, as it contains at most (sx-1)
		{
			for (Index iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (Index iz = ind[4]; iz <= ind[5]; iz++)
				{
					ld = data[GlobalIndex(ix, iy, iz)].NumberOfItems();
					for (Index i = 0; i < ld; i++)
					{
						items.AppendPure((data[GlobalIndex(ix, iy, iz)])[i]);
					}
				}
			}
		}
	}

	//add items in box defined by 6 indices: minx, maxx, miny, maxy, minz, maxz
	//does not reset items list
	void AddItemsInBox(Index6& ind, ArrayIndex& items) const
	{
		CHECKandTHROW(data != 0, "AddItemsInBox: data=0");

		Index ld;
		for (Index ix = ind[0]; ix <= ind[1]; ix++)
		{
			for (Index iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (Index iz = ind[4]; iz <= ind[5]; iz++)
				{
					ld = data[GlobalIndex(ix, iy, iz)].NumberOfItems();
					for (Index i = 0; i < ld; i++)
					{
						items.AppendPure((data[GlobalIndex(ix, iy, iz)])[i]);
					}
				}
			}
		}
	}

	//get only items in box
	void GetItemsInBox(const Box3D& b, ArrayIndex& items) const
	{
		items.SetNumberOfItems(0);
		Index ind[6];
		ind[0] = IndX(b.PMin().X());
		ind[1] = IndX(b.PMax().X());
		ind[2] = IndY(b.PMin().Y());
		ind[3] = IndY(b.PMax().Y());
		ind[4] = IndZ(b.PMin().Z());
		ind[5] = IndZ(b.PMax().Z());
		Index ix, iy, iz, i;
		ArrayIndex* id;

		for (ix = ind[0]; ix <= ind[1]; ix++)
		{
			for (iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (iz = ind[4]; iz <= ind[5]; iz++)
				{
					id = &data[GlobalIndex(ix, iy, iz)];
					for (i = 0; i < id->NumberOfItems(); i++)
					{
						items.AppendPure(id->GetItemUnsafe(i));
					}
				}
			}
		}
	}

	//get items in box b; do not add duplicates by using indexFlags array, having one bool per index, all initialized with false
	//leave out items with index >= currentIndex
	void GetSingleItemsInBoxHalf(const Box3D& b, ArrayIndex& items, ResizableArray<bool>& indexFlags, Index currentIndex, bool clearIndexFlags = true) const
	{
		items.SetNumberOfItems(0);
		Index ind[6];
		ind[0] = IndX(b.PMin().X());
		ind[1] = IndX(b.PMax().X());
		ind[2] = IndY(b.PMin().Y());
		ind[3] = IndY(b.PMax().Y());
		ind[4] = IndZ(b.PMin().Z());
		ind[5] = IndZ(b.PMax().Z());
		Index ix, iy, iz, i;
		ArrayIndex* id;

		for (ix = ind[0]; ix <= ind[1]; ix++)
		{
			for (iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (iz = ind[4]; iz <= ind[5]; iz++)
				{
					id = &data[GlobalIndex(ix, iy, iz)];
					for (i = 0; i < id->NumberOfItems(); i++)
					{
						Index newItem = id->GetItemUnsafe(i);
						if (!indexFlags[newItem] && newItem < currentIndex)
						{
							items.AppendPure(newItem);
							indexFlags[newItem] = true;
						}
					}
				}
			}
		}
		//reset indexFlags back to original state, containing only false
		if (clearIndexFlags)
		{
			for (Index i : items)
			{
				indexFlags[i] = false;
			}
		}
	}
	
	//get items in box b; do not add duplicates by using indexFlags array, having one bool per index, all initialized with false
	void GetSingleItemsInBox(const Box3D& b, ArrayIndex& items, ResizableArray<bool>& indexFlags, bool clearIndexFlags=true) const
	{
		items.SetNumberOfItems(0);
		Index ind[6];
		ind[0] = IndX(b.PMin().X());
		ind[1] = IndX(b.PMax().X());
		ind[2] = IndY(b.PMin().Y());
		ind[3] = IndY(b.PMax().Y());
		ind[4] = IndZ(b.PMin().Z());
		ind[5] = IndZ(b.PMax().Z());
		Index ix, iy, iz, i;
		ArrayIndex* id;

		for (ix = ind[0]; ix <= ind[1]; ix++)
		{
			for (iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (iz = ind[4]; iz <= ind[5]; iz++)
				{
					id = &data[GlobalIndex(ix, iy, iz)];
					for (i = 0; i < id->NumberOfItems(); i++)
					{
						Index newItem = id->GetItemUnsafe(i);
						if (!indexFlags[newItem])
						{
							items.AppendPure(newItem);
							indexFlags[newItem] = true;
						}
					}
				}
			}
		}
		//reset indexFlags back to original state, containing only false
		if (clearIndexFlags)
		{
			for (Index i : items)
			{
				indexFlags[i] = false;
			}
		}
	}

	//get items in box, do not reset items list
	void AddItemsInBox(const Box3D& b, ArrayIndex& items) const
	{
		Index ind[6];
		ind[0] = IndX(b.PMin().X());
		ind[1] = IndX(b.PMax().X());
		ind[2] = IndY(b.PMin().Y());
		ind[3] = IndY(b.PMax().Y());
		ind[4] = IndZ(b.PMin().Z());
		ind[5] = IndZ(b.PMax().Z());
		Index ix, iy, iz, i;
		ArrayIndex* id;

		for (ix = ind[0]; ix <= ind[1]; ix++)
		{
			for (iy = ind[2]; iy <= ind[3]; iy++)
			{
				for (iz = ind[4]; iz <= ind[5]; iz++)
				{
					id = &data[GlobalIndex(ix, iy, iz)];
					for (i = 0; i < id->NumberOfItems(); i++)
					{
						items.AppendPure(id->GetItem(i));
					}
				}
			}
		}
	}

	//! get all items of the box with global index ind
	void GetItemsOfBox(Index ind, ArrayIndex& items) const
	{
		items = data[ind];
	}

	//! get all items of the box in which the point 'pos' lies
	void GetItemsOfBox(const Vector3D& pos, ArrayIndex& items) const
	{
		GetItemsOfBox(GlobalIndex(IndX(pos.X()), IndY(pos.Y()), IndZ(pos.Z())), items);
	}

	//add all items of the box with index ind
	void AddItemsOfBox(Index ind, ArrayIndex& items) const
	{
		items.AppendArray(data[ind]);
	}

	//! add all items of the box in which the point lies
	void AddItemsOfPoint(const Vector3D& pos, ArrayIndex& items) const
	{
		AddItemsOfBox(GlobalIndex(IndX(pos.X()), IndY(pos.Y()), IndZ(pos.Z())), items);
	}

	//! add item to search tree
	void AddItem(const Box3D& b, Index identifier)
	{
		Index pMinX = IndX(b.PMin().X());
		Index pMaxX = IndX(b.PMax().X());
		Index pMinY = IndY(b.PMin().Y());
		Index pMaxY = IndY(b.PMax().Y());
		Index pMinZ = IndZ(b.PMin().Z());
		Index pMaxZ = IndZ(b.PMax().Z());
		//Index gi = iX + iY * sx + iZ * (sx*sy);
		Index sxsy = sx * sy;
		Index gi;
		for (Index iz = pMinZ; iz <= pMaxZ; iz++)
		{
			gi = pMinY * sx + iz * sxsy;
			for (Index iy = pMinY; iy <= pMaxY; iy++)
			{
				for (Index ix = pMinX; ix <= pMaxX; ix++)
				{
					//data[GlobalIndex(ix, iy, iz)].AppendPure(identifier);
					data[gi+ix].AppendPure(identifier);
				}
				gi += sx;
			}
		}
		//old: worse regarding caching and precomputed values:
		//for (Index ix = IndX(b.PMin().X()); ix <= IndX(b.PMax().X()); ix++)
		//{
		//	for (Index iy = IndY(b.PMin().Y()); iy <= IndY(b.PMax().Y()); iy++)
		//	{
		//		for (Index iz = IndZ(b.PMin().Z()); iz <= IndZ(b.PMax().Z()); iz++)
		//		{
		//			data[GlobalIndex(ix, iy, iz)].AppendPure(identifier);
		//		}
		//	}
		//}
	}

	//! return x-index for a Real x-value in Box
	Index IndX(Real x) const
	{
		//(Index) would behave wrong between -1 to +1: (int)0.8 => 0, (int)(-0.8) => 0
		//        floor(...) corrects this: floor(0.8) => 0, fllor(-0.8) => -1
		Index i = (Index)floor(((x - box.PMin().X())*(Real)sx) / box.SizeX());
		if (i < 0) { i = 0; }
		if (i >= sx) { i = sx - 1; }
		return i;
	}

	//! return y-index for a Real y-value in Box
	Index IndY(Real y) const
	{
		Index i = (Index)floor(((y - box.PMin().Y())*(Real)sy) / box.SizeY());
		if (i < 0) { i = 0; }
		if (i >= sy) { i = sy - 1; }
		return i;
	}

	//! return z-index for a Real z-value in Box
	Index IndZ(Real z) const
	{
		Index i = (Index)floor(((z - box.PMin().Z())*(Real)sz) / box.SizeZ());
		if (i < 0) { i = 0; }
		if (i >= sz) { i = sz - 1; }
		return i;
	}

	//! get index in 3D list from 3 indices
	//! could be improved, especially by storing sx*sy
	Index GlobalIndex(Index iX, Index iY, Index iZ) const
	{
		Index gi = iX + iY * sx + iZ * (sx*sy);
		//not necessary lateron, as IndX, etc. add checks!
		//CHECKandTHROW((gi < TotalSize()) && (gi >= 0), "GlobalIndex: computation of global index failed");
		return gi;
	}

	//! empty searchtree, but keep memory
	void ClearItems()
	{
		for (Index i = 0; i < TotalSize(); i++)
		{
			data[i].SetNumberOfItems(0);
		}
		//for (Index ix=0; ix < sx; ix++)
		//{
		//	for (Index iy=0; iy < sy; iy++)
		//	{
		//		for (Index iz=0; iz < sz; iz++)
		//		{
		//			data[GlobalIndex(ix,iy,iz)].SetNumberOfItems(0);
		//		}
		//	}
		//}
	}

	void GetStatistics(Index& numberOfTreeItems, Real& averageFill, Index& numberOfZeros, Index& maxFill, Index& numberOf10average) const
	{
		numberOfTreeItems = 0;
		averageFill = -1;
		numberOf10average = -1;
		maxFill = 0;
		numberOfZeros = 0;
		for (Index i = 0; i < TotalSize(); i++)
		{
			Index value = data[i].NumberOfItems();
			numberOfTreeItems += value;
			maxFill = EXUstd::Maximum(value, maxFill);
			if (value == 0) { numberOfZeros++; }
		}
		if (TotalSize() != 0)
		{
			averageFill = (Real)numberOfTreeItems / (Real)TotalSize();
			numberOf10average = 0;
			for (Index i = 0; i < TotalSize(); i++)
			{
				if (data[i].NumberOfItems() > (Index)(10. * averageFill))
				{
					numberOf10average++;
				}
			}
		}

	}

};




#endif
