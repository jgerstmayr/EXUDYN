/** ***********************************************************************************************
* @class		SlimVectorBase
* @brief		A slim, fast, templated vector with constant size to be allocated on stack or within large (dynamic) arrays
* @details		Details:
                    - a vector of Real entries (double/float);
                    - templated number of Reals using 'dataSize'
                    - this vector can be used in (possibly huge) dynamic arrays (std::vector, ResizableArray)
                    - data can be copied with memcopy
                    - efficient implementation for 2D/3D case (PLANNED)

* @author		Gerstmayr Johannes
* @date			2018-04-25 (generated)
* @date			2018-04-30 (last modified)
* @pre			Indizes of []-operator run from 0 to dataSize-1;
* 				Use SlimVector for small vector sizes (<100; better: <=12)
* @copyright		This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
* *** Example code ***
*
* @code{.cpp}
* SlimVectorBase<T, 3> v1({1.1, 2.7, 3.0}); //create a vector with 3 Real
* v2 = v1;                           //assign v1 to v2
* v1 += v2;                          //add v2 to v1
* cout << v1 << "\n";                //write "[1.1, 2.7, 3.0]" to cout
* @endcode
************************************************************************************************ */
#ifndef SLIMVECTORBASE__H
#define SLIMVECTORBASE__H

#include "Utilities/ReleaseAssert.h"
#include <initializer_list>
#include <vector>
#include <array>

//template <typename T, Index dataSize> class SlimVectorBase;

template<typename T, Index dataSize> //! dataSize number of Reals in SlimVector; must be constexpr
class SlimVectorBase
{
protected:
    T data[dataSize]; //!< const number of T given by template parameter 'dataSize'

public:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CONSTRUCTOR, DESTRUCTOR
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! default constructor: no initialization here, in order to allow simple copy of data
	SlimVectorBase() {}; //rule of 5

	//! copy constructor
	SlimVectorBase(const SlimVectorBase<T, dataSize>& other)
	{
		Index cnt = 0;
		for (auto val : other) {
			data[cnt++] = val;
		}
	}

	//! constructor with a single scalar value used for all vector components.
	SlimVectorBase(T scalarValue)
    {
        for (auto &item : *this) { item = scalarValue; }
    }

    //! initializer list: e.g. SlimVectorBase<T, 3> ({1.0, 3.14, 5.5}); dataSize must exactly match the initializer_list.size() which is different from ConstSizeVector
    //! @todo CHECK PERFORMANCE performance of initializer_list SlimVectorBase<T, 3> ({1.0, 3.14, 5.5})
	SlimVectorBase(std::initializer_list<T> listOfItems) //pass by value as a standard in C++11
	//SlimVector(const T(&listOfItems)[dataSize]) //immediate check of initializer_list size, but would allow to cast to std::vector<> ==> dangerous!!! //pass by value as a standard in C++11
	{
		//not needed in C++14 and above; 
		CHECKandTHROW(dataSize == listOfItems.size(), "ERROR: SlimVector::constructor, initializer_list.size() must match template dataSize");
		//static_assert supported by C++14 (supports listOfReals.size() as constexpr)
		
        Index cnt = 0;
        for (auto val : listOfItems) {
            (*this)[cnt++] = val;
        }
    }

    //! constructor with Vector; 
    //! @brief Initialize SlimVector by data given from vector at startPositionVector=0; 
    //! copies 'dataSize' items, independently of vector size (might cause memory access error)
	SlimVectorBase(const VectorBase<T>& vector, Index startPositionVector) //remove default argument for startPositionVector in order to avoid unwanted casting from Vector
    {
		CHECKandTHROW(startPositionVector >= 0, "ERROR: SlimVector(const VectorBase<T>&, Index), startPositionVector < 0");
		CHECKandTHROW(dataSize + startPositionVector <= vector.NumberOfItems(), "ERROR:  SlimVector(const VectorBase<T>&, Index), dataSize mismatch with initializer_list");

        Index cnt = startPositionVector;
        for (auto& item : *this) {
            item = vector[cnt++];
        }
    }


	//! constructor with std::vector
	SlimVectorBase(const std::vector<T> vector)
	{
		CHECKandTHROW(vector.size() == dataSize, "ERROR: SlimVector(const std::vector<T> vector), dataSize mismatch");

		//better?: std::copy(vector.begin(), vector.end(), this->begin());
		Index cnt = 0;
		for (auto& item : *this) {
			item = vector[cnt++];
		}
	}

	SlimVectorBase(const std::array<T, dataSize> vector)
	{
		CHECKandTHROW(vector.size() == dataSize, "ERROR: SlimVector(const std::array<T> vector), dataSize mismatch");

		//better?: std::copy(vector.begin(), vector.end(), this->begin());
		Index cnt = 0;
		for (auto& item : *this) {
			item = vector[cnt++];
		}
	}

	~SlimVectorBase() { }; //rule of 5

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // BASIC FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    T* begin() { return &data[0]; }					//!< C++11 std::begin() for iterators.
    T* end() { return &data[dataSize]; }				//!< C++11 std::end() for iterators.
    const T* begin() const { return &data[0]; }		//!< C++11 std::begin() for iterators, const version needed for ==, +=, etc.
    const T* end() const { return &data[dataSize]; } //!< C++11 std::end() for iterators, const version needed for ==, +=, etc.
    Index NumberOfItems() const { return dataSize; }	//!< number of ('T') components in vector (for 'SlimVectorBase<T, 3> v;' NumberOfItems() returns 3).
	const T* GetDataPointer() const { return &data[0]; }         //!< return pointer to first data containing T numbers.
	T* GetDataPointer() { return &data[0]; }         //!< return pointer to first data containing T numbers.

    //! set all Reals to given value.
    void SetAll(T value)
    {
        for (auto &item : *this) { item = value; }
    }

	//! set vector to data given by initializer list
	void SetVector(std::initializer_list<T> listOfItems)
	{
		CHECKandTHROW(dataSize == listOfItems.size(), "ERROR: SlimVector::SetVector, initializer_list.size() must match template dataSize");

		Index cnt = 0;
		for (auto val : listOfItems) {
			(*this)[cnt++] = val;
		}
	}


	//! for compatibility with Vector and ConstVector
	void SetNumberOfItems(Index numberOfItems)
	{
		CHECKandTHROW(numberOfItems == dataSize, "SlimVectorBase<T, >::SetNumberOfItems size mismatch");
	}

	//! copy from other vector and perform type conversion (e.g. for graphics)
	template<class TVector>
	void CopyFrom(const TVector& vector)
	{
		CHECKandTHROW(vector.NumberOfItems() == dataSize, "SlimVectorBase<T, >::CopyFrom(TVector) size mismatch");
		Index cnt = 0;
		for (auto val : vector) {
			(*this)[cnt++] = (T)val;
		}
	}
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // OPERATORS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //! reference (write) access-operator.
    T& operator[](Index item)
    {
		CHECKandTHROW((item >= 0), "ERROR: SlimVector T& operator[]: item < 0");
		CHECKandTHROW((item < dataSize), "ERROR: SlimVector T& operator[]: item >= dataSize");

        return data[item];
    };

    //! const (read) access-operator
    const T& operator[](Index item) const
    {
		CHECKandTHROW((item >= 0), "ERROR: SlimVector T operator[] const: item < 0");
		CHECKandTHROW((item < dataSize), "ERROR: SlimVector T operator[] const: item >= dataSize");

        return data[item];
    };

    //this is dangerous: use SetAll(...) instead!
    ////! assign a scalar value to all components of the vector
    //SlimVectorBase<T, dataSize>& operator= (T scalarValue)
    //{
    //    for (auto &item : *this) {
    //        item = scalarValue;
    //    }
    //    return *this;
    //}

	//! copy assignment operator
	SlimVectorBase<T,dataSize>& operator= (const SlimVectorBase<T, dataSize>& other)
	{
		if (this == &other) { return *this; }

		Index cnt = 0;
		for (auto item : other) {
			(*this)[cnt++] = item;
		}
		return *this;
	}


    //! comparison operator, component-wise compare; returns true, if all components are equal
    bool operator== (const SlimVectorBase<T, dataSize>& v) const
    {
        Index cnt = 0;
        for (auto item : v)
        {
            if (item != (*this)[cnt++]) { return false; }
        }
        return true;
    }

	//! comparison operator for scalar value; returns true, if all components are equal to value
	bool operator==(T value) const
	{
		for (auto item : (*this))
		{
			if (item != value) { return false; }
		}
		return true;
	}

    //! add vector v to *this vector (for each component); both vectors must have same size
    SlimVectorBase<T,dataSize>& operator+= (const SlimVectorBase<T, dataSize>& v)
    {
        Index cnt = 0;
        for (auto item : v) {
            (*this)[cnt++] += item;
        }
        return *this;
    }

    //! substract vector v from *this vector (for each component); both vectors must have same size
	SlimVectorBase<T, dataSize>& operator-= (const SlimVectorBase<T, dataSize>& v)
    {
        Index cnt = 0;
        for (auto item : v) {
            (*this)[cnt++] -= item;
        }
        return *this;
    }
    //! scalar multiply vector *this with scalar (for each component)
	SlimVectorBase<T, dataSize>& operator*= (T scalar)
    {
        for (auto &item : *this) {
            item *= scalar;
        }
        return *this;
    }
    //! scalar division of vector v through scalar (for each component)
	SlimVectorBase<T, dataSize>& operator/= (T scalar)
    {
        for (auto &item : *this) {
            item /= scalar;
        }
        return *this;
    }

    //! add two vectors, result = v1+v2 (for each component)
    friend SlimVectorBase<T, dataSize> operator+ (const SlimVectorBase<T, dataSize>& v1, const SlimVectorBase<T, dataSize>& v2)
    {
		SlimVectorBase<T, dataSize> result;
        Index cnt = 0;
        for (auto &item : result) {
            item = v1[cnt] + v2[cnt];
            cnt++;
        }
        return result;
    }

	//! subtract two vectors, result = v1-v2 (for each component)
	friend SlimVectorBase<T, dataSize> operator- (const SlimVectorBase<T, dataSize>& v1, const SlimVectorBase<T, dataSize>& v2)
	{
		SlimVectorBase<T, dataSize> result;
		Index cnt = 0;
		for (auto &item : result) {
			item = v1[cnt] - v2[cnt];
			cnt++;
		}
		return result;
	}

	//! unary minus; result = -v1 (for each component)
	friend SlimVectorBase<T, dataSize> operator- (const SlimVectorBase<T, dataSize>& v1)
	{
		SlimVectorBase<T, dataSize> result;
		Index cnt = 0;
		for (auto &item : result) {
			item = -v1[cnt];
			cnt++;
		}
		return result;
	}

	//! scalar multiply, result = scalar * v (for each component)
    friend SlimVectorBase<T, dataSize> operator* (const SlimVectorBase<T, dataSize>& v, T scalar)
    {
        SlimVectorBase<T, dataSize> result;
        Index cnt = 0;
        for (auto &item : result) {
            item = scalar * v[cnt++];
        }
        return result;
    }

    //! scalar multiply, result = v * scalar (for each component)
    friend SlimVectorBase<T, dataSize> operator* (T scalar, const SlimVectorBase<T, dataSize>& v)
    {
        SlimVectorBase<T, dataSize> result;
        Index cnt = 0;
        for (auto &item : result) {
            item = scalar * v[cnt++];
        }
        return result;
    }

    //! scalar product, result = v1 * v2 (scalar result)
    friend T operator* (const SlimVectorBase<T, dataSize>& v1, const SlimVectorBase<T, dataSize>& v2)
    {
        T result = 0;
        Index cnt = 0;
        for (auto &item : v1) {
            result += item * v2[cnt++];
        }
        return result;
    }

	//! conversion of SlimVector into std::vector (needed e.g. in pybind)
	operator std::vector<T>() const
	{
		return std::vector<T>(begin(), end());
	}

	//! conversion of SlimVector into std::array (needed e.g. in pybind)
	operator std::array<T, dataSize>() const
	{
		std::array<T, dataSize> v;
		std::copy(begin(), end(), v.begin());

		//for (Index i = 0; i < dataSize; i++)
		//{
		//	v[i] = data[i];
		//}
		return v;
	}

	// conversion to Vector does not work ==> use ToVector(SlimVectorBase<T, size>&) template
	//operator Vector() const
	//{
	//	return Vector(dataSize, &data[0]);
	//}

    /*
    friend SlimVector operator* (const Matrix& m, const SlimVector& v);
    friend SlimVector operator* (const Matrix3D& m, const SlimVector& v);
    friend SlimVector operator* (const SlimVector& v, const Matrix3D& m); //res=v^T*m =m^T*v
    friend SlimVector operator* (const Matrix3D& m, const Vector& v);
    friend SlimVector operator* (const Matrix3D& m, const Vector& v);

    friend void Mult(const Matrix3D& m, const SlimVector& v, Vector& res);
    friend void Mult(const Matrix& m, const SlimVector& v, Vector& res); //computes res=m*v
    friend void Mult(const MatrixXD& m, const Vector& v, SlimVector& res);
    friend void Mult(const Matrix3D& m, const SlimVector& v, SlimVector& res);
    friend void Mult(const Matrix& m, const Vector& v, SlimVector& res); //computes res=m*v
    friend void MultTp(const Matrix& m, const Vector& v, SlimVector& res); //computes res=m*v

    */

    //! @brief Output operator << generates ostream "[v[0] v[1] .... v[dataSize-]]" for a vector v;
    //! the FORMAT IS DIFFERENT TO HOTINT1 ==> no separating comma ','
    friend std::ostream& operator<<(std::ostream& os, const SlimVectorBase<T, dataSize>& v)
    {
		char s = ' ';
		if (linalgPrintUsePythonFormat) { s = ','; }
		os << "[";
		for (Index i = 0; i < v.NumberOfItems(); i++) {
			os << v[i];
			if (i < v.NumberOfItems() - 1) { os << s; }
		}
		os << "]";
		//os << "[";
  //      for (Index i = 0; i < v.NumberOfItems(); i++) {
  //          os << v[i];
  //          if (i < v.NumberOfItems() - 1) { os << " "; }
  //      }

  //      os << "]";
        return os;
    }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXTENDED FUNCTIONS
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //! returns the sum of squared components (v[0]^2 + v[1]^2 + v[2]^2 ....)
    T GetL2NormSquared() const
    {
        T result = 0.;
        for (auto item : *this) { result += item * item; }
        return result;
    }

    //! returns the square norm of a vector
    T GetL2Norm() const
    {
        return sqrt(GetL2NormSquared());
    }

    //! normalizes the vector; divide each component by vector square norm
    void Normalize()
    {
        T norm = GetL2Norm();
		CHECKandTHROW(norm != 0., "SlimVector::Normalized() called with GetL2Norm() == 0.");

        for (auto &item : *this) { item /= norm; }
    }

    T& X()
    {
        static_assert(dataSize >= 1, "ERROR: SlimVector dataSize < 1 for function T& X()");
        return data[0];
    }
    T& Y()
    {
        static_assert(dataSize >= 2, "ERROR: SlimVector dataSize < 2 for function T& Y()");
        return data[1];
    }
    T& Z()
    {
        static_assert(dataSize >= 3, "ERROR: SlimVector dataSize < 3 for function T& Z()");
        return data[2];
    }

    T X() const
    {
        static_assert(dataSize >= 1, "ERROR: SlimVector dataSize < 1 for function T X() const");
        return data[0];
    }
    T Y() const
    {
        static_assert(dataSize >= 2, "ERROR: SlimVector dataSize < 2 for function T Y() const");
        return data[1];
    }
    T Z() const
    {
        static_assert(dataSize >= 3, "ERROR: SlimVector dataSize < 3 for function T Z() const");
        return data[2];
    }

	SlimVectorBase<T, dataSize> CrossProduct(const SlimVectorBase& v) const
	{
		static_assert((dataSize == 3), "SlimVectorBase::CrossProduct: only implemented for 3D case");

		return SlimVectorBase<T, dataSize>({ data[1] * v.data[2] - data[2] * v.data[1],
			data[2] * v.data[0] - data[0] * v.data[2],
			data[0] * v.data[1] - data[1] * v.data[0] });
	}

	T CrossProduct2D(const SlimVectorBase& v) const
	{
		static_assert((dataSize == 2), "SlimVectorBase::CrossProduct2D: only implemented for 2D case");

		return data[0] * v.data[1] - data[1] * v.data[0];
	}
};

template<Index dataSize>
using SlimVector = SlimVectorBase<Real, dataSize>;

template<Index dataSize>
using SlimVectorF = SlimVectorBase<float, dataSize>;

typedef SlimVector<1> Vector1D; //needed to multiply 1D-coordinate vector with matrix ... (ContactCoordinate, ...)
typedef SlimVector<2> Vector2D;
typedef SlimVector<3> Vector3D;
typedef SlimVector<4> Vector4D;
//typedef SlimVector<5> Vector5D; //uncomment as soon it is needed
typedef SlimVector<6> Vector6D; //inertia parameters, stresses, ...
typedef SlimVector<7> Vector7D; //rigid body initial/reference/... coordinates

typedef SlimVectorBase<float, 2> Float2; //!< a triple of float values => for OpenGL
typedef SlimVectorBase<float, 3> Float3; //!< a triple of float values => for OpenGL
typedef SlimVectorBase<float, 4> Float4; //!< a triple of float values => for OpenGL
typedef SlimVectorBase<float, 9> Float9; //!< a triple of float values => for OpenGL
typedef SlimVectorBase<float, 16> Float16; //!< a triple of float values => for OpenGL

//typedefs for std::array, used for consistent data transfer with python during time-critical functions such as userDefinedFunctions
typedef std::array<Real, 1> StdVector1D;
typedef std::array<Real, 2> StdVector2D;
typedef std::array<Real, 3> StdVector3D;
typedef std::array<Real, 4> StdVector4D;
//typedef std::array<Real, 5> StdVector5D;
typedef std::array<Real, 6> StdVector6D;

typedef std::array<StdVector3D, 3> StdMatrix3D;
typedef std::array<StdVector6D, 6> StdMatrix6D;

//this way would allow direct size check, but invalid size casted to std::vector<T>
//template<> inline SlimVectorBase<T, 2>::SlimVector(const T(&listOfItems)[2]) {
//	//static_assert(2 == listOfItems.size() && "ERROR: SlimVectorBase<T, 3>::constructor, initializer_list.size() must match template dataSize");
//	data[0] = listOfItems[0]; 
//	data[1] = listOfItems[1];
//};
//template<> inline SlimVectorBase<T, 3>::SlimVector(const T(&listOfItems)[3]) {
//	//static_assert(3 == listOfItems.size() && "ERROR: SlimVectorBase<T, 3>::constructor, initializer_list.size() must match template dataSize");
//	data[0] = listOfItems[0]; 
//	data[1] = listOfItems[1]; 
//	data[2] = listOfItems[2];
//	//approx. 3 times faster than in for-loop!
//};
//template<> inline SlimVectorBase<T, 4>::SlimVector(const T(&listOfItems)[4]) {
//	//static_assert(4 == listOfItems.size() && "ERROR: SlimVectorBase<T, 3>::constructor, initializer_list.size() must match template dataSize");
//	data[0] = listOfItems[0];
//	data[1] = listOfItems[1];
//	data[2] = listOfItems[2];
//	data[3] = listOfItems[3];
//};


//does not work with static assert in VS2017
template<> inline SlimVectorBase<Real, 1>::SlimVectorBase(std::initializer_list<Real> listOfReals) {
	CHECKandTHROW(1 == listOfReals.size(), "ERROR: SlimVectorBase<T, 1>::constructor, initializer_list.size() must match template dataSize");
	data[0] = listOfReals.begin()[0];
};
template<> inline SlimVectorBase<Real, 2>::SlimVectorBase(std::initializer_list<Real> listOfReals) {
	CHECKandTHROW(2 == listOfReals.size(), "ERROR: SlimVectorBase<T, 2>::constructor, initializer_list.size() must match template dataSize");
	data[0] = listOfReals.begin()[0]; 
	data[1] = listOfReals.begin()[1];
};
template<> inline SlimVectorBase<Real, 3>::SlimVectorBase(std::initializer_list<Real> listOfReals) {
	CHECKandTHROW(3 == listOfReals.size(), "ERROR: SlimVectorBase<T, 3>::constructor, initializer_list.size() must match template dataSize");
    data[0] = listOfReals.begin()[0]; 
	data[1] = listOfReals.begin()[1]; 
	data[2] = listOfReals.begin()[2];
    //approx. 3 times faster than in for-loop!
};
template<> inline SlimVectorBase<Real, 4>::SlimVectorBase(std::initializer_list<Real> listOfReals) {
	CHECKandTHROW(4 == listOfReals.size(), "ERROR: SlimVectorBase<T, 4>::constructor, initializer_list.size() must match template dataSize");
	data[0] = listOfReals.begin()[0];
	data[1] = listOfReals.begin()[1];
	data[2] = listOfReals.begin()[2];
	data[3] = listOfReals.begin()[3];
};




/*
//possible specializations for 3D vector:
//Returns the length of a vector
void Set(T x, T y, T z)
{
vec[0] = x; vec[1] = y; vec[2] = z;
}
void Get(T& x, T& y, T& z) //JG2012-01: this is for easier access to SlimVector data
{
x = vec[0]; y = vec[1]; z = vec[2];
}

void Scale(T x, T y, T z)
{
vec[0] /= x; vec[1] /= y; vec[2] /= z;
}
//normalizes *this and gets some orthogonal vectors n1 and n2
void SetNormalBasis(SlimVector& n1, SlimVector& n2)
{
Normalize();
SlimVector nx;
if (fabs(vec[0]) > 0.5 && fabs(vec[1]) < 0.1 && fabs(vec[2]) < 0.1) nx.Set(0., 1., 0.);
else nx.Set(1., 0., 0.);

Real h = nx*(*this);
n1 = nx - h*(*this);
n1.Normalize();
n2 = this->Cross(n1);
}

//Project n into normal plane of *this
void GramSchmidt(SlimVector& n) const
{
Real h = n*(*this) / ((*this)*(*this));
n -= h*(*this);
}

//Project n into normal plane of *this and normalize
void GramSchmidtNormalized(SlimVector& n) const
{
GramSchmidt(n);
n.Normalize();
}

//Returns the normalized planar perpendicular vector of a vector
SlimVector Cross(const SlimVector& v) const
{
return SlimVector(vec[1] * v.vec[2] - vec[2] * v.vec[1],
vec[2] * v.vec[0] - vec[0] * v.vec[2],
vec[0] * v.vec[1] - vec[1] * v.vec[0]);
}

*/

#endif
