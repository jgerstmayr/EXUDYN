
/**************************************************************************/
/* File:   autodiff.hpp                                                   */
/* Author: Joachim Schoeberl                                              */
/* Date:   24. Oct. 02                                                    */
/**************************************************************************/

//This file is taken from a special ngs-core_master project provided thanks to Joachim Sch�berl!
//this file has been modified in order to meet the requirements for EXUDYN and is currently in an evaluation mode

#pragma once

#ifndef INLINE
#define INLINE inline
#endif

namespace ngstd
{

	// Automatic differentiation datatype


	/**
	   Datatype for automatic differentiation.
	   Contains function value and D derivatives. Algebraic
	   operations are overloaded by using product-rule etc. etc.
	**/
	template <int D, typename SCAL = double>
	class AutoDiff
	{
		SCAL val;
		SCAL dval[D ? D : 1];
	public:

		typedef AutoDiff<D, SCAL> TELEM;
		typedef SCAL TSCAL;


		/// elements are undefined
		// INLINE AutoDiff  () throw() { };
		AutoDiff() = default;
		// { val = 0; for (int i = 0; i < D; i++) dval[i] = 0; }  // !

		/// copy constructor
		AutoDiff(const AutoDiff & ad2) = default;
		/*
		INLINE AutoDiff  (const AutoDiff & ad2) throw()
		{
		  val = ad2.val;
		  for (int i = 0; i < D; i++)
			dval[i] = ad2.dval[i];
		}
		*/
		/// initial object with constant value
		INLINE AutoDiff(SCAL aval) throw()
		{
			val = aval;
			for (int i = 0; i < D; i++)
				dval[i] = 0;
		}

		/// init object with (val, e_diffindex)
		INLINE AutoDiff(SCAL aval, int diffindex)  throw()
		{
			val = aval;
			for (int i = 0; i < D; i++)
				dval[i] = 0;
			dval[diffindex] = 1;
		}

		INLINE AutoDiff(SCAL aval, const SCAL * grad)
		{
			val = aval;
			LoadGradient(grad);
		}

		/// assign constant value
		INLINE AutoDiff & operator= (SCAL aval) throw()
		{
			val = aval;
			for (int i = 0; i < D; i++)
				dval[i] = 0;
			return *this;
		}

		AutoDiff & operator= (const AutoDiff & ad2) = default;

		/// returns value
		INLINE SCAL Value() const throw() { return val; }

		/// returns partial derivative
		INLINE SCAL DValue(int i) const throw() { return dval[i]; }

		///
		INLINE void StoreGradient(SCAL * p) const
		{
			for (int i = 0; i < D; i++)
				p[i] = dval[i];
		}

		INLINE void LoadGradient(const SCAL * p)
		{
			for (int i = 0; i < D; i++)
				dval[i] = p[i];
		}

		/// access value
		INLINE SCAL & Value() throw() { return val; }

		/// accesses partial derivative 
		INLINE SCAL & DValue(int i) throw() { return dval[i]; }
	};


	/// prints AutoDiff
	template<int D, typename SCAL>
	inline std::ostream & operator<< (std::ostream & ost, const AutoDiff<D, SCAL> & x)
	{
		ost << x.Value() << ", D = ";
		for (int i = 0; i < D; i++)
			ost << x.DValue(i) << " ";
		return ost;
	}

	/// AutoDiff plus AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> operator+ (const AutoDiff<D, SCAL> & x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x.Value() + y.Value();
		// AutoDiff<D,SCAL> res(x.Value()+y.Value());
		for (int i = 0; i < D; i++)
			res.DValue(i) = x.DValue(i) + y.DValue(i);
		return res;
	}


	/// AutoDiff minus AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> operator- (const AutoDiff<D, SCAL> & x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x.Value() - y.Value();
		// AutoDiff<D,SCAL> res (x.Value()-y.Value());
		for (int i = 0; i < D; i++)
			res.DValue(i) = x.DValue(i) - y.DValue(i);
		return res;
	}

	/// double plus AutoDiff
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator+ (SCAL2 x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x + y.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = y.DValue(i);
		return res;
	}

	/// AutoDiff plus double
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator+ (const AutoDiff<D, SCAL> & y, SCAL2 x) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x + y.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = y.DValue(i);
		return res;
	}


	/// minus AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> operator- (const AutoDiff<D, SCAL> & x) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = -x.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = -x.DValue(i);
		return res;
	}

	/// AutoDiff minus double
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator- (const AutoDiff<D, SCAL> & x, SCAL2 y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x.Value() - y;
		for (int i = 0; i < D; i++)
			res.DValue(i) = x.DValue(i);
		return res;
	}

	///
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator- (SCAL2 x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x - y.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = -y.DValue(i);
		return res;
	}


	/// double times AutoDiff
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator* (SCAL2 x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x * y.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = x * y.DValue(i);
		return res;
	}

	/// AutoDiff times double
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>

		INLINE AutoDiff<D, SCAL> operator* (const AutoDiff<D, SCAL> & y, SCAL2 x) throw()
	{
		AutoDiff<D, SCAL> res;
		res.Value() = x * y.Value();
		for (int i = 0; i < D; i++)
			res.DValue(i) = x * y.DValue(i);
		return res;
	}

	/// AutoDiff times AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> operator* (const AutoDiff<D, SCAL> & x, const AutoDiff<D, SCAL> & y) throw()
	{
		AutoDiff<D, SCAL> res;
		SCAL hx = x.Value();
		SCAL hy = y.Value();

		res.Value() = hx * hy;
		for (int i = 0; i < D; i++)
			res.DValue(i) = hx * y.DValue(i) + hy * x.DValue(i);

		return res;
	}

	/// AutoDiff times AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> sqr(const AutoDiff<D, SCAL> & x) throw()
	{
		AutoDiff<D, SCAL> res;
		SCAL hx = x.Value();
		res.Value() = hx * hx;
		hx *= 2;
		for (int i = 0; i < D; i++)
			res.DValue(i) = hx * x.DValue(i);
		return res;
	}

	/// Inverse of AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> Inv(const AutoDiff<D, SCAL> & x)
	{
		AutoDiff<D, SCAL> res(1.0 / x.Value());
		for (int i = 0; i < D; i++)
			res.DValue(i) = -x.DValue(i) / (x.Value() * x.Value());
		return res;
	}


	/// AutoDiff div AutoDiff
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> operator/ (const AutoDiff<D, SCAL> & x, const AutoDiff<D, SCAL> & y)
	{
		return x * Inv(y);
	}

	/// AutoDiff div double
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator/ (const AutoDiff<D, SCAL> & x, SCAL2 y)
	{
		return (1.0 / y) * x;
	}

	/// double div AutoDiff
	template<int D, typename SCAL, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, SCAL>::value, int>::type = 0>
		INLINE AutoDiff<D, SCAL> operator/ (SCAL2 x, const AutoDiff<D, SCAL> & y)
	{
		return x * Inv(y);
	}




	template <int D, typename SCAL, typename SCAL2>
	INLINE AutoDiff<D, SCAL> & operator+= (AutoDiff<D, SCAL> & x, SCAL2 y) throw()
	{
		x.Value() += y;
		return x;
	}


	/// 
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> & operator+= (AutoDiff<D, SCAL> & x, AutoDiff<D, SCAL> y)
	{
		x.Value() += y.Value();
		for (int i = 0; i < D; i++)
			x.DValue(i) += y.DValue(i);
		return x;
	}

	///
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> & operator-= (AutoDiff<D, SCAL> & x, AutoDiff<D, SCAL> y)
	{
		x.Value() -= y.Value();
		for (int i = 0; i < D; i++)
			x.DValue(i) -= y.DValue(i);
		return x;

	}

	template <int D, typename SCAL, typename SCAL2>
	INLINE AutoDiff<D, SCAL> & operator-= (AutoDiff<D, SCAL> & x, SCAL2 y)
	{
		x.Value() -= y;
		return x;
	}

	///
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> & operator*= (AutoDiff<D, SCAL> & x, AutoDiff<D, SCAL> y)
	{
		for (int i = 0; i < D; i++)
			x.DValue(i) = x.DValue(i)*y.Value() + x.Value() * y.DValue(i);
		x.Value() *= y.Value();
		return x;
	}

	///
	template <int D, typename SCAL, typename SCAL2>
	INLINE AutoDiff<D, SCAL> & operator*= (AutoDiff<D, SCAL> & x, SCAL2 y)
	{
		x.Value() *= y;
		for (int i = 0; i < D; i++)
			x.DValue(i) *= y;
		return x;
	}

	///
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> & operator/= (AutoDiff<D, SCAL> & x, SCAL y)
	{
		SCAL iy = 1.0 / y;
		x.Value() *= iy;
		for (int i = 0; i < D; i++)
			x.DValue(i) *= iy;
		return x;
	}




	/// 
	template <int D, typename SCAL>
	INLINE bool operator== (AutoDiff<D, SCAL> x, SCAL val2)
	{
		return x.Value() == val2;
	}

	///
	template <int D, typename SCAL>
	INLINE bool operator!= (AutoDiff<D, SCAL> x, SCAL val2) throw()
	{
		return x.Value() != val2;
	}

	///
	template <int D, typename SCAL>
	INLINE bool operator< (AutoDiff<D, SCAL> x, SCAL val2) throw()
	{
		return x.Value() < val2;
	}

	///
	template <int D, typename SCAL>
	INLINE bool operator> (AutoDiff<D, SCAL> x, SCAL val2) throw()
	{
		return x.Value() > val2;
	}




	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> fabs(const AutoDiff<D, SCAL> & x)
	{
		double abs = fabs(x.Value());
		AutoDiff<D, SCAL> res(abs);
		if (abs != 0.0)
			for (int i = 0; i < D; i++)
				res.DValue(i) = x.Value()*x.DValue(i) / abs;
		else
			for (int i = 0; i < D; i++)
				res.DValue(i) = 0.0;
		return res;
	}

	using std::sqrt;
	template<int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> sqrt(const AutoDiff<D, SCAL> & x)
	{
		AutoDiff<D, SCAL> res;
		res.Value() = sqrt(x.Value());
		for (int j = 0; j < D; j++)
			res.DValue(j) = 0.5 / res.Value() * x.DValue(j);
		return res;
	}

	using std::log;
	template <int D, typename SCAL>
	AutoDiff<D, SCAL> log(AutoDiff<D, SCAL> x)
	{
		AutoDiff<D, SCAL> res;
		res.Value() = log(x.Value());
		for (int k = 0; k < D; k++)
			res.DValue(k) = x.DValue(k) / x.Value();
		return res;
	}

	using std::exp;
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> exp(AutoDiff<D, SCAL> x)
	{
		AutoDiff<D, SCAL> res;
		res.Value() = exp(x.Value());
		for (int k = 0; k < D; k++)
			res.DValue(k) = x.DValue(k) * res.Value();
		return res;
	}

	using std::sin;
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> sin(AutoDiff<D, SCAL> x)
	{
		AutoDiff<D, SCAL> res;
		res.Value() = sin(x.Value());
		SCAL c = cos(x.Value());
		for (int k = 0; k < D; k++)
			res.DValue(k) = x.DValue(k) * c;
		return res;
	}

	using std::cos;
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> cos(AutoDiff<D, SCAL> x)
	{
		AutoDiff<D, SCAL> res;
		res.Value() = cos(x.Value());
		SCAL ms = -sin(x.Value());
		for (int k = 0; k < D; k++)
			res.DValue(k) = x.DValue(k) * ms;
		return res;
	}

	using std::tan;
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> tan(AutoDiff<D, SCAL> x)
	{
		return sin(x) / cos(x);
	}

	using std::atan;
	template <int D, typename SCAL>
	INLINE AutoDiff<D, SCAL> atan(AutoDiff<D, SCAL> x)
	{
		AutoDiff<D> res;
		double a = atan(x.Value());
		res.Value() = a;
		for (int k = 0; k < D; k++)
			res.DValue(k) = x.DValue(k) / (1 + x.Value()*x.Value());
		return res;
	}




	template <int D, typename SCAL, typename TB, typename TC>
	auto IfPos(AutoDiff<D, SCAL> a, TB b, TC c) -> decltype(IfPos(a.Value(), b, c))
	{
		return IfPos(a.Value(), b, c);
	}

	//template <int D, typename SCAL>
	//INLINE AutoDiff<D, SCAL> IfPos(SIMD<double> a, AutoDiff<D, SCAL> b, AutoDiff<D, SCAL> c)
	//{
	//	AutoDiff<D, SCAL> res;
	//	res.Value() = IfPos(a, b.Value(), c.Value());
	//	for (int j = 0; j < D; j++)
	//		res.DValue(j) = IfPos(a, b.DValue(j), c.DValue(j));
	//	return res;
	//}

	//template <int D, typename SCAL, typename TC>
	//INLINE AutoDiff<D, SCAL> IfPos(SIMD<double> a, AutoDiff<D, SCAL> b, TC c)
	//{
	//	return IfPos(a, b, AutoDiff<D, SCAL>(c));
	//}

}; //ngstd
