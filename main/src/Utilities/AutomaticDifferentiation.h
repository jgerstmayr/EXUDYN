/** ***********************************************************************************************
* @file			AutomaticDifferentiation.h
* @brief		This file contains a class for simple automatic differentiation of functions
* @details		Details:
*               - this file is adapted from autodiff.hpp from Netgen/NGsolve (thanks to Joachim Schoeberl!)
* 				- do automatic differentiation by overloading a function with AutoDiff<nDiff, Real> 
*               - with nDiff being the number of differentiations
*
* @author		Joachim Schoeberl; adapted by Gerstmayr Johannes
* @date			2002-10-24 
* @copyright	This file is adapted from Netgen/NGsolve. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
*
*
************************************************************************************************ */


#ifndef AUTOMATICDIFFERENTIATION__H
#define AUTOMATICDIFFERENTIATION__H

namespace EXUmath 
{

	//! class for automatic differentiation.
	//! specify scalar data type and nDiff derivatives. 
	//! standard operations are overloaded by using product-rule etc. etc.
	template <int nDiff, typename TReal = Real>
	class AutoDiff
	{
		TReal val;
		TReal dval[nDiff ? nDiff : 1];
	public:

		typedef AutoDiff<nDiff, TReal> TELEM;
		typedef TReal TSCAL;

		/// elements are undefined
		// inline AutoDiff  () throw() { };
		AutoDiff() = default;
		// { val = 0; for (int i = 0; i < nDiff; i++) dval[i] = 0; }  // !

		/// copy constructor
		AutoDiff(const AutoDiff & ad2) = default;
		/*
		inline AutoDiff  (const AutoDiff & ad2) throw()
		{
		  val = ad2.val;
		  for (int i = 0; i < nDiff; i++)
			dval[i] = ad2.dval[i];
		}
		*/
		/// initial object with constant value
		inline AutoDiff(TReal aval) throw()
		{
			val = aval;
			for (int i = 0; i < nDiff; i++)
				dval[i] = 0;
		}

		/// init object with (val, e_diffindex)
		inline AutoDiff(TReal aval, int diffindex)  throw()
		{
			val = aval;
			for (int i = 0; i < nDiff; i++)
				dval[i] = 0;
			dval[diffindex] = 1;
		}

		inline AutoDiff(TReal aval, const TReal * grad)
		{
			val = aval;
			LoadGradient(grad);
		}

		/// assign constant value
		inline AutoDiff & operator= (TReal aval) throw()
		{
			val = aval;
			for (int i = 0; i < nDiff; i++)
				dval[i] = 0;
			return *this;
		}

		AutoDiff & operator= (const AutoDiff & ad2) = default;

		/// returns value
		inline TReal Value() const throw() { return val; }

		/// returns partial derivative
		inline TReal DValue(int i) const throw() { return dval[i]; }

		/// explicit cast operator, will cast (Real)DReal into real
		inline explicit operator TReal() const { return val; }

		///
		inline void StoreGradient(TReal * p) const
		{
			for (int i = 0; i < nDiff; i++)
				p[i] = dval[i];
		}

		inline void LoadGradient(const TReal * p)
		{
			for (int i = 0; i < nDiff; i++)
				dval[i] = p[i];
		}

		/// access value
		inline TReal & Value() throw() { return val; }

		/// accesses partial derivative 
		inline TReal & DValue(int i) throw() { return dval[i]; }
	};


	/// prints AutoDiff
	template<int nDiff, typename TReal>
	inline std::ostream & operator<< (std::ostream & ost, const AutoDiff<nDiff, TReal> & x)
	{
		ost << x.Value() << ", nDiff = ";
		for (int i = 0; i < nDiff; i++)
			ost << x.DValue(i) << " ";
		return ost;
	}

	/// AutoDiff plus AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> operator+ (const AutoDiff<nDiff, TReal> & x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x.Value() + y.Value();
		// AutoDiff<nDiff,TReal> res(x.Value()+y.Value());
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = x.DValue(i) + y.DValue(i);
		return res;
	}


	/// AutoDiff minus AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> operator- (const AutoDiff<nDiff, TReal> & x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x.Value() - y.Value();
		// AutoDiff<nDiff,TReal> res (x.Value()-y.Value());
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = x.DValue(i) - y.DValue(i);
		return res;
	}

	/// double plus AutoDiff
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator+ (SCAL2 x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x + y.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = y.DValue(i);
		return res;
	}

	/// AutoDiff plus double
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator+ (const AutoDiff<nDiff, TReal> & y, SCAL2 x) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x + y.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = y.DValue(i);
		return res;
	}


	/// minus AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> operator- (const AutoDiff<nDiff, TReal> & x) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = -x.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = -x.DValue(i);
		return res;
	}

	/// AutoDiff minus double
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator- (const AutoDiff<nDiff, TReal> & x, SCAL2 y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x.Value() - y;
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = x.DValue(i);
		return res;
	}

	///
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator- (SCAL2 x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x - y.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = -y.DValue(i);
		return res;
	}


	/// double times AutoDiff
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator* (SCAL2 x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x * y.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = x * y.DValue(i);
		return res;
	}

	/// AutoDiff times double
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>

		inline AutoDiff<nDiff, TReal> operator* (const AutoDiff<nDiff, TReal> & y, SCAL2 x) throw()
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = x * y.Value();
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = x * y.DValue(i);
		return res;
	}

	/// AutoDiff times AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> operator* (const AutoDiff<nDiff, TReal> & x, const AutoDiff<nDiff, TReal> & y) throw()
	{
		AutoDiff<nDiff, TReal> res;
		TReal hx = x.Value();
		TReal hy = y.Value();

		res.Value() = hx * hy;
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = hx * y.DValue(i) + hy * x.DValue(i);

		return res;
	}

	/// AutoDiff times AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> sqr(const AutoDiff<nDiff, TReal> & x) throw()
	{
		AutoDiff<nDiff, TReal> res;
		TReal hx = x.Value();
		res.Value() = hx * hx;
		hx *= 2;
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = hx * x.DValue(i);
		return res;
	}

	/// Inverse of AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> Inv(const AutoDiff<nDiff, TReal> & x)
	{
		AutoDiff<nDiff, TReal> res(1.0 / x.Value());
		for (int i = 0; i < nDiff; i++)
			res.DValue(i) = -x.DValue(i) / (x.Value() * x.Value());
		return res;
	}


	/// AutoDiff div AutoDiff
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> operator/ (const AutoDiff<nDiff, TReal> & x, const AutoDiff<nDiff, TReal> & y)
	{
		return x * Inv(y);
	}

	/// AutoDiff div double
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator/ (const AutoDiff<nDiff, TReal> & x, SCAL2 y)
	{
		return (1.0 / y) * x;
	}

	/// double div AutoDiff
	template<int nDiff, typename TReal, typename SCAL2,
		typename std::enable_if<std::is_convertible<SCAL2, TReal>::value, int>::type = 0>
		inline AutoDiff<nDiff, TReal> operator/ (SCAL2 x, const AutoDiff<nDiff, TReal> & y)
	{
		return x * Inv(y);
	}




	template <int nDiff, typename TReal, typename SCAL2>
	inline AutoDiff<nDiff, TReal> & operator+= (AutoDiff<nDiff, TReal> & x, SCAL2 y) throw()
	{
		x.Value() += y;
		return x;
	}


	/// 
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> & operator+= (AutoDiff<nDiff, TReal> & x, AutoDiff<nDiff, TReal> y)
	{
		x.Value() += y.Value();
		for (int i = 0; i < nDiff; i++)
			x.DValue(i) += y.DValue(i);
		return x;
	}

	///
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> & operator-= (AutoDiff<nDiff, TReal> & x, AutoDiff<nDiff, TReal> y)
	{
		x.Value() -= y.Value();
		for (int i = 0; i < nDiff; i++)
			x.DValue(i) -= y.DValue(i);
		return x;

	}

	template <int nDiff, typename TReal, typename SCAL2>
	inline AutoDiff<nDiff, TReal> & operator-= (AutoDiff<nDiff, TReal> & x, SCAL2 y)
	{
		x.Value() -= y;
		return x;
	}

	///
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> & operator*= (AutoDiff<nDiff, TReal> & x, AutoDiff<nDiff, TReal> y)
	{
		for (int i = 0; i < nDiff; i++)
			x.DValue(i) = x.DValue(i)*y.Value() + x.Value() * y.DValue(i);
		x.Value() *= y.Value();
		return x;
	}

	///
	template <int nDiff, typename TReal, typename SCAL2>
	inline AutoDiff<nDiff, TReal> & operator*= (AutoDiff<nDiff, TReal> & x, SCAL2 y)
	{
		x.Value() *= y;
		for (int i = 0; i < nDiff; i++)
			x.DValue(i) *= y;
		return x;
	}

	///
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> & operator/= (AutoDiff<nDiff, TReal> & x, TReal y)
	{
		TReal iy = 1.0 / y;
		x.Value() *= iy;
		for (int i = 0; i < nDiff; i++)
			x.DValue(i) *= iy;
		return x;
	}




	/// 
	template <int nDiff, typename TReal>
	inline bool operator== (AutoDiff<nDiff, TReal> x, TReal val2)
	{
		return x.Value() == val2;
	}

	///
	template <int nDiff, typename TReal>
	inline bool operator!= (AutoDiff<nDiff, TReal> x, TReal val2) throw()
	{
		return x.Value() != val2;
	}

	///
	template <int nDiff, typename TReal>
	inline bool operator< (AutoDiff<nDiff, TReal> x, TReal val2) throw()
	{
		return x.Value() < val2;
	}

	///
	template <int nDiff, typename TReal>
	inline bool operator> (AutoDiff<nDiff, TReal> x, TReal val2) throw()
	{
		return x.Value() > val2;
	}




	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> fabs(const AutoDiff<nDiff, TReal> & x)
	{
		double abs = fabs(x.Value());
		AutoDiff<nDiff, TReal> res(abs);
		if (abs != 0.0)
			for (int i = 0; i < nDiff; i++)
				res.DValue(i) = x.Value()*x.DValue(i) / abs;
		else
			for (int i = 0; i < nDiff; i++)
				res.DValue(i) = 0.0;
		return res;
	}

	using std::sqrt;
	template<int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> sqrt(const AutoDiff<nDiff, TReal> & x)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = sqrt(x.Value());
		for (int j = 0; j < nDiff; j++)
			res.DValue(j) = 0.5 / res.Value() * x.DValue(j);
		return res;
	}

	using std::log;
	template <int nDiff, typename TReal>
	AutoDiff<nDiff, TReal> log(AutoDiff<nDiff, TReal> x)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = log(x.Value());
		for (int k = 0; k < nDiff; k++)
			res.DValue(k) = x.DValue(k) / x.Value();
		return res;
	}

	using std::exp;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> exp(AutoDiff<nDiff, TReal> x)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = exp(x.Value());
		for (int k = 0; k < nDiff; k++)
			res.DValue(k) = x.DValue(k) * res.Value();
		return res;
	}

	using std::pow;
	template <int nDiff, typename TReal, typename SCAL2>
	inline AutoDiff<nDiff, TReal> pow(AutoDiff<nDiff, TReal> x, SCAL2 exponent)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = pow(x.Value(), exponent);
		TReal base_pow_exponent_minus_one = std::pow(x.Value(), exponent - 1);

		for (int k = 0; k < nDiff; ++k)
			res.DValue(k) = exponent * base_pow_exponent_minus_one * x.DValue(k);

		return res;
	}

	using std::pow;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> pow(const AutoDiff<nDiff, TReal>& x, const AutoDiff<nDiff, TReal>& y)
	{
		CHECKandTHROW(x.Value() > 0, "AutoDiff::pow: Base x must be positive for general exponent y.");

		AutoDiff<nDiff, TReal> res;
		res.Value() = pow(x.Value(), y.Value());

		TReal x_pow_y_minus_one = pow(x.Value(), y.Value() - 1);
		TReal x_pow_y_ln_x = res.Value() * std::log(x.Value());

		for (int i = 0; i < nDiff; ++i) {
			res.DValue(i) = y.DValue(i) * x_pow_y_ln_x + y.Value() * x_pow_y_minus_one * x.DValue(i);
		}

		return res;
	}

	using std::sin;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> sin(AutoDiff<nDiff, TReal> x)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = sin(x.Value());
		TReal c = cos(x.Value());
		for (int k = 0; k < nDiff; k++)
			res.DValue(k) = x.DValue(k) * c;
		return res;
	}

	using std::cos;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> cos(AutoDiff<nDiff, TReal> x)
	{
		AutoDiff<nDiff, TReal> res;
		res.Value() = cos(x.Value());
		TReal ms = -sin(x.Value());
		for (int k = 0; k < nDiff; k++)
			res.DValue(k) = x.DValue(k) * ms;
		return res;
	}

	using std::tan;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> tan(AutoDiff<nDiff, TReal> x)
	{
		return sin(x) / cos(x);
	}

	using std::atan;
	template <int nDiff, typename TReal>
	inline AutoDiff<nDiff, TReal> atan(AutoDiff<nDiff, TReal> x)
	{
		AutoDiff<nDiff> res;
		double a = atan(x.Value());
		res.Value() = a;
		for (int k = 0; k < nDiff; k++)
			res.DValue(k) = x.DValue(k) / (1 + x.Value()*x.Value());
		return res;
	}




	template <int nDiff, typename TReal, typename TB, typename TC>
	auto IfPos(AutoDiff<nDiff, TReal> a, TB b, TC c) -> decltype(IfPos(a.Value(), b, c))
	{
		return IfPos(a.Value(), b, c);
	}

	//template <int nDiff, typename TReal>
	//inline AutoDiff<nDiff, TReal> IfPos(SIMD<double> a, AutoDiff<nDiff, TReal> b, AutoDiff<nDiff, TReal> c)
	//{
	//	AutoDiff<nDiff, TReal> res;
	//	res.Value() = IfPos(a, b.Value(), c.Value());
	//	for (int j = 0; j < nDiff; j++)
	//		res.DValue(j) = IfPos(a, b.DValue(j), c.DValue(j));
	//	return res;
	//}

	//template <int nDiff, typename TReal, typename TC>
	//inline AutoDiff<nDiff, TReal> IfPos(SIMD<double> a, AutoDiff<nDiff, TReal> b, TC c)
	//{
	//	return IfPos(a, b, AutoDiff<nDiff, TReal>(c));
	//}

}; //EXUmath


#endif
