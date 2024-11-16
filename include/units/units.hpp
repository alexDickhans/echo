// "RQuantity.h" header file

#pragma once

#include <ratio>
#include <math.h>

// The "RQuantity" class is the prototype template container class, that just holds a float value. The
// class SHOULD NOT BE INSTANTIATED directly by itself, rather use the quantity types defined below.
template<typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim>
class RQuantity
{
private:

public:
	float value;
	constexpr RQuantity() : value(0.0) {}
	constexpr RQuantity(float val) : value(val) {}

	// The intrinsic operations for a quantity with a unit is addition and subtraction
	constexpr RQuantity const& operator+=(const RQuantity& rhs)
	{
		value += rhs.value;
		return *this;
	}
	constexpr RQuantity const& operator-=(const RQuantity& rhs)
	{
		value -= rhs.value;
		return *this;
	}

	constexpr RQuantity const& operator-() {
		value = -value;

		return *this;
	}

	constexpr bool const& operator!=(RQuantity rhs) {
		return this->getValue() != rhs.getValue();
	}

	// Returns the value of the quantity in multiples of the specified unit
	[[nodiscard]] constexpr float Convert(const RQuantity& rhs) const
	{
		return value / rhs.value;
	}

	// returns the raw value of the quantity (should not be used)
	[[nodiscard]] constexpr float getValue() const
	{
		return value;
	}
};


// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, name) \
    typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>> name;

#define LEGACY_TYPEDEF(old_name, new_name) \
    using old_name [[deprecated("use " #new_name " instead")]] = new_name

// Replacement of "float" type
QUANTITY_TYPE(0, 0, 0, 0, Number);

// Physical quantity types
QUANTITY_TYPE(1, 0, 0, 0, QMass);
QUANTITY_TYPE(0, 1, 0, 0, QLength);
QUANTITY_TYPE(0, 2, 0, 0, QArea);
QUANTITY_TYPE(0, 3, 0, 0, QVolume);
QUANTITY_TYPE(0, 0, 1, 0, QTime);
QUANTITY_TYPE(0, 1, -1, 0, QVelocity);
LEGACY_TYPEDEF(QSpeed, QVelocity);
QUANTITY_TYPE(0, 1, -2, 0, QAcceleration);
QUANTITY_TYPE(0, 1, -3, 0, QJerk);
QUANTITY_TYPE(0, 0, -1, 0, QFrequency);
QUANTITY_TYPE(1, 1, -2, 0, QForce);
QUANTITY_TYPE(1, -1, -2, 0, QPressure);
QUANTITY_TYPE(0, -1, 0, 1, QCurvature);
QUANTITY_TYPE(0, 1, 0, -1, QRadius);
QUANTITY_TYPE(0, 0, -1, 1, QAngularVelocity);
QUANTITY_TYPE(0, 0, -2, 1, QAngularAcceleration);
//QUANTITY_TYPE()

// Angle type:
QUANTITY_TYPE(0, 0, 0, 1, Angle);


// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A>
operator+(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return RQuantity<M, L, T, A>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A>
operator-(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return RQuantity<M, L, T, A>(lhs.getValue() - rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1,
	typename M2, typename L2, typename T2, typename A2>
constexpr RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>,
	std::ratio_add<T1, T2>, std::ratio_add<A1, A2>>
	operator*(const RQuantity<M1, L1, T1, A1>& lhs, const RQuantity<M2, L2, T2, A2>& rhs)
{
	return RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>,
		std::ratio_add<T1, T2>, std::ratio_add<A1, A2>>
		(lhs.getValue() * rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A>
operator*(const float& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return RQuantity<M, L, T, A>(lhs * rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1,
	typename M2, typename L2, typename T2, typename A2>
constexpr RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
	std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>>
	operator/(const RQuantity<M1, L1, T1, A1>& lhs, const RQuantity<M2, L2, T2, A2>& rhs)
{
	return RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
		std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>>
		(lhs.getValue() / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
	std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>>
	operator/(long double x, const RQuantity<M, L, T, A>& rhs)
{
	return RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
		std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>>
		(x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A>
operator/(const RQuantity<M, L, T, A>& rhs, long double x)
{
	return RQuantity<M, L, T, A>(rhs.getValue() / x);
}


// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A>
constexpr bool operator==(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator!=(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<=(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>=(const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator< (const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() < rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator> (const RQuantity<M, L, T, A>& lhs, const RQuantity<M, L, T, A>& rhs)
{
	return (lhs.getValue() > rhs.getValue());
}


// Predefined units:
// -----------------

// Predefined mass units:
constexpr QMass kg(1.0);                            // SI base unit
constexpr QMass gramme = 0.001 * kg;
constexpr QMass tonne = 1000 * kg;
constexpr QMass ounce = 0.028349523125 * kg;
constexpr QMass pound = 16 * ounce;
constexpr QMass stone = 14 * pound;

// Predefined length-derived units
constexpr QLength metre(1.0);                   // SI base unit
constexpr QLength decimetre = metre / 10;
constexpr QLength centimetre = metre / 100;
constexpr QLength millimetre = metre / 1000;
constexpr QLength kilometre = 1000 * metre;
constexpr QLength inch = 2.54 * centimetre;
constexpr QLength foot = 12 * inch;
constexpr QLength yard = 3 * foot;
constexpr QLength mile = 5280 * foot;

constexpr QArea kilometre2 = kilometre * kilometre;
constexpr QArea metre2 = metre * metre;
constexpr QArea decimetre2 = decimetre * decimetre;
constexpr QArea centimetre2 = centimetre * centimetre;
constexpr QArea millimetre2 = millimetre * millimetre;
constexpr QArea inch2 = inch * inch;
constexpr QArea foot2 = foot * foot;
constexpr QArea mile2 = mile * mile;

constexpr QVolume kilometre3 = kilometre2 * kilometre;
constexpr QVolume metre3 = metre2 * metre;
constexpr QVolume decimetre3 = decimetre2 * decimetre;
constexpr QVolume litre = decimetre3;
constexpr QVolume centimetre3 = centimetre2 * centimetre;
constexpr QVolume millimetre3 = millimetre2 * millimetre;
constexpr QVolume inch3 = inch2 * inch;
constexpr QVolume foot3 = foot2 * foot;
constexpr QVolume mile3 = mile2 * mile;

// Predefined time-derived units:
constexpr QTime second(1.0);                        // SI base unit
constexpr QTime millisecond = second / 1000;
constexpr QTime minute = 60 * second;
constexpr QTime hour = 60 * minute;
constexpr QTime day = 24 * hour;
constexpr QTime year = 365 * day;

constexpr QFrequency Hz(1.0);

// Predefined mixed units:
constexpr QAcceleration G = 9.80665 * metre / (second * second);
constexpr QAcceleration inchs2 = inch / (second * second);

constexpr QForce newton(1.0);
constexpr QForce poundforce = pound * G;
constexpr QForce kilopond = kg * G;

constexpr QPressure Pascal(1.0);

#ifndef SIM
constexpr QPressure bar = 100000 * Pascal;
#endif // !SIM

constexpr QPressure psi = pound * G / inch2;

// Physical unit literals:
// -----------------------

// literals for length units
constexpr QLength operator"" _mm(long double x) { return static_cast<float>(x) * millimetre; }
constexpr QLength operator"" _cm(long double x) { return static_cast<float>(x) * centimetre; }
constexpr QLength operator"" _m(long double x) { return static_cast<float>(x) * metre; }
constexpr QLength operator"" _km(long double x) { return static_cast<float>(x) * kilometre; }
constexpr QLength operator"" _mi(long double x) { return static_cast<float>(x) * mile; }
constexpr QLength operator"" _yd(long double x) { return static_cast<float>(x) * yard; }
constexpr QLength operator"" _ft(long double x) { return static_cast<float>(x) * foot; }
constexpr QLength operator"" _in(long double x) { return static_cast<float>(x) * inch; }
constexpr QLength operator"" _mm(unsigned long long int x) { return static_cast<float>(x) * millimetre; }
constexpr QLength operator"" _cm(unsigned long long int  x) { return static_cast<float>(x) * centimetre; }
constexpr QLength operator"" _m(unsigned long long int  x) { return static_cast<float>(x) * metre; }
constexpr QLength operator"" _km(unsigned long long int  x) { return static_cast<float>(x) * kilometre; }
constexpr QLength operator"" _mi(unsigned long long int  x) { return static_cast<float>(x) * mile; }
constexpr QLength operator"" _yd(unsigned long long int  x) { return static_cast<float>(x) * yard; }
constexpr QLength operator"" _ft(unsigned long long int  x) { return static_cast<float>(x) * foot; }
constexpr QLength operator"" _in(unsigned long long int  x) { return static_cast<float>(x) * inch; }

// literals for speed units
constexpr QVelocity operator"" _mps(long double x) { return x; };
constexpr QVelocity operator"" _inchs(long double x) { return static_cast<float>(x) * inch / second; };
constexpr QVelocity operator"" _inchs(unsigned long long int x) { return static_cast<float>(x) * inch / second; };
constexpr QVelocity operator"" _miph(long double x) { return static_cast<float>(x) * mile / hour; };
constexpr QVelocity operator"" _kmph(long double x) { return static_cast<float>(x) * kilometre / hour; };
constexpr QVelocity operator"" _mps(unsigned long long int x) { return {static_cast<float>(x)}; };
constexpr QVelocity operator"" _miph(unsigned long long int x)
{
	return static_cast<float>(x) * mile / hour;
};
constexpr QVelocity operator"" _kmph(unsigned long long int x)
{
	return static_cast<float>(x) * kilometre / hour;
};

// literal for frequency unit
constexpr QFrequency operator"" _Hz(long double x) { return QFrequency(x); };
constexpr QFrequency operator"" _Hz(unsigned long long int x)
{
	return QFrequency(static_cast<float>(x));
};

// literals for time units
constexpr QTime operator"" _s(long double x) { return QTime(x); };
constexpr QTime operator"" _ms(long double x) { return static_cast<float>(x) * millisecond; };
constexpr QTime operator"" _min(long double x) { return static_cast<float>(x) * minute; };
constexpr QTime operator"" _h(long double x) { return static_cast<float>(x) * hour; };
constexpr QTime operator"" _day(long double x) { return static_cast<float>(x) * day; };
constexpr QTime operator"" _s(unsigned long long int x) { return QTime(static_cast<float>(x)); };
constexpr QTime operator"" _ms(unsigned long long int x) { return static_cast<float>(x) * millisecond; };
constexpr QTime operator"" _min(unsigned long long int x) { return static_cast<float>(x) * minute; };
constexpr QTime operator"" _h(unsigned long long int x) { return static_cast<float>(x) * hour; };
constexpr QTime operator"" _day(unsigned long long int x) { return static_cast<float>(x) * day; };

// literals for mass units
constexpr QMass operator"" _kg(long double x) { return QMass(x); };
constexpr QMass operator"" _g(long double x) { return static_cast<float>(x) * gramme; };
constexpr QMass operator"" _t(long double x) { return static_cast<float>(x) * tonne; };
constexpr QMass operator"" _oz(long double x) { return static_cast<float>(x) * ounce; };
constexpr QMass operator"" _lb(long double x) { return static_cast<float>(x) * pound; };
constexpr QMass operator"" _st(long double x) { return static_cast<float>(x) * stone; };
constexpr QMass operator"" _kg(unsigned long long int x) { return QMass(static_cast<float>(x)); };
constexpr QMass operator"" _g(unsigned long long int x) { return static_cast<float>(x) * gramme; };
constexpr QMass operator"" _t(unsigned long long int x) { return static_cast<float>(x) * tonne; };
constexpr QMass operator"" _oz(unsigned long long int x) { return static_cast<float>(x) * ounce; };
constexpr QMass operator"" _lb(unsigned long long int x) { return static_cast<float>(x) * pound; };
constexpr QMass operator"" _st(unsigned long long int x) { return static_cast<float>(x) * stone; };

// literals for acceleration units
constexpr QAcceleration operator"" _mps2(long double x) { return QAcceleration(x); };
constexpr QAcceleration operator"" _mps2(unsigned long long int x) { return QAcceleration(static_cast<float>(x)); };
constexpr QAcceleration operator"" _inchs2(long double x) { return static_cast<float>(x) * inchs2; };
constexpr QAcceleration operator"" _inchs2(unsigned long long int x) { return static_cast<float>(x) * inchs2; };
constexpr QAcceleration operator"" _G(long double x) { return static_cast<float>(x) * G; };
constexpr QAcceleration operator"" _G(unsigned long long int x) { return static_cast<float>(x) * G; }

// literals for force units
constexpr QForce operator"" _Newton(long double x) { return QForce(x); };
constexpr QForce operator"" _Newton(unsigned long long int x) { return QForce(static_cast<float>(x)); };
constexpr QForce operator"" _lbf(long double x) { return static_cast<float>(x) * poundforce; };
constexpr QForce operator"" _lbf(unsigned long long int x) { return static_cast<float>(x) * poundforce; };
constexpr QForce operator"" _kp(long double x) { return static_cast<float>(x) * kilopond; };
constexpr QForce operator"" _kp(unsigned long long int x) { return static_cast<float>(x) * kilopond; };

// literals for pressure units
constexpr QPressure operator"" _Pa(long double x) { return QPressure(x); };
constexpr QPressure operator"" _Pa(unsigned long long int x)
{
	return QPressure(static_cast<float>(x));
};
#ifndef SIM
constexpr QPressure operator"" _bar(long double x) { return static_cast<float>(x) * bar; };
constexpr QPressure operator"" _bar(unsigned long long int x) { return static_cast<float>(x) * bar; };
#endif // !SIM
constexpr QPressure operator"" _psi(long double x) { return static_cast<float>(x) * psi; };
constexpr QPressure operator"" _psi(unsigned long long int x) { return static_cast<float>(x) * psi; };

// Angular unit literals:
// ----------------------
constexpr float operator"" _pi(long double x)
{
	return static_cast<float>(x) * 3.1415926535897932384626433832795;
}
constexpr float operator"" _pi(unsigned long long int x)
{
	return static_cast<float>(x) * 3.1415926535897932384626433832795;
}

// Predefined angle units:
constexpr Angle radian(1.0);
constexpr Angle revolution = 2_pi;
constexpr Angle degree = 1_pi / 180.0 * radian;

// literals for angle units
constexpr Angle operator"" _rad(long double x) { return Angle(x); };
constexpr Angle operator"" _rad(unsigned long long int x) { return Angle(static_cast<float>(x)); };
constexpr Angle operator"" _deg(long double x) { return static_cast<float>(x) * degree; };
constexpr Angle operator"" _deg(unsigned long long int x) { return static_cast<float>(x) * degree; };

constexpr QCurvature RadM(1.0);
constexpr QCurvature DegM = degree/metre;
constexpr QCurvature RadIn = degree/inch; 
constexpr QCurvature DegIn = degree/inch; 

constexpr QCurvature operator"" _radm(long double x) { return static_cast<float>(x); };
constexpr QCurvature operator"" _radm(unsigned long long int x) { return static_cast<float>(x); };
constexpr QCurvature operator"" _degm(long double x) { return static_cast<float>(x) * DegM; };
constexpr QCurvature operator"" _degm(unsigned long long int x) { return static_cast<float>(x) * DegM; };
constexpr QCurvature operator"" _radin(long double x) { return static_cast<float>(x) * RadIn; };
constexpr QCurvature operator"" _radin(unsigned long long int x) { return static_cast<float>(x) * RadIn; };
constexpr QCurvature operator"" _degin(long double x) { return static_cast<float>(x) * DegIn; };
constexpr QCurvature operator"" _degin(unsigned long long int x) { return static_cast<float>(x) * DegIn; };

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).Convert(1.0_##_y)

// Typesafe mathematical operations:
// ---------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
	std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>>
	Qsqrt(const RQuantity<M, L, T, A>& num)
{
	return RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
		std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>>
		(sqrt(num.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A>
Qabs(const RQuantity<M, L, T, A>& num)
{
	return RQuantity<M, L, T, A>
			(fabs(num.getValue()));
}

// Typesafe mathematical operations:
// ---------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<1, 2>>, std::ratio_divide<L, std::ratio<1, 2>>,
	std::ratio_divide<T, std::ratio<1, 2>>, std::ratio_divide<A, std::ratio<1, 2>>>
	Qsq(const RQuantity<M, L, T, A>& num)
{
	return RQuantity<std::ratio_divide<M, std::ratio<1, 2>>, std::ratio_divide<L, std::ratio<1, 2>>,
		std::ratio_divide<T, std::ratio<1, 2>>, std::ratio_divide<A, std::ratio<1, 2>>>
		(num * num);
}

// Typesafe trigonometric operations
inline float sin(const Angle& num) {
	return sin(num.getValue());
}
inline float cos(const Angle& num) {
	return cos(num.getValue());
}

inline float tan(const Angle& num) {
	return tan(num.getValue());
}
