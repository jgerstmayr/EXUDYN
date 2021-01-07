/** ***********************************************************************************************
* @class        CSystemState
* @brief        data structure for representation of system state for instant of time or configuration (initial, current, reference, ...)
*
* @author       Gerstmayr Johannes
* @date         2018-05-18 (generated)
* @date         2019-05-09 (last modfied)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
				- email: johannes.gerstmayr@uibk.ac.at
				- weblink: https://github.com/jgerstmayr/EXUDYN
				
************************************************************************************************ */
#ifndef CSYSTEMSTATE__H
#define CSYSTEMSTATE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

class CSystemState // 
{
public: // 
	Vector ODE1Coords;                              //!< first order differential equations (ODE1) (displacement) coordinates
	Vector ODE1Coords_t;                            //!< first order differential equations (ODE1) (velocity) coordinates
	Vector ODE2Coords;                              //!< second order differential equations (ODE2) displacement coordinates; in static mode: (unknown) displacement coordinates
	Vector ODE2Coords_t;                            //!< second order differential equations (ODE2) velocity coordinates
	Vector ODE2Coords_tt;                           //!< second order differential equations (ODE2) acceleration coordinates
	Vector AECoords;                                //!< algebraic equations coordinates (e.g. Lagrange multipliers)
	Vector dataCoords;                              //!< data coordinates (e.g. contact state or other non-DAE coordinates)
	Real time;										//!< time or quasi-time (static solver) to which the data is linked to

public: // 

	//! Write access to: time
	void SetTime(const Real& timeInit) { time = timeInit; }
	//! Read (Reference) access to: time
	const Real& GetTime() const { return time; }

	//! Reset data: reset all vectors to zero size and free memory; needed for solvers
	void Reset() {
		ODE1Coords.Reset();
		ODE1Coords_t.Reset();
		ODE2Coords.Reset();
		ODE2Coords_t.Reset();
		ODE2Coords_tt.Reset();
		AECoords.Reset();
		dataCoords.Reset();
		time = 0;
	}

	//! Write access to: first order differential equations (ODE1) coordinates
	void SetODE1Coords(const Vector& ODE1CoordsInit) { ODE1Coords = ODE1CoordsInit; }
	//! Read (Reference) access to: first order differential equations (ODE1) coordinates
	const Vector& GetODE1Coords() const { return ODE1Coords; }
	//const std::vector<Real> GetODE1CoordsStdVec() const { return ODE1Coords; }

	//! Write access to: first order differential equations (ODE1) velocity coordinates
	void SetODE1Coords_t(const Vector& ODE1Coords_tInit) { ODE1Coords_t = ODE1Coords_tInit; }
	//! Read (Reference) access to: first order differential equations (ODE1) velocity coordinates
	const Vector& GetODE1Coords_t() const { return ODE1Coords_t; }
	//const std::vector<Real> GetODE1Coords_tStdVec() const { return ODE1Coords_t; }

	//! Write access to: second order differential equations (ODE2) displacement coordinates; in static mode: (unknown) displacement coordinates
	void SetODE2Coords(const Vector& ODE2CoordsInit) { ODE2Coords = ODE2CoordsInit; }
	//! Read (Reference) access to: second order differential equations (ODE2) displacement coordinates; in static mode: (unknown) displacement coordinates
	const Vector& GetODE2Coords() const { return ODE2Coords; }
	//const std::vector<Real> GetODE2CoordsStdVec() const { return ODE2Coords; }

	//! Write access to: second order differential equations (ODE2) velocity coordinates
	void SetODE2Coords_t(const Vector& ODE2Coords_tInit) { ODE2Coords_t = ODE2Coords_tInit; }
	//! Read (Reference) access to: second order differential equations (ODE2) velocity coordinates
	const Vector& GetODE2Coords_t() const { return ODE2Coords_t; }
	//const std::vector<Real> GetODE2Coords_tStdVec() const { return ODE2Coords_t; }

	//! Write access to: second order differential equations (ODE2) acceleration coordinates
	void SetODE2Coords_tt(const Vector& ODE2Coords_ttInit) { ODE2Coords_tt = ODE2Coords_ttInit; }
	//! Read (Reference) access to: second order differential equations (ODE2) acceleration coordinates
	const Vector& GetODE2Coords_tt() const { return ODE2Coords_tt; }
	//const std::vector<Real> GetODE2Coords_ttStdVec() const { return ODE2Coords_tt; }

	//! Write access to: algebraic equations coordinates (e.g. Lagrange multipliers)
	void SetAECoords(const Vector& AECoordsInit) { AECoords = AECoordsInit; }
	//! Read (Reference) access to: algebraic equations coordinates (e.g. Lagrange multipliers)
	const Vector& GetAECoords() const { return AECoords; }
	//const std::vector<Real> GetAECoordsStdVec() const { return AECoords; }

	//! Write access to: data coordinates (e.g. contact state or other non-DAE coordinates)
	void SetDataCoords(const Vector& dataCoordsInit) { dataCoords = dataCoordsInit; }
	//! Read (Reference) access to: data coordinates (e.g. contact state or other non-DAE coordinates)
	const Vector& GetDataCoords() const { return dataCoords; }
	//const std::vector<Real> GetDataCoordsStdVec() const { return dataCoords; }

	void Print(std::ostream& os) const
	{
		os << "  CSystemState:\n";
		os << "    ODE1Coords = " << ODE1Coords << "\n";
		os << "    ODE2Coords = " << ODE2Coords << "\n";
		os << "    ODE2Coords_t = " << ODE2Coords_t << "\n";
		os << "    AECoords = " << AECoords << "\n";
		os << "    dataCoords = " << dataCoords << "\n";
		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const CSystemState& object)
	{
		object.Print(os);
		return os;
	}

};

#endif
