/** ***********************************************************************************************
* @file         WriteBinary.h
* @brief
* @details		Details: Helper functions for binary file output
*
* @author		Gerstmayr Johannes
* @date			2022-02-06 (generated)
* @pre			...
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
* @note			Bug reports, support and further information:
* 				- email: johannes.gerstmayr@uibk.ac.at
* 				- weblink: https://github.com/jgerstmayr/EXUDYN
* 				
*
************************************************************************************************ */

//#include "Linalg/BasicLinalg.h"

namespace ExuFile {

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ASCII functions
	//write vector data, starting with colon;
	inline void ASCIIwrite(const Vector& vector, std::ofstream& file)
	{
		for (Index k = 0; k < vector.NumberOfItems(); k++)
		{
			file << "," << vector[k];
		}
	}

	inline void ASCIIwrite(const Real& value, std::ofstream& file)
	{
		file << value;
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//BINARY functions

	//! determine system endianness
	inline bool SystemIsLittleEndian()
	{
		const int value{ 0x01 };
		const void * address = static_cast<const void *>(&value);
		const unsigned char * leastSignificantAddress = static_cast<const unsigned char *>(address);
		return (*leastSignificantAddress == 0x01);
	}

	//! structure defining settings for file write; may be eiterh system settings, or user-defined settings (e.g. to store at float accuracy)
	class BinaryFileSettings
	{
	public:
		Index indexSize;
		Index realSize; //4=float, 8=double
		Index pointerSize;
		bool bigEndian; //change does not affect behavior

		BinaryFileSettings()
		{
			indexSize = sizeof(Index);
			realSize = sizeof(Real);
			Index a = 1;
			pointerSize = sizeof(&a);
			bigEndian = (!SystemIsLittleEndian());
		}
	};



	//write 16 bytes header, providing information about stored file structure
	inline void BinaryWriteHeader(std::ofstream& file, const BinaryFileSettings& bSettings)
	{
		file << "EXUBIN"; //identifier for all formats; add end of line to allow simple reading in ascii mode!
		file << (char)'\n'; //should be only 1 byte
		file << (char)bSettings.indexSize;
		file << (char)bSettings.realSize;
		file << (char)bSettings.pointerSize;
		file << (char)bSettings.bigEndian;
		char x = 0; //dummy bytes
		for (Index i = 0; i < 16 - 11; i++) //11 bytes already used
		{
			file << x;
		}
	}

	//! write Index according to settings; includeSize ignored
	inline void BinaryWrite(Index value, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = false)
	{
		if (bSettings.indexSize == 4)
		{
			std::int32_t castedValue = (std::int32_t)value;
			file.write(reinterpret_cast<const char*>(&castedValue), sizeof(std::int32_t));
		}
		else if (bSettings.indexSize == 8)
		{
			std::int64_t castedValue = (std::int64_t)value;
			file.write(reinterpret_cast<const char*>(&castedValue), sizeof(std::int64_t));
		}
		else { CHECKandTHROWstring("ExuFile::BinaryWrite(Index value, ...): illegal Index size"); }
	}

	//! write string in binary format; add stringlength before bytes of string
	inline void BinaryWrite(const STDstring& str, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		if (includeSize) { BinaryWrite((Index)str.length(), file, bSettings); }
		file.write(str.c_str(), str.length());
	}

	//! write Real according to settings; includeSize ignored
	inline void BinaryWrite(Real value, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = false)
	{
		if (bSettings.realSize == 4)
		{
			float castedValue = (float)value;
			file.write(reinterpret_cast<const char*>(&castedValue), sizeof(float));
		}
		else if (bSettings.realSize == 8)
		{
			double castedValue = (double)value;
			file.write(reinterpret_cast<const char*>(&castedValue), sizeof(double));
		}
		else { CHECKandTHROWstring("ExuFile::BinaryWrite(Real value, ...): illegal Real size"); }
	}

//#define FileDebug
	//! write vector in binary format; add vector length before data of vector
	template<class TVector>
	void BinaryWriteVectorTemplate(const TVector& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		if (includeSize) { BinaryWrite((Index)vector.NumberOfItems(), file, bSettings); }
		for (Real item : vector)
		{
			BinaryWrite(item, file, bSettings);
		}
	}

	//! specific functions to write vectors and arrays
	inline void BinaryWrite(const Vector& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteVectorTemplate<Vector>(vector, file, bSettings, includeSize);
	}
	template<Index dataSize>
	void BinaryWrite(const ConstSizeVector<dataSize>& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteVectorTemplate<ConstSizeVector<dataSize>>(vector, file, bSettings, includeSize);
	}
	inline void BinaryWrite(const ResizableVector& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteVectorTemplate<ResizableVector>(vector, file, bSettings, includeSize);
	}
	template<Index dataSize>
	void BinaryWrite(const SlimVector<dataSize>& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteVectorTemplate<SlimVector<dataSize>>(vector, file, bSettings, includeSize);
	}
	inline void BinaryWrite(const ResizableArray<Real>& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteVectorTemplate<ResizableArray<Real>>(vector, file, bSettings, includeSize);
	}

	//! write Index vector or array in binary format; add vector length before data of vector
	template<class TVector>
	void BinaryWriteIndexVectorTemplate(const TVector& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		if (includeSize) { BinaryWrite((Index)vector.NumberOfItems(), file, bSettings); }
		for (Index item : vector)
		{
			BinaryWrite(item, file, bSettings);
		}
	}

	inline void BinaryWrite(const ArrayIndex& vector, std::ofstream& file, const BinaryFileSettings& bSettings, bool includeSize = true)
	{
		BinaryWriteIndexVectorTemplate<ArrayIndex>(vector, file, bSettings, includeSize);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//mixed functions
	//write ASCII or binary depending on flag;
	template<class DataType>
	inline void Write(const DataType& data, std::ofstream& file, const BinaryFileSettings& bSettings, bool isBinary, bool includeSize=true)
	{
		if (isBinary)
		{
			BinaryWrite(data, file, bSettings, includeSize);
		}
		else
		{
			ASCIIwrite(data, file);
		}
	}

};