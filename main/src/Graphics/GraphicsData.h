/** ***********************************************************************************************
* @class        GraphicsData
* @brief        Data for 3D graphics; interface between visualization and OpenGL rendering engine
*			    This is a very simple prototype for graphics visulization to be extended to
*				OpenGL vertex shaders
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef GRAPHICSDATA__H
#define GRAPHICSDATA__H

#include <ostream>

#include "Linalg/BasicLinalg.h"	
#include "Main/OutputVariable.h"		//for ItemType conversion, used in GlfwClient and others

#include <array>						//std::array

typedef SlimArray<float, 16> hMatrix4f; //introduce this typedef to enable switch to other matrix representations

//! structure for colored line
class GLLine
{
public:
	Index itemID;			//!< itemID according to ItemType and index, see Index2ItemID(...)
	Float3 point1;			//!< 3D point coordinates
	Float3 point2;			//!< 3D point coordinates

	Float4 color1;			//!< RGBA color in range 0.f - 1.f; A ... alpha
	Float4 color2;			//!< RGBA color in range 0.f - 1.f; A ... alpha
};

//! structure for a point (node); drawing might be realized as point, circle or sphere
class GLPoint
{
public:
	Index itemID;			//!< itemID according to ItemType and index, see Index2ItemID(...)
	Float3 point;			//!< 3D point coordinates
	Float4 color;			//!< RGBA color in range 0.f - 1.f; A ... alpha
};

//! structure for a circle in XY-plane with radius
class GLCircleXY
{
public:
	Index itemID;			//!< itemID according to ItemType and index, see Index2ItemID(...)
	Float3 point;			//!< 3D point coordinates
	Float4 color;			//!< RGBA color in range 0.f - 1.f; A ... alpha
	float radius;			//!< circle radius
	Index numberOfSegments; //!< set to 0 to use default number of segments
};

//! structure for colored text at position
class GLText
{
public:
	Index itemID;			//!< itemID according to ItemType and index, see Index2ItemID(...)
	Float3 point;			//!< 3D point coordinates
	Float4 color;			//!< RGBA color in range 0.f - 1.f; A ... alpha
	float size;				//!< size of text; if size==0 --> use default text size
	float offsetX;			//!< offset of text in x-direction relative to textsize; not corotated with model
	float offsetY;			//!< offset of text in y-direction relative to textsize; not corotated with model
	char* text;				//!< pointer to 0-terminated string
};

//! structure for colored triangle: points, normals and colors per triangle node point
//! 
//! local indices: triangle viewd from outside (counter-clockwise local numbering)
//! 3
//! |\
//! | \
//! |  \
//! 1---2
//! 
class GLTriangle
{
public:
	Index itemID;			//!< itemID according to ItemType and index, see Index2ItemID(...)
	std::array< Float3, 3> points;	//!< 3D point coordinates
	std::array< Float3, 3> normals;	//!< 3D normal coordinates, pointing outwards; [0,0,0] if unused
	std::array< Float4, 3> colors;	//!< RGBA color in range 0.f - 1.f; A ... alpha
};

//!interface for system graphics data
// data is read by glfwClient (other thread) and Visualization
class GraphicsData
{
public:
	ResizableArray<GLLine> glLines;				//!< lines to be displayed
	ResizableArray<GLPoint> glPoints;			//!< points to be displayed
	ResizableArray<GLCircleXY> glCirclesXY;		//!< circles to be displayed
	ResizableArray<GLText> glTexts;				//!< texts to be displayed
	ResizableArray<GLTriangle> glTriangles;		//!< triangles to be displayed

	bool isStatic;				//!< true, if object is fixed to world-frame (e.g. background or groundObject)
	bool isRigid;				//!< signals that after creation of the object, all points just undergo a rigidbody transformation
	hMatrix4f transformation;	//!< used for rigidbody transformation, if object is rigid

private:
	std::atomic_flag lock;		
	uint64_t visualizationCounter;
	bool updateGraphicsDataNow; //! flag set by Renderer to recompute graphics data (e.g. when settings changed)
	float contourCurrentMinValue; //! current minimum value for contour plot
	float contourCurrentMaxValue; //! current maximum value for contour plot

public:
	GraphicsData()
	{
		contourCurrentMinValue = EXUstd::_MAXFLOAT;
		contourCurrentMaxValue = EXUstd::_MINFLOAT;
		ClearLock();
	}
	//! Aquire lock for data, such that computation / visualization thread does not access data at the same time
	void LockData() 
	{ 
		EXUstd::WaitAndLockSemaphore(lock);
		//now data is locked until it is cleared!!!
		//hereafter, lock must be cleared, otherwise, a new lock is not possible
	}
	//! Release the lock of a previous LockData()
	void ClearLock() 
	{ 
		EXUstd::ReleaseSemaphore(lock);
	}

	virtual ~GraphicsData()
	{
		FlushData();
	}

	const uint64_t& GetVisualizationCounter() const { return visualizationCounter; };
	uint64_t& GetVisualizationCounter() { return visualizationCounter; };

	const bool& GetUpdateGraphicsDataNow() const { return updateGraphicsDataNow; };
	bool& GetUpdateGraphicsDataNow() { return updateGraphicsDataNow; };

	const float& GetContourCurrentMinValue() const { return contourCurrentMinValue; };
	float& GetContourCurrentMinValue() { return contourCurrentMinValue; };

	const float& GetContourCurrentMaxValue() const { return contourCurrentMaxValue; };
	float& GetContourCurrentMaxValue() { return contourCurrentMaxValue; };

	//! clear lists (keep allocated data) and deallocate data of texts, allocated with new
	void FlushData()
	{
		LockData();
		for (auto item : glTexts)
		{
			delete[] item.text;
		}
		glLines.SetNumberOfItems(0);
		glPoints.SetNumberOfItems(0);
		glCirclesXY.SetNumberOfItems(0);
		glTexts.SetNumberOfItems(0);
		glTriangles.SetNumberOfItems(0);

		ClearLock();
	}

	Index AddLine(const Vector3D& point1, const Vector3D& point2, const Float4& color1, const Float4& color2, 
		Index itemID)
	{
		GLLine line;
		line.itemID = itemID;
		line.point1 = Float3({ (float)point1[0],(float)point1[1],(float)point1[2] });
		line.point2 = Float3({ (float)point2[0],(float)point2[1],(float)point2[2] });
		line.color1 = color1;
		line.color2 = color2;
		return glLines.Append(line);
	}

	Index AddPoint(const Vector3D& point, const Float4& color, Index itemID)
	{
		GLPoint glPoint;
		glPoint.itemID = itemID;
		glPoint.point = Float3({ (float)point[0],(float)point[1],(float)point[2] });
		glPoint.color = color;

		return glPoints.Append(glPoint);
	}

	//! create circle i XY-plane with centerPoint, radius, color and numberOfSegments (0 ... use default value)
	//Index AddCircleXY(const Vector3D& centerPoint, float radius, const Float4& color, Index numberOfSegments = 0, Index index, ItemType itemType)
	Index AddCircleXY(const Vector3D& centerPoint, float radius, const Float4& color, Index numberOfSegments, 
		Index itemID)
	{
		GLCircleXY circle;
		circle.itemID = itemID;
		circle.point = Float3({ (float)centerPoint[0],(float)centerPoint[1],(float)centerPoint[2] });
		circle.radius = radius;
		circle.color = color;
		circle.numberOfSegments = numberOfSegments;
		return glCirclesXY.Append(circle);
	}

	//! create a triangle with local colors; normals not considered in this call!
	Index AddTriangle(const std::array<Vector3D, 3>& points, const std::array<Float4, 3>& colors, 
		Index itemID)
	{
		GLTriangle trig;
		trig.itemID = itemID;

		for (Index i = 0; i < (Index)points.size(); i++)
		{
			trig.points[i] = Float3({ (float)(points[i][0]), (float)(points[i][1]), (float)(points[i][2]) });
			trig.normals[i] = Float3({ 0.,0.,0. });
			trig.colors[i] = colors[i];
		}
		return glTriangles.Append(trig);
	}

	//! create a triangle with local colors; normals not considered in this call!
	Index AddTriangle(const std::array<Vector3D, 3>& points, const std::array<Vector3D, 3>& normals, const std::array<Float4, 3>& colors, 
		Index itemID)
	{
		GLTriangle trig;
		trig.itemID = itemID;

		for (Index i = 0; i < (Index)points.size(); i++)
		{
			trig.points[i] = Float3({ (float)(points[i][0]), (float)(points[i][1]), (float)(points[i][2]) });
			trig.normals[i] = Float3({ (float)(normals[i][0]), (float)(normals[i][1]), (float)(normals[i][2]) });;
			trig.colors[i] = colors[i];
		}
		return glTriangles.Append(trig);
	}

	//! create text from string with 3D-point, color and size (0 ... use default text size)
	//Index AddText(const Vector3D& point, const Float4& color, const STDstring& text, float size = 0.f, float offsetX = 0.f, float offsetY = 0.f, Index index, ItemType itemType)
	Index AddText(const Vector3D& point, const Float4& color, const STDstring& text, float size, float offsetX, float offsetY, 
		Index itemID)
	{
		GLText glText;
		glText.itemID = itemID;

		glText.point = Float3({ (float)point[0],(float)point[1],(float)point[2] });
		glText.color = color;
		glText.size = size;
		glText.offsetX = offsetX;
		glText.offsetY = offsetY;

		int len = (int)text.size();
		glText.text = new char[len + 1]; //will be deleted in destructor of GraphicsData
		//strcpy_s(glText.text, len + 1, text.c_str()); //not working with gcc
		strcpy(glText.text, text.c_str());

		return glTexts.Append(glText);
	}

	virtual void Print(std::ostream& os) const
	{
		os << "GraphicsData:\n";
		os << "  number of lines  = " << glLines.NumberOfItems() << "\n";
		os << "  number of points = " << glPoints.NumberOfItems() << "\n";
		os << "  number of circles= " << glCirclesXY.NumberOfItems() << "\n";
		os << "  number of texts  = " << glTexts.NumberOfItems() << "\n";
		os << "  number of trigs  = " << glTriangles.NumberOfItems() << "\n";

		os << "\n";
	}

	friend std::ostream& operator<<(std::ostream& os, const GraphicsData& object)
	{
		object.Print(os);
		return os;
	}


};

#endif
