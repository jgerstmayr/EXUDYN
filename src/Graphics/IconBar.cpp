/** ***********************************************************************************************
* @brief        The icons of the icon bar (#3026): a small rasterizer with anti-aliased lines, arcs, discs and
*               polygons by their distance to the pixel center, and one drawing per icon on a 64 x 64 canvas
*
* @author       Gerstmayr Johannes
* @date         2026-10-19
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#include "Graphics/IconBar.h"
#include "Graphics/TextureStore.h"
#include <cmath>
#include <algorithm>

namespace IconBar
{
	//! a point of the canvas, in pixels from the bottom left corner
	struct P { float x, y; };

	//! an RGBA canvas of floats, y up; shapes are blended over what is there with the coverage of each pixel, which is
	//! 0.5 plus the signed distance of the pixel center to the border of the shape, clamped to [0, 1]
	class Canvas
	{
	public:
		Index size;
		std::vector<float> rgba;

		Canvas(Index sizeInit) : size(sizeInit), rgba((size_t)(sizeInit*sizeInit*4), 0.f) {}

		void Blend(Index i, Index j, const Float4& color, float coverage)
		{
			if (i < 0 || j < 0 || i >= size || j >= size || coverage <= 0.f) { return; }
			float a = std::min(1.f, coverage)*color[3];
			float* p = &rgba[(size_t)((j*size + i)*4)];
			float outA = a + p[3]*(1.f - a);
			for (Index k = 0; k < 3; k++) { p[k] = outA > 0.f ? (color[k]*a + p[k]*p[3]*(1.f - a))/outA : 0.f; }
			p[3] = outA;
		}

		//! apply a coverage function f(x, y) over the bounding box [x0, x1] x [y0, y1]
		template<class TCoverage>
		void Fill(float x0, float y0, float x1, float y1, const Float4& color, const TCoverage& coverage)
		{
			Index i0 = std::max((Index)0, (Index)std::floor(x0 - 1)), i1 = std::min(size - 1, (Index)std::ceil(x1 + 1));
			Index j0 = std::max((Index)0, (Index)std::floor(y0 - 1)), j1 = std::min(size - 1, (Index)std::ceil(y1 + 1));
			for (Index j = j0; j <= j1; j++)
			{
				for (Index i = i0; i <= i1; i++) { Blend(i, j, color, coverage((float)i + 0.5f, (float)j + 0.5f)); }
			}
		}

		static float SegmentDistance(P a, P b, float x, float y)
		{
			float dx = b.x - a.x, dy = b.y - a.y;
			float length2 = dx*dx + dy*dy;
			float t = length2 > 0.f ? std::max(0.f, std::min(1.f, ((x - a.x)*dx + (y - a.y)*dy)/length2)) : 0.f;
			float ex = x - (a.x + t*dx), ey = y - (a.y + t*dy);
			return std::sqrt(ex*ex + ey*ey);
		}

		void Line(P a, P b, float width, const Float4& color)
		{
			float h = 0.5f*width;
			Fill(std::min(a.x, b.x) - h, std::min(a.y, b.y) - h, std::max(a.x, b.x) + h, std::max(a.y, b.y) + h, color,
				[&](float x, float y) { return std::max(0.f, std::min(1.f, h + 0.5f - SegmentDistance(a, b, x, y))); });
		}

		void PolyLine(const std::vector<P>& points, float width, const Float4& color, bool closed = false)
		{
			//as one shape, so that the joints are not blended twice
			float h = 0.5f*width;
			float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
			for (const P& p : points) { x0 = std::min(x0, p.x - h); y0 = std::min(y0, p.y - h); x1 = std::max(x1, p.x + h); y1 = std::max(y1, p.y + h); }
			size_t n = points.size();
			Fill(x0, y0, x1, y1, color, [&](float x, float y)
			{
				float d = 1e9f;
				for (size_t k = 0; k + 1 < n + (closed ? 1 : 0); k++) { d = std::min(d, SegmentDistance(points[k], points[(k + 1) % n], x, y)); }
				return std::max(0.f, std::min(1.f, h + 0.5f - d));
			});
		}

		//! an arc of radius r about c from angle a0 to a1 (radians, counterclockwise from +x)
		void Arc(P c, float r, float width, const Float4& color, float a0 = 0.f, float a1 = 6.2831853f)
		{
			float h = 0.5f*width;
			bool full = (a1 - a0) >= 6.283f;
			P e0{ c.x + r*std::cos(a0), c.y + r*std::sin(a0) }, e1{ c.x + r*std::cos(a1), c.y + r*std::sin(a1) };
			Fill(c.x - r - h, c.y - r - h, c.x + r + h, c.y + r + h, color, [&](float x, float y)
			{
				float dx = x - c.x, dy = y - c.y;
				float d = std::fabs(std::sqrt(dx*dx + dy*dy) - r);
				if (!full)
				{
					float a = std::atan2(dy, dx);
					while (a < a0) { a += 6.2831853f; }
					if (a > a1) //outside the arc: the distance to its ends
					{
						d = std::min(std::sqrt((x - e0.x)*(x - e0.x) + (y - e0.y)*(y - e0.y)), std::sqrt((x - e1.x)*(x - e1.x) + (y - e1.y)*(y - e1.y)));
					}
				}
				return std::max(0.f, std::min(1.f, h + 0.5f - d));
			});
		}

		void Disc(P c, float r, const Float4& color)
		{
			Fill(c.x - r, c.y - r, c.x + r, c.y + r, color, [&](float x, float y)
			{
				return std::max(0.f, std::min(1.f, r + 0.5f - std::sqrt((x - c.x)*(x - c.x) + (y - c.y)*(y - c.y))));
			});
		}

		//! a filled polygon, by the even-odd rule
		void Polygon(const std::vector<P>& points, const Float4& color)
		{
			float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
			for (const P& p : points) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
			size_t n = points.size();
			Fill(x0, y0, x1, y1, color, [&](float x, float y)
			{
				bool inside = false;
				float d = 1e9f;
				for (size_t k = 0, l = n - 1; k < n; l = k++)
				{
					const P& a = points[k];
					const P& b = points[l];
					if ((a.y > y) != (b.y > y) && x < (b.x - a.x)*(y - a.y)/(b.y - a.y) + a.x) { inside = !inside; }
					d = std::min(d, SegmentDistance(a, b, x, y));
				}
				return std::max(0.f, std::min(1.f, inside ? 0.5f + d : 0.5f - d));
			});
		}

		void Rectangle(float x0, float y0, float x1, float y1, const Float4& color)
		{
			Polygon({ {x0, y0}, {x1, y0}, {x1, y1}, {x0, y1} }, color);
		}

		//! an arrow from a to b with a filled head
		void Arrow(P a, P b, float width, float head, const Float4& color)
		{
			float dx = b.x - a.x, dy = b.y - a.y;
			float length = std::sqrt(dx*dx + dy*dy);
			if (length == 0.f) { return; }
			dx /= length; dy /= length;
			P base{ b.x - head*dx, b.y - head*dy };
			Line(a, base, width, color);
			Polygon({ b, {base.x - 0.55f*head*dy, base.y + 0.55f*head*dx}, {base.x + 0.55f*head*dy, base.y - 0.55f*head*dx} }, color);
		}
	};

	const Float4 ink({ 0.18f, 0.2f, 0.24f, 1.f });		//lines and outlines
	const Float4 accent({ 0.15f, 0.42f, 0.8f, 1.f });	//the blue of selections and items
	const Float4 lightAccent({ 0.55f, 0.72f, 0.95f, 1.f });
	const Float4 warning({ 0.78f, 0.15f, 0.12f, 1.f });
	const float pi = 3.14159265f;

	const char* Name(Index icon)
	{
		static const char* names[] = { "quit", "zoomAll", "zoomRegion", "viewXY", "viewYZ", "viewXZ", "viewIsometric",
			"ruler", "mouseCoordinates", "lines", "edges", "faces", "transparency", "updateSpeed",
			"nodes", "bodies", "connectors", "markers", "loads", "sensors", "config0", "config1", "config2", "config3" };
		return (icon >= 0 && icon < NumberOfIcons()) ? names[icon] : "";
	}

	const char* Tooltip(Index icon)
	{
		static const char* tooltips[] = {
			"close the renderer (ESCAPE)",
			"zoom all (A)",
			"zoom to a region: click two of its corners (or SHIFT+drag; CTRL+A zooms to the selected item)",
			"view in the X-Y plane (CTRL: Y-X; SHIFT: from behind)",
			"view in the Y-Z plane (CTRL: Z-Y; SHIFT: from behind)",
			"view in the X-Z plane (CTRL: Z-X; SHIFT: from behind)",
			"3D view (CTRL+7)",
			"ruler (F5)",
			"mouse coordinates (F3)",
			"show lines",
			"show edges of faces and meshes",
			"show faces",
			"transparent faces",
			"graphics update interval (keys 1 to 5)",
			"show nodes (N; SHIFT: numbers, CTRL+N)",
			"show bodies (B; SHIFT: numbers, CTRL+B)",
			"show connectors (C; SHIFT: numbers, CTRL+C)",
			"show markers (M; SHIFT: numbers, CTRL+M)",
			"show loads (L; SHIFT: numbers, CTRL+L)",
			"show sensors (S; SHIFT: numbers, CTRL+S)",
			"switch the visualization settings to the configuration CFG0 stored in config.json (CTRL: store the current settings as CFG0)",
			"switch the visualization settings to the configuration CFG1 stored in config.json (CTRL: store the current settings as CFG1)",
			"switch the visualization settings to the configuration CFG2 stored in config.json (CTRL: store the current settings as CFG2)",
			"switch the visualization settings to the configuration CFG3 stored in config.json (CTRL: store the current settings as CFG3)" };
		return (icon >= 0 && icon < NumberOfIcons()) ? tooltips[icon] : "";
	}

	//! two axes of a view; the names of the axes are written over the icon by the bar, as they change with CTRL
	void DrawAxes(Canvas& c)
	{
		c.Arrow({ 12, 12 }, { 52, 12 }, 4, 11, ink);
		c.Arrow({ 12, 12 }, { 12, 52 }, 4, 11, ink);
		c.Disc({ 12, 12 }, 4, ink);
	}

	//! a cube in an oblique view: its front, top and right faces, or its edges with the three hidden ones thin
	void DrawCube(Canvas& c, bool faces)
	{
		P f0{ 8, 8 }, f1{ 40, 8 }, f2{ 40, 40 }, f3{ 8, 40 };	//the front face, counterclockwise from the bottom left
		P b0{ 24, 22 }, b1{ 56, 22 }, b2{ 56, 54 }, b3{ 24, 54 };	//the back face
		if (faces)
		{
			c.Polygon({ f0, f1, f2, f3 }, accent);
			c.Polygon({ f3, f2, b2, b3 }, lightAccent);
			c.Polygon({ f1, b1, b2, f2 }, Float4({ 0.1f, 0.3f, 0.6f, 1.f }));
		}
		else
		{
			const Float4 hidden({ 0.18f, 0.2f, 0.24f, 0.45f });
			c.PolyLine({ f0, b0, b3 }, 2, hidden);
			c.Line(b0, b1, 2, hidden);
			c.PolyLine({ f0, f1, b1, b2, b3, f3 }, 3.5f, ink, true);
			c.PolyLine({ f3, f2, f1 }, 3.5f, ink);
			c.Line(f2, b2, 3.5f, ink);
		}
	}

	void DrawIcon(Index icon, std::vector<unsigned char>& rgba)
	{
		Canvas c(iconPixels);
		switch ((Icon)icon)
		{
		case Icon::Quit: //the power symbol
			c.Arc({ 32, 29 }, 19, 6, warning, 0.5f*pi + 0.55f, 2.5f*pi - 0.55f);
			c.Line({ 32, 31 }, { 32, 56 }, 6, warning);
			break;
		case Icon::ZoomAll: //four corners around a box
			for (float sx : { -1.f, 1.f }) for (float sy : { -1.f, 1.f })
			{
				P corner{ 32 + 23*sx, 32 + 23*sy };
				c.PolyLine({ {corner.x - 11*sx, corner.y}, corner, {corner.x, corner.y - 11*sy} }, 5, ink);
			}
			c.Rectangle(21, 21, 43, 43, accent);
			break;
		case Icon::ZoomRegion: //a magnifier on a region drawn dashed
			for (float t = 0.f; t < 1.f; t += 0.25f) //the dashes of the four sides
			{
				c.Line({ 6 + 44*t, 58 }, { 6 + 44*(t + 0.14f), 58 }, 3, accent);
				c.Line({ 6 + 44*t, 14 }, { 6 + 44*(t + 0.14f), 14 }, 3, accent);
				c.Line({ 6, 14 + 44*t }, { 6, 14 + 44*(t + 0.14f) }, 3, accent);
				c.Line({ 50, 14 + 44*t }, { 50, 14 + 44*(t + 0.14f) }, 3, accent);
			}
			c.Line({ 40, 24 }, { 56, 8 }, 7, ink);
			c.Arc({ 29, 35 }, 14, 5, ink);
			break;
		case Icon::ViewXY:
		case Icon::ViewYZ:
		case Icon::ViewXZ:
			DrawAxes(c);
			break;
		case Icon::ViewIsometric: //a cube in a 3D view
		{
			P a{ 32, 32 }, top{ 32, 55 }, left{ 12, 21 }, right{ 52, 21 };
			c.Polygon({ top, {52, 44}, a, {12, 44} }, Float4({ 0.78f, 0.86f, 0.98f, 1.f }));	//top face
			c.Polygon({ {12, 44}, a, {32, 9}, left }, lightAccent);							//left face
			c.Polygon({ a, {52, 44}, right, {32, 9} }, accent);									//right face
			c.PolyLine({ top, {52, 44}, right, {32, 9}, left, {12, 44} }, 4, ink, true);
			c.PolyLine({ {12, 44}, a, {52, 44} }, 3, ink);
			c.Line(a, { 32, 9 }, 3, ink);
			break;
		}
		case Icon::Ruler:
			c.Rectangle(5, 22, 59, 44, Float4({ 0.95f, 0.85f, 0.45f, 1.f }));
			c.PolyLine({ {5, 22}, {59, 22}, {59, 44}, {5, 44} }, 3, ink, true);
			for (Index k = 1; k < 10; k++)
			{
				float x = 5.f + 5.4f*(float)k;
				c.Line({ x, 44 }, { x, k % 5 == 0 ? 28.f : (k % 2 == 0 ? 33.f : 37.f) }, 2.5f, ink);
			}
			break;
		case Icon::MouseCoordinates: //a crosshair
			c.Line({ 32, 6 }, { 32, 58 }, 3, ink);
			c.Line({ 6, 32 }, { 58, 32 }, 3, ink);
			c.Arc({ 32, 32 }, 13, 4, accent);
			break;
		case Icon::Lines: //the edges of a cube, the hidden ones thin
			DrawCube(c, false);
			break;
		case Icon::Edges: //a rectangle meshed into 8 triangles
			c.PolyLine({ {7, 10}, {57, 10}, {57, 54}, {7, 54} }, 3.5f, ink, true);
			c.Line({ 32, 10 }, { 32, 54 }, 2.5f, ink);
			c.Line({ 7, 32 }, { 57, 32 }, 2.5f, ink);
			c.Line({ 7, 10 }, { 57, 54 }, 2.5f, ink);
			c.Line({ 32, 10 }, { 57, 32 }, 2.5f, ink);
			c.Line({ 7, 32 }, { 32, 54 }, 2.5f, ink);
			break;
		case Icon::Faces: //the faces of a cube
			DrawCube(c, true);
			break;
		case Icon::Transparency: //two squares, the front one transparent
			c.Rectangle(6, 22, 42, 58, accent);
			c.Rectangle(22, 6, 58, 42, Float4({ 0.85f, 0.88f, 0.92f, 0.65f }));
			c.PolyLine({ {22, 6}, {58, 6}, {58, 42}, {22, 42} }, 3, ink, true);
			break;
		case Icon::UpdateSpeed: //a clock; the bar writes the number of the interval
			c.Arc({ 32, 32 }, 25, 4, ink);
			for (Index k = 0; k < 12; k++)
			{
				float a = 2.f*pi*(float)k/12.f;
				c.Line({ 32 + 20*std::cos(a), 32 + 20*std::sin(a) }, { 32 + 23*std::cos(a), 32 + 23*std::sin(a) }, 2, ink);
			}
			c.Line({ 32, 32 }, { 32, 50 }, 4, ink);
			c.Line({ 32, 32 }, { 45, 26 }, 4, accent);
			break;
		case Icon::Nodes:
			c.Disc({ 32, 32 }, 15, accent);
			c.Arc({ 32, 32 }, 15, 3, ink);
			break;
		case Icon::Bodies: //a box
			c.Polygon({ {8, 12}, {42, 12}, {42, 44}, {8, 44} }, accent);
			c.Polygon({ {8, 44}, {42, 44}, {56, 54}, {22, 54} }, lightAccent);
			c.Polygon({ {42, 12}, {56, 22}, {56, 54}, {42, 44} }, Float4({ 0.1f, 0.3f, 0.6f, 1.f }));
			c.PolyLine({ {8, 12}, {42, 12}, {56, 22}, {56, 54}, {22, 54}, {8, 44} }, 3, ink, true);
			break;
		case Icon::Connectors: //a spring between two points
			c.PolyLine({ {10, 32}, {18, 32}, {22, 44}, {28, 20}, {34, 44}, {40, 20}, {46, 44}, {50, 32}, {54, 32} }, 3.5f, ink);
			c.Disc({ 8, 32 }, 6, accent);
			c.Disc({ 56, 32 }, 6, accent);
			break;
		case Icon::Markers: //a frame
			c.Arrow({ 20, 20 }, { 56, 20 }, 4, 10, warning);
			c.Arrow({ 20, 20 }, { 20, 56 }, 4, 10, Float4({ 0.1f, 0.6f, 0.15f, 1.f }));
			c.Arrow({ 20, 20 }, { 7, 7 }, 4, 9, accent);
			break;
		case Icon::Loads: //a force on a point
			c.Arrow({ 32, 58 }, { 32, 18 }, 6, 16, warning);
			c.Disc({ 32, 11 }, 7, ink);
			break;
		case Icon::Sensors: //a gauge
			c.Arc({ 32, 20 }, 25, 5, ink, 0.f, pi);
			c.Line({ 7, 20 }, { 57, 20 }, 4, ink);
			c.Line({ 32, 20 }, { 47, 38 }, 4, warning);
			c.Disc({ 32, 20 }, 5, ink);
			break;
		case Icon::Config0: //three sliders of settings; the bar writes the number of the configuration
		case Icon::Config1:
		case Icon::Config2:
		case Icon::Config3:
			for (Index k = 0; k < 3; k++)
			{
				float y = 14.f + 16.f*(float)k;
				float knob = (k == 1) ? 36.f : (k == 0 ? 20.f : 28.f);
				c.Line({ 8, y }, { 52, y }, 3.5f, ink);
				c.Disc({ knob, y }, 6.5f, accent);
			}
			break;
		default:
			break;
		}
		rgba.resize((size_t)(iconPixels*iconPixels*4));
		for (size_t k = 0; k < rgba.size(); k++) { rgba[k] = (unsigned char)std::lround(255.f*std::max(0.f, std::min(1.f, c.rgba[k]))); }
	}

	void AddIconTextures()
	{
		std::vector<unsigned char> rgba;
		for (Index icon = 0; icon < NumberOfIcons(); icon++)
		{
			STDstring name = STDstring("exudyn.icon.") + Name(icon);
			if (textureStore.GetID(name) != -1) { continue; }
			DrawIcon(icon, rgba);
			textureStore.Set(name, iconPixels, iconPixels, TextureInterpretation::RGBA, (Index)TextureFlags::Clamp, std::vector<unsigned char>(rgba));
		}
	}
}
