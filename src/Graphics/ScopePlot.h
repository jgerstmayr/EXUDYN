/** ***********************************************************************************************
* @brief        The scopes of a view (#3237): stored sensor values plotted in a region of the render window, built as
*               one list of rectangles, lines and texts in pixels of the window, which OpenGL and the raytracer draw
*
* @author       Gerstmayr Johannes
* @date         2026-11-04
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */
#ifndef SCOPEPLOT__H
#define SCOPEPLOT__H

#include "Utilities/BasicDefinitions.h"
#include "Linalg/BasicLinalg.h"
#include <vector>
#include <atomic>

class VisualizationSystemContainerBase;
class VSettingsScope;

namespace ScopePlot
{
	//! the number of scopes of a view: scope0, scope1, scope2
	const Index numberOfScopes = 3;
	//! at most this many lines in one scope: 16 colors, solid, dotted and dashed
	const Index maxLines = 48;

	//! a character cell of the bitmap font, relative to the font size, as GlfwRenderer::DrawString and the raytracer draw it
	const float characterWidthFactor = 47.f / 64.f;
	const float characterHeightFactor = 100.f / 64.f;

	enum class ItemType { Rectangle, Line, Text };

	//! a primitive in pixels of the window from its bottom left corner: a filled rectangle p0-p1, a line p0-p1 of the
	//! given width, or a text with the bottom left corner of its first character cell at p0 and the font size as width
	class Item
	{
	public:
		ItemType type;
		Float2 p0;
		Float2 p1;
		Float4 color;
		float width;
		STDstring text;
	};

	//! the drawing of the scopes of a view; the items are drawn in their order
	class Graphics
	{
	public:
		std::vector<Item> items;
		STDstring message;	//!< a line that cannot be shown, e.g. of a sensor without storeInternal; the renderer shows it once

		void Clear() { items.clear(); message.clear(); }
		void AddRectangle(const Float2& p0, const Float2& p1, const Float4& color) { items.push_back({ ItemType::Rectangle, p0, p1, color, 0.f, "" }); }
		void AddLine(const Float2& p0, const Float2& p1, const Float4& color, float width) { items.push_back({ ItemType::Line, p0, p1, color, width, "" }); }
		void AddText(const Float2& position, const STDstring& text, const Float4& color, float fontSize) { items.push_back({ ItemType::Text, position, position, color, fontSize, text }); }
	};

	enum class StaticItemType { Line, Text, Point };

	//! an item of a scope in the coordinates of its graph, set from Python with SC.renderer.scopes (#3237): a polyline,
	//! a text with the bottom left corner of its first character at the position, or a point drawn as a circle
	class StaticItem
	{
	public:
		StaticItemType type = StaticItemType::Line;
		Index view = 0;
		Index scope = 0;
		std::vector<Vector2D> points;	//!< the points of a line; the position of a text or a point
		Float4 color = Float4({ 0.f, 0.f, 0.f, 1.f });
		float width = -1.f;			//!< the line width in pixels of a line or a point; -1: the lineWidth of the scope
		bool front = false;			//!< true: drawn over the sensor lines
		STDstring name;				//!< a line with a name has an entry in the legend
		STDstring text;
		float size = 1.f;			//!< the font size of a text relative to the tick labels
		float radius = 4.f;			//!< the radius of a point in pixels
	};

	//! the static items of all scopes of a SystemContainer, written by Python and read by BuildScopes under the lock
	class StaticGraphics
	{
	public:
		std::vector<StaticItem> items;
		std::atomic_flag lock = ATOMIC_FLAG_INIT;
	};

	//! one plotted line: its values, already reduced to what the plot can show, and its legend
	class Curve
	{
	public:
		STDstring label;
		std::vector<Real> x;
		std::vector<Real> y;
	};

	//! the color of line i: 16 colors, the first seven those of PlotLineCode as matplotlib draws them
	Float4 LineColor(Index i);
	//! the style of line i: 0 solid, 1 dotted, 2 dashed; it changes every 16 lines
	Index LineStyle(Index i);

	//! the distance of the ticks for a range: 1, 2 or 5 times a power of 10, at most about targetCount ticks
	Real TickSpacing(Real range, Real targetCount);
	//! the label of a tick, with the digits the spacing needs
	STDstring TickLabel(Real value, Real spacing);

	//! the drawing of one scope: regionPixels is [x0, y0, x1, y1] from the bottom left corner of the window, xRange
	//! the range of the horizontal axis, fontSize the font size in pixels; the static items behind and in front of
	//! the lines, outside the automatic range
	void AddScope(const VSettingsScope& scope, const std::vector<Curve>& curves, const Vector2D& xRange,
		const Float4& regionPixels, float fontSize, Graphics& graphics, const std::vector<StaticItem>& statics = {});

	//! the drawing of the scopes of a view from the stored sensor data, read under the lock of each sensor; nothing if
	//! window.showScopes is False
	void BuildScopes(VisualizationSystemContainerBase& container, Index viewID, Index width, Index height,
		float fontSize, Graphics& graphics);

	//! true the first time a message is given after ResetMessage(): the message of a line that cannot be shown comes
	//! once per start of the renderer
	bool MessageIsNew(const STDstring& message);
	//! called when the renderer starts
	void ResetMessage();
}

#endif
