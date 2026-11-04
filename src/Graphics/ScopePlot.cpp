/** ***********************************************************************************************
* @brief        The scopes of a view (#3237): the stored sensor values up to the visualization time, reduced to what
*               the plot can show, and the layout of axes, ticks, grid, lines and legend in pixels of the window
*
* @author       Gerstmayr Johannes
* @date         2026-11-04
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */

#include "Graphics/ScopePlot.h"
#include "Graphics/VisualizationSystemContainer.h"
#include "Main/MainSystem.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <array>
#include <memory>

namespace ScopePlot
{
	Float4 LineColor(Index i)
	{
		static const float colors[16][3] = {
			{ 1.f, 0.f, 0.f }, { 0.f, 0.5f, 0.f }, { 0.f, 0.f, 1.f }, { 0.f, 0.f, 0.f },		//r, g, b, k of PlotLineCode
			{ 0.f, 0.75f, 0.75f }, { 0.75f, 0.f, 0.75f }, { 0.75f, 0.75f, 0.f },				//c, m, y
			{ 1.f, 0.5f, 0.f }, { 0.f, 0.5f, 1.f }, { 0.65f, 0.2f, 0.2f }, { 0.5f, 0.f, 1.f },	//orange, dodger blue, brown, violet
			{ 1.f, 0.f, 0.5f }, { 0.5f, 0.5f, 0.5f }, { 0.f, 0.6f, 0.3f }, { 0.5f, 0.5f, 0.f },	//pink, grey, sea green, olive
			{ 0.f, 0.f, 0.5f } };																//navy
		Index k = ((i % 16) + 16) % 16;
		return Float4({ colors[k][0], colors[k][1], colors[k][2], 1.f });
	}

	Index LineStyle(Index i) { return (i / 16) % 3; }

	Real TickSpacing(Real range, Real targetCount)
	{
		Real target = range / std::max(targetCount, (Real)1.);
		if (!(target > 0.) || !std::isfinite(target)) { return 1.; }
		Real power = std::pow(10., std::floor(std::log10(target)));
		Real spacing = 10. * power;
		for (Real factor : { 1., 2., 5., 10. })
		{
			if (factor * power >= target * (1. - 1e-12)) { spacing = factor * power; break; }
		}
		//at least two ticks in the range: the next smaller spacing of 1, 2, 5 times a power of 10
		while (std::floor(range / spacing + 1e-9) < 2. && spacing > 1e-300)
		{
			Real factor = spacing / std::pow(10., std::floor(std::log10(spacing) + 1e-9));
			spacing *= (std::fabs(factor - 2.) < 1e-6) ? 0.5 : (std::fabs(factor - 5.) < 1e-6 ? 0.4 : 0.5);
		}
		return spacing;
	}

	STDstring TickLabel(Real value, Real spacing)
	{
		char buffer[40];
		if (std::fabs(value) < 1e-9 * spacing) { value = 0.; }
		Real magnitude = std::max(std::fabs(value), spacing);
		if (magnitude >= 1e5 || spacing < 1e-4) //exponent form, with the significant digits the spacing needs
		{
			int digits = (int)std::max(1., std::floor(std::log10(magnitude)) - std::floor(std::log10(spacing) + 1e-9) + 1.);
			snprintf(buffer, sizeof(buffer), "%.*g", digits, value);
		}
		else
		{
			int digits = (int)std::max(0., -std::floor(std::log10(spacing) + 1e-9));
			snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
		}
		return STDstring(buffer);
	}

	//! the part of the segment a-b inside box [x0, y0, x1, y1] (Liang-Barsky); false if nothing is inside
	bool ClipSegment(Float2& a, Float2& b, const Float4& box)
	{
		float t0 = 0.f, t1 = 1.f;
		float dx = b[0] - a[0], dy = b[1] - a[1];
		const float p[4] = { -dx, dx, -dy, dy };
		const float q[4] = { a[0] - box[0], box[2] - a[0], a[1] - box[1], box[3] - a[1] };
		for (Index k = 0; k < 4; k++)
		{
			if (p[k] == 0.f)
			{
				if (q[k] < 0.f) { return false; }
			}
			else
			{
				float t = q[k] / p[k];
				if (p[k] < 0.f) { if (t > t1) { return false; } t0 = std::max(t0, t); }
				else { if (t < t0) { return false; } t1 = std::min(t1, t); }
			}
		}
		Float2 start = a;
		a = Float2({ start[0] + t0 * dx, start[1] + t0 * dy });
		b = Float2({ start[0] + t1 * dx, start[1] + t1 * dy });
		return true;
	}

	//! a line through points, clipped to box; style 1 dotted and 2 dashed by the length along the line
	void AddPolyline(const std::vector<Float2>& points, const Float4& box, const Float4& color, float width, Index style, Graphics& graphics)
	{
		const float on = (style == 1) ? width : 6.f * width;
		const float period = on + ((style == 1) ? 2.f * width : 4.f * width);
		float phase = 0.f; //the position in the pattern
		for (size_t i = 1; i < points.size(); i++)
		{
			Float2 a = points[i - 1], b = points[i];
			if (style == 0)
			{
				if (ClipSegment(a, b, box)) { graphics.AddLine(a, b, color, width); }
				continue;
			}
			float length = (b - a).GetL2Norm();
			float s = 0.f;
			while (s < length)
			{
				float inPattern = std::fmod(phase, period);
				float end = std::min(length, s + (inPattern < on ? on - inPattern : period - inPattern));
				if (inPattern < on)
				{
					Float2 d0 = a + (s / length) * (b - a), d1 = a + (end / length) * (b - a);
					if (ClipSegment(d0, d1, box)) { graphics.AddLine(d0, d1, color, width); }
				}
				phase = std::fmod(phase + (end - s), period);
				s = end;
			}
		}
	}

	void AddFrame(const Float2& p0, const Float2& p1, const Float4& color, float width, Graphics& graphics)
	{
		graphics.AddLine(Float2({ p0[0], p0[1] }), Float2({ p1[0], p0[1] }), color, width);
		graphics.AddLine(Float2({ p1[0], p0[1] }), Float2({ p1[0], p1[1] }), color, width);
		graphics.AddLine(Float2({ p1[0], p1[1] }), Float2({ p0[0], p1[1] }), color, width);
		graphics.AddLine(Float2({ p0[0], p1[1] }), Float2({ p0[0], p0[1] }), color, width);
	}

	//! the sub-ticks between two ticks: 4 for a spacing of 2 times a power of 10, else 5
	Index SubTicks(Real spacing)
	{
		Real factor = spacing / std::pow(10., std::floor(std::log10(spacing) + 1e-9));
		return (std::fabs(factor - 2.) < 1e-6) ? 4 : 5;
	}

	//! the static items of a scope with the given front flag, clipped to box; a text only if its position is inside
	template<class TX, class TY>
	void AddStaticItems(const std::vector<StaticItem>& statics, bool front, const Float4& box, float lineWidth,
		float fontSize, TX X, TY Y, Graphics& graphics)
	{
		const Index circleSegments = 16;
		std::vector<Float2> points;
		for (const StaticItem& item : statics)
		{
			if (item.front != front || item.points.empty()) { continue; }
			const float width = item.width > 0.f ? item.width : lineWidth;
			const Float2 p0({ X(item.points[0][0]), Y(item.points[0][1]) });
			points.clear();
			if (item.type == StaticItemType::Line)
			{
				for (const Vector2D& point : item.points) { points.push_back(Float2({ X(point[0]), Y(point[1]) })); }
			}
			else if (item.type == StaticItemType::Point)
			{
				for (Index k = 0; k <= circleSegments; k++)
				{
					float phi = 2.f * (float)EXUstd::pi * (float)k / (float)circleSegments;
					points.push_back(Float2({ p0[0] + item.radius * std::cos(phi), p0[1] + item.radius * std::sin(phi) }));
				}
			}
			else if (p0[0] >= box[0] && p0[0] <= box[2] && p0[1] >= box[1] && p0[1] <= box[3])
			{
				graphics.AddText(p0, item.text, item.color, item.size * fontSize);
			}
			AddPolyline(points, box, item.color, width, 0, graphics);
		}
	}

	void AddScope(const VSettingsScope& scope, const std::vector<Curve>& curves, const Vector2D& xRange,
		const Float4& regionPixels, float fontSize, Graphics& graphics, const std::vector<StaticItem>& statics)
	{
		const float lineWidth = scope.lineWidth;
		const float cw = characterWidthFactor * fontSize;
		const float ch = characterHeightFactor * fontSize;
		const float pad = 0.4f * fontSize;
		const float x0 = regionPixels[0], y0 = regionPixels[1], x1 = regionPixels[2], y1 = regionPixels[3];

		graphics.AddRectangle(Float2({ x0, y0 }), Float2({ x1, y1 }), scope.backgroundColor);
		AddFrame(Float2({ x0, y0 }), Float2({ x1, y1 }), scope.frameColor, lineWidth, graphics);

		//the ranges
		Real xMin = xRange[0], xMax = xRange[1];
		if (!(xMax > xMin)) { xMax = xMin + 1.; }
		Real yMin = scope.verticalZoom[0], yMax = scope.verticalZoom[1];
		if (yMin == 0. && yMax == 0.)
		{
			yMin = EXUstd::MAXREAL; yMax = EXUstd::LOWESTREAL;
			for (const Curve& curve : curves)
			{
				for (size_t i = 0; i < curve.y.size(); i++)
				{
					if (curve.x[i] >= xMin && curve.x[i] <= xMax && std::isfinite(curve.y[i])) { yMin = std::min(yMin, curve.y[i]); yMax = std::max(yMax, curve.y[i]); }
				}
			}
			if (yMin > yMax) { yMin = 0.; yMax = 1.; }		//no values
			Real size = yMax - yMin;
			if (size <= 1e-12 * std::max((Real)1., std::fabs(yMin))) //a constant signal
			{
				size = (yMin != 0.) ? 0.02 * std::fabs(yMin) : 2.;
				yMin -= 0.5 * size; yMax += 0.5 * size;
			}
			else { yMin -= 0.05 * size; yMax += 0.05 * size; }
		}
		if (!(yMax > yMin)) { yMax = yMin + 1.; }

		//the plot area; the labels of the vertical axis decide the left margin; a small scope leaves out the axis labels
		const bool withLabels = (y1 - y0) > 5.5f * ch;
		const STDstring xLabel = withLabels ? scope.xLabel : STDstring();
		const STDstring yLabel = withLabels ? scope.yLabel : STDstring();
		const float top = y1 - pad - (yLabel.size() ? ch : 0.5f * ch);
		const float bottom = y0 + pad + ch + (xLabel.size() ? ch : 0.f);
		if (top - bottom < 1.5f * ch) { return; } //nothing more fits
		Real ySpacing = TickSpacing(yMax - yMin, std::max(2.f, (top - bottom) / (2.5f * ch)));
		Index yFirst = (Index)std::ceil(yMin / ySpacing - 1e-9), yLast = (Index)std::floor(yMax / ySpacing + 1e-9);
		if (yLast - yFirst > 100) { yLast = yFirst + 100; }
		size_t yLabelChars = 1;
		for (Index k = yFirst; k <= yLast; k++) { yLabelChars = std::max(yLabelChars, TickLabel(k * ySpacing, ySpacing).size()); }
		const float left = x0 + pad + (float)yLabelChars * cw + 0.5f * pad;
		const float right = x1 - pad - 2.f * cw;
		if (right - left < 4.f * fontSize) { return; }
		const Float4 box({ left, bottom, right, top });

		auto X = [&](Real x) { return (float)(left + (x - xMin) / (xMax - xMin) * (right - left)); };
		auto Y = [&](Real y) { return (float)(bottom + (y - yMin) / (yMax - yMin) * (top - bottom)); };

		Float4 gridColor = scope.axesColor;
		gridColor[3] *= 0.25f;
		Float4 subGridColor = scope.axesColor;
		subGridColor[3] *= 0.12f;
		const float tickLength = 0.4f * fontSize;

		//the ticks, sub-ticks, grid and labels of both axes
		Real xSpacing = TickSpacing(xMax - xMin, std::max(2.f, (right - left) / std::max(80.f, 8.f * cw)));
		for (Index axis = 0; axis < 2; axis++)
		{
			Real spacing = axis == 0 ? xSpacing : ySpacing;
			Real minValue = axis == 0 ? xMin : yMin;
			Real maxValue = axis == 0 ? xMax : yMax;
			Index nSub = SubTicks(spacing);
			Real subSpacing = spacing / (Real)nSub;
			Index first = (Index)std::ceil(minValue / subSpacing - 1e-9), last = (Index)std::floor(maxValue / subSpacing + 1e-9);
			if (last - first > 1000) { last = first + 1000; }
			for (Index k = first; k <= last; k++)
			{
				bool major = (k % nSub) == 0;
				Real value = k * subSpacing;
				float width = major ? lineWidth : 0.5f * lineWidth;
				float length = major ? tickLength : 0.6f * tickLength;
				if (axis == 0)
				{
					float x = X(value);
					if ((major && scope.useGrid >= 1) || (!major && scope.useGrid >= 2)) { graphics.AddLine(Float2({ x, bottom }), Float2({ x, top }), major ? gridColor : subGridColor, 0.5f * lineWidth); }
					graphics.AddLine(Float2({ x, bottom }), Float2({ x, bottom + length }), scope.axesColor, width);
					if (major)
					{
						STDstring label = TickLabel(value, spacing);
						graphics.AddText(Float2({ x - 0.5f * (float)label.size() * cw, bottom - 0.2f * fontSize - ch }), label, scope.axesColor, fontSize);
					}
				}
				else
				{
					float y = Y(value);
					if ((major && scope.useGrid >= 1) || (!major && scope.useGrid >= 2)) { graphics.AddLine(Float2({ left, y }), Float2({ right, y }), major ? gridColor : subGridColor, 0.5f * lineWidth); }
					graphics.AddLine(Float2({ left, y }), Float2({ left + length, y }), scope.axesColor, width);
					if (major)
					{
						STDstring label = TickLabel(value, spacing);
						graphics.AddText(Float2({ left - 0.3f * fontSize - (float)label.size() * cw, y - 0.5f * ch }), label, scope.axesColor, fontSize);
					}
				}
			}
		}
		AddFrame(Float2({ left, bottom }), Float2({ right, top }), scope.axesColor, lineWidth, graphics);
		if (xLabel.size())
		{
			graphics.AddText(Float2({ 0.5f * (left + right) - 0.5f * (float)xLabel.size() * cw, y0 + 0.5f * pad }), xLabel, scope.axesColor, fontSize);
		}
		if (yLabel.size())
		{
			graphics.AddText(Float2({ left, top + 0.2f * pad }), yLabel, scope.axesColor, fontSize);
		}

		//the lines, between the static items behind and in front of them
		AddStaticItems(statics, false, box, lineWidth, fontSize, X, Y, graphics);
		std::vector<Float2> points;
		for (size_t i = 0; i < curves.size(); i++)
		{
			const Curve& curve = curves[i];
			points.clear();
			for (size_t k = 0; k < curve.y.size(); k++)
			{
				if (std::isfinite(curve.x[k]) && std::isfinite(curve.y[k])) { points.push_back(Float2({ X(curve.x[k]), Y(curve.y[k]) })); }
			}
			AddPolyline(points, box, LineColor((Index)i), lineWidth, LineStyle((Index)i), graphics);
		}
		AddStaticItems(statics, true, box, lineWidth, fontSize, X, Y, graphics);

		//the legend in the top right corner, as many entries as fit: the sensor lines, then the named static lines
		class LegendEntry
		{
		public:
			STDstring label;
			Float4 color;
			Index style;
			float width;
		};
		std::vector<LegendEntry> legend;
		for (size_t i = 0; i < curves.size(); i++) { legend.push_back({ curves[i].label, LineColor((Index)i), LineStyle((Index)i), lineWidth }); }
		for (const StaticItem& item : statics)
		{
			if (item.type == StaticItemType::Line && item.name.size()) { legend.push_back({ item.name, item.color, 0, item.width > 0.f ? item.width : lineWidth }); }
		}
		if (scope.showLegend && legend.size())
		{
			const float sample = 2.f * fontSize;
			size_t maxChars = 3;
			for (const LegendEntry& entry : legend) { maxChars = std::max(maxChars, entry.label.size()); }
			size_t fittingChars = (size_t)std::max(3.f, (0.7f * (right - left) - sample - 2.f * pad) / cw);
			maxChars = std::min(maxChars, fittingChars);
			size_t entries = std::min(legend.size(), (size_t)std::max(1.f, (top - bottom - pad) / ch));
			bool truncated = entries < legend.size();
			const float lx1 = right - 0.5f * pad, lx0 = lx1 - (sample + 2.f * pad + (float)maxChars * cw);
			const float ly1 = top - 0.5f * pad, ly0 = ly1 - (float)entries * ch - 0.5f * pad;
			Float4 legendColor = scope.backgroundColor;
			legendColor[3] *= 0.6f; //the lines stay visible under the legend
			graphics.AddRectangle(Float2({ lx0, ly0 }), Float2({ lx1, ly1 }), legendColor);
			AddFrame(Float2({ lx0, ly0 }), Float2({ lx1, ly1 }), gridColor, 0.5f * lineWidth, graphics);
			for (size_t i = 0; i < entries; i++)
			{
				float y = ly1 - ((float)i + 1.f) * ch;
				STDstring label = (truncated && i == entries - 1) ? STDstring("...") : legend[i].label;
				if (label.size() > maxChars) { label = label.substr(0, maxChars - 2) + ".."; }
				if (!(truncated && i == entries - 1))
				{
					std::vector<Float2> sampleLine = { Float2({ lx0 + 0.5f * pad, y + 0.45f * ch }), Float2({ lx0 + 0.5f * pad + sample, y + 0.45f * ch }) };
					AddPolyline(sampleLine, Float4({ lx0, ly0, lx1, ly1 }), legend[i].color, legend[i].width, legend[i].style, graphics);
				}
				graphics.AddText(Float2({ lx0 + pad + sample, y }), label, scope.axesColor, fontSize);
			}
		}
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//the data

	//! the component of a sensor in a legend: X, Y, Z or XX ... XY as PlotSensor names them, else [i]
	STDstring ComponentName(const STDstring& variable, Index component)
	{
		STDstring name = variable;
		size_t position = name.find("Local");
		if (position != STDstring::npos) { name.erase(position, 5); }
		static const char* vectors[] = { "Force", "Torque", "Position", "Displacement", "Velocity", "Acceleration", "Rotation",
			"AngularVelocity", "AngularAcceleration" };
		static const char* xyz[] = { "X", "Y", "Z" };
		static const char* tensors[] = { "XX", "YY", "ZZ", "YZ", "XZ", "XY" };
		if (component >= 0 && component < 3)
		{
			for (const char* vector : vectors) { if (name == vector) { return xyz[component]; } }
		}
		if (component >= 0 && component < 6 && (name == "Strain" || name == "Stress")) { return tensors[component]; }
		return "[" + EXUstd::ToString(component) + "]";
	}

	//! the name of the measured quantity of a sensor: its output variable, or Load or UserFunction
	STDstring VariableName(const CSensor& sensor)
	{
		SensorType type = sensor.GetType();
		if (type == SensorType::Load || type == SensorType::UserFunction) { return GetSensorTypeString(type); }
		return GetOutputVariableTypeString(sensor.GetOutputVariableType());
	}

	//! a line in the legend: the name of the sensor, and for a sensor of more than one component the component,
	//! "m0.Pos-X" or "m0.F[3]"; the name says what the sensor measures (#3248)
	STDstring LineName(const MainSensor& mainSensor, const CSensor& sensor, Index component, Index components)
	{
		if (components <= 1) { return mainSensor.GetName(); }
		STDstring componentName = ComponentName(VariableName(sensor), component);
		return mainSensor.GetName() + (componentName[0] == '[' ? "" : "-") + componentName;
	}

	static STDstring scopeMessage;

	bool MessageIsNew(const STDstring& message)
	{
		if (message.empty() || message == scopeMessage) { return false; }
		scopeMessage = message;
		return true;
	}

	void ResetMessage() { scopeMessage.clear(); }

	//! one line of a scope: a component of sensorY over time (sensorX nullptr) or over a component of sensorX
	class LineSource
	{
	public:
		const CSensor* sensorY;
		const CSensor* sensorX;
		Index componentY;
		Index componentX;
		Real time;		//!< the visualization time of its system
		STDstring label;
	};

	//! the values of a row of the stored data that are shown: up to the visualization time, and the last timeHistory seconds
	inline bool UseRow(const ResizableMatrix& data, Index i, Real time, Real timeHistory)
	{
		Real t = data(i, 0);
		return t <= time + 1e-10 * std::max((Real)1., std::fabs(time)) && (timeHistory <= 0. || t >= time - timeHistory);
	}

	//! the lines of a scope from its rows [sensorY, componentY, sensorX, componentX, mbsNumber]
	void LineSources(VisualizationSystemContainerBase& container, const std::vector<std::array<Index, 5>>& rows,
		std::vector<LineSource>& sources, STDstring& message)
	{
		auto Message = [&message](const STDstring& text) { if (message.empty()) { message = "Scope: " + text; } };
		for (const std::array<Index, 5>& row : rows)
		{
			Index mbsNumber = row[4];
			MainSystem* mainSystem = (mbsNumber >= 0 && mbsNumber < container.NumberOFMainSystemsBacklink()) ? container.GetMainSystemBacklink(mbsNumber) : nullptr;
			if (mainSystem == nullptr) { Message("there is no mbs " + EXUstd::ToString(mbsNumber)); continue; }
			const ResizableArray<CSensor*>& sensors = mainSystem->GetCSystem().GetSystemData().GetCSensors();
			const ResizableArray<MainSensor*>& mainSensors = mainSystem->GetMainSystemData().GetMainSensors();
			auto Sensor = [&](Index number) -> const CSensor*
			{
				if (number < 0 || number >= sensors.NumberOfItems() || number >= mainSensors.NumberOfItems())
				{
					Message("there is no sensor " + EXUstd::ToString(number) + " in mbs " + EXUstd::ToString(mbsNumber));
					return nullptr;
				}
				if (!sensors[number]->GetStoreInternalFlag())
				{
					Message("sensor " + EXUstd::ToString(number) + " of mbs " + EXUstd::ToString(mbsNumber) + " is not shown, as it needs storeInternal=True");
					return nullptr;
				}
				return sensors[number];
			};
			const CSensor* sensorY = Sensor(row[0]);
			const CSensor* sensorX = row[2] >= 0 ? Sensor(row[2]) : nullptr;
			if (sensorY == nullptr || (row[2] >= 0 && sensorX == nullptr)) { continue; }

			sensorY->LockInternalStorage();
			Index componentsY = std::max((Index)0, sensorY->GetInternalStorage().NumberOfColumns() - 1);
			sensorY->ReleaseInternalStorage();
			Index componentsX = 0;
			if (sensorX)
			{
				sensorX->LockInternalStorage();
				componentsX = std::max((Index)0, sensorX->GetInternalStorage().NumberOfColumns() - 1);
				sensorX->ReleaseInternalStorage();
				if (componentsX != 0 && (row[3] < 0 || row[3] >= componentsX))
				{
					Message("sensor " + EXUstd::ToString(row[2]) + " has no component " + EXUstd::ToString(row[3]));
					continue;
				}
			}
			if (componentsY == 0 || (sensorX && componentsX == 0)) { continue; } //no values stored yet
			if (row[1] >= componentsY)
			{
				Message("sensor " + EXUstd::ToString(row[0]) + " has no component " + EXUstd::ToString(row[1]));
				continue;
			}
			Real time = mainSystem->GetCSystem().GetSystemData().GetCData().GetVisualization().GetTime();
			Index firstComponent = row[1] < 0 ? 0 : row[1];
			Index lastComponent = row[1] < 0 ? componentsY - 1 : row[1];
			STDstring overX; //a line over another sensor: "m0.Pos-Y over m0.Pos-X"
			if (sensorX) { overX = " over " + LineName(*mainSensors[row[2]], *sensorX, row[3], componentsX); }
			for (Index component = firstComponent; component <= lastComponent && (Index)sources.size() < maxLines; component++)
			{
				sources.push_back({ sensorY, sensorX, component, row[3], time,
					LineName(*mainSensors[row[0]], *sensorY, component, componentsY) + overX });
			}
		}
	}

	//! the range of the horizontal axis: horizontalZoom, else the last timeHistory seconds of the time, else the values
	Vector2D HorizontalRange(const VSettingsScope& scope, const std::vector<LineSource>& sources)
	{
		if (scope.horizontalZoom[0] != 0.f || scope.horizontalZoom[1] != 0.f)
		{
			return Vector2D({ (Real)scope.horizontalZoom[0], (Real)scope.horizontalZoom[1] });
		}
		Real xMin = EXUstd::MAXREAL, xMax = EXUstd::LOWESTREAL;
		Real tFirst = EXUstd::MAXREAL, tLatest = EXUstd::LOWESTREAL;
		bool overTime = false, overSensor = false;
		for (const LineSource& source : sources)
		{
			const CSensor* sensor = source.sensorX ? source.sensorX : source.sensorY;
			sensor->LockInternalStorage();
			const ResizableMatrix& data = sensor->GetInternalStorage();
			for (Index i = 0; i < data.NumberOfRows(); i++)
			{
				if (!UseRow(data, i, source.time, scope.timeHistory)) { continue; }
				if (source.sensorX)
				{
					Real x = data(i, source.componentX + 1);
					if (std::isfinite(x)) { xMin = std::min(xMin, x); xMax = std::max(xMax, x); overSensor = true; }
				}
				else
				{
					tFirst = std::min(tFirst, data(i, 0)); tLatest = std::max(tLatest, data(i, 0)); overTime = true;
				}
			}
			sensor->ReleaseInternalStorage();
		}
		if (overSensor) //a margin as in the vertical direction
		{
			Real size = std::max(xMax - xMin, 1e-12 * std::max((Real)1., std::fabs(xMin)));
			xMin -= 0.05 * size; xMax += 0.05 * size;
		}
		if (overTime)
		{
			if (scope.timeHistory > 0.) //a window of timeHistory seconds, which scrolls once the time exceeds it
			{
				Real time = sources[0].time;
				tFirst = std::max(tFirst, time - (Real)scope.timeHistory);
				tLatest = tFirst + (Real)scope.timeHistory;
			}
			xMin = std::min(xMin, tFirst); xMax = std::max(xMax, tLatest);
		}
		if (xMin > xMax) { return Vector2D({ 0., 1. }); }
		return Vector2D({ xMin, xMax });
	}

	//! the values of a line, reduced to bins values per range: over time the minimum and maximum of each bin in their
	//! order, so that peaks stay, with one value on each side outside the range; over a sensor every k-th value
	void CollectCurve(const LineSource& source, const VSettingsScope& scope, const Vector2D& xRange, Index bins, Curve& curve)
	{
		curve.label = source.label;
		source.sensorY->LockInternalStorage();
		if (source.sensorX && source.sensorX != source.sensorY) { source.sensorX->LockInternalStorage(); }
		const ResizableMatrix& dataY = source.sensorY->GetInternalStorage();
		const ResizableMatrix& dataX = source.sensorX ? source.sensorX->GetInternalStorage() : dataY;
		Index rows = std::min(dataY.NumberOfRows(), dataX.NumberOfRows());
		Index columnY = source.componentY + 1;
		Real timeHistory = scope.timeHistory;
		if (source.sensorX)
		{
			Index used = 0;
			for (Index i = 0; i < rows; i++) { if (UseRow(dataY, i, source.time, timeHistory)) { used++; } }
			Index stride = std::max((Index)1, used / std::max((Index)1, bins));
			Index count = 0;
			for (Index i = 0; i < rows; i++)
			{
				if (!UseRow(dataY, i, source.time, timeHistory)) { continue; }
				if (count++ % stride == 0 || count == used) { curve.x.push_back(dataX(i, source.componentX + 1)); curve.y.push_back(dataY(i, columnY)); }
			}
		}
		else
		{
			const Real xMin = xRange[0], xMax = xRange[1];
			const Real binSize = (xMax - xMin) / (Real)std::max((Index)1, bins);
			Index bin = -1;
			Real xLow = 0., yLow = 0., xHigh = 0., yHigh = 0.; //the minimum and the maximum of the bin
			auto Flush = [&]()
			{
				if (bin < 0) { return; }
				bool lowFirst = xLow <= xHigh;
				curve.x.push_back(lowFirst ? xLow : xHigh); curve.y.push_back(lowFirst ? yLow : yHigh);
				if (xHigh != xLow || yHigh != yLow) { curve.x.push_back(lowFirst ? xHigh : xLow); curve.y.push_back(lowFirst ? yHigh : yLow); }
				bin = -1;
			};
			bool insideBefore = false;
			bool hasOutside = false;
			Real xOutside = 0., yOutside = 0.;
			for (Index i = 0; i < rows; i++)
			{
				if (!UseRow(dataY, i, source.time, timeHistory)) { continue; }
				Real x = dataY(i, 0), y = dataY(i, columnY);
				if (!std::isfinite(y)) { continue; }
				if (x < xMin || x > xMax)
				{
					if (insideBefore) { Flush(); curve.x.push_back(x); curve.y.push_back(y); insideBefore = false; } //the first value after the range
					else { hasOutside = true; xOutside = x; yOutside = y; } //the last value before it
					continue;
				}
				if (hasOutside) { curve.x.push_back(xOutside); curve.y.push_back(yOutside); hasOutside = false; }
				insideBefore = true;
				Index newBin = std::min(bins - 1, (Index)((x - xMin) / binSize));
				if (newBin != bin)
				{
					Flush();
					bin = newBin; xLow = xHigh = x; yLow = yHigh = y;
				}
				else
				{
					if (y < yLow) { yLow = y; xLow = x; }
					if (y > yHigh) { yHigh = y; xHigh = x; }
				}
			}
			Flush();
		}
		if (source.sensorX && source.sensorX != source.sensorY) { source.sensorX->ReleaseInternalStorage(); }
		source.sensorY->ReleaseInternalStorage();
	}

	void BuildScopes(VisualizationSystemContainerBase& container, Index viewID, Index width, Index height,
		float fontSize, Graphics& graphics)
	{
		graphics.Clear();
		const VSettingsWindow& window = GetSettingsView(viewID, container.GetVisualizationSettings()).window;
		if (!window.showScopes || width <= 0 || height <= 0) { return; }
		const VSettingsScope* scopes[numberOfScopes] = { &window.scope0, &window.scope1, &window.scope2 };
		std::vector<std::unique_ptr<SystemAccessGuard>> systemAccess; //the sensors stay while they are read (#3239)
		for (Index i = 0; i < container.NumberOFMainSystemsBacklink(); i++)
		{
			if (container.GetMainSystemBacklink(i))
			{
				systemAccess.push_back(std::make_unique<SystemAccessGuard>(*container.GetMainSystemBacklink(i)->GetCSystem().GetPostProcessData(), false));
			}
		}

		//without any sensor list, the first three sensors of mbs 0 that store their values, one per scope
		bool automatic = true;
		for (const VSettingsScope* scope : scopes) { if (scope->sensors.NumberOfRows() != 0) { automatic = false; } }
		std::vector<Index> automaticSensors;
		if (automatic && container.NumberOFMainSystemsBacklink() > 0 && container.GetMainSystemBacklink(0))
		{
			const ResizableArray<CSensor*>& sensors = container.GetMainSystemBacklink(0)->GetCSystem().GetSystemData().GetCSensors();
			for (Index i = 0; i < sensors.NumberOfItems() && (Index)automaticSensors.size() < numberOfScopes; i++)
			{
				if (sensors[i]->GetStoreInternalFlag()) { automaticSensors.push_back(i); }
			}
		}

		std::vector<StaticItem> viewStatics; //copied, so that Python may set them meanwhile
		StaticGraphics& staticGraphics = container.GetScopeStaticGraphics();
		EXUstd::WaitAndLockSemaphore(staticGraphics.lock);
		for (const StaticItem& item : staticGraphics.items) { if (item.view == viewID) { viewStatics.push_back(item); } }
		EXUstd::ReleaseSemaphore(staticGraphics.lock);

		for (Index j = 0; j < numberOfScopes; j++)
		{
			const VSettingsScope& scope = *scopes[j];
			std::vector<std::array<Index, 5>> rows;
			if (automatic)
			{
				if (j < (Index)automaticSensors.size()) { rows.push_back({ automaticSensors[j], -1, -1, 0, 0 }); }
			}
			else
			{
				const MatrixI& sensors = scope.sensors;
				if (sensors.NumberOfRows() != 0 && sensors.NumberOfColumns() != 2 && sensors.NumberOfColumns() != 5)
				{
					if (graphics.message.empty()) { graphics.message = "Scope: the sensors of scope" + EXUstd::ToString(j) + " need rows of 2 or 5 integers"; }
					continue;
				}
				for (Index i = 0; i < sensors.NumberOfRows(); i++)
				{
					if (sensors.NumberOfColumns() == 2) { rows.push_back({ sensors(i, 0), sensors(i, 1), -1, 0, 0 }); }
					else { rows.push_back({ sensors(i, 0), sensors(i, 1), sensors(i, 2), sensors(i, 3), sensors(i, 4) }); }
				}
			}
			std::vector<StaticItem> statics;
			for (const StaticItem& item : viewStatics) { if (item.scope == j) { statics.push_back(item); } }
			if (rows.empty() && statics.empty()) { continue; }

			Float4 regionPixels({ scope.region[0] * (float)width, (1.f - scope.region[3]) * (float)height,
				scope.region[2] * (float)width, (1.f - scope.region[1]) * (float)height });
			if (!(regionPixels[2] > regionPixels[0] && regionPixels[3] > regionPixels[1])) { continue; }

			std::vector<LineSource> sources;
			LineSources(container, rows, sources, graphics.message);
			std::vector<Curve> curves(sources.size());
			Vector2D xRange = sources.size() ? HorizontalRange(scope, sources) : Vector2D({ 0., 1. });
			Index bins = std::max((Index)1, (Index)(scope.xResolution * (regionPixels[2] - regionPixels[0])));
			for (size_t i = 0; i < sources.size(); i++) { CollectCurve(sources[i], scope, xRange, bins, curves[i]); }
			AddScope(scope, curves, xRange, regionPixels, fontSize, graphics, statics);
		}
	}
}
