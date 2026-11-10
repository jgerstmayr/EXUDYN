/** ***********************************************************************************************
* @brief        The buttons and sliders of the render window
* @details      Details:
*               Elements drawn over the scene like the icon bar, each at a location relative to the window, which
*               SC.renderer.AddButton and AddSlider create. A click or a drag changes the state of an element in the
*               renderer thread, which queues the event; the main (Python) thread calls the one user function of
*               the controls, f(id, data) with data = [state, 0, 0] (#3400).
*
* @author       Gerstmayr Johannes
* @date         2026-11-10 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */
#ifndef RENDERERCONTROLS__H
#define RENDERERCONTROLS__H

#include "Utilities/BasicDefinitions.h"
#include "Utilities/BasicFunctions.h"
#include "Graphics/GraphicsData.h" //Float4
#include <atomic>
#include <array>
#include <functional>
#include <cstdio> //snprintf
#include <vector>

namespace RendererControls
{
	//! the kinds of elements
	enum class ControlType { Button, Slider };

	//! one button or slider
	class Control
	{
	public:
		Index id = -1;								//!< the ID that AddButton or AddSlider returned
		ControlType type = ControlType::Button;
		Index view = 0;								//!< the view (window) it is drawn in
		std::array<std::array<Real, 2>, 2> location = { { { 0., 0. }, { 0.1, 0.05 } } }; //!< [[x0, y0], [x1, y1]] relative to the window, from its top left corner
		STDstring text;								//!< the label, centered; of a slider with format, the text before the value
		float fontSize = 12.f;						//!< in points as window.globalFontSize, scaled with the display scaling
		Float4 color = Float4({ 0.f, 0.f, 0.f, 1.f });				//!< the color of the text
		Float4 backgroundColor = Float4({ 0.85f, 0.87f, 0.9f, 0.8f });	//!< the alpha value is the transparency
		bool toggle = false;						//!< a button: True switches between 0 and 1, False sends 1 when released
		Real state = 0.;							//!< a button: 0 or 1; a slider: its value
		std::array<Real, 2> minMax = { 0., 1. };	//!< the range of a slider
		STDstring format = "{:.3g}";				//!< the value of a slider in its label: {} or {:.Nf}, {:.Ne}, {:.Ng}; empty: no value

		bool pressed = false;						//!< the mouse is pressed on it (drawn darker)

		//! the value of a slider at the relative position s in [0, 1] of its track
		Real SliderValue(Real s) const
		{
			s = EXUstd::Minimum(1., EXUstd::Maximum(0., s));
			return minMax[0] + s * (minMax[1] - minMax[0]);
		}
		//! the relative position in [0, 1] of the value of a slider
		Real SliderPosition() const
		{
			Real range = minMax[1] - minMax[0];
			if (range == 0.) { return 0.; }
			return EXUstd::Minimum(1., EXUstd::Maximum(0., (state - minMax[0]) / range));
		}
	};

	//! the printf format of a Python format string {} or {:.Nf}, {:.Ne}, {:.Ng}, and the texts before and after it;
	//! false for another format
	inline bool ConvertFormat(const STDstring& format, STDstring& before, STDstring& printfFormat, STDstring& after)
	{
		before = format; printfFormat = ""; after = "";
		if (format.empty()) { return true; }
		size_t open = format.find('{');
		size_t close = format.find('}', open == STDstring::npos ? 0 : open);
		if (open == STDstring::npos || close == STDstring::npos) { before = format; printfFormat = "%g"; return true; } //only a text before the value
		before = format.substr(0, open);
		after = format.substr(close + 1);
		STDstring spec = format.substr(open + 1, close - open - 1);
		if (spec.empty() || spec == ":") { printfFormat = "%g"; return true; }
		if (spec.size() >= 4 && spec[0] == ':' && spec[1] == '.')
		{
			STDstring digits = spec.substr(2, spec.size() - 3);
			char kind = spec.back();
			bool allDigits = !digits.empty() && digits.size() <= 2;
			for (char c : digits) { if (c < '0' || c > '9') { allDigits = false; } }
			if (allDigits && (kind == 'f' || kind == 'e' || kind == 'g'))
			{
				printfFormat = "%." + digits + kind;
				return true;
			}
		}
		return false;
	}

	//! the label of an element: the text of a button; of a slider the format with its value, after the text
	inline STDstring Label(const Control& control)
	{
		if (control.type == ControlType::Button || control.format.empty()) { return control.text; }
		STDstring before, printfFormat, after;
		if (!ConvertFormat(control.format, before, printfFormat, after)) { return control.text; }
		char buffer[64];
		std::snprintf(buffer, sizeof(buffer), printfFormat.c_str(), control.state);
		STDstring label = control.text;
		if (!label.empty() && !before.empty() && label.back() != ' ' && before[0] != ' ') { label += ' '; }
		return label + before + buffer + after;
	}

	//! the elements of a SystemContainer and the user function, shared by the renderer thread and Python
	class ControlList
	{
	public:
		std::atomic_flag lock = ATOMIC_FLAG_INIT;	//!< held while the list is read or changed
		std::vector<Control> items;
		Index nextID = 0;							//!< the ID of the next element
		Index activeID = -1;						//!< the element the mouse is pressed on, -1 for none
		std::function<void(Index, std::vector<Real>)> userFunction; //!< f(id, data); empty: none

		//! the index in items of an ID, -1 if there is none
		Index Find(Index id) const
		{
			for (Index i = 0; i < (Index)items.size(); i++) { if (items[i].id == id) { return i; } }
			return -1;
		}
	};
}

#endif
