/** ***********************************************************************************************
* @brief        The icons of the icon bar of the renderer (#3026): drawn by code into RGBA images when the renderer
*               starts, kept as textures 'exudyn.icon.<name>' of the TextureStore; no files
*
* @author       Gerstmayr Johannes
* @date         2026-10-19
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN

************************************************************************************************ */
#ifndef ICONBAR__H
#define ICONBAR__H

#include "Utilities/BasicDefinitions.h"
#include <vector>

namespace IconBar
{
	//! the size of an icon image in pixels
	const Index iconPixels = 64;

	//! the icons in the order of the bar (maintainer 2026-10-19)
	enum class Icon {
		Quit, ZoomAll, ZoomRegion, ViewXY, ViewYZ, ViewXZ, ViewIsometric, Ruler, MouseCoordinates, Lines, Edges,
		Faces, Transparency, Shadow, UpdateSpeed, Nodes, Bodies, Connectors, Markers, Loads, Sensors, Settings,
		Config0, Config1, Config2, Config3, NumberOfIcons	//the stored configurations CFG0 to CFG3 (#3043)
	};

	inline Index NumberOfIcons() { return (Index)Icon::NumberOfIcons; }

	//! the name of an icon, e.g. 'zoomAll'; the texture is 'exudyn.icon.' and the name
	const char* Name(Index icon);
	//! what an icon does, shown beside it when the mouse is over it
	const char* Tooltip(Index icon);

	//! the image of an icon: iconPixels x iconPixels RGBA bytes, row by row from the bottom row, as the TextureStore keeps them
	void DrawIcon(Index icon, std::vector<unsigned char>& rgba);

	//! add the icons to the TextureStore as 'exudyn.icon.<name>', if they are not there yet
	void AddIconTextures();
}

#endif
