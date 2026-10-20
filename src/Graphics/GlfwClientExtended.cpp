/** ***********************************************************************************************
* @brief        Implementation of extended functions for GlfwClient (bitmap, texture, ...)
*
* @author       Gerstmayr Johannes
* @date         2020-12-29 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "Utilities/AdvancedStuff.h"

#include "Utilities/SlimArray.h"
//#include <string>
#include "Utilities/SlimArray.h"

//the following is used for Sphere generation, could be replaced!
#include "Linalg/RigidBodyMath.h"
#include "Graphics/VisualizationSystemContainer.h"
#include "Graphics/VisualizationPrimitives.h"

#ifdef USE_GLFW_GRAPHICS

#include <ostream>
//#include <stdlib.h> //only works in MSVC for initialization with std::vector
#include <array>
#include <vector>
 
//different gllists for nodes/etc
//clean up texture extensions

////#define GLFW_INCLUDE_ES3 //open gl ES version
//#define GLFW_INCLUDE_GLEXT
//#include <GLFW/glfw3.h>

////#define MULTISAMPLING_TEXTURE //not needed, as this is done already with linear filter; however, RGB-sub-pixel rasterization would improve, but not available for lower OpenGL versions;
//#ifdef MULTISAMPLING_TEXTURE
////for multisampling: (only supported in OpenGL 3.2 onwards, which does not work in Windows out of the box ...)
//#define myGlTexImage2D glTexImage2DMultisample 
//#define MY_GL_TEXTURE_2D GL_TEXTURE_2D_MULTISAMPLE
//#else
////without multisampling:
//#define myGlTexImage2D glTexImage2D
//#define MY_GL_TEXTURE_2D GL_TEXTURE_2D
//#endif

#define MY_GL_TEXTURE_2D GL_TEXTURE_2D
#include "Graphics/GlfwClient.h"
#include "Graphics/characterBitmap.h"
#include "Graphics/IconBar.h"
#include "Graphics/TextureStore.h"


//for GetItemInformation, MainSystem*
#include "Main/MainSystemData.h"
#include "Main/MainSystem.h"
//#include "Pymodules/PybindUtilities.h"
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>       //interface to numpy
#include <pybind11/buffer_info.h> //passing reference to matrix to numpy
#include <pybind11/embed.h>       //scoped interpreter
//does not work globally: #include <pybind11/iostream.h> //used to redirect cout:  py::scoped_ostream_redirect output;
#include <pybind11/cast.h> //for arguments

//extern py::dict PyGetInternalSelectionDict();
//extern void PySetInternalSelectionDict(py::dict dict);
extern void PyWriteToSysDictionary(const STDstring& key, py::object item);


//! write dictionary for selected item; return true if success; MAY ONLY BE CALLED FROM PYTHON THREAD!!!
bool GlfwRenderer::PySetRendererSelectionDict(Index itemID)
{
	Index itemIndex;
	ItemType itemType;
	Index mbsNumber;
	ItemID2IndexType(itemID, itemIndex, itemType, mbsNumber);
	if (mbsNumber >= basicVisualizationSystemContainer->NumberOFMainSystemsBacklink()) { return false; }
	MainSystem* mainSystem = basicVisualizationSystemContainer->GetMainSystemBacklink(mbsNumber);
	py::dict itemDict;
	switch (itemType)
	{
	case ItemType::_None:
		return false; break;
	case ItemType::Node:
		if (itemIndex < mainSystem->mainSystemData.GetMainNodes().NumberOfItems()) {
			itemDict = mainSystem->mainSystemData.GetMainNodes().GetItem(itemIndex)->GetDictionary();
			//itemTypeName = "Node" + py::cast<STDstring>(itemDict["nodeType"]);
			//itemName = py::cast<STDstring>(itemDict["name"]);
		}
		else { return false; }
		break;
	case ItemType::Object:
		if (itemIndex < mainSystem->mainSystemData.GetMainObjects().NumberOfItems()) {
			itemDict = mainSystem->mainSystemData.GetMainObjects().GetItem(itemIndex)->GetDictionary(visSettings->interactive.advanced.selectionRightMouseGraphicsData);
			//itemTypeName = "Object" + py::cast<STDstring>(itemDict["objectType"]);
			//itemName = py::cast<STDstring>(itemDict["name"]);
		}
		else { return false; }
		break;
	case ItemType::Marker:
		if (itemIndex < mainSystem->mainSystemData.GetMainMarkers().NumberOfItems()) {
			itemDict = mainSystem->mainSystemData.GetMainMarkers().GetItem(itemIndex)->GetDictionary();
			//itemTypeName = "Marker" + py::cast<STDstring>(itemDict["markerType"]);
			//itemName = py::cast<STDstring>(itemDict["name"]);
		}
		else { return false; }
		break;
	case ItemType::Load:
		if (itemIndex < mainSystem->mainSystemData.GetMainLoads().NumberOfItems()) {
			itemDict = mainSystem->mainSystemData.GetMainLoads().GetItem(itemIndex)->GetDictionary();
			//itemTypeName = "Load" + py::cast<STDstring>(itemDict["loadType"]);
			//itemName = py::cast<STDstring>(itemDict["name"]);
		}
		else { return false; }
		break;
	case ItemType::Sensor:
		if (itemIndex < mainSystem->mainSystemData.GetMainSensors().NumberOfItems()) {
			itemDict = mainSystem->mainSystemData.GetMainSensors().GetItem(itemIndex)->GetDictionary();
			//itemTypeName = "Sensor" + py::cast<STDstring>(itemDict["sensorType"]);
			//itemName = py::cast<STDstring>(itemDict["name"]);
		}
		else { return false; }
		break;
	default:
		return false;
		break;
	}
	PyWriteToSysDictionary("currentRendererSelectionDict", itemDict);
	return true;
}

//! retrieve basic item information from MainSystemBacklink; return true if success; thread safe (no Python calls)
bool GlfwRenderer::GetItemInformation(Index itemID, STDstring& itemTypeName, STDstring& itemName)//, STDstring& itemInfo)
{
	Index itemIndex;
	ItemType itemType;
	Index mbsNumber;
	ItemID2IndexType(itemID, itemIndex, itemType, mbsNumber);
	if (mbsNumber >= basicVisualizationSystemContainer->NumberOFMainSystemsBacklink()) { return false; }
	MainSystem* mainSystem = basicVisualizationSystemContainer->GetMainSystemBacklink(mbsNumber);
	switch (itemType)
	{
	case ItemType::_None:
		return false; break;
	case ItemType::Node:
		if (itemIndex < mainSystem->mainSystemData.GetMainNodes().NumberOfItems()) {
			itemTypeName = STDstring("Node") + mainSystem->mainSystemData.GetMainNodes().GetItem(itemIndex)->GetTypeName();
			itemName = mainSystem->mainSystemData.GetMainNodes().GetItem(itemIndex)->GetName();
			return true;
		}
		break;
	case ItemType::Object:
		if (itemIndex < mainSystem->mainSystemData.GetMainObjects().NumberOfItems()) {
			itemTypeName = STDstring("Object") + mainSystem->mainSystemData.GetMainObjects().GetItem(itemIndex)->GetTypeName();
			itemName = mainSystem->mainSystemData.GetMainObjects().GetItem(itemIndex)->GetName();
			return true;
		}
		break;
	case ItemType::Marker:
		if (itemIndex < mainSystem->mainSystemData.GetMainMarkers().NumberOfItems()) {
			itemTypeName = STDstring("Marker") + mainSystem->mainSystemData.GetMainMarkers().GetItem(itemIndex)->GetTypeName();
			itemName = mainSystem->mainSystemData.GetMainMarkers().GetItem(itemIndex)->GetName();
			return true;
		}
		break;
	case ItemType::Load:
		if (itemIndex < mainSystem->mainSystemData.GetMainLoads().NumberOfItems()) {
			itemTypeName = STDstring("Load") + mainSystem->mainSystemData.GetMainLoads().GetItem(itemIndex)->GetTypeName();
			itemName = mainSystem->mainSystemData.GetMainLoads().GetItem(itemIndex)->GetName();
			return true;
		}
		break;
	case ItemType::Sensor:
		if (itemIndex < mainSystem->mainSystemData.GetMainSensors().NumberOfItems()) {
			itemTypeName = STDstring("Sensor") + mainSystem->mainSystemData.GetMainSensors().GetItem(itemIndex)->GetTypeName();
			itemName = mainSystem->mainSystemData.GetMainSensors().GetItem(itemIndex)->GetName();
			return true;
		}
		break;
	default:
		break;
	}
	return false; //nothing found, succuess=false
}

void GlfwRenderer::SetGLLights(Index viewID)
{
	if (visSettings->openGL.advanced.shadeModelSmooth) { glShadeModel(GL_SMOOTH); }
	else { glShadeModel(GL_FLAT); }

	//+++++++++++++++++++++++++++++++++++
	//light; see https://www.glprogramming.com/red/chapter05.html#name4

	//glDisable(GL_LIGHTING); //changed 2020-12-05
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_COLOR); //GL_ONE_MINUS_DST_COLOR, GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR, GL_SRC_ALPHA

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	for (Index lightID = 0; lightID < MAX_LIGHTS_GLFW; lightID++)
	{
		glLoadIdentity();
		//changed 2020-12-05; not needed any more, because SetGLLights moved to different place
		int GL_LIGHT_used = 0;
		switch (lightID)
		{
		case 0: GL_LIGHT_used = GL_LIGHT0; break;
		case 1: GL_LIGHT_used = GL_LIGHT1; break;
		case 2: GL_LIGHT_used = GL_LIGHT2; break;
		case 3: GL_LIGHT_used = GL_LIGHT3; break;
		default: CHECKandTHROWstring("GlfwRenderer::SetGLLights: illegal lightID");
		}
		const VSettingsLight& settingsLight = GetSettingsLight(lightID, *visSettings);

		//apply inverse model rotation to allow global lights
		if (!settingsLight.useCameraFrame) { SetModelRotationTranslation(viewID); } //needs to be applied to have light co-moving with modelview


		glEnable(GL_LIGHTING);

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//light sources:
		float ambientLightDeactivated[] = { 0.f, 0.f, 0.f, 1.0f }; //do not use 

		//float a0 = visSettings->openGL.light0ambient; //disabled; use global ambient light for all lights: openGL.lightModelAmbient
		float d0 = settingsLight.diffuse;
		float s0 = settingsLight.specular;
		//float ambientLight0[] = { a0, a0, a0, 1.0f };
		float diffuseLight0[] = { d0, d0, d0, 1.0f };
		float specularLight0[] = { s0, s0, s0, 1.0f };

		// Assign created components to GL_LIGHT_used
		//glLightfv(GL_LIGHT_used, GL_AMBIENT, ambientLight0);
		glLightfv(GL_LIGHT_used, GL_AMBIENT, ambientLightDeactivated);
		glLightfv(GL_LIGHT_used, GL_DIFFUSE, diffuseLight0);
		glLightfv(GL_LIGHT_used, GL_SPECULAR, specularLight0);
		//transform light to camera frame:

		Float4 lightPos4F = settingsLight.position; //in cameraFrame, just do nothing!

		glLightfv(GL_LIGHT_used, GL_POSITION, lightPos4F.GetDataPointer());
		//attenuation = 1/(kc + kl*d + kq*(d*d))
		glLightf(GL_LIGHT_used, GL_CONSTANT_ATTENUATION, settingsLight.constantAttenuation);
		glLightf(GL_LIGHT_used, GL_LINEAR_ATTENUATION, settingsLight.linearAttenuation);
		glLightf(GL_LIGHT_used, GL_QUADRATIC_ATTENUATION, settingsLight.quadraticAttenuation);

		if (settingsLight.enable) { glEnable(GL_LIGHT_used); }
		else { glDisable(GL_LIGHT_used); }

		////float a1 = visSettings->openGL.light1ambient; //disabled; use global ambient light: openGL.lightModelAmbient
		////float ambientLight1[] = { a1, a1, a1, 1.0f };
		//float d1 = visSettings->openGL.light1diffuse;
		//float s1 = visSettings->openGL.light1specular;
		//float diffuseLight1[] = { d1, d1, d1, 1.0f };
		//float specularLight1[] = { s1, s1, s1, 1.0f };

		//// Assign created components to GL_LIGHT1
		//glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLightDeactivated);
		//glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight1);
		//glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight1);
		//glLightfv(GL_LIGHT1, GL_POSITION, visSettings->openGL.light1position.GetDataPointer());

		//glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, visSettings->openGL.light1constantAttenuation);
		//glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, visSettings->openGL.light1linearAttenuation);
		//glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, visSettings->openGL.light1quadraticAttenuation);

		//if (visSettings->openGL.enableLight1) { glEnable(GL_LIGHT1); }
		//else { glDisable(GL_LIGHT1); }

	}
	glPopMatrix();

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//MATERIAL and Light model:
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, visSettings->openGL.materialShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, visSettings->openGL.materialSpecular.GetDataPointer());

	GLfloat ambient_diffuseMaterial[4] = { 0.6f, 0.6f, 0.6f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuseMaterial); //just for safety for cases without glColor
	//not needed: each glColor command overrides the ambientAndDiffuse value; glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, visSettings->openGL.materialAmbientAndDiffuse.GetDataPointer());

	glEnable(GL_COLOR_MATERIAL); //otherwise colors only use GL_AMBIENT_AND_DIFFUSE material (=grey)

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, visSettings->openGL.advanced.lightModelTwoSide);
	//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, true);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, visSettings->openGL.lightModelAmbient.GetDataPointer());
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, visSettings->openGL.advanced.lightModelLocalViewer);

	//obtain max. number of lights in your OpenGL version:
	//int v[10];
	//glGetIntegerv(GL_MAX_LIGHTS, v);
	//std::cout << "GL_MAX_LIGHTS=" << v[0] << "\n"; //==> gives 8 on OpenGL 1.1
	//+++++++++++++++++++++++++++++++++++
	//glPopMatrix();

	glDisable(GL_LIGHTING); //only enabled when drawing triangle faces

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! initialize some GLlists, e.g., for spheres

void GlfwRenderer::InitGLlists()
{
	spheresListBase = glGenLists(maxSpheresLists);
	float radius = 1.f;
	Index itemID = 0;
	GraphicsData graphicsData; //temporary
	for (guint loop = 0; loop < maxSpheresLists; loop++)
	{
		Index nTiles = Index(pow(2, loop));
		graphicsData.FlushData();
		EXUvis::DrawSphereTriangles(Vector3D(0.), radius, EXUvis::grey2, graphicsData, itemID, nTiles, true);

		glNewList(spheresListBase + loop, GL_COMPILE);

		for (const GLTriangle& trig : graphicsData.glTriangles)
		{ //draw faces
			glBegin(GL_TRIANGLES);
			for (Index i = 0; i < 3; i++)
			{
				glNormal3fv(trig.normals[i].GetDataPointer());
				glVertex3fv(trig.points[i].GetDataPointer());
			}
			glEnd();
		}

		glEndList();
	}

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++                         FONTS                              +++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! initialize bitmap for bitmap font (loaded from characterBitmap.h; switch between pure bitmap or textured based (faster!) fonts
void GlfwRenderer::InitFontBitmap(guint fontSize)
//, guint fontSizeSmall,	guint fontSizeLarge, guint fontSizeHuge)
{
	//create textures for fonts
	CreateFontTextures();

	CreateTexturedQuadsLists(bitmapFontListBase, &textureNumberRGBbitmap[0],
		bitmapFont.nCharacters, bitmapFont.characterByteWidth * 8,
		bitmapFont.characterWidth, bitmapFont.characterHeight);

}

//! draw a 0-terminated text string with fontSize, including monitor scaling factor; (for line-characters: size=1: height=1; width=0.5 for one character; distance = 0.25)
void GlfwRenderer::DrawString(Index viewID, const char* text, float fontSizeScaled, const Float3& p, const Float3& offset, 
	Float4 color, bool transparent)
{
	RenderState* state = renderViews.State(viewID);

	if (visSettings->general.useBitmapText)
	{
		float scale = 2.f*fontSizeScaled*state->zoom / ((float)state->currentWindowSize[1] * (float)bitmapFont.fontSize);
		//offset is given relative to character width, height and scene max size
		Float3 offsetScaled({ scale * bitmapFont.characterWidth * offset[0], 
			scale * bitmapFont.characterHeight * offset[1],
			state->maxSceneSize * offset[2] });

		if (!transparent)
        {
            float avColor = (3.f)*(color[0] + color[1] + color[2]);
            if (avColor < 0.5) //very dark colors are not visible
            {
                color[0] = EXUstd::Minimum(1.f, color[0] + 1.f - avColor);
                color[1] = EXUstd::Minimum(1.f, color[1] + 1.f - avColor);
                color[2] = EXUstd::Minimum(1.f, color[2] + 1.f - avColor);
            }
        }
		DrawStringWithTextures(text, scale, p + offsetScaled, color, bitmapFont,
			charBuffer, bitmapFontListBase, transparent);
	}
	else //old mode with lines used for text
	{
		float scale = 2.f*fontSizeScaled*state->zoom / ((float)state->currentWindowSize[1]);
		Float3 offsetScaled({ 0.7f * scale * offset[0], 1.4f * scale * offset[1], state->maxSceneSize * offset[2] });

		glLineWidth(visSettings->openGL.advanced.textLineWidth);
		if (visSettings->openGL.advanced.textLineSmooth) { glEnable(GL_LINE_SMOOTH); }
		OpenGLText::DrawString(text, scale, p + offsetScaled, color);
		if (visSettings->openGL.advanced.textLineSmooth) { glDisable(GL_LINE_SMOOTH); }
	}
}

//! now draw a string with internal (close to) unicode string, with length stringLen
void GlfwRenderer::DrawStringWithGLlistTextures(const Float3& p, float fontSizeScaled, GLuint listBase,
	GLubyte *string, guint stringLen, GLuint listOffset)
{
	glPushMatrix();
	glTranslatef(p[0], p[1], p[2]);
	glScalef(fontSizeScaled, fontSizeScaled, fontSizeScaled);

	glListBase(listBase - listOffset); //assign base of string list, 32 MUST be smallest value
	glCallLists(stringLen, GL_UNSIGNED_BYTE, string);
	glPopMatrix();
}


void GlfwRenderer::DrawStringWithTextures(const char* text, float fontSizeScaled, const Float3& p, const Float4& color,
	BitmapFont& font, ResizableArray<GLubyte>& charBuffer, GLuint listBase, bool transparent)
{
	//GLsizei w = font.characterWidth;
	GLsizei h = font.characterHeight;
	float vOff = 0; //offset for multiple lines

	//Draws the bitmap specified by bitmap, which is a pointer to the bitmap image.
	//The origin of the bitmap is placed at the current raster position.If the current raster position is invalid, nothing is drawn, and the raster position remains invalid.
	//The width and height arguments indicate the width and height, in pixels, of the bitmap.
	//The width need not be a multiple of 8, although the data is stored in unsigned characters of 8 bits each. 
	//(In the F example, it wouldn't matter if there were garbage bits in the data beyond the tenth bit; since glBitmap() was called with a width of 10, only 10 bits of the row are rendered.) 
	//Use xorig and yorig to define the origin of the bitmap (positive values move the origin up and to the right of the raster position; 
	//negative values move it down and to the left); 

	//guint i = 0;
	//guint lineNumber = 0; //for several lines of text
	guint columnNumber = 0;

	guint updatedIndex = 0;
	gchar cUnicode = font.GetUnicodeCharacterFromUTF8(text, updatedIndex);
	charBuffer.SetNumberOfItems(0);

	//SetglDepthMask(GL_FALSE); //done outside, can distinguish between always on top text (status information) and node numbers, etc.
	//glDisable(GL_LIGHTING);
	//glColor4f(0.f, 0.f, 0.f, 1.f); //on texture, this color will influence the appearance (similar to lighting)
	glEnable(MY_GL_TEXTURE_2D); //must be disabled if no textures drawn!
	glColor4f(color[0], color[1], color[2], color[3]); //on texture, this color will influence the appearance (similar to lighting)
	
    if (!transparent && NUMBER_OF_TEXTUREFONT_LISTS == 2)
    {
        listBase += font.nCharacters; //switch to second list
    }

	while (cUnicode != 0)
	{
		//use UTF-8 encoding: https://de.wikipedia.org/wiki/UTF-8
		//accept unicode characters: https://www.utf8-chartable.de/unicode-utf8-table.pl?unicodeinhtml=dec&htmlent=1
		if (cUnicode != (guint)'\n')
		{
			//std::cout << c << ":" << ci << "\n";
			if (cUnicode >= font.characterOffset && cUnicode < font.nCharacters + font.characterOffset) //do not print control characters ...
			{
				charBuffer[columnNumber] = cUnicode;
			}
			else
			{
                charBuffer[columnNumber] = font.characterOffset; // 32; //should not happen!
			}
			columnNumber++;
		}
		else
		{
			DrawStringWithGLlistTextures(p + Float3({0,vOff,0}), fontSizeScaled, listBase, charBuffer.GetDataPointer(), 
                charBuffer.NumberOfItems(), font.characterOffset);
			charBuffer.SetNumberOfItems(0);
			vOff -= (float)h*fontSizeScaled;
			//lineNumber++; 
			columnNumber = 0;
		}
		cUnicode = font.GetUnicodeCharacterFromUTF8(text, updatedIndex);
	}
	if (charBuffer.NumberOfItems())
	{
		DrawStringWithGLlistTextures(p + Float3({ 0,vOff,0 }), fontSizeScaled, listBase, charBuffer.GetDataPointer(), 
            charBuffer.NumberOfItems(), font.characterOffset);
	}
	glDisable(MY_GL_TEXTURE_2D); //must be disabled if no textures drawn!

}

//! create glLists for texture with textureNumber
void GlfwRenderer::CreateTexturedQuadsLists(GLuint& listBase, GLuint* textureNumber,
	guint nCharacters, guint wCharacter8, guint wCharacter, guint hCharacter, bool itemTags)
{
	//GLfloat cx, cy;         /* the character coordinates in our texture */
	listBase = glGenLists(nCharacters*NUMBER_OF_TEXTUREFONT_LISTS);

	GLfloat wFact = (float)wCharacter / (float)wCharacter8 - 0.001f; //0.001 needed to remove artifacts (interpolation?)

	for (guint loop = 0; loop < nCharacters*NUMBER_OF_TEXTUREFONT_LISTS; loop++)
	{
		glNewList(listBase + loop, GL_COMPILE);
		glBindTexture(MY_GL_TEXTURE_2D, textureNumber[loop]);
        //glColor4f(0.f, 0.f, 0.f, 0.f); //do not add color here; color is added to drawing of texts, allowing for colored texts!
        glBegin(GL_QUADS);
        glTexCoord2f(0.001f, 0); //0.001 needed to remove artifacts (interpolation?)
		glVertex2i(0, 0);
		glTexCoord2f(wFact, 0);
		glVertex2i(wCharacter, 0);
		glTexCoord2f(wFact, 1);
		glVertex2i(wCharacter, hCharacter);
		glTexCoord2f(0.001f, 1); //0.001 needed to remove artifacts (interpolation?)
		glVertex2i(0, hCharacter);
		glEnd();
		glTranslated(wCharacter, 0, 0); //this causes that every additional character is drawn to the right of the last when drawing a list of characters one!
		glBindTexture(MY_GL_TEXTURE_2D, 0);
		glEndList();
	}
}

//! create glTexImage2D objects for font characters, stored in textureNumberRGBbitmap
void GlfwRenderer::CreateFontTextures()
{
	bitmapFont = BitmapFont(charBitmap64::fontSize, charBitmap64::nCharacters, charBitmap64::characterOffset,
		charBitmap64::characterWidth, charBitmap64::characterHeight,
		charBitmap64::characterByteWidth, charBitmap64::characterBytes,
		charBitmap64::OpenGLtextBitmap);

	IconBar::AddIconTextures(); //the icons of the icon bar, uploaded with the other textures (#3026)

	//create texture bitmapFont (could also be loaded from file)
	glGenTextures(bitmapFont.nCharacters*NUMBER_OF_TEXTUREFONT_LISTS, &textureNumberRGBbitmap[0]);   //create one texture

    for (Index j = 0; j < NUMBER_OF_TEXTUREFONT_LISTS; j++)
    {
        for (GLuint iChar = 0; iChar < bitmapFont.nCharacters; iChar++)
        {
            GLubyte* textureRGB = bitmapFont.GetRGBAFontCharacter(iChar, (bool)(1-j));

            glBindTexture(MY_GL_TEXTURE_2D, textureNumberRGBbitmap[iChar+ bitmapFont.nCharacters*j]);
            /* actually generate the texture */
            glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //linear filter give nicer results than GL_NEAREST
            glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(MY_GL_TEXTURE_2D, 0, 4, bitmapFont.characterByteWidth * 8, //bitmapFont.characterWidth,
                bitmapFont.characterHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, textureRGB);
            delete[] textureRGB; //not needed lateron
        }
    }
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++                        ICON BAR                            +++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//the icon bar (#3026): folds in from the right border when the mouse comes near it without a button pressed; each icon
//does what a key does - the key is sent to key_callback, so that a key and its icon do the same - or switches a setting
//of the scene; drawn after the frame is saved, so not part of saved images

GlfwRenderer::IconBarState GlfwRenderer::iconBarStates[MAX_VIEWS_GLFW];
float GlfwRenderer::iconBarStandardUpdateInterval = -1.f;

//! icon size, gap and the width of the bar, in pixels of the frame buffer
void GlfwRenderer::IconBarGeometry(Index viewID, int height, float& iconSize, float& gap, float& barWidth)
{
	RenderState* state = renderViews.State(viewID);
	float scaling = EXUstd::Maximum(0.1f, visSettings->general.iconBarScaling)*(state->displayScaling > 0.f ? state->displayScaling : 1.f);
	Index n = IconBar::NumberOfIcons();
	gap = 4.f*scaling;
	iconSize = 54.f*scaling;
	if (n*(iconSize + gap) + gap > (float)height) //shrunk to the height of the window
	{
		iconSize = EXUstd::Maximum(8.f, ((float)height - gap*(n + 1))/(float)n);
	}
	barWidth = iconSize + 2.f*gap;
}

//! the icon at a mouse position in pixels of the frame buffer from the top left, or -1
Index GlfwRenderer::IconBarIconAt(Index viewID, int width, int height, double x, double y)
{
	const IconBarState& bar = iconBarStates[viewID];
	if (bar.visible <= 0.f) { return -1; }
	float iconSize, gap, barWidth;
	IconBarGeometry(viewID, height, iconSize, gap, barWidth);
	float x0 = (float)width - barWidth*bar.visible;
	if (x < x0 + gap || x > x0 + gap + iconSize) { return -1; }
	Index icon = (Index)std::floor((y - gap)/(iconSize + gap));
	if (icon < 0 || icon >= IconBar::NumberOfIcons() || y - gap - icon*(iconSize + gap) > iconSize) { return -1; }
	return icon;
}

//! the mouse position in pixels of the frame buffer, which is larger than the window on a retina display
void GlfwRenderer::IconBarMouse(GLFWwindow* window, int width, int height, double& x, double& y)
{
	int windowWidth = 1, windowHeight = 1;
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	glfwGetCursorPos(window, &x, &y);
	x *= (double)width/(double)EXUstd::Maximum(1, windowWidth);
	y *= (double)height/(double)EXUstd::Maximum(1, windowHeight);
}

//! true, if an icon is pressed (shown as pressed in the bar)
bool GlfwRenderer::IconBarIconIsOn(Index viewID, Index icon)
{
	const VSettingsView& view = GetSettingsView(viewID, *visSettings);
	switch ((IconBar::Icon)icon)
	{
	case IconBar::Icon::Ruler: return view.window.showRuler;
	case IconBar::Icon::MouseCoordinates: return view.window.showMouseCoordinates;
	case IconBar::Icon::Lines: return view.scene.showLines;
	case IconBar::Icon::Edges: return view.scene.showFaceEdges;
	case IconBar::Icon::Faces: return view.scene.showFaces;
	case IconBar::Icon::Transparency: return view.scene.facesTransparent;
	case IconBar::Icon::Shadow: return IconBarShadowIsOn();
	case IconBar::Icon::Nodes: return visSettings->nodes.show;
	case IconBar::Icon::Bodies: return visSettings->bodies.show;
	case IconBar::Icon::Connectors: return visSettings->connectors.show;
	case IconBar::Icon::Markers: return visSettings->markers.show;
	case IconBar::Icon::Loads: return visSettings->loads.show;
	case IconBar::Icon::Sensors: return visSettings->sensors.show;
	default: return false;
	}
}

//! true if a light that is on casts a shadow
bool GlfwRenderer::IconBarShadowIsOn()
{
	const VSettingsOpenGL& gl = visSettings->openGL;
	return (gl.light0.enable && gl.light0.shadow > 0.f) || (gl.light1.enable && gl.light1.shadow > 0.f) ||
		(gl.light2.enable && gl.light2.shadow > 0.f) || (gl.light3.enable && gl.light3.shadow > 0.f);
}

//! the graphics update intervals the icon cycles through: those of the keys 1 to 5 and, sorted in, the interval the
//! model set when the bar was first drawn
std::vector<float> GlfwRenderer::IconBarUpdateIntervals()
{
	std::vector<float> intervals = { 0.02f, 0.1f, 0.5f, 2.f, 100.f };
	float standard = iconBarStandardUpdateInterval;
	if (standard >= 0.f && std::find_if(intervals.begin(), intervals.end(),
		[standard](float value) { return std::fabs(standard - value) <= 1e-6f*value; }) == intervals.end())
	{
		intervals.insert(std::upper_bound(intervals.begin(), intervals.end(), standard), standard);
	}
	return intervals;
}

//! the index in IconBarUpdateIntervals() of the graphics update interval that is set; -1 for another value
Index GlfwRenderer::IconBarUpdateInterval()
{
	std::vector<float> intervals = IconBarUpdateIntervals();
	for (Index k = 0; k < (Index)intervals.size(); k++)
	{
		if (std::fabs(visSettings->general.graphicsUpdateInterval - intervals[k]) <= 1e-6f*intervals[k]) { return k; }
	}
	return -1;
}

//! the tooltip of an icon; the one of the update interval lists the intervals
STDstring GlfwRenderer::IconBarTooltip(Index icon)
{
	STDstring text = IconBar::Tooltip(icon);
	if ((IconBar::Icon)icon == IconBar::Icon::UpdateSpeed)
	{
		STDstring values;
		for (float value : IconBarUpdateIntervals()) { values += (values.size() ? ", " : "") + EXUstd::Num2String(value, 4); }
		text = "graphics update interval: " + values + " s (keys 1 to 5)";
	}
	return text;
}

//! the text written over an icon: the axes of a view, the number of the update interval, # for shown item numbers
STDstring GlfwRenderer::IconBarIconText(Index viewID, Index icon, bool control)
{
	switch ((IconBar::Icon)icon)
	{
	case IconBar::Icon::ViewXY: return control ? "YX" : "XY";
	case IconBar::Icon::ViewYZ: return control ? "ZY" : "YZ";
	case IconBar::Icon::ViewXZ: return control ? "ZX" : "XZ";
	case IconBar::Icon::UpdateSpeed: return IconBarUpdateInterval() >= 0 ? EXUstd::ToString(IconBarUpdateInterval() + 1) : "";
	case IconBar::Icon::Nodes: return visSettings->nodes.showNumbers ? "#" : "";
	case IconBar::Icon::Bodies: return visSettings->bodies.showNumbers ? "#" : "";
	case IconBar::Icon::Connectors: return visSettings->connectors.showNumbers ? "#" : "";
	case IconBar::Icon::Markers: return visSettings->markers.showNumbers ? "#" : "";
	case IconBar::Icon::Loads: return visSettings->loads.showNumbers ? "#" : "";
	case IconBar::Icon::Sensors: return visSettings->sensors.showNumbers ? "#" : "";
	case IconBar::Icon::Config0: case IconBar::Icon::Config1: case IconBar::Icon::Config2: case IconBar::Icon::Config3:
		return "CFG" + EXUstd::ToString(icon - (Index)IconBar::Icon::Config0);
	default: return "";
	}
}

//! the action of an icon; SHIFT and CTRL as described in its tooltip
void GlfwRenderer::IconBarExecute(GLFWwindow* window, Index viewID, Index icon, int mods)
{
	VSettingsView& view = GetSettingsView(viewID, *visSettings);
	bool shift = (mods & GLFW_MOD_SHIFT) != 0;
	bool control = (mods & GLFW_MOD_CONTROL) != 0;
	auto Key = [&](int key, int keyMods) { key_callback(window, key, 0, GLFW_PRESS, keyMods); };
	int viewMods = GLFW_MOD_CONTROL + (shift ? GLFW_MOD_SHIFT : 0);
	switch ((IconBar::Icon)icon)
	{
	case IconBar::Icon::Quit: Key(GLFW_KEY_ESCAPE, 0); break;
	case IconBar::Icon::Settings: Key(GLFW_KEY_V, 0); break;
	case IconBar::Icon::ZoomAll: Key(GLFW_KEY_A, 0); break;
	case IconBar::Icon::ZoomRegion: //the next two clicks are the corners of the region
		iconBarStates[viewID].regionStep = (iconBarStates[viewID].regionStep == 0) ? 1 : 0;
		ShowMessage(iconBarStates[viewID].regionStep ? "zoom to a region: click two of its corners (right mouse button: cancel; or SHIFT+drag)" : "", 5);
		break;
	case IconBar::Icon::ViewXY: Key(control ? GLFW_KEY_4 : GLFW_KEY_1, viewMods); break;
	case IconBar::Icon::ViewYZ: Key(control ? GLFW_KEY_6 : GLFW_KEY_3, viewMods); break;
	case IconBar::Icon::ViewXZ: Key(control ? GLFW_KEY_5 : GLFW_KEY_2, viewMods); break;
	case IconBar::Icon::ViewIsometric: Key(GLFW_KEY_7, GLFW_MOD_CONTROL); break;
	case IconBar::Icon::Ruler: Key(GLFW_KEY_F5, 0); break;
	case IconBar::Icon::MouseCoordinates: Key(GLFW_KEY_F3, 0); break;
	case IconBar::Icon::Lines: view.scene.showLines = !view.scene.showLines; UpdateGraphicsDataNow(); break;
	case IconBar::Icon::Edges:
		view.scene.showFaceEdges = !view.scene.showFaceEdges;
		view.scene.showMeshEdges = view.scene.showFaceEdges;
		UpdateGraphicsDataNow();
		break;
	case IconBar::Icon::Faces:
		view.scene.showFaces = !view.scene.showFaces;
		view.scene.showMeshFaces = view.scene.showFaces;
		UpdateGraphicsDataNow();
		break;
	case IconBar::Icon::Transparency: view.scene.facesTransparent = !view.scene.facesTransparent; UpdateGraphicsDataNow(); break;
	case IconBar::Icon::Shadow: //light0 casts 0.25, or no light casts one (#3044)
	{
		bool on = !IconBarShadowIsOn();
		visSettings->openGL.light0.shadow = on ? 0.25f : 0.f;
		if (!on) { visSettings->openGL.light1.shadow = visSettings->openGL.light2.shadow = visSettings->openGL.light3.shadow = 0.f; }
		if (on) { visSettings->openGL.light0.enable = true; }
		UpdateGraphicsDataNow();
		break;
	}
	case IconBar::Icon::UpdateSpeed: { //the next interval; the keys 1 to 5 do not reach the interval of the model
		std::vector<float> intervals = IconBarUpdateIntervals();
		float interval = intervals[(size_t)((IconBarUpdateInterval() + 1) % (Index)intervals.size())];
		visSettings->general.graphicsUpdateInterval = interval;
		ShowMessage("Visualization update: " + EXUstd::Num2String(interval, 4) + "s", 2);
		break;
	}
	case IconBar::Icon::Nodes: Key(GLFW_KEY_N, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Bodies: Key(GLFW_KEY_B, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Connectors: Key(GLFW_KEY_C, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Markers: Key(GLFW_KEY_M, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Loads: Key(GLFW_KEY_L, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Sensors: Key(GLFW_KEY_S, shift ? GLFW_MOD_CONTROL : 0); break;
	case IconBar::Icon::Config0: case IconBar::Icon::Config1: case IconBar::Icon::Config2: case IconBar::Icon::Config3:
	{
		//in the Python thread, as the dialogs: the configurations are in config.json, which Python reads and writes (#3043)
		STDstring number = EXUstd::ToString(icon - (Index)IconBar::Icon::Config0);
		STDstring call = control ? "StoreVisualizationSettings(CFG=" + number + ", storeSettings=True, storeModelView=True)" :
			"ApplyStoredVisualizationSettings(CFG=" + number + ")";
		PyQueueExecutableString("__import__('exudyn.misc.GUI', fromlist=['GUI'])." + call + "\n");
		ShowMessage(control ? "store the visualization settings and the model view as configuration CFG" + number :
			"switch to configuration CFG" + number, 3);
		break;
	}
	default: break;
	}
	SetCallBackRefreshSignal();
}

//! a click on the bar: the action of the icon on press; the press and its release do not reach the selection and the
//! rotation of the model; the corners of the region of the icon zoomRegion, or SHIFT+drag with the left button for
//! the same zoom, also with the bar off; true if the bar took the event
bool GlfwRenderer::IconBarMouseButton(GLFWwindow* window, Index viewID, int button, int action, int mods)
{
	IconBarState& bar = iconBarStates[viewID];
	int width, height;
	GetWindowSize(window, width, height);
	double x, y;
	IconBarMouse(window, width, height, x, y);
	if (action == GLFW_RELEASE && bar.pressed)
	{
		bar.pressed = false;
		if (bar.regionStep == 3) //the end of SHIFT+drag; a click without drag does nothing
		{
			bar.regionStep = 0;
			if (std::fabs(x - bar.regionX) >= 3. || std::fabs(y - bar.regionY) >= 3.) { IconBarZoomToRegion(viewID, bar.regionX, bar.regionY, x, y); }
		}
		return true;
	}
	if (action != GLFW_PRESS) { return false; }
	float iconSize, gap, barWidth;
	IconBarGeometry(viewID, height, iconSize, gap, barWidth);
	bool showBar = GetSettingsView(viewID, *visSettings).window.showIconBar;
	bool onBar = showBar && bar.visible >= 0.5f && x >= (double)width - barWidth*bar.visible;
	if (bar.regionStep == 0 && !onBar && button == GLFW_MOUSE_BUTTON_LEFT && mods == GLFW_MOD_SHIFT) //SHIFT+drag
	{
		bar.pressed = true;
		bar.regionX = x;
		bar.regionY = y;
		bar.regionStep = 3;
		return true;
	}
	if (bar.regionStep != 0 && !onBar) //the corners of the region to zoom to
	{
		bar.pressed = true;
		if (button != GLFW_MOUSE_BUTTON_LEFT) { bar.regionStep = 0; ShowMessage("", 0); return true; } //cancelled
		if (bar.regionStep == 1)
		{
			bar.regionX = x;
			bar.regionY = y;
			bar.regionStep = 2;
		}
		else
		{
			bar.regionStep = 0;
			ShowMessage("", 0);
			IconBarZoomToRegion(viewID, bar.regionX, bar.regionY, x, y);
		}
		return true;
	}
	if (!onBar) { return false; }
	bar.pressed = true;
	Index icon = IconBarIconAt(viewID, width, height, x, y);
	if (icon >= 0 && button == GLFW_MOUSE_BUTTON_LEFT) { IconBarExecute(window, viewID, icon, mods); }
	return true;
}

//! the region between two corners, in pixels of the frame buffer, fills the window: its center becomes the center of
//! the view, the zoom shrinks by the larger of its relative width and height
void GlfwRenderer::IconBarZoomToRegion(Index viewID, double x0, double y0, double x1, double y1)
{
	RenderState* state = renderViews.State(viewID);
	double width = (double)EXUstd::Maximum((Index)1, state->currentWindowSize[0]);
	double height = (double)EXUstd::Maximum((Index)1, state->currentWindowSize[1]);
	double factor = EXUstd::Maximum(std::fabs(x1 - x0)/width, std::fabs(y1 - y0)/height);
	if (factor*height < 3.) { ShowMessage("zoom to a region: the region is too small", 3); return; }
	Vector2D center = basicVisualizationSystemContainer->PixelsToOpenGLCoordinates2D(0.5*(x0 + x1), 0.5*(y0 + y1), *state);
	state->centerPoint[0] = (float)center[0];
	state->centerPoint[1] = (float)center[1];
	state->zoom *= (float)factor;
	stateMachine.hasLastMousePressed = 0;
	SetCallBackRefreshSignal();
}

//! draw the bar in pixels of the frame buffer; it slides in and out over iconBarFoldTime seconds
void GlfwRenderer::DrawIconBar(GLFWwindow* window, Index viewID, int width, int height)
{
	IconBarState& bar = iconBarStates[viewID];
	if (iconBarStandardUpdateInterval < 0.f) { iconBarStandardUpdateInterval = visSettings->general.graphicsUpdateInterval; }
	bool showBar = GetSettingsView(viewID, *visSettings).window.showIconBar;
	if (!showBar) { bar.visible = 0.f; }
	if ((!showBar && bar.regionStep < 2) || height <= 0) { return; }
	if (bar.textureIDs.size() == 0) //the icons are in the TextureStore (CreateFontTextures)
	{
		for (Index icon = 0; icon < IconBar::NumberOfIcons(); icon++) { bar.textureIDs.push_back(textureStore.GetID(STDstring("exudyn.icon.") + IconBar::Name(icon))); }
	}

	float iconSize, gap, barWidth;
	IconBarGeometry(viewID, height, iconSize, gap, barWidth);
	double x, y;
	IconBarMouse(window, width, height, x, y);
	bool hovered = glfwGetWindowAttrib(window, GLFW_HOVERED) != 0;
	bool buttonPressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
	float foldDistance = visSettings->general.iconBarFoldDistance*(renderViews.State(viewID)->displayScaling > 0.f ? renderViews.State(viewID)->displayScaling : 1.f);
	bool show = showBar && hovered && bar.regionStep != 3 && ((!buttonPressed && x >= (double)width - foldDistance) ||
		(bar.visible > 0.f && x >= (double)width - barWidth - 2.f*gap));

	//slide in or out; after the mouse left, the bar stays general.iconBarTimeout
	double now = EXUstd::GetTimeInSeconds();
	if (show) { bar.lastShownTime = now; }
	else if (showBar && bar.visible > 0.f && now - bar.lastShownTime < 0.001*(double)visSettings->general.iconBarTimeout)
	{
		show = true;
		SetCallBackRefreshSignal(); //the next frame checks again
	}
	float step = (float)((now - bar.lastTime)/iconBarFoldTime);
	bar.lastTime = now;
	float target = show ? 1.f : 0.f;
	if (GetSettingsView(viewID, *visSettings).camera.useRaytracer) { bar.visible = target; } //a raytraced frame is slow: no animation (#3050)
	if (bar.visible != target)
	{
		step = EXUstd::Minimum(EXUstd::Maximum(step, 0.f), 0.25f);
		bar.visible = show ? EXUstd::Minimum(1.f, bar.visible + step) : EXUstd::Maximum(0.f, bar.visible - step);
		SetCallBackRefreshSignal(); //the next frame continues the animation
	}
	bool drawRegion = (bar.regionStep >= 2);
	if (bar.visible <= 0.f && !drawRegion) { return; }

	glPushAttrib(GL_ENABLE_BIT | GL_TEXTURE_BIT | GL_CURRENT_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_LINE_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, width, height, 0.0, -1.0, 1.0); //y down, as the mouse
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	auto Quad = [](float x0, float y0, float x1, float y1)
	{
		glBegin(GL_QUADS);
		glVertex2f(x0, y0); glVertex2f(x1, y0); glVertex2f(x1, y1); glVertex2f(x0, y1);
		glEnd();
	};
	if (drawRegion) //the region to zoom to, from its first corner to the mouse (#3026)
	{
		float rx0 = (float)bar.regionX, ry0 = (float)bar.regionY, rx1 = (float)x, ry1 = (float)y;
		glColor4f(0.15f, 0.42f, 0.8f, 0.15f);
		Quad(rx0, ry0, rx1, ry1);
		glColor4f(0.15f, 0.42f, 0.8f, 1.f);
		glLineWidth(1.5f);
		glBegin(GL_LINE_LOOP); glVertex2f(rx0, ry0); glVertex2f(rx1, ry0); glVertex2f(rx1, ry1); glVertex2f(rx0, ry1); glEnd();
	}
	if (bar.visible > 0.f)
	{
		float x0 = (float)width - barWidth*bar.visible;
		glColor4f(0.93f, 0.94f, 0.96f, 0.92f);
		Quad(x0, 0.f, x0 + barWidth, (float)height);
		glColor4f(0.6f, 0.62f, 0.66f, 1.f);
		glLineWidth(1.f);
		glBegin(GL_LINES); glVertex2f(x0 + 0.5f, 0.f); glVertex2f(x0 + 0.5f, (float)height); glEnd();

		Index hover = (bar.visible >= 1.f && hovered) ? IconBarIconAt(viewID, width, height, x, y) : -1;
		bool control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
		float fontPixels = 0.3f*iconSize;
		float fontScale = fontPixels/(float)bitmapFont.fontSize;
		for (Index icon = 0; icon < IconBar::NumberOfIcons(); icon++)
		{
			float ix = x0 + gap;
			float iy = gap + icon*(iconSize + gap);
			if (IconBarIconIsOn(viewID, icon) || icon == hover)
			{
				if (IconBarIconIsOn(viewID, icon)) { glColor4f(0.68f, 0.8f, 0.97f, 1.f); }
				else { glColor4f(0.84f, 0.86f, 0.9f, 1.f); }
				Quad(ix - 0.5f*gap, iy - 0.5f*gap, ix + iconSize + 0.5f*gap, iy + iconSize + 0.5f*gap);
			}
			Index id = bar.textureIDs[icon];
			if (id >= 0 && id < (Index)sceneTextures.size() && sceneTextures[id].name != 0)
			{
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, sceneTextures[id].name);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				glColor4f(1.f, 1.f, 1.f, 1.f);
				glBegin(GL_QUADS); //the image has its bottom row first: v=0 at the bottom of the icon, y grows down
				glTexCoord2f(0.f, 1.f); glVertex2f(ix, iy);
				glTexCoord2f(1.f, 1.f); glVertex2f(ix + iconSize, iy);
				glTexCoord2f(1.f, 0.f); glVertex2f(ix + iconSize, iy + iconSize);
				glTexCoord2f(0.f, 0.f); glVertex2f(ix, iy + iconSize);
				glEnd();
				glBindTexture(GL_TEXTURE_2D, 0);
				glDisable(GL_TEXTURE_2D);
			}
			STDstring text = IconBarIconText(viewID, icon, control);
			if (text.size())
			{
				//the text at the top right of the icon; the font draws upward from its point, so the y axis is turned for it
				glPushMatrix();
				float textWidth = fontScale*(float)bitmapFont.characterWidth*(float)text.size();
				float textX = (icon >= (Index)IconBar::Icon::Config0) ? ix + 0.5f*(iconSize - textWidth) : ix + iconSize - textWidth - 0.5f*gap; //CFG0 ... centered on the button
				float textY = (icon >= (Index)IconBar::Icon::Config0) ? iy + 0.42f*iconSize : iy + 1.1f*fontPixels; //CFG0 ... inside the button
				glTranslatef(textX, textY, 0.f);
				glScalef(1.f, -1.f, 1.f);
				DrawStringWithTextures(text.c_str(), fontScale, Float3({ 0.f, 0.f, 0.f }), (icon >= (Index)IconBar::Icon::Config0 ? Float4({ 0.1f, 0.15f, 0.3f, 1.f }) : Float4({ 0.7f, 0.1f, 0.1f, 1.f })), bitmapFont,
					charBuffer, bitmapFontListBase, true);
				glPopMatrix();
			}
		}

		if (hover >= 0) //the tooltip left of the bar, in the font size of the version string, at the display scaling
		{
			STDstring text = IconBarTooltip(hover);
			float tipFont = GetSettingsView(viewID, *visSettings).window.globalFontSize*fontSmallFactor*GetFontScaling(viewID);
			float tipScale = tipFont/(float)bitmapFont.fontSize;
			float textWidth = tipScale*(float)bitmapFont.characterWidth*(float)text.size();
			float margin = 0.4f*tipFont;
			float ty = gap + hover*(iconSize + gap) + 0.5f*iconSize;
			float tx1 = x0 - gap;
			float tx0 = EXUstd::Maximum(0.f, tx1 - textWidth - 2.f*margin);
			glColor4f(1.f, 1.f, 0.88f, 0.95f);
			Quad(tx0, ty - 1.12f*tipFont, tx1, ty + 0.64f*tipFont);
			glPushMatrix();
			glTranslatef(tx0 + margin, ty + 0.45f*tipFont, 0.f);
			glScalef(1.f, -1.f, 1.f);
			DrawStringWithTextures(text.c_str(), tipScale, Float3({ 0.f, 0.f, 0.f }), Float4({ 0.1f, 0.1f, 0.1f, 1.f }),
				bitmapFont, charBuffer, bitmapFontListBase, true);
			glPopMatrix();
		}
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}


void GlfwRenderer::DeleteFonts()
{
	glDeleteTextures(bitmapFont.nCharacters*NUMBER_OF_TEXTUREFONT_LISTS, &textureNumberRGBbitmap[0]);
	glDeleteLists(bitmapFontListBase, bitmapFont.nCharacters*NUMBER_OF_TEXTUREFONT_LISTS);
}








#endif //USE_GLFW_GRAPHICS
