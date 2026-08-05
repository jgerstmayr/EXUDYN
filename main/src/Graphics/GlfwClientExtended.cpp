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
		EXUvis::DrawSphere(Vector3D(0.), radius, EXUvis::grey2, graphicsData, itemID, nTiles, true);

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




void GlfwRenderer::DeleteFonts()
{
	glDeleteTextures(bitmapFont.nCharacters*NUMBER_OF_TEXTUREFONT_LISTS, &textureNumberRGBbitmap[0]);
	glDeleteLists(bitmapFontListBase, bitmapFont.nCharacters*NUMBER_OF_TEXTUREFONT_LISTS);
}








#endif //USE_GLFW_GRAPHICS
