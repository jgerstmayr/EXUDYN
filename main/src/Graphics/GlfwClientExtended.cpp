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
#include "Utilities/SlimArray.h"
//#include <string>

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
			itemDict = mainSystem->mainSystemData.GetMainObjects().GetItem(itemIndex)->GetDictionary();
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

void GlfwRenderer::SetGLLights()
{
	if (visSettings->openGL.shadeModelSmooth) { glShadeModel(GL_SMOOTH); }
	else { glShadeModel(GL_FLAT); }

	//glDisable(GL_LIGHTING); //changed 2020-12-05
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//changed 2020-12-05; not needed any more, because SetGLLights moved to different place
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();


	//+++++++++++++++++++++++++++++++++++
	//light; see https://www.glprogramming.com/red/chapter05.html#name4

	glEnable(GL_LIGHTING);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//light sources:
	float a0 = visSettings->openGL.light0ambient;
	float d0 = visSettings->openGL.light0diffuse;
	float s0 = visSettings->openGL.light0specular;
	float ambientLight0[] = { a0, a0, a0, 1.0f };
	float diffuseLight0[] = { d0, d0, d0, 1.0f };
	float specularLight0[] = { s0, s0, s0, 1.0f };

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight0);
	glLightfv(GL_LIGHT0, GL_POSITION, visSettings->openGL.light0position.GetDataPointer());
	//attenuation = 1/(kc + kl*d + kq*(d*d))
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, visSettings->openGL.light0constantAttenuation);
	glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, visSettings->openGL.light0linearAttenuation);
	glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, visSettings->openGL.light0quadraticAttenuation);

	float a1 = visSettings->openGL.light1ambient;
	float d1 = visSettings->openGL.light1diffuse;
	float s1 = visSettings->openGL.light1specular;
	float ambientLight1[] = { a1, a1, a1, 1.0f };
	float diffuseLight1[] = { d1, d1, d1, 1.0f };
	float specularLight1[] = { s1, s1, s1, 1.0f };

	// Assign created components to GL_LIGHT0
	//only glLightfv works properly, not glLightf ...:
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight1);
	glLightfv(GL_LIGHT1, GL_POSITION, visSettings->openGL.light1position.GetDataPointer());

	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, visSettings->openGL.light1constantAttenuation);
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, visSettings->openGL.light1linearAttenuation);
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, visSettings->openGL.light1quadraticAttenuation);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//MATERIAL and Light model:
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, visSettings->openGL.materialShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, visSettings->openGL.materialSpecular.GetDataPointer());

	//GLfloat diffuseMaterial[4] = { 0.5, 0.5, 0.5, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, visSettings->openGL.materialAmbientAndDiffuse.GetDataPointer());
	glEnable(GL_COLOR_MATERIAL); //otherwise colors are grey
	//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); //do not use this!

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, visSettings->openGL.lightModelTwoSide);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, visSettings->openGL.lightModelAmbient.GetDataPointer());
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, visSettings->openGL.lightModelLocalViewer);

	if (visSettings->openGL.enableLight0) { glEnable(GL_LIGHT0); }
	else { glDisable(GL_LIGHT0); }

	if (visSettings->openGL.enableLight1) { glEnable(GL_LIGHT1); }
	else { glDisable(GL_LIGHT1); }

	//obtain max. number of lights in your OpenGL version:
	//int v[10];
	//glGetIntegerv(GL_MAX_LIGHTS, v);
	//std::cout << "GL_MAX_LIGHTS=" << v[0] << "\n"; //==> gives 8 on OpenGL 1.1
	//+++++++++++++++++++++++++++++++++++
	glPopMatrix();

	glDisable(GL_LIGHTING); //only enabled when drawing triangle faces

}

//!transform pixel coordinates (from bottom/left, [0..width-1, 0..height-1]) into vertex coordinates
//!works if model view is initialized with 	glMatrixMode(GL_MODELVIEW);glLoadIdentity();
Float2 GlfwRenderer::PixelToVertexCoordinates(float x, float y)
{
	float width = (float)state->currentWindowSize[0];
	float height = (float)state->currentWindowSize[1];

	if (height == 0) { height = 1; }
	if (width == 0) { width = 1; }
	float ratio = width / (float)height;

	//x,y in pixels; origin is top/left
	float vx = state->zoom*ratio*(-1.f + 2.f * (x + 1e-2f) / width); //add 1e-2 because of round off errors (ratio)
	float vy = state->zoom*(-1.f + 2.f * y / height);
	return Float2({ vx, vy });
}





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++                         FONTS                              +++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! initialize bitmap for bitmap font (loaded from characterBitmap.h; switch between pure bitmap or textured based (faster!) fonts
void GlfwRenderer::InitFontBitmap(guint fontSize)
//, guint fontSizeSmall,	guint fontSizeLarge, guint fontSizeHuge)
{
#ifndef USE_TEXTURED_BITMAP_FONTS
	BitmapFont bitmap(charBitmap64::fontSize, charBitmap64::nCharacters, charBitmap64::characterOffset,
		charBitmap64::characterWidth, charBitmap64::characterHeight,
		charBitmap64::characterByteWidth, charBitmap64::characterBytes,
		charBitmap64::OpenGLtextBitmap);

	guint fontSizeSmall = (guint)(fontSize*fontSmallFactor);
	guint fontSizeLarge = (guint)(fontSize*fontLargeFactor);
	guint fontSizeHuge = (guint)(fontSize*fontHugeFactor);

	bitmapFont.DeleteBitmap(); //if existing from earlier StartRenderer()
	bitmapFont = bitmap.GetScaledFont(fontSize);
	bitmapFont.toBeDeleted = true; //delete bitmap when object is deleted

	bitmapFontSmall.DeleteBitmap(); //if existing from earlier StartRenderer()
	bitmapFontSmall = bitmap.GetScaledFont(fontSizeSmall);
	bitmapFontSmall.toBeDeleted = true; //delete bitmap when object is deleted

	bitmapFontLarge.DeleteBitmap(); //if existing from earlier StartRenderer()
	bitmapFontLarge = bitmap.GetScaledFont(fontSizeLarge);
	bitmapFontLarge.toBeDeleted = true; //delete bitmap when object is deleted

	bitmapFontHuge.DeleteBitmap(); //if existing from earlier StartRenderer()
	bitmapFontHuge = bitmap.GetScaledFont(fontSizeHuge);
	bitmapFontHuge.toBeDeleted = true; //delete bitmap when object is deleted

#else
	//create textures for fonts
	CreateFontTextures();

	CreateTexturedQuadsLists(bitmapFontListBase, &textureNumberRGBbitmap[0],
		bitmapFont.nCharacters, bitmapFont.characterByteWidth * 8,
		bitmapFont.characterWidth, bitmapFont.characterHeight);

	//std::cout << "texture list=" << bitmapFontListBase << "\n";
	//std::cout << "texture num =" << textureNumberRGBbitmap[0] << "\n";


	//check hardware:
	//GLint sizeMax;
	//glGetIntegerv(GL_MAX_TEXTURE_SIZE, &sizeMax);
	//std::cout << "max texture size=" << sizeMax << "\n";

#endif 

}

//! draw a 0-terminated text string with fontSize, including monitor scaling factor; (for line-characters: size=1: height=1; width=0.5 for one character; distance = 0.25)
void GlfwRenderer::DrawString(const char* text, float fontSizeScaled, const Float3& p, const Float4& color)
{
	if (visSettings->general.useBitmapText)
	{
#ifndef USE_TEXTURED_BITMAP_FONTS
		BitmapFont* bmf;
		if ((guint)fontSizeScaled <= bitmapFontSmall.fontSize) { bmf = &bitmapFontSmall; }
		else if ((guint)fontSizeScaled >= bitmapFontHuge.fontSize) { bmf = &bitmapFontHuge; }
		else if ((guint)fontSizeScaled >= bitmapFontLarge.fontSize) { bmf = &bitmapFontLarge; }
		else { bmf = &bitmapFont; }

		glDisable(GL_LIGHTING);
		bmf->DrawString(text, p, color);
		glEnable(GL_LIGHTING);
#else
		float scale = 2.f*fontSizeScaled*state->zoom / ((float)state->currentWindowSize[1] * (float)bitmapFont.fontSize);
		//std::cout << "scale=" << scale << "\n";
		DrawStringWithTextures(text, scale, p, color, bitmapFont,
			charBuffer, bitmapFontListBase);
#endif
	}
	else
	{
#ifdef OPENGLTEXT_EXISTS //defined in GlfwClientText
		float scale = 2.f*fontSizeScaled*state->zoom / ((float)state->currentWindowSize[1]);
		glLineWidth(visSettings->openGL.textLineWidth);
		if (visSettings->openGL.textLineSmooth) { glEnable(GL_LINE_SMOOTH); }
		OpenGLText::DrawString(text, scale, p, color);
		if (visSettings->openGL.textLineSmooth) { glDisable(GL_LINE_SMOOTH); }
#endif
	}
}

//! now draw a string with internal (close to) unicode string, with length stringLen
void GlfwRenderer::DrawStringWithGLlistTextures(const Float3& p, float fontSizeScaled, GLuint listBase,
	GLubyte *string, guint stringLen)
{
	//std::cout << "test\n";
	//glDisable(GL_DEPTH_TEST);

	//glMatrixMode(GL_MODELVIEW);	//must be done in function previous to call of DrawString(...)
	glPushMatrix();
	//glLoadIdentity();				//must be done in function previous to call of DrawString(...)
	glTranslatef(p[0], p[1], p[2]);
	glScalef(fontSizeScaled, fontSizeScaled, fontSizeScaled);

	glListBase(listBase - 32); //assign base of string list, 32 MUST be smallest value
	glCallLists(stringLen, GL_UNSIGNED_BYTE, string);
	glPopMatrix();
	//glEnable(GL_DEPTH_TEST);
}


void GlfwRenderer::DrawStringWithTextures(const char* text, float fontSizeScaled, const Float3& p, const Float4& color,
	BitmapFont& font, ResizableArray<GLubyte>& charBuffer, GLuint listBase)
{
#ifdef USE_TEXTURED_BITMAP_FONTS
	//glColor4f(color[0], color[1], color[2], color[3]);
	//glRasterPos3f(p[0], p[1], p[2]);

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

	//glDepthMask(GL_FALSE); //done outside, can distinguish between always on top text (status information) and node numbers, etc.
	//glDisable(GL_LIGHTING);
	//glColor4f(0.f, 0.f, 0.f, 1.f); //on texture, this color will influence the appearance (similar to lighting)
	glEnable(MY_GL_TEXTURE_2D); //must be disabled if no textures drawn!
	glColor4f(color[0], color[1], color[2], color[3]); //on texture, this color will influence the appearance (similar to lighting)
	

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
				charBuffer[columnNumber] = 32; //should not happen!
			}
			columnNumber++;
		}
		else
		{
			DrawStringWithGLlistTextures(p + Float3({0,vOff,0}), fontSizeScaled, listBase, charBuffer.GetDataPointer(), charBuffer.NumberOfItems());
			charBuffer.SetNumberOfItems(0);
			vOff -= (float)h*fontSizeScaled;
			//lineNumber++; 
			columnNumber = 0;
		}
		cUnicode = font.GetUnicodeCharacterFromUTF8(text, updatedIndex);
	}
	if (charBuffer.NumberOfItems())
	{
		DrawStringWithGLlistTextures(p + Float3({ 0,vOff,0 }), fontSizeScaled, listBase, charBuffer.GetDataPointer(), charBuffer.NumberOfItems());
	}
	glDisable(MY_GL_TEXTURE_2D); //must be disabled if no textures drawn!
	//glDepthMask(GL_TRUE);
	//glEnable(GL_LIGHTING);

#endif
}

//! create glLists for texture with textureNumber
void GlfwRenderer::CreateTexturedQuadsLists(GLuint& listBase, GLuint* textureNumber,
	guint nCharacters, guint wCharacter8, guint wCharacter, guint hCharacter, bool itemTags)
{
	//GLfloat cx, cy;         /* the character coordinates in our texture */
	listBase = glGenLists(nCharacters);
	//glBindTexture(MY_GL_TEXTURE_2D, textureNumber);
	//glEnable(MY_GL_TEXTURE_2D);
	GLfloat wFact = (float)wCharacter / (float)wCharacter8 - 0.001f;
	//GLfloat wFact = 1.f;

	for (guint loop = 0; loop < nCharacters; loop++)
	{
		glNewList(listBase + loop, GL_COMPILE);
		glBindTexture(MY_GL_TEXTURE_2D, textureNumber[loop]);
		//glColor3f(0.9f, 0.9f, 0.9f);
		glBegin(GL_QUADS);
		glTexCoord2f(0.001f, 0);
		glVertex2i(0, 0);
		glTexCoord2f(wFact, 0);
		glVertex2i(wCharacter, 0);
		glTexCoord2f(wFact, 1);
		glVertex2i(wCharacter, hCharacter);
		glTexCoord2f(0.001f, 1);
		glVertex2i(0, hCharacter);
		glEnd();
		glTranslated(wCharacter, 0, 0);
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
	glGenTextures(bitmapFont.nCharacters, &textureNumberRGBbitmap[0]);   //create one texture

	for (GLuint iChar = 0; iChar < bitmapFont.nCharacters; iChar++)
	{
		GLubyte* textureRGB = bitmapFont.GetRGBFontCharacter(iChar);

		glBindTexture(MY_GL_TEXTURE_2D, textureNumberRGBbitmap[iChar]);
		/* actually generate the texture */
		//glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		//glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //linear filter give nicer results than GL_NEAREST
		glTexParameteri(MY_GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(MY_GL_TEXTURE_2D, 0, 4, bitmapFont.characterByteWidth * 8, //bitmapFont.characterWidth,
			bitmapFont.characterHeight/* *bitmapFont.nCharacters*/, 0, GL_RGBA, GL_UNSIGNED_BYTE,
			textureRGB);
		delete[] textureRGB; //not needed lateron

	}
}




void GlfwRenderer::DeleteFonts()
{
#ifdef USE_TEXTURED_BITMAP_FONTS
	glDeleteTextures(bitmapFont.nCharacters, &textureNumberRGBbitmap[0]);
	glDeleteLists(bitmapFontListBase, bitmapFont.nCharacters);
#endif
}








#endif //USE_GLFW_GRAPHICS
