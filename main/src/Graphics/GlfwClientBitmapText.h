/** ***********************************************************************************************
* @class        GlfwClientText
* @brief        Link to 2D Text drawing library
*
* @author       Gerstmayr Johannes
* @date         2019-05-24 (generated)
*
* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.
* @note         Bug reports, support and further information:
                - email: johannes.gerstmayr@uibk.ac.at
                - weblink: https://github.com/jgerstmayr/EXUDYN
                
************************************************************************************************ */
#ifndef GLFWCLIENTBITMAPTEXT__H
#define GLFWCLIENTBITMAPTEXT__H

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"

#ifdef USE_GLFW_GRAPHICS


typedef unsigned int guint; //unsigned int used for graphics module
typedef unsigned char gchar; //unsigned char used for graphics module

//this class is used to set up scaled fonts; this avoids including lots of font libraries
class BitmapFont
{
public:
	guint nCharacters;			//!< number of characters in bitmap
	guint fontSize;				//!< underlying font size
	guint characterOffset;		//!< offset for first character (e.g., index 0 == char(32) ) 
	guint characterWidth;		//!< width of one character in pixel
	guint characterHeight;		//!< height of one character in pixel
	guint characterByteWidth;	//!< width in bytes
	guint characterBytes;		//!< number of bytes per character
	gchar* openGLBitmap;				//!< black/white bitmap stored for all characters; size=characterBytes*nCharacters; every byte stores 8 pixels
	bool toBeDeleted;					//!< set this flag such that delete operator deletes bitmap

public:
	BitmapFont() {
		toBeDeleted = false; nCharacters = 0; characterOffset = 0; fontSize = 1; characterWidth
			= 1; characterHeight = 1; characterBytes = 0; characterByteWidth = 1; };
	BitmapFont(guint fontSizeInit, guint nCharactersInit, guint characterOffsetInit,
		guint characterWidthInit, guint characterHeightInit,
		guint characterByteWidthInit, guint characterBytesInit, gchar* bitmap)
	{
		fontSize = fontSizeInit;
		nCharacters = nCharactersInit;
		characterOffset = characterOffsetInit;
		characterWidth = characterWidthInit;
		characterHeight = characterHeightInit;
		characterByteWidth = characterByteWidthInit;
		characterBytes = characterBytesInit;
		openGLBitmap = bitmap;
		toBeDeleted = false;
	}

	//! erase bitmaps which are created with new
	~BitmapFont()
	{
		DeleteBitmap();
	}

	void DeleteBitmap() 
	{
		if (toBeDeleted)
		{
			delete[] openGLBitmap;
		}
	}

	gchar* GetCharacterBitmap(guint iChar) 
	{
		return &openGLBitmap[iChar * characterBytes];
	}

	//! get a new BitmapFont instance with a newly created bitmap set
	//! alternatives using glDrawPixels are slow or complicated and unsure to work on different platforms
	BitmapFont GetScaledFont(guint newFontSize)
	{
		if (newFontSize > 256)
		{
			newFontSize = 256; //this leads already to 15Mpixels to store for 1 font
		}
		float fact = (float)newFontSize / (float)fontSize;
		guint hNew = (guint)(fact*characterHeight);
		guint wNew = (guint)(fact*characterWidth);

		//guint hNew = newFontSize;
		//float fact = (float)hNew / (float)characterHeight;
		//guint wNew = (guint)(fact*characterWidth);

		BitmapFont newFont;
		newFont.fontSize = newFontSize;
		newFont.nCharacters = nCharacters;
		newFont.characterOffset = characterOffset;

		newFont.characterWidth = wNew;
		newFont.characterHeight = hNew;
		newFont.characterByteWidth = wNew/8;
		if (wNew % 8 != 0) { newFont.characterByteWidth++; } //add another byte for last bits if they exist

		newFont.characterBytes = newFont.characterByteWidth * hNew;
		gchar* bitmap = new gchar[newFont.characterBytes * nCharacters];
		newFont.openGLBitmap = bitmap;
		newFont.toBeDeleted = false; //set this to true in the returned font object!

		//now copy scaled pixels:
		for (guint iChar = 0; iChar < newFont.nCharacters; iChar++)
		{
			for (guint yNew = 0; yNew < newFont.characterHeight; yNew++)
			{
				for (guint xNew = 0; xNew < newFont.characterWidth; xNew++)
				{
					guint x = (guint)(xNew/fact);
					guint y = (guint)(yNew/fact);
					if (x >= characterWidth) { x = characterWidth - 1; }
					if (y >= characterHeight) { y = characterHeight - 1; }
					if (fact > 1 && x < characterWidth-1 && y < characterHeight-1)
					{
						guint pix = 0;
						if ((GetPixel(iChar, x, y) + 
							GetPixel(iChar, (guint)((xNew+1) / fact), y) +
							GetPixel(iChar, x, (guint)((yNew+1) / fact)) +
							GetPixel(iChar, (guint)((xNew + 1) / fact), (guint)((yNew + 1) / fact))) > 1)
						{
							pix = 1;
						}
						newFont.SetPixel(iChar, xNew, yNew, pix);
					}
					else
					{
						newFont.SetPixel(iChar, xNew, yNew, GetPixel(iChar, x, y));
					}
				}
			}
		}
		return newFont;
	}

	//! get new RGB image for opengl texture2D for given character (needs to be deleted hereafter)
	gchar* GetRGBFontCharacter(guint iChar)
	{
		const int colorSize = 4; //RGBA
		gchar* font;
		guint w = characterByteWidth * 8; //make multiple of 8 (problems with overlapping area?)
		//guint w = characterWidth; 
		guint h = characterHeight;
		font = new gchar[w*h * colorSize]; //RGB bytes

		for (guint y = 0; y < h; y++)
		{
			for (guint x = 0; x < w; x++)
			{
				gchar b = (1 - GetPixel(iChar, x, y)) * 255; 
				//gchar b = GetPixel(iChar, x, y) * 255;

				font[y*w * colorSize + x * colorSize + 0] = 255; //all color white ==> text color will be added; 
				font[y*w * colorSize + x * colorSize + 1] = 255;
				font[y*w * colorSize + x * colorSize + 2] = 255;
				font[y*w * colorSize + x * colorSize + 3] = 255-b; //color has no effect ==> character will be visible ...
			}
		}
		return font;
	}

	//! get new RGBA image for opengl texture2D for given character (needs to be deleted hereafter)
	gchar* GetRGBAFontBitmap()
	{
		const int colorSize = 4; //RGBA
		gchar* font;
		//guint w = characterByteWidth * 8; //make multiple of 8 (problems with overlapping area?)
		guint w = characterWidth; 
		guint h = characterHeight;
		font = new gchar[nCharacters* w*h * colorSize]; //RGB bytes

		for (guint iChar = 0; iChar < nCharacters; iChar++)
		{
			for (guint y = 0; y < h; y++)
			{
				for (guint x = 0; x < w; x++)
				{
					//gchar b = (1 - GetPixel(iChar, x, y)) * 255; //for RGB, use black as color
					gchar b = GetPixel(iChar, x, y) * 255; //use black for background, white for color (color can then be changed with glColorf)

					font[iChar*h*w * colorSize + y*w * colorSize + x * colorSize + 0] = b;
					font[iChar*h*w * colorSize + y*w * colorSize + x * colorSize + 1] = b;
					font[iChar*h*w * colorSize + y * w * colorSize + x * colorSize + 2] = b;
					font[iChar*h*w * colorSize + y * w * colorSize + x * colorSize + 3] = 255-b; //for white pixels, use no transparency; for black pixels full transparency
				}
			}
		}
		//for (int i = 0; i < nCharacters* w*h * colorSize; i++)
		//{
		//	font[i] = 0;
		//}
		return font;
	}

	//! get pixel of current bitmap for char iChar at position x/y
	guint GetPixel(guint iChar, guint x, guint y) const
	{
		guint rv = 0;
		guint bitIndex = x % 8;
		guint bitNumber = 1 << bitIndex;

		guint byteIndex = x / 8;
		guint offset = iChar * characterBytes + y * characterByteWidth + byteIndex;

		if ((x < characterWidth) && ((openGLBitmap[offset] & bitNumber) != 0)) { rv = 1; }
		//if (openGLBitmap[offset] & bitNumber) { rv = 1; }
		return rv;
	}

	//! set pixel of current bitmap for char iChar at position x/y, value = 0/1
	void SetPixel(guint iChar, guint x, guint y, guint value)
	{
		guint bitIndex = x % 8;
		guint bitNumber = 1 << bitIndex;

		guint byteIndex = x / 8;
		guint offset = iChar * characterBytes + y * characterByteWidth + byteIndex;

		if (value)
		{
			openGLBitmap[offset] |= bitNumber;
		}
		else
		{
			openGLBitmap[offset] &= 255-bitNumber;
		}
	}

	//! return a unicode char converted from UTF8 character encoding
	//! updatedIndex jumps to next UTF8 char position after readout
	//! returns 0xBF (inverted '?') if character cannot be converted to unicode
	//! returns 0xA1 (inverted '!') if error occurs
	//! take care: char is signed and conversion to unsigned int yields something like 0xFFFFFFxx for chars >= 0x80
	gchar GetUnicodeCharacterFromUTF8(const char* text, guint& updatedIndex)
	{
		gchar ci0 = (gchar)(text[updatedIndex]); //first character

		//use UTF-8 encoding: https://de.wikipedia.org/wiki/UTF-8
		//accept unicode characters: https://www.utf8-chartable.de/unicode-utf8-table.pl?unicodeinhtml=dec&htmlent=1
		if (ci0 < 0x80)
		{
			updatedIndex++;
			return ci0;
		}
		else
		{
			gchar ci; //unicode character which is returned
			guint sequenceLen = 0;

			if ((ci0 & (0x80 + 0x40 + 0x20 + 0x10 + 0x08)) == 0xF0) //start of 4-byte sequence: 11110xxx ...
			{
				sequenceLen = 4;
			}
			else if ((ci0 & (0x80 + 0x40 + 0x20 + 0x10)) == 0xE0) //start of 3-byte sequence: 1110xxxx ...
			{
				sequenceLen = 3;
				//std::cout << "start of 3-byte-UTF8\n";
			}
			else if ((ci0 & (0x80 + 0x40 + 0x20)) == 0xC0) //start of 2-byte sequence: 110xxxxx ...
			{
				sequenceLen = 2;
				//std::cout << "start of 2-byte-UTF8\n";
			}
			else //probably a unicode character encoding; simply do not show this!
			{ 
				updatedIndex++; 
				//std::cout << "not expected:" << (guint)ci0 << "\n";
				return 0xA1; 
			} //not expected; return inverted ! and increment

			bool ok = true; //check if zero occurs (which is wrong, but could happen in some weird encodings)
			for (guint i = 1; i < sequenceLen; i++)
			{
				if (text[updatedIndex + i] == 0) { ok = false; }
			}

			if (!ok) { return 0; } //if zero occurs, string ends
			else
			{
				gchar ci1 = (gchar)(text[updatedIndex+1]); //second character (must exist at this point)

				ci = 0xBF; //return inverted '?' if character cannot be converted to unicode
				if (sequenceLen == 2)
				{
					if (ci0 == 0xC2 && ci1 >= 0xA0 && ci1 <= 0xBF)
					{
						ci = ci1; //second char directly maps to unicode character
					}
					else if (ci0 == 0xC3 && ci1 >= 0x80 && ci1 <= 0xBF)
					{
						ci = ci1+0x40; //second char maps to unicode character + 0x40: 0x80 -> 0xC0
					}
					else if (ci0 == 0xCE && ci1 == 0xB1) { ci = 144; } //alpha
					else if (ci0 == 0xCE && ci1 == 0xB2) { ci = 145; } //...
					else if (ci0 == 0xCE && ci1 == 0xB3) { ci = 146; } //...
					else if (ci0 == 0xCE && ci1 == 0xB4) { ci = 147; } //...
					else if (ci0 == 0xCE && ci1 == 0xB5) { ci = 148; } //...
					else if (ci0 == 0xCE && ci1 == 0xB6) { ci = 149; } //...
					else if (ci0 == 0xCE && ci1 == 0xB7) { ci = 150; } //...
					else if (ci0 == 0xCE && ci1 == 0xB8) { ci = 151; } //...
					else if (ci0 == 0xCE && ci1 == 0xBA) { ci = 152; } //...
					else if (ci0 == 0xCE && ci1 == 0xBB) { ci = 153; } //...
					else if (ci0 == 0xCE && ci1 == 0xBD) { ci = 154; } //...
					else if (ci0 == 0xCE && ci1 == 0xBE) { ci = 155; } //...
					else if (ci0 == 0xCF && ci1 == 0x80) { ci = 156; } //...
					else if (ci0 == 0xCF && ci1 == 0x81) { ci = 157; } //...
					else if (ci0 == 0xCF && ci1 == 0x83) { ci = 158; } //...
					else if (ci0 == 0xCF && ci1 == 0x95) { ci = 159; } //...
					//
					else if (ci0 == 0xCF && ci1 == 0x87) { ci = 130; } //...
					else if (ci0 == 0xCF && ci1 == 0x88) { ci = 131; } //...
					else if (ci0 == 0xCF && ci1 == 0x89) { ci = 132; } //...
					else if (ci0 == 0xCE && ci1 == 0x94) { ci = 133; } //...
					else if (ci0 == 0xCE && ci1 == 0xA0) { ci = 134; } //...
					else if (ci0 == 0xCE && ci1 == 0xA3) { ci = 135; } //...
					else if (ci0 == 0xCE && ci1 == 0xA9) { ci = 136; } //...
					//

					//std::cout << "c0=" << ci0 << ", c1=" << ci1 << "\n";
				}
				else if (sequenceLen == 3)
				{
					gchar ci2 = (gchar)(text[updatedIndex + 2]); //third character (must exist at this point)
					if (ci0 == 0xE2 && ci1 == 0x88 && ci2 == 0x82) { ci = 128; } //partial
					else if (ci0 == 0xE2 && ci1 == 0x88 && ci2 == 0xAB) { ci = 129; } //int
					else if (ci0 == 0xE2 && ci1 == 0x99 && ci2 == 0xA5) { ci = 137; } //like
					else if (ci0 == 0xE2 && ci1 == 0x88 && ci2 == 0x9A) { ci = 139; } //sqrt
					else if (ci0 == 0xE2 && ci1 == 0x89 && ci2 == 0x88) { ci = 140; } //approx
					else if (ci0 == 0xE2 && ci1 == 0x88 && ci2 == 0x9E) { ci = 143; } //infinity
				}
				else if (sequenceLen == 4)
				{
					gchar ci2 = (gchar)(text[updatedIndex + 2]); //third character (must exist at this point)
					gchar ci3 = (gchar)(text[updatedIndex + 3]); //fourth character (must exist at this point)
					if (ci0 == 0xF0 && ci1 == 0x9F && ci2 == 0x99 && ci3 == 0x82) { ci = 141; } //smiley
					else if (ci0 == 0xF0 && ci1 == 0x9F && ci2 == 0x98 && ci3 == 0x92) { ci = 142; } //frowney
				}
				//if (ci == 0xBF) {std::cout << "unknown:" << (guint)ci0 << "\n";}
			}
			updatedIndex += sequenceLen;
			return ci;
		}
	}

	//! function draws string as bitmap text, using the internal bitmap font
	void DrawString(const char* text, const Float3& p, const Float4& color)
	{
		glColor4f(color[0], color[1], color[2], color[3]);  /* white */
		glRasterPos3f(p[0], p[1], p[2]);

		glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		//GLsizei w = characterWidth;
		//GLsizei h = characterHeight;

		//Draws the bitmap specified by bitmap, which is a pointer to the bitmap image.
		//The origin of the bitmap is placed at the current raster position.If the current raster position is invalid, nothing is drawn, and the raster position remains invalid.
		//The width and height arguments indicate the width and height, in pixels, of the bitmap.
		//The width need not be a multiple of 8, although the data is stored in unsigned characters of 8 bits each. 
		//(In the F example, it wouldn't matter if there were garbage bits in the data beyond the tenth bit; since glBitmap() was called with a width of 10, only 10 bits of the row are rendered.) 
		//Use xorig and yorig to define the origin of the bitmap (positive values move the origin up and to the right of the raster position; 
		//negative values move it down and to the left); 
		//xbi and ybi indicate the x and y increments that are added to the raster position after the bitmap is rasterized (see Figure 8-2).
		
		//guint i = 0;
		guint lineNumber = 0; //for several lines of text
		guint columnNumber = 0;
		//std::string ss(text);
		//std::cout << ss << "\n";
		
		guint updatedIndex = 0;
		guint cUnicode = (guint)GetUnicodeCharacterFromUTF8(text, updatedIndex);
		while (cUnicode != 0)
		{
			//use UTF-8 encoding: https://de.wikipedia.org/wiki/UTF-8
			//accept unicode characters: https://www.utf8-chartable.de/unicode-utf8-table.pl?unicodeinhtml=dec&htmlent=1
			//unsigned char c = text[i];
			//guint ci = (guint)c;
			if (cUnicode != '\n')
			{
				//std::cout << c << ":" << ci << "\n";
				if (cUnicode >= characterOffset && cUnicode < nCharacters + characterOffset) //do not print control characters ...
				{
					glBitmap(characterWidth, characterHeight, 
						-(GLfloat)(columnNumber*characterWidth), (GLfloat)(lineNumber*characterHeight),
						(GLfloat)(characterWidth)*0, 0, GetCharacterBitmap(cUnicode -characterOffset)); // &bitmapFont.openGLBitmap[ii*bitmapFont.characterBytes]);
				}
				columnNumber++;
			}
			else 
			{ 
				lineNumber++; columnNumber = 0; 
			}
			//i++;
			cUnicode = GetUnicodeCharacterFromUTF8(text, updatedIndex);
		}
	}
};


#endif //USE_GLFW_GRAPHICS
#endif //include once
