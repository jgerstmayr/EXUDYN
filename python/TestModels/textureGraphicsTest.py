#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Textures in the raytracer and in exudyn.graphics (#3031, #3034, #3035, #3036): the raytracer samples a
#           texture with Nearest and Replace, cuts out the mask of RGBAMask and draws the colors of the points with
#           openGL.showTextures=False; graphics.Quad and graphics.Cylinder with a texture, graphics.ProjectTexture
#           planar, box, cylindrical and spherical; graphics.CheckerBoard with its texture looks like the one of
#           triangles; Transform, MergeTriangleLists and InvertTriangles keep the texture, a list merged with one without
#           texture has its triangles drawn without; graphics.ImageFromFile reads a PNG written before and the JPEG of
#           a wooden floor of the examples (testData/hardwood_small.jpg, OpenGameArt, CC0), drawn brown.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
import numpy as np
import os

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('textureGraphicsTest: ' + message)
        testError += 1

def Render(graphicsDataList, showTextures=True, size=80):
    """the ground with graphicsDataList seen from above through the raytracer, without texts"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    mbs.CreateGround(graphicsDataList=graphicsDataList)
    mbs.Assemble()
    v = SC.visualizationSettings
    v.view0.window.renderWindowSize = [size, size]
    v.view0.window.showComputationInfo = False
    v.view0.scene.drawWorldBasis = False
    v.view0.scene.drawCoordinateSystem = 0
    v.openGL.showTextures = showTextures
    SC.renderer.ZoomAll()
    return SC.renderer.RedrawAndGetImage(useRaytracer=True).astype(int)

def Dominant(pixel): #0 red, 1 green, 2 blue
    return int(np.argmax(pixel))

square = [[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]]
SC = exu.SystemContainer()

#the raytracer samples the texture: red at the top left of the image, as in the image
image = np.zeros((2, 2, 3), dtype=np.uint8)
image[0, 0] = image[1, 1] = [255, 0, 0]
image[0, 1] = image[1, 0] = [0, 0, 255]
SC.renderer.AddTexture('redBlue', image, interpretation=exu.TextureInterpretation.RGB,
                       flags=exu.TextureFlags.Nearest | exu.TextureFlags.Replace)
im = Render([graphics.Quad(square, texture='redBlue')])
quadrants = [Dominant(im[20, 20]), Dominant(im[20, 60]), Dominant(im[60, 20]), Dominant(im[60, 60])]
Check(quadrants == [0, 2, 2, 0], 'raytraced texture: ' + str(quadrants))
im = Render([graphics.Quad(square, texture='redBlue')], showTextures=False)
Check(np.ptp(im[20, 20]) < 30 and np.ptp(im[60, 60]) < 30, 'showTextures=False: not the white of the points')

#the mask: the left half cut out, the green quad below is seen there
mask = np.zeros((4, 4, 4))
mask[:, 2:] = [1, 0, 0, 1]
SC.renderer.AddTexture('halfMask', mask, interpretation=exu.TextureInterpretation.RGBAMask, flags=exu.TextureFlags.Nearest)
green = graphics.Quad([[-1, -1, -0.1], [1, -1, -0.1], [1, 1, -0.1], [-1, 1, -0.1]], color=[0, 1, 0, 1])
im = Render([graphics.Quad(square, texture='halfMask'), green])
Check(Dominant(im[40, 20]) == 1 and Dominant(im[40, 60]) == 0, 'mask: ' + str(im[40, 20]) + str(im[40, 60]))

#graphics.Quad and graphics.Cylinder with a texture
quad = graphics.Quad(square, texture='redBlue', textureTiles=[2, 3])
Check(np.allclose(quad['textureCoordinates'], [[0, 0], [2, 0], [2, 3], [0, 3]]) and np.allclose(quad['colors'], 1),
      'Quad texture coordinates')
cylinder = graphics.Cylinder(pAxis=[0, 0, 0], vAxis=[0, 0, 2], radius=0.5, nTiles=16, texture='redBlue', textureTiles=[3, 2])
uv = np.asarray(cylinder['textureCoordinates'])[np.asarray(cylinder['triangles']).flatten()].reshape((-1, 3, 2))
points = np.asarray(cylinder['points'])[np.asarray(cylinder['triangles']).flatten()].reshape((-1, 3, 3))
mantle = np.abs(points[:, :, 2] - points[:, :, 2].mean(axis=1, keepdims=True)).max(axis=1) > 1e-10 #not on a face
Check('triangles6' not in cylinder and np.ptp(uv[mantle][:, :, 0], axis=1).max() < 0.5, 'Cylinder: a triangle across the seam')
Check(abs(np.ptp(uv[mantle][:, :, 0]) - 3) < 0.2 and abs(np.ptp(uv[mantle][:, :, 1]) - 2) < 1e-12, 'Cylinder: tiles around and along')
Check(uv[~mantle].min() > -1e-12 and uv[~mantle].max() < 1 + 1e-12, 'Cylinder: faces once across the diameter')
u = round(float(np.sum(np.abs(uv))), 6) #rounded: the angles come from numpy, whose last bits may differ between platforms

#ProjectTexture
brick = graphics.ProjectTexture(graphics.Brick(size=[2, 1, 0.5]), 'redBlue', projection='box', size=0.5)
uvBrick = np.asarray(brick['textureCoordinates'])[np.asarray(brick['triangles']).flatten()].reshape((-1, 3, 2))
Check(np.allclose(np.sort(np.ptp(uvBrick, axis=1), axis=1).max(axis=0), [2, 4]), 'box: the extent of the faces in images')
planar = graphics.ProjectTexture(graphics.Brick(size=[2, 1, 0.5]), 'redBlue', projection='planar', size=[2, 1])
Check(len(planar['points']) == len(graphics.Brick(size=[2, 1, 0.5])['points']) and abs(np.ptp(planar['textureCoordinates'][:, 0]) - 1) < 1e-12,
      'planar: the points stay')
sphere = graphics.ProjectTexture(graphics.Sphere(radius=1, nTiles=16), 'redBlue', projection='spherical', tiles=2)
uvSphere = np.asarray(sphere['textureCoordinates'])[np.asarray(sphere['triangles']).flatten()].reshape((-1, 3, 2))
Check(uvSphere[:, :, 1].min() > -1e-12 and uvSphere[:, :, 1].max() < 1 + 1e-12 and np.ptp(uvSphere[:, :, 0], axis=1).max() < 0.5,
      'spherical: v from pole to pole, no triangle across the seam')
u += round(float(np.sum(np.abs(uvBrick))) + float(np.sum(np.abs(uvSphere))), 6)

#the texture stays with Transform, MergeTriangleLists and InvertTriangles; two textures are not merged
moved = graphics.Move(quad, [0, 0, 1])
merged = graphics.MergeTriangleLists(quad, moved)
inverted = graphics.InvertTriangles(quad)
Check(moved['texture'] == 'redBlue' and np.allclose(moved['textureCoordinates'], quad['textureCoordinates']), 'Move')
Check(merged['texture'] == 'redBlue' and len(merged['textureCoordinates']) == 8, 'MergeTriangleLists')
Check(inverted['texture'] == 'redBlue' and np.allclose(inverted['textureCoordinates'], quad['textureCoordinates']), 'InvertTriangles')
try:
    graphics.MergeTriangleLists(quad, graphics.Quad(square, texture='halfMask'))
    Check(False, 'MergeTriangleLists of two textures not refused')
except ValueError:
    pass

#a list with a texture merged with one without: the points of the second have the texture coordinates nan, their
#triangles are drawn without the texture
mixed = graphics.MergeTriangleLists(graphics.Quad(square, texture='redBlue'),
                                    graphics.Quad([[-1, -1, 0.1], [0, -1, 0.1], [0, 1, 0.1], [-1, 1, 0.1]], color=[0, 1, 0, 1]))
Check(mixed['texture'] == 'redBlue' and np.isnan(mixed['textureCoordinates'][4:]).all(), 'merged with a list without texture')
im = Render([mixed])
Check(Dominant(im[40, 20]) == 1 and Dominant(im[20, 60]) == 2, 'merged: ' + str(im[40, 20]) + str(im[20, 60]))

#CheckerBoard: one quad with a texture of its colors, which looks like the one of triangles
board = graphics.CheckerBoard(size=2, nTiles=4)
boardTriangles = graphics.CheckerBoard(size=2, nTiles=4, useTexture=False)
Check(board['texture'].startswith('exudyn.checkerBoard.') and board['texture'] in SC.renderer.GetTextureNames()
      and len(board['triangles']) == 2 and len(boardTriangles['triangles']) == 2*4*4, 'CheckerBoard')
imTexture = Render([board])
imTriangles = Render([boardTriangles])
differing = np.mean(np.abs(imTexture - imTriangles).max(axis=2) > 20)
Check(differing < 0.05, 'CheckerBoard with texture differs from the one of triangles in ' + str(differing) + ' of the pixels')

#ImageFromFile: a PNG written before, with matplotlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from exudyn.basicUtilities import OutputFilePath
fileName = OutputFilePath('textureGraphicsTest.png', 'textureGraphicsTest')
if os.path.dirname(fileName) != '':
    os.makedirs(os.path.dirname(fileName), exist_ok=True)
written = np.zeros((3, 5, 3), dtype=np.uint8)
written[0, :] = [255, 0, 0]
written[:, 4] = [0, 0, 255]
plt.imsave(fileName, written)
read = graphics.ImageFromFile(fileName)
Check(read.dtype == np.uint8 and read.shape[0:2] == (3, 5) and np.array_equal(read[:, :, 0:3], written), 'ImageFromFile: ' + str(read.shape))
os.remove(fileName)

#a real texture: the JPEG of a wooden floor (OpenGameArt, CC0) of the examples, brown on a quad in the raytracer
wood = graphics.ImageFromFile('../Examples/testData/hardwood_small.jpg')
meanColor = wood.reshape((-1, 3)).mean(axis=0)
Check(wood.dtype == np.uint8 and wood.shape == (1024, 1024, 3) and meanColor[0] > meanColor[1] > meanColor[2],
      'hardwood_small.jpg: ' + str(wood.shape) + str(meanColor))
SC.renderer.AddTexture('hardwood', wood, interpretation=exu.TextureInterpretation.RGB)
im = Render([graphics.Quad(square, texture='hardwood', textureTiles=[2, 2])])
Check(im[40, 40, 0] > im[40, 40, 1] > im[40, 40, 2], 'raytraced wood is not brown: ' + str(im[40, 40]))

for name in ['redBlue', 'halfMask', 'hardwood']:
    SC.renderer.RemoveTexture(name)

u += testError
exu.Print('solution of textureGraphicsTest=', u)

exu.sys['testResult'] = u
