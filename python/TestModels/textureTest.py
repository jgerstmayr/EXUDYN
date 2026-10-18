#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Textures (#2154, #2155, #3029, #3030): SC.renderer.AddTexture with images of uint8 and of floats, grey,
#           RGB and RGBA, the names, replacing and removing; a TriangleList with 'texture' and 'textureCoordinates'
#           on a ground and on a rigid body, also with 6-node triangles, which become four flat textured triangles;
#           SC.renderer.GetGraphicsData returns them with their texture coordinates, moved with the body; the
#           graphics data of an object returns them as a TriangleList with the texture; what is refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-18
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('textureTest: ' + message)
        testError += 1

def Names(): #without the textures of Exudyn, e.g. of graphics.CheckerBoard in models before
    return [name for name in SC.renderer.GetTextureNames() if not name.startswith('exudyn.')]

def Refused(function, errorType=ValueError):
    try:
        function()
    except errorType:
        return True
    return False

#the textures: a checkerboard of uint8, a grey gradient of floats, a mask of RGBA as nested list
checker = np.zeros((8, 8, 3), dtype=np.uint8)
checker[(np.add.outer(np.arange(8), np.arange(8)) % 2) == 0] = [255, 255, 255]
SC.renderer.AddTexture('checker', checker, interpretation=exu.TextureInterpretation.RGB, flags=exu.TextureFlags.Nearest)
SC.renderer.AddTexture('gradient', np.linspace(0, 1, 16).reshape((4, 4)), interpretation=exu.TextureInterpretation.Grey)
SC.renderer.AddTexture('mask', [[[1, 0, 0, 1], [0, 0, 0, 0]], [[0, 0, 0, 0], [1, 0, 0, 1]]], interpretation=exu.TextureInterpretation.RGBAMask,
                       flags=exu.TextureFlags.Clamp | exu.TextureFlags.Replace)
SC.renderer.AddTexture('temporary', checker) #RGB image as RGBA: opaque
Check(Names() == ['checker', 'gradient', 'mask', 'temporary'], 'names: ' + str(Names()))
SC.renderer.AddTexture('checker', checker[:4, :4]) #replaces the image, the name stays in place
Check(SC.renderer.RemoveTexture('temporary') and not SC.renderer.RemoveTexture('temporary'), 'RemoveTexture')
Check(Names() == ['checker', 'gradient', 'mask'], 'names after remove: ' + str(Names()))
Check([n for n in exu.SystemContainer().renderer.GetTextureNames() if not n.startswith('exudyn.')] == ['checker', 'gradient', 'mask'],
      'textures are shared by the SystemContainers')

#what AddTexture refuses
Check(Refused(lambda: SC.renderer.AddTexture('a', np.full((2, 2, 3), 2.))), 'floats above 1 not refused')
Check(Refused(lambda: SC.renderer.AddTexture('a', np.zeros((2, 2, 2)))), '2 channels not refused')
Check(Refused(lambda: SC.renderer.AddTexture('a', checker, interpretation=exu.TextureInterpretation.Grey)), 'Grey of a color image not refused')
Check(Refused(lambda: SC.renderer.AddTexture('exudyn.icon', checker)), 'reserved name not refused')
Check(Refused(lambda: SC.renderer.AddTexture('', checker)), 'empty name not refused')
Check(Refused(lambda: SC.renderer.AddTexture('a', np.zeros((0, 4, 3)))), 'empty image not refused')
Check(Refused(lambda: SC.renderer.AddTexture('a', checker, flags=16)), 'unknown flag bits not refused')
Check(Refused(lambda: SC.renderer.AddTexture('a', checker, flags=[exu.TextureFlags.Nearest]), TypeError), 'a list of flags not refused')
Check('a' not in Names(), 'a refused texture was added')

#a quad of two triangles with the checkerboard, on the ground
quad = {'type': 'TriangleList', 'points': [[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]], 'triangles': [[0, 1, 2], [0, 2, 3]],
        'colors': [graphics.color.white] * 4, 'texture': 'checker', 'textureCoordinates': [[0, 0], [4, 0], [4, 2], [0, 2]]}
mbs.CreateGround(graphicsDataList=[quad, graphics.Brick(centerPoint=[0, 0, -1], size=[0.2, 0.2, 0.2])])

#a 6-node triangle with the gradient, on a rigid body at [5,0,0]: four flat textured triangles
triangle6 = graphics.FromPointsAndTrigs([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.5, 0, 0], [0.5, 0.5, 0], [0, 0.5, 0]],
                                        [[0, 1, 2, 3, 4, 5]], color=graphics.color.red)
triangle6['texture'] = 'gradient'
triangle6['textureCoordinates'] = [[0, 0], [1, 0], [0, 1], [0.5, 0], [0.5, 0.5], [0, 0.5]]
body = mbs.CreateRigidBody(referencePosition=[5, 0, 0], inertia=InertiaCuboid(1000, [1, 1, 1]),
                           graphicsDataList=[triangle6])
mbs.Assemble()

data = SC.renderer.GetGraphicsData()
textured = data['texturedTriangles']
Check(len(textured['items']) == 6 and textured['texture'] == ['checker'] * 2 + ['gradient'] * 4,
      'textured triangles: ' + str(textured['texture']))
Check(len(data['triangles6']['items']) == 0, 'the textured 6-node triangle is not split')
Check(np.allclose(textured['textureCoordinates'][1], [[0, 0], [4, 2], [0, 2]]), 'texture coordinates of the quad')
Check(np.allclose(textured['points'][2:, :, 0].min(), 5), 'the triangles of the body are not moved with it')
Check(np.allclose(textured['textureCoordinates'][5], [[0.5, 0], [0.5, 0.5], [0, 0.5]]), 'texture coordinates of the middle triangle')
Check(np.all(textured['items'][2:, 2] == body.GetIndex()), 'item of the body triangles')
u = float(np.sum(textured['textureCoordinates']) + np.sum(textured['points']))

#the graphics data of the body, back as a TriangleList with the texture
graphicsList = mbs.GetObjectParameter(body, 'VgraphicsData')
texturedList = [g for g in graphicsList if 'texture' in g]
Check(len(texturedList) == 1 and texturedList[0]['texture'] == 'gradient' and len(texturedList[0]['triangles']) == 4
      and np.asarray(texturedList[0]['textureCoordinates']).shape == (12, 2), 'graphics data of the body: ' + str(texturedList))

#a removed texture: the triangles stay and are drawn with the colors of their points
SC.renderer.RemoveTexture('gradient')
Check(SC.renderer.GetGraphicsData()['texturedTriangles']['texture'][2:] == ['gradient'] * 4, 'triangles of a removed texture')

#what a TriangleList with a texture refuses
def Ground(**keys):
    g = dict(quad)
    g.update(keys)
    g = {k: v for k, v in g.items() if v is not None}
    mbs.CreateGround(graphicsDataList=[g])
Check(Refused(lambda: Ground(texture='gradient')), 'removed texture not refused')
Check(Refused(lambda: Ground(texture='unknown')), 'unknown texture not refused')
Check(Refused(lambda: Ground(textureCoordinates=None)), 'texture without textureCoordinates not refused')
Check(Refused(lambda: Ground(textureCoordinates=[[0, 0]] * 3)), 'wrong number of textureCoordinates not refused')
Check(Refused(lambda: Ground(texture=None)), 'textureCoordinates without texture not refused')
Check(Refused(lambda: Ground(texture=3), TypeError), 'a texture that is not a name not refused')
Check(mbs.systemData.NumberOfObjects() == 2, 'a refused ground was added')

for name in Names():
    SC.renderer.RemoveTexture(name) #the store is shared by the module, the other tests start without textures

u += testError
exu.Print('solution of textureTest=', u)

exu.sys['testResult'] = u
