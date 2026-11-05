#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SC.renderer.GetDictionary and SetDictionary (#2075): the materials, the static graphics of the scopes and
#           the render states of the enabled views of one SystemContainer, changed and set on a second one, which then
#           gives the same dictionary; a key left out stays, an unknown key is refused. The textures as dicts (#3268):
#           SC.renderer.GetTexture gives the image as AddTexture took it, for one, three and four channels; a texture
#           removed is added again from its dict by SetDictionary; SC.GetDictionary has the textures too.
#           SC.GetDictionary and SC.SetDictionary warn of no deprecated function, as they call none (#3271).
#           exudyn.config has its GetDictionary and SetDictionary too.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import numpy as np

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('rendererDictionaryTest: ' + message)
        testError += 1

SC = exu.SystemContainer()
values = SC.renderer.GetDictionary()
Check(sorted(values.keys()) == ['materials', 'scopeGraphics', 'states', 'textures'], 'keys: ' + str(values.keys()))
Check(len(values['materials']) == 10 and values['scopeGraphics'] == [] and list(values['states'].keys()) == [0],
      'the defaults: ten materials, no static graphics, the state of view 0')

#changed and set on a second SystemContainer
values['materials'][1]['baseColor'] = [0.1, 0.2, 0.3]
values['materials'].append(dict(values['materials'][0], name='extra'))
values['scopeGraphics'] = [{'type': 'Line', 'points': [[0, 1], [2, 1]], 'name': 'limit'}]
values['states'][0]['zoom'] = 3.5
SC2 = exu.SystemContainer()
SC2.renderer.SetDictionary(values)
values2 = SC2.renderer.GetDictionary()
Check(np.allclose(values2['materials'][1]['baseColor'], [0.1, 0.2, 0.3]) and len(values2['materials']) == 11
      and values2['materials'][10]['name'] == 'extra', 'the materials')
Check(len(values2['scopeGraphics']) == 1 and values2['scopeGraphics'][0]['name'] == 'limit', 'the static graphics')
Check(abs(values2['states'][0]['zoom'] - 3.5) < 1e-6, 'the render state')

#a key left out stays; an unknown key is refused
SC2.renderer.SetDictionary({'scopeGraphics': []})
Check(SC2.renderer.GetDictionary()['materials'][10]['name'] == 'extra' and SC2.renderer.scopes.GetGraphics() == [],
      'a key left out')
try:
    SC2.renderer.SetDictionary({'material': []})
    Check(False, 'an unknown key is not refused')
except exu.ExudynError:
    pass

#the textures as dicts (#3268): the image as it was added, the first row at the top
imageRGB = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]], [[10, 20, 30], [40, 50, 60], [70, 80, 90]]], dtype=np.uint8)
imageGrey = np.array([[0., 0.5], [1., 0.25]])
imageRGBA = np.random.default_rng(1).integers(0, 256, size=(4, 5, 4)).astype(np.uint8)
SC.renderer.AddTexture('dictTestRGB', imageRGB, interpretation=exu.TextureInterpretation.RGB, flags=exu.TextureFlags.Nearest)
SC.renderer.AddTexture('dictTestGrey', imageGrey, interpretation=exu.TextureInterpretation.Grey)
SC.renderer.AddTexture('dictTestRGBA', imageRGBA, interpretation=exu.TextureInterpretation.RGBAMask,
                       flags=exu.TextureFlags.Clamp | exu.TextureFlags.Replace)
texture = SC.renderer.GetTexture('dictTestRGB')
Check(np.array_equal(texture['image'], imageRGB) and texture['image'].dtype == np.uint8 and texture['interpretation'] == 'RGB'
      and texture['flags'] == int(exu.TextureFlags.Nearest) and texture['name'] == 'dictTestRGB', 'GetTexture RGB')
textureGrey = SC.renderer.GetTexture('dictTestGrey')
Check(np.array_equal(textureGrey['image'], np.round(imageGrey*255)) and textureGrey['image'].shape == (2, 2)
      and textureGrey['interpretation'] == 'Grey', 'GetTexture Grey')
textureRGBA = SC.renderer.GetTexture('dictTestRGBA')
Check(np.array_equal(textureRGBA['image'], imageRGBA) and textureRGBA['interpretation'] == 'RGBAMask'
      and textureRGBA['flags'] == int(exu.TextureFlags.Clamp | exu.TextureFlags.Replace), 'GetTexture RGBA')
Check([t['name'] for t in SC.renderer.GetDictionary()['textures'] if t['name'].startswith('dictTest')]
      == ['dictTestRGB', 'dictTestGrey', 'dictTestRGBA'], 'the textures of GetDictionary')
Check('dictTestRGB' in [t['name'] for t in SC.GetDictionary()['textures']], 'the textures of SC.GetDictionary')
SC.renderer.RemoveTexture('dictTestRGB')
SC2.renderer.SetDictionary({'textures': [texture]})
Check(np.array_equal(SC.renderer.GetTexture('dictTestRGB')['image'], imageRGB), 'a texture added again from its dict')
try:
    SC.renderer.GetTexture('noTexture')
    Check(False, 'an unknown texture is not refused')
except exu.ExudynError:
    pass
for name in ['dictTestRGB', 'dictTestGrey', 'dictTestRGBA']:
    SC.renderer.RemoveTexture(name)

#SC.GetDictionary and SetDictionary with the render state, without a deprecation warning (#3271)
import warnings
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter('always')
    SC3 = exu.SystemContainer()
    SC3.SetDictionary(SC.GetDictionary())
Check([str(w.message) for w in caught if issubclass(w.category, DeprecationWarning)] == [],
      'SC.GetDictionary/SetDictionary warn: ' + str([str(w.message) for w in caught]))

#exudyn.config: the same pair
config = exu.config.GetDictionary()
Check('printToConsole' in config and 'outputPrecision' in config, 'exudyn.config.GetDictionary: ' + str(config.keys()))
exu.config.SetDictionary({'outputPrecision': config['outputPrecision']})

u = testError + len(values2['materials'])
exu.Print('solution of rendererDictionaryTest=', u)

exu.sys['testResult'] = u
