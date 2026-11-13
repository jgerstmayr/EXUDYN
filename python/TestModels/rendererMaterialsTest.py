#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The materials of the raytracer in both of their places (#3490): the ten predefined materials are
#           SC.visualizationSettings.raytracer.material0 to material9 and the first ten of SC.renderer.materials.
#           A change in the settings is seen by Get, GetDict, [] and GetDictionary of the material list; Set of one
#           material keeps the changes of the others made in the settings; a material renamed in the settings is
#           found by its new name; Set writes into the settings. GetDict, Set with a dict and the renderer's
#           dictionary carry the roughness of blurred reflections. An appended material (index 10) is drawn by the
#           raytracer with the color 1010.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-13
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.graphics as graphics
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('rendererMaterialsTest: ' + message)
        testError += 1

SC = exu.SystemContainer()
settings = SC.visualizationSettings.raytracer
materials = SC.renderer.materials

#a change in the settings is seen by every access of the material list
settings.material4.reflectivity = 0.5 #chrome
Check(materials.Get('chrome').reflectivity == 0.5, 'Get after a change of the settings')
Check(materials.GetDict(4)['reflectivity'] == 0.5, 'GetDict after a change of the settings')
Check(materials[4].reflectivity == 0.5, '[] after a change of the settings')
Check(SC.renderer.GetDictionary()['materials'][4]['reflectivity'] == 0.5, 'GetDictionary after a change of the settings')

#Set of one material keeps the changes of the others in the settings, and writes its own into them
settings.material2.shininess = 7. #steel
chrome = materials.Get('chrome')
chrome.roughness = 0.03
materials.Set('chrome', chrome)
Check(settings.material2.shininess == 7., 'Set of chrome undid the change of steel in the settings')
Check(settings.material4.reflectivity == 0.5, 'Set of chrome undid the change of its own reflectivity')
Check(abs(settings.material4.roughness - 0.03) < 1e-7, 'Set did not write the roughness into the settings')

#a material renamed in the settings is found by its new name
settings.material5.name = 'gold'
Check(materials.Get('gold').reflectivity == settings.material5.reflectivity, 'Get by the new name')

#the roughness in the dictionaries
material = materials.GetDict('glass')
Check('roughness' in material, 'GetDict has no roughness')
material['roughness'] = 0.05
materials.Set('glass', material)
Check(abs(settings.material7.roughness - 0.05) < 1e-7, 'Set with a dict did not set the roughness')
values = SC.renderer.GetDictionary()
SC2 = exu.SystemContainer()
SC2.renderer.SetDictionary(values)
Check(abs(SC2.visualizationSettings.raytracer.material7.roughness - 0.05) < 1e-7, 'the roughness through SetDictionary')
Check(SC2.visualizationSettings.raytracer.material2.shininess == 7., 'the shininess through SetDictionary')

#an appended material, drawn by the raytracer with the color 1000 + its index
mbs = SC.AddSystem()
gold = materials.New()
gold.name = 'yellow'
gold.baseColor = [0.9, 0.7, 0.2]
index = materials.Append(gold)
Check(index == 10, 'the index of the appended material: ' + str(index))
mbs.CreateGround(graphicsDataList=[graphics.Sphere(radius=1, color=[-1, -1, -1, 1000 + index], nTiles=16)])
mbs.Assemble()
SC.visualizationSettings.view0.window.renderWindowSize = [80, 60]
SC.renderer.ZoomAll()
image = SC.renderer.RedrawAndGetImage(True)
center = image[30, 40].astype(float)
Check(center[0] > center[2] + 40 and center[1] > center[2] + 20, 'the color of the appended material: ' + str(center))

if not testIsActive:
    import matplotlib.pyplot as plt
    plt.imshow(image)
    plt.axis('off')
    plt.show()

u = testError + len(materials.GetDict(10)) #the number of properties of a material: 9
exu.Print('solution of rendererMaterialsTest=', u)

exu.sys['testResult'] = u
