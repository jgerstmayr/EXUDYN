#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The defaults of the ten raytracer materials and of the four lights. Until
#           revision2026b step RG6.2.20 (#2626) they were set by C++ AFTER construction -
#           VisualizationSystemContainer() dimmed light1 to light3 and
#           MainGraphicsMaterialList::Reset() filled the materials - so they were invisible to the
#           generated reference, which printed the defaults of VSettingsLight and
#           VSettingsMaterial instead, and to the settings dialog, which reported 59 settings as
#           changed that nobody had touched.
#
#           They are defaults of the structure now, written as memberDefaults in
#           definitions/structureDefsVisualizationSettings.py. These tests pin the values, so that
#           an edit there is a decision and not an accident, and they check the two paths that
#           have to agree: a fresh VisualizationSettings, and SC.renderer.materials after a Reset.
#
# Usage:    pytest python/testing/test_settingsDefaults.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import pytest

import exudyn

materialNames = ['default', 'matt', 'steel', 'plastic', 'chrome', 'shiny', 'transparent',
                 'glass', 'mirror', 'emission']

#one value per material that says it is the right one, and not merely a material with that name
materialMarks = {0: ('baseColor', [0.4, 0.4, 0.9]), 1: ('shininess', 5.),
                 4: ('reflectivity', 0.25), 6: ('alpha', 0.3), 7: ('ior', 1.5),
                 8: ('reflectivity', 0.8), 9: ('emission', [0.8, 0.8, 0.7])}

#light0 keeps what VSettingsLight defines; the other three do not
lightMarks = {'light1': {'diffuse': 0.25, 'specular': 0.25, 'enable': True},
              'light2': {'diffuse': 0.2, 'specular': 0.2, 'enable': False},
              'light3': {'diffuse': 0.2, 'specular': 0.2, 'enable': False}}


def Material(settings, index):
    return getattr(settings.raytracer, 'material' + str(index))


def Close(value, expected):
    if isinstance(expected, list):
        return all(abs(a - b) < 1e-7 for (a, b) in zip(list(value), expected))
    return abs(float(value) - float(expected)) < 1e-7


@pytest.mark.parametrize('index', range(10))
def testTheMaterialDefaultsAreInTheStructure(index):
    """a plain VisualizationSettings carries the ten materials - no SystemContainer needed"""
    settings = exudyn.VisualizationSettings()
    material = Material(settings, index)
    assert material.name == materialNames[index]
    if index in materialMarks:
        (member, expected) = materialMarks[index]
        assert Close(getattr(material, member), expected), (
            'material' + str(index) + '.' + member)


def testTheLightDefaultsAreInTheStructure():
    """light1 is dimmed and moved, light2 and light3 are dimmed and off"""
    settings = exudyn.VisualizationSettings()
    for (name, expected) in lightMarks.items():
        light = getattr(settings.openGL, name)
        for (member, value) in expected.items():
            if isinstance(value, bool):
                assert getattr(light, member) == value, name + '.' + member
            else:
                assert Close(getattr(light, member), value), name + '.' + member
    assert Close(list(settings.openGL.light1.position), [2., 2., -10., 0.])
    #light0 is the one that keeps the defaults of the type
    assert Close(settings.openGL.light0.diffuse, 0.5)
    assert settings.openGL.light0.enable


def testAContainerStartsFromTheSameMaterials():
    """SC.visualizationSettings and the material list of the renderer must agree, which is what
    MainGraphicsMaterialList::Reset() now guarantees by copying from a fresh VSettingsRaytracer"""
    SC = exudyn.SystemContainer()
    for index in range(10):
        assert Material(SC.visualizationSettings, index).name == materialNames[index]


def testResettingTheMaterialsRestoresThem():
    """the Reset that is exposed to Python must still produce the ten default materials"""
    SC = exudyn.SystemContainer()
    SC.renderer.materials.Reset()
    for index in range(10):
        assert Material(SC.visualizationSettings, index).name == materialNames[index]
