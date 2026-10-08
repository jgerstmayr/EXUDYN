#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  ZoomAll with a tracked marker (view0.camera.trackMarker): the view translation is the
#           center point of the render state plus what the tracked marker adds, so ZoomAll has to
#           subtract that - otherwise the scene is shifted by the marker position (#2309). Also the
#           orientation of a tracked marker: the bounding box is taken in the rotation the view draws.
#           Headless: the inactive renderer computes the render state (SC.renderer.ZoomAll()).
#
# Usage:    pytest python/testing/test_zoomAllTrackMarker.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-30
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
import exudyn.graphics as graphics
from exudyn.utilities import (ObjectGround, VObjectGround, MarkerBodyRigid, MarkerBodyPosition,
                              RotationMatrixZ)

exu.special.userInterface.SuppressAll(True)   #no window, whatever a test asks for

markerPosition = np.array([3., 2., 0.])
markerRotation = RotationMatrixZ(np.pi/2)


def Model(track, rigid=False):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    #the scene: a brick from (10,0) to (12,1), and the ground carrying the tracked marker
    mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[
        graphics.Brick(centerPoint=[11, 0.5, 0.25], size=[2, 1, 0.5], color=graphics.color.red)])))
    oCarrier = mbs.AddObject(ObjectGround(referencePosition=list(markerPosition),
                                          referenceRotation=markerRotation if rigid else np.eye(3)))
    if rigid:
        m = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCarrier))
    else:
        m = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oCarrier))
    mbs.Assemble()
    SC.visualizationSettings.view0.scene.drawWorldBasis = False
    SC.visualizationSettings.markers.show = False
    SC.visualizationSettings.view0.scene.drawCoordinateSystem = False
    if track:
        SC.visualizationSettings.view0.camera.trackMarker = int(m)
        if rigid:
            SC.visualizationSettings.view0.camera.trackMarkerOrientation = [1, 1, 1]
    return SC, mbs


def ViewCenterAfterZoomAll(SC):
    """zoom and centerPoint of the render state after ZoomAll; the view translation is centerPoint plus what the
    tracked marker adds"""
    SC.renderer.ZoomAll()
    state = SC.renderer.GetState()
    return state['zoom'], np.array(state['centerPoint'])


def test_zoomAllCentersTheSceneWithATrackedMarker():
    SC0, _ = Model(track=False)
    zoom0, center0 = ViewCenterAfterZoomAll(SC0)
    SC1, _ = Model(track=True)
    zoom1, center1 = ViewCenterAfterZoomAll(SC1)
    #the view translation with tracking is center1 + markerPosition (identity model rotation); it must equal
    #the center of the untracked zoom all, in x and y
    assert np.allclose(center1[:2] + markerPosition[:2], center0[:2], atol=1e-5), (center0, center1)
    assert zoom1 == pytest.approx(zoom0)


def test_zoomAllUsesTheOrientationOfATrackedMarker():
    SC0, _ = Model(track=False, rigid=True)
    zoom0, center0 = ViewCenterAfterZoomAll(SC0)
    SC1, _ = Model(track=True, rigid=True)
    zoom1, center1 = ViewCenterAfterZoomAll(SC1)
    #the view rotates the scene by the marker orientation (90 degrees about z): the brick, 2 wide and 1 high,
    #becomes 1 wide and 2 high, and the view is centered on it in that rotation
    #(RenderState multiplies a row vector with the rotation matrix, hence p @ R)
    points = np.array([[10, 0, 0], [12, 0, 0], [12, 1, 0], [10, 1, 0]], float)
    rotated = points @ np.array(markerRotation)
    boxCenter = 0.5*(rotated.min(axis=0) + rotated.max(axis=0))
    tracked = markerPosition @ np.array(markerRotation)
    assert np.allclose(center1[:2] + tracked[:2], boxCenter[:2], atol=1e-5), (center1, boxCenter, tracked)
    assert zoom1 > zoom0   #the brick stands upright in the view: a larger zoom for the same window
