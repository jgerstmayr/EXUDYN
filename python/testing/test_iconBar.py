#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# test_iconBar - the icons of the icon bar of the render window (#3026)
#
# The icons are drawn by C++ code into 64 x 64 RGBA images when the renderer starts; these tests read the images
# through the internal exudyn._compiledModule._IconBarImages, without a window: every icon is there, in the order
# of the bar, and draws something that stays inside its square with a transparent border.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np

import exudyn as exu

order = ['quit', 'zoomAll', 'zoomRegion', 'viewXY', 'viewYZ', 'viewXZ', 'viewIsometric', 'ruler',
         'mouseCoordinates', 'lines', 'edges', 'faces', 'transparency', 'shadow', 'updateSpeed',
         'nodes', 'bodies', 'connectors', 'markers', 'loads', 'sensors', 'scope', 'settings',
         'config0', 'config1', 'config2', 'config3']


def testTheIconsAreThoseOfTheBarInItsOrder():
    names = [name for (name, image) in exu._compiledModule._IconBarImages()]
    assert names == order


def testEveryIconDrawsInsideItsSquare():
    for (name, image) in exu._compiledModule._IconBarImages():
        assert image.shape == (64, 64, 4) and image.dtype == np.uint8, name
        alpha = image[:, :, 3].astype(float)/255
        covered = np.mean(alpha > 0.5)
        maxCovered = 0.97 if name.startswith('config') else 0.7    #the configurations are buttons (#3044)
        assert 0.03 < covered < maxCovered, (name, covered)  #something, but not a filled square
        border = np.concatenate([alpha[0, :], alpha[-1, :], alpha[:, 0], alpha[:, -1]])
        assert border.max() < 0.6, (name, border.max())     #nothing cut at the border of the square


def testTheIconsDiffer():
    images = [image for (name, image) in exu._compiledModule._IconBarImages()]
    #the three views have the same picture, and the four stored configurations (#3043); their axes and
    #numbers are written over them by the bar
    distinct = {image.tobytes() for image in images}
    assert len(distinct) == len(images) - 2 - 3
