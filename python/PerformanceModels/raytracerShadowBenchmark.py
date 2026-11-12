#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Benchmark of the raytracer and its shadows, run by hand (not part of the performance tests, whose
#           results must agree on every platform): the scene of TestModels/raytracerNOGLFWtest.py at 1200 x 1000,
#           the ten bodies on their circle, an oblique light with a radius, a chrome floor with reflections.
#           Each configuration (no shadow, hard shadow, soft shadows per pixel and at reduced resolution, a light
#           without radius) is rendered several times; the best time and an MD5 of the image (the upper 90 %, without the version text) are printed, so
#           that a change of the raytracer is checked for speed and for an unchanged image (#3468).
#           Arguments: --out FILE writes the results as JSON, --compare FILE compares with such a file,
#           --reference adds the mean difference to a reference with 240 shadow samples per pixel (slow),
#           --repeat N the number of images per configuration (default 3), --save DIRECTORY the images as .npy.
#           The design and the measurements: docs/dev/design/raytracerImprovements.md.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys
import time
import json
import hashlib
import numpy as np

#the scene is the one of the test model, read from its file, so that it exists only once
modelFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'TestModels', 'raytracerNOGLFWtest.py')
source = open(modelFile, encoding='utf-8').read()
source = source.split('mbs.SetPreStepUserFunction(PreStepUserFunction)')[0]
exec(source)    #defines SC, mbs, A, PreStepUserFunction, ...

PreStepUserFunction(mbs, 1.)   #the bodies on their circle, as at t = 1

settings = SC.visualizationSettings
settings.view0.window.renderWindowSize = [1200, 1000]
settings.general.useMultiThreadedRendering = False
settings.openGL.multiSampling = 1
settings.openGL.light0.position = [8, -6, 10, 1]
settings.openGL.light0.shadow = 0.2
settings.raytracer.numberOfThreads = 0
settings.raytracer.maxReflectionDepth = 2
settings.raytracer.maxTransparencyDepth = 2
settings.raytracer.verbose = 0
settings.raytracer.imageSizeFactor = 1
settings.view0.camera.perspective = 1
settings.general.useGradientBackground = True
SC.renderer.SetModelView(5.7, RotationMatrix2RotationVector(A), [0, 0, 0])

#name: (shadow, lightRadius, lightRadiusVariations, shadowScalingFactor, shadowSmoothingSteps)
configurations = {
    'no shadow':              (0.,  0.5, 1,  1, 0),
    'hard shadow':            (0.2, 0.5, 1,  1, 0),
    'N=13, S=3 (test model)': (0.2, 0.5, 13, 3, 2),
    'N=50, S=3':              (0.2, 0.5, 50, 3, 2),
    'N=13, per pixel':        (0.2, 0.5, 13, 0, 0),
    'N=13, S=3, radius 0':    (0.2, 0.,  13, 3, 2),
    }

def Configure(shadow, lightRadius, variations, scaling, smoothing):
    settings.openGL.light0.shadow = shadow
    settings.openGL.light0.lightRadius = lightRadius
    settings.raytracer.lightRadiusVariations = variations
    settings.raytracer.advanced.shadowScalingFactor = scaling
    settings.raytracer.advanced.shadowSmoothingSteps = smoothing

def Render(repeat):
    """the best time of repeat images and the last image"""
    best = 1e9
    for i in range(repeat):
        start = time.perf_counter()
        image = SC.renderer.RedrawAndGetImage(True)
        best = min(best, time.perf_counter() - start)
    return best, image

def MeanDifference(image, reference):
    """the mean absolute difference in 0..255 over the RGB channels of the upper 90 % (without the version text)"""
    rows = int(0.9 * image.shape[0])
    return float(np.mean(np.abs(image[:rows, :, :3].astype(float) - reference[:rows, :, :3].astype(float))))

if __name__ == '__main__':
    arguments = sys.argv[1:]
    def Argument(name, default=None):
        return arguments[arguments.index(name) + 1] if name in arguments else default
    repeat = int(Argument('--repeat', 3))

    reference = None
    if '--reference' in arguments:
        Configure(0.2, 0.5, 240, 0, 0)
        settings.raytracer.advanced.adaptiveShadowSamples = 0 #every sample traced
        timeReference, reference = Render(1)
        settings.raytracer.advanced.adaptiveShadowSamples = 6
        print('reference with 240 samples per pixel: %.3f s' % timeReference)

    results = {}
    print('%-26s %9s  %-32s %s' % ('configuration', 'time[s]', 'MD5 of the image', 'difference' if reference is not None else ''))
    for name, configuration in configurations.items():
        Configure(*configuration)
        timeBest, image = Render(repeat)
        rows = int(0.9 * image.shape[0]) #without the version text at the bottom, which changes with every version
        md5 = hashlib.md5(np.ascontiguousarray(image[:rows]).tobytes()).hexdigest()
        results[name] = {'time': timeBest, 'md5': md5}
        if Argument('--save') is not None:
            np.save(os.path.join(Argument('--save'), name.replace(',', '').replace(' ', '_').replace('=', '') + '.npy'), image)
        line = '%-26s %9.4f  %-32s' % (name, timeBest, md5)
        if reference is not None:
            results[name]['difference'] = MeanDifference(image, reference)
            line += ' %.3f' % results[name]['difference']
        print(line, flush=True)

    if Argument('--out') is not None:
        with open(Argument('--out'), 'w') as file:
            json.dump(results, file, indent=1)
    if Argument('--compare') is not None:
        with open(Argument('--compare')) as file:
            before = json.load(file)
        print('\ncompared with ' + Argument('--compare') + ':')
        for name, result in results.items():
            if name in before:
                change = 100 * (result['time'] - before[name]['time']) / before[name]['time']
                print('%-26s %9.4f -> %9.4f  %+6.1f %%  %s' % (name, before[name]['time'], result['time'], change,
                      'same image' if result['md5'] == before[name]['md5'] else 'IMAGE CHANGED'))
