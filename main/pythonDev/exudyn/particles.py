#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  This module offers methods for GeneralContact, in particular particles (DEM - discrete element method)
#
# Author:   Johannes Gerstmayr
# Date:     2024-10-19 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# import exudyn
# import exudyn.basicUtilities as ebu
import numpy as np #LoadSolutionFile
# import copy as copy #to be able to copy e.g. lists

#**function: create set of spherical particles densly packed inside box using hexagonal closest packing (HCP); radius is randomized between minRadius and maxRadius
#**input:
# minPointBox: [xMin,yMin,zMin] minimal cartesian coordinates for box
# maxPointBox: [xMax,yMax,zMax] maximal cartesian coordinates for box
# minRadius: minimal or nominal radius
# maxRadius: maximal radius for randomized variations of radius or None to use minRadius
# maxNumberOfParticles: if not None, this limits the amount of created particles; otherwise number of particles depends on geometry
# offsetRadius: additional space between spheres (by assuming a larger radius for packing)
# verbose: if > 0 some main parameters are printed
#**output: [(point0, radius0), ...] a list of point-radius tuples containing the information of created particles
def CreateParticlesInBox(minPointBox, maxPointBox, minRadius, maxRadius=None, 
                         maxNumberOfParticles=None, offsetRadius=0, verbose=0):
    if maxRadius == None:
        maxRadius = minRadius
    
    calcRadius = maxRadius + offsetRadius
    
    listPR = [] #list of point-radius tuples
    
    minP = np.array(minPointBox, dtype=float)
    maxP = np.array(maxPointBox, dtype=float)
    
    dvec = maxP - minP  # box dimensions (x, y, z)
    
    # Spacing between particles not including shifting
    spacing = np.array([2 * calcRadius, np.sqrt(3) * calcRadius, np.sqrt(6) * calcRadius * 2 / 3])
    maxShifting = np.array([calcRadius, np.sqrt(3)/3*calcRadius, 0])
    sizeSphere = np.array([2*calcRadius, 2*calcRadius, 2*calcRadius])
    
    # Calculate number of particles that can fit in each direction
    nvec = np.floor((dvec - sizeSphere - maxShifting) / spacing).astype(int) + 1
    
    if min(nvec) < 1:
        raise ValueError('CreateParticlesInBox: no particles fit into box, at least in one dimension')
    
    # Total number of available positions; not needed
    totalParticles = nvec[0] * nvec[1] * nvec[2]
    
    spaceNeeded = (nvec-1)*spacing + maxShifting + sizeSphere
    
    
    # Offset to center the grid in the box
    ovec = (dvec - spaceNeeded) / 2. + minP + 0.5*sizeSphere
    
    if verbose:
        print('box size         =',dvec)
        print('particles (x,y,z)=',nvec)
        print('offsets          =',ovec)
        print('spacing (x,y,z)  =',spacing)
        print('shifting (x,y,z) =',maxShifting)
        print('space needed     =',spaceNeeded)
        print('total particles  =',totalParticles)
    
    # Generate particles with HCP (hexagonal close packing) arrangement
    count = 0
    for k in range(nvec[2]):  # z-layer
        for j in range(nvec[1]):  # y-row
            for i in range(nvec[0]):  # x-column
                if maxNumberOfParticles is not None and count >= maxNumberOfParticles:
                    break
                
                #random radius for the particle
                radius = np.random.uniform(minRadius, maxRadius)
                
                #base position without shifts
                position = np.array([i, j, k], dtype=float) * spacing

                #shifting depending on layer
                if k%2 == 0:
                    position[0] += maxShifting[0]*(j%2)
                else:
                    position[0] += maxShifting[0]*((j+1)%2)
                position[1] += maxShifting[1]*(k%2)
                
                # Add the offset to center the grid inside the box
                position += ovec
                
                #ensure the particle fits within the box given its radius
                if not (minPointBox[0] + radius <= position[0] <= maxPointBox[0] - radius and
                    minPointBox[1] + radius <= position[1] <= maxPointBox[1] - radius and
                    minPointBox[2] + radius <= position[2] <= maxPointBox[2] - radius):
                    
                    print('WARNING: CreateParticlesInBox: particle not inside box: p=',position,', r=', radius)

                # Append the particle's position and radius as a tuple
                listPR.append((np.array(position), radius))
                count += 1
    
    return listPR


if __name__ == '__main__':

    # print(particles)    
    import exudyn as exu
    from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
    import exudyn.graphics as graphics #only import if it does not conflict
    
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()


    #test particles in box
    particles = CreateParticlesInBox(minPointBox = [0, 0, 0], 
                                     maxPointBox = [10, 10, 10], 
                                     minRadius = 0.4, maxRadius = 0.7,
                                     maxNumberOfParticles = 800,
                                     verbose=1)
    print('n particles=', len(particles))

    for (p, r) in particles:
        #print(p)
        colorInd = int(p[2]/2)%16
        #print(colorInd)
        mbs.CreateGround(referencePosition=p,
                         graphicsDataList=[graphics.Sphere(radius=r, 
                                                           color=graphics.colorList[colorInd],
                                                           nTiles=16)])
    mbs.CreateGround(graphicsDataList=[graphics.Brick(centerPoint=[5,5,5], size=[10,10,10],
                                                      addEdges=True, color=[0.7,0.7,0.7,0.5])]) 

    mbs.Assemble()

    exu.StartRenderer()
    mbs.WaitForUserToContinue()
    exu.StopRenderer()
    
