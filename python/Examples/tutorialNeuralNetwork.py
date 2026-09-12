#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  tutorial for machine learning with Exudyn;
#           correct positioning of rigid body mounted on strings by model trained with machine learning
#           data is created with static computations, then inverse model is trained with pytorch
#
# Model:    A rigid body with height 0.4, width 0.2 and depth 0.2 and 1 kg, mounted on two soft strings with stiffness 500N/m
#
# Author:   Johannes Gerstmayr
# Date:     2024-02-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

import sys
import numpy as np
# #from math import sin, cos, sqrt,pi
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE" #for multiprocessing problems with pytorch

from numpy.random import rand
np.random.seed(0)

doTraining = True

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create an Exudyn model of a rigid body on two strings
#this is a simple model for a 2D cable-driven robot
    
SC = exu.SystemContainer()
mbs = SC.AddSystem()


np.random.seed(0)

#Geometry:
#[0,0,0] is at bottom of left tower
#[L,0,0] is at bottom of right tower
#[L,H,0] is at top of left tower

L = 4       #m; distance of columns
H = 3       #m; height of columns
w = 0.2     #m; size of rigid body
m = 1       #kg; weight of mass
g = 9.81    #m/s^2; gravity

sideLengths = [w,w*2,w]
density = m/w**3 #average density of rigid body with dimensions

#left and right tower
pTower0 = np.array([0,H,0])
pTower1 = np.array([L,H,0])
#local attachment points at mass
localPosMass0=[-0.5*w,w,0]
localPosMass1=[ 0.5*w,w,0]
#center location of rigid body
#pRigidMid = np.array([0.5*L, 0.5*H,0])
pRigidMid = np.array([0.5*L, 0.5*H,0])

# pInit = [1.2,0.78,0] #for testing
pInit = pRigidMid

k = 500         #string stiffness
d = 0.01*k      #string damping for dynamic simulation
tEnd = 1
stepSize = 0.01

gGround = [graphics.Brick(centerPoint=[0,0.5*H,0],size=[0.5*w,H,w],color=graphics.color.grey)]
gGround += [graphics.Brick(centerPoint=[L,0.5*H,0],size=[0.5*w,H,w],color=graphics.color.grey)]
gGround += [graphics.Brick(centerPoint=[0.5*L,-0.5*w,0],size=[L,w,w],color=graphics.color.grey)]
oGround = mbs.CreateGround(graphicsDataList=gGround)

gRigid = [graphics.Brick(size=sideLengths, color=graphics.color.dodgerblue)]
oRigid = mbs.CreateRigidBody(referencePosition=pInit,
                             inertia = InertiaCuboid(density, sideLengths),
                             gravity = [0,-g,0],
                             graphicsDataList=gRigid)
nRigid = mbs.GetObject(oRigid)['nodeNumber'] #used later

oString0 = mbs.CreateSpringDamper(bodyNumbers=[oGround, oRigid],
                                  localPosition0=[0,H,0],
                                  localPosition1=localPosMass0,
                                  stiffness = k,
                                  damping = d,
                                  drawSize = 0, #draw as line
                                  )
oString1 = mbs.CreateSpringDamper(bodyNumbers=[oGround, oRigid],
                                  localPosition0=[L,H,0],
                                  localPosition1=localPosMass1,
                                  stiffness = k,
                                  damping = d,
                                  drawSize = 0, #draw as line
                                  )
sRigid = mbs.AddSensor(SensorBody(bodyNumber=oRigid, storeInternal=True,
                                outputVariableType=exu.OutputVariableType.Position))

# compute string lengths for given rigid body center position in straight configuration
# used for initialization of static computation 
def ComputeStringLengths(pRigid):
    L0 = np.array(pRigid)+localPosMass0-pTower0
    L1 = np.array(pRigid)+localPosMass1-pTower1
    
    return [NormL2(L0), NormL2(L1)]


mbs.Assemble()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solutionSettings.writeSolutionToFile = not doTraining

# # this leads to flipped results => good example !
# simulationSettings.staticSolver.numberOfLoadSteps = 10
# simulationSettings.staticSolver.stabilizerODE2term = 2        #add virtual stiffness due to mass; helps static solver to converge for such cases

simulationSettings.staticSolver.numberOfLoadSteps = 2
simulationSettings.staticSolver.stabilizerODE2term = 20       #add virtual stiffness due to mass; helps static solver to converge for such cases
simulationSettings.staticSolver.computeLoadsJacobian = False #due to bug in loadsJacobian
simulationSettings.staticSolver.verboseMode = 0


if False: #set to true to perform static/dynamic analysis and visualize results
    tEnd = 20 #for visualization of dynamic case
    simulationSettings.solutionSettings.sensorsWritePeriod = stepSize
    # simulationSettings.timeIntegration.simulateInRealtime = True
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
    
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()
    
    mbs.SolveStatic(simulationSettings)
    SC.renderer.DoIdleTasks()
    mbs.SolveDynamic(simulationSettings)
    
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop()

    mbs.PlotSensor(sRigid, components=[0,1,2]) #plot vertical displacement

    #this shows the deviation due to string stiffness and rotation of rigid body
    print('final pos=',mbs.GetSensorValues(sRigid))

    sys.exit()

#this function is called to compute real position from ideal position p
def ComputePositionFromStringLengths(p):
    [L0,L1] = ComputeStringLengths(p)
    refCoordsRigid[0:3] = p #override position
    mbs.SetNodeParameter(nodeNumber=nRigid, 
                         parameterName='referenceCoordinates',
                         value=refCoordsRigid)
    mbs.SetObjectParameter(objectNumber=oString0, 
                           parameterName='referenceLength',
                           value=L0)
    mbs.SetObjectParameter(objectNumber=oString1, 
                           parameterName='referenceLength',
                           value=L1)
    mbs.Assemble()
    
    try:
        mbs.SolveStatic(simulationSettings)
        positionList.append(p) #this is the ideal position; used to calculate deviation
        #we map targeted (real) positions to original lengths
        realPos = mbs.GetSensorValues(sRigid)
        #compute original lengths to realPos
        [L0orig,L1orig] = ComputeStringLengths(realPos)

        #targetList.append([L0,L1]) #we only need to store the deviation
        diff = [L0-L0orig,L1-L1orig]
        return [realPos, diff, [L0, L1]]
    except:
        print('solver failed for:',p,',Ls=',[L0,L1])
        return [None]

#%%+++++++++++++++++++++++++++++++++++++++++++++
#now create data: mapping of 2D position of object to difference of 
#  ideal lengths to real lengths (which gives the necessary correction 
#  for positioning)

gridX = 20*2 # gridX * gridY is the number of samples for training
gridY = 20*2
nExamples = gridX*gridY
nExamplesTest = int(nExamples*0.1) # additional samples for testing
pRangeX = 2.4
pRangeY = 2.4
positionList = []
inputList = []
targetList = []
#store reference coordinates for rotations
refCoordsRigid = mbs.GetNodeParameter(nodeNumber=nRigid, parameterName='referenceCoordinates')

gridValues=np.zeros((gridX,gridY,4))
i=0
ix = 0
iy = 0
while i < nExamples+nExamplesTest:
    if i < nExamples:
        ix = i%gridX
        iy = int(i/gridX)
        x0 = pRangeX*(ix/gridX-0.5)
        y0 = pRangeY*(iy/gridY-0.5)
    else:
        x0 = pRangeX*(rand()-0.5)
        y0 = pRangeY*(rand()-0.5)

    p = pRigidMid + [x0, y0, 0]

    rv = ComputePositionFromStringLengths(p)        
    
    if rv[0] is not None:
        realPos = rv[0]
        diff = rv[1]
        [L0,L1] = rv[2]
        targetList.append(diff)
        inputList.append(list(realPos)[0:2]) #correction on position
        
        if i < nExamples:
            gridValues[ix,iy,0:2] = diff
            gridValues[ix,iy,2:4] = realPos[0:2]
        
        if max(diff)>0.5:
            print('++++++++++++++++++++++++++')
            print('ideal pos=',p)
            print('realPos  =',realPos)
            print('lengths  =',L0,L1)
        
        i += 1    
    
inputsExudynList = inputList[0:nExamples]
targetsExudynList = targetList[0:nExamples]
inputsExudynTestList = inputList[nExamples:]
targetsExudynTestList = targetList[nExamples:]

print('created',nExamples+nExamplesTest,'samples')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#function to 3D-plot errors and corrections
def PlotGridFunction(data, name='error', closeAll=False):
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    if closeAll: plt.close('all')
    
    # Generate grid coordinates (if not already generated)
    # For example, if gridX and gridY are the dimensions of your grid
    gridX, gridY = data.shape
    x = np.linspace( -0.5*pRangeX, 0.5*pRangeX, gridX)
    y = np.linspace(-0.5*pRangeY, 0.5*pRangeY, gridY)
    X, Y = np.meshgrid(x, y)
    
    # Create the contour plot
    fig=plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # contour = plt.contourf(X, Y, dataX, cmap='hot', levels=100)
    contour = ax.plot_surface(X, Y, data, cmap='viridis')

    plt.colorbar(contour)
    plt.title(name)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    # Display the plot
    plt.show()

PlotGridFunction(gridValues[:, :, 0], name='positioning error X', closeAll=True)
PlotGridFunction(gridValues[:, :, 1], name='positioning error Y')


if not doTraining:
    sys.exit()





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# MACHINE LEARNING: Model and training
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#now train
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

hiddenLayerSize = 8*4       #example size, adjust as needed
batchSize = 16*8
learningRate = 0.002
nTrainingEpochs = 10000     #max epochs
lossThreshold = 0.0002      #desired max. loss

# torch.set_num_threads(1)
#prepare data:

# Convert lists to PyTorch tensors
dtype=torch.float32
inputsExudyn = torch.tensor(inputsExudynList, dtype=dtype)
targetsExudyn = torch.tensor(targetsExudynList, dtype=dtype)
inputsExudynTest = torch.tensor(inputsExudynTestList, dtype=dtype)
targetsExudynTest = torch.tensor(targetsExudynTestList, dtype=dtype)

# Create TensorDatasets
train_dataset = TensorDataset(inputsExudyn, targetsExudyn)
test_dataset = TensorDataset(inputsExudynTest, targetsExudynTest)

# Create DataLoaders
trainLoader = DataLoader(train_dataset, batch_size=batchSize, shuffle=True)
testLoader = DataLoader(test_dataset, batch_size=batchSize, shuffle=False)


class ModelNN(nn.Module):
    def __init__(self, input_size, hiddenLayerSize, output_size):
        super(ModelNN, self).__init__()
        self.fc1 = nn.Linear(input_size, hiddenLayerSize,dtype=dtype)
        self.relu = nn.ReLU()
        self.leakyRelu= nn.LeakyReLU()
        self.elu = nn.ELU()
        # self.relu = nn.Sigmoid() #alternatives
        # self.relu = nn.Tanh()
        self.fc2 = nn.Linear(hiddenLayerSize, hiddenLayerSize,dtype=dtype)
        self.fc3 = nn.Linear(hiddenLayerSize, hiddenLayerSize,dtype=dtype)
        self.lastLayer = nn.Linear(hiddenLayerSize, output_size,dtype=dtype)

    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        x = self.leakyRelu(x)
        x = self.fc3(x)
        x = self.elu(x)
        x = self.lastLayer(x)
        return x



input_size = inputsExudyn.shape[1]  # Number of input features
output_size = targetsExudyn.shape[1]  # Assuming regression task, adjust for classification

model = ModelNN(input_size, hiddenLayerSize, output_size)
lossFunction = nn.MSELoss()  # Mean Squared Error Loss for regression, adjust for classification
optimizer = optim.Adam(model.parameters(), lr=learningRate,)

#++++++++++++++++++++++++++++++++++++++++++++++++++++
#perform training and store loss over epochs
#stop when lossThreshold reached

lossHistory = []
minLoss = 1e10
# Train the network
for epoch in range(nTrainingEpochs):  # 100 epochs
    for inputs, targets in trainLoader:
        optimizer.zero_grad()

        # Forward pass
        outputs = model(inputs)
        loss = lossFunction(outputs, targets)

        # Backward pass and optimization
        loss.backward()
        optimizer.step()
        
    lossHistory.append([epoch, np.sqrt(loss.item())])
    minLoss = min(minLoss, np.sqrt(loss.item()))
    lossVal = np.sqrt(loss.item())
    
    if lossVal < lossThreshold:
        print(f'loss threshold reached at: epoch {epoch+1}/{nTrainingEpochs}, Loss: {lossVal}')
        break

    if (epoch+1) % 50 == 0:
        print(f'Epoch {epoch+1}/{nTrainingEpochs}, Loss: {lossVal}')

print('min loss=',minLoss)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
#evaluate model using test data:
model.eval()  # Set the model to evaluation mode
totalLoss = 0
count = 0

with torch.no_grad():  # No need to track gradients for evaluation
    for inputs, targets in testLoader:
        outputs = model(inputs)
        loss = torch.sqrt(((outputs - targets) ** 2).mean())  # Calculating RMSE for each batch
        totalLoss += loss.item()
        count += 1

averageRMSE = totalLoss / count

# Call the evaluate_model function with the test_loader and your model
print(f"\nTest RMSE: {averageRMSE}\n")
    
for test in range(10):
    x = inputsExudynTest[test:test+1]
    print('x=',x,', shape',x.shape)
    y = model(x).tolist()[0] #convert output to list
    
    yRef = targetsExudynTest[test:test+1]

    print('++++++++++++++++++++')
    print('input:  ',x,'\ntarget: ',yRef,'\npredict:',y)
    # mbs.PlotSensor(result, components=[0], closeAll=test==0, newFigure=False,
    #                labels=['RNN'], yLabel='displacement (m)',
    #                colorCodeOffset=test)
    # mbs.PlotSensor(result, components=[1], newFigure=False,
    #                labels=['reference'], yLabel='displacement (m)',
    #                colorCodeOffset=test,lineStyles=[':'])


#compute 2D grid with error
testErrorGrid = np.zeros((gridX, gridY))
maxError = 0
for iy in range(gridY):
    for ix in range(gridX):
        x0 = pRangeX*((ix+0.5)/gridX-0.5)*0.5
        y0 = pRangeY*((iy+0.5)/gridY-0.5)*0.5

        p = pRigidMid + [x0, y0, 0]

        x = torch.tensor([list(p[0:2])], dtype=dtype)
        y = model(x).tolist()[0]

        rv = ComputePositionFromStringLengths(p)        
        
        diff = rv[1]
        err = np.array(diff) - y
        maxError = max(maxError, abs(err[0]))
        
        testErrorGrid[ix,iy] = err[0]

print('maxError', maxError)
PlotGridFunction(testErrorGrid, name='test error X', closeAll=False)

#%%+++++++++++++++    
if True: #plot loss history
    lossData = np.array(lossHistory)
    import matplotlib.pyplot as plt
    from exudyn.plot import PlotSensor
    PlotSensor(None,lossData,components=[0], closeAll=False, logScaleY=True,
               labels=['loss'])



