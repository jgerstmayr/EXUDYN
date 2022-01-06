#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Data and information that support the findings of the article:
# M. Pieber, K. Ntarladima, R. Winkler and J. Gerstmayr, A Hybrid ALE 
# Formulation for the Investigation of the Stability of Pipes Conveying Fluid 
# and Axially Moving Beams. Journal of Computational and Nonlinear Dynamics
#
# Details:  Postprocessing for discrete masses parameter variation
#
# Author:   Johannes Gerstmayr
# Date:     2022-01-04
# Testet with exudyn version:  1.1.71
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn.basicUtilities import *
import exudyn as exu
from exudyn.utilities import *

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

plt.close('all')
#plt.rcParams['text.usetex'] = True #slows down figures
saveSolutions = False

fontSize=14

dmFList=['0.5','0.75','0.9']

massList=[0,1,2,3,4,8,12,16,20,24,30] #this is the conversion of nMasses parameter
plotCodes = ['k-','b-','g-','r-','c-','m-','y-']
#%%++++++++++++++++++++++++++++++++++++
#evaluate parameter variation    

for dMF in dmFList:

    for i in range(2):
        fig = plt.figure()
        ax=fig.gca() # get current axes
        plt.rc('font', size=fontSize) 
        plt.rc('axes', titlesize=fontSize)     # fontsize of the axes title
        plt.rc('axes', labelsize=fontSize)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=fontSize)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=fontSize)    # fontsize of the tick labels
        plt.rc('legend', fontsize=fontSize)    # legend fontsize    
        
        
        
        #relevant tests for ASME paper:
        data = np.loadtxt('solution/parvarForce4Eval12discreteMass'+dMF+'.txt', comments='#', delimiter=',')
        
        #default values are:
        #nElements=16
        #damping=0.1
        #h=4e-4
        #n (variations) = 320+1
            
        cnt = 0
        #rangeVal = [1,2,3,4,5,6,0]
        if i==0:
            rangeVal = [1,2,3,0]
            iCalc='123'
        else:
            rangeVal = [4,5,7,0]
            iCalc='81216'
        for n in rangeVal:
            #create new lists for a certain mass number:
            vALE = [] 
            values = []
            nMasses = massList[n]
            for row in data:
                if row[3] == n:
                # if row[3+1] == n and abs(row[3]-0.5) < 1e-6:
                    values+=[row[1]]
                    vALE+=[row[2]]
            strLabel = 'pure beam'
            if nMasses==1:
                strLabel = str(nMasses)+' mass'
            elif nMasses > 1:
                strLabel = str(nMasses)+' masses'
            
            plotCode=plotCodes[cnt]
            if nMasses!=0:
                plotCode=plotCode.replace('-','.')
            plt.plot(vALE, values, plotCode, label=strLabel)
            cnt+=1
    
        ax.set_ylabel(r'max amplitude in m')
        ax.set_xlabel(r'$v_E$ in m/s')
    
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    
    
        plt.axis([0, 8.1, -0.001, 0.03])
            
        plt.tight_layout()
        plt.legend()
        plt.legend(loc='upper left')   
        plt.show() 
    

        if saveSolutions:
            fig.savefig('plots/parvarForce4Eval12discreteMass'+dMF+'and'+iCalc+'.pdf',format='pdf')
        
    
    
    
    
dmFList=['09']

massList=[8,9,10,11,16,24,0] #this is the conversion of nElements parameter
massList=[10,16,20,32,40,0] #this is the conversion of nElements parameter
plotCodes = ['k-','b-','g-','r-','c-','m-','y-']
#%%++++++++++++++++++++++++++++++++++++
#evaluate parameter variation    

for dMF in dmFList:

    for i in range(2):
        fig = plt.figure()
        ax=fig.gca() # get current axes
        plt.rc('font', size=fontSize) 
        plt.rc('axes', titlesize=fontSize)     # fontsize of the axes title
        plt.rc('axes', labelsize=fontSize)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=fontSize)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=fontSize)    # fontsize of the tick labels
        plt.rc('legend', fontsize=fontSize)    # legend fontsize    
        
        
        
        #relevant tests for ASME paper:
        data = np.loadtxt('solution/parameterVariationMdmf05masses3vALE36elem1016203240.txt', comments='#', delimiter=',')
        data2 = np.loadtxt('solution/parvarForce4Eval12discreteMass0.9.txt', comments='#', delimiter=',')

            
        cnt = 0
        if i==0:
            rangeVal = [0,1,2,3,4]
            iCalc='123'
        else:
            rangeVal = [0,1,2,3,4]
            iCalc='81216'
        for n in rangeVal:
            #create new lists for a certain element number:
            vALE = [] 
            values = []
            nMasses = massList[n]
            for row in data:
                if row[3] == massList[n]:
                # if row[3+1] == n and abs(row[3]-0.5) < 1e-6:
                    values+=[row[1]]
                    vALE+=[row[2]]
                    
                    
            if nMasses==0:
                for row in data2:
                    if row[3] == massList[n]:
                    # if row[3+1] == n and abs(row[3]-0.5) < 1e-6:
                        values+=[row[1]]
                        vALE+=[row[2]]                
                strLabel = 'pure beam'
                plt.plot(vALE, values, 'r-', label=strLabel)
                
            if nMasses==1:
                strLabel = str(nMasses)+' elements'
            elif nMasses > 1:
                strLabel = str(nMasses)+' elements'
            
            plotCode=plotCodes[cnt]
            if nMasses!=0:
                plotCode=plotCode.replace('-','.')
            if nMasses>0:
                plt.plot(vALE, values, plotCode, label=strLabel)
                cnt+=1
        
        #ax.plot(pDict['vALE0'], values, 'b-', label='vALE') 
        ax.set_ylabel(r'max amplitude in m')
        ax.set_xlabel(r'$v_E$ in m/s')
    
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        
        plt.axis([3, 6.1, -0.001, 0.012])
            
        plt.tight_layout()
        plt.legend()
        plt.legend(loc='upper left')   
        plt.show() 
    
        if saveSolutions:
            fig.savefig('plots/parvarForce4Eval12discreteElements'+dMF+'and'+iCalc+'.pdf',format='pdf')  

