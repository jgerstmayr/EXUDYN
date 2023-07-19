#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The signal library supports processing of signals for import (e.g. measurement data)
#           and for filtering result data.
#
# Authors:  Johannes Gerstmayr, Stefan Holzinger 
# Date:     2020-12-10
# Notes:    This module is still under construction and should be used with care!
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from exudyn.advancedUtilities import IsListOrArray
import copy

#**function: filter output of sensors (using numpy savgol filter) as well as numerical differentiation to compute derivative of signal
#**input:
#   signal: numpy array (2D array with column-wise storage of signals, as exported by EXUDYN position, displacement, etc. sensors); first column = time, other columns = signals to operate on; note that it is assumed, that time devided in almost constant steps!
#   derivative: 0=no derivative, 1=first derivative, 2=second derivative, etc. (>2 only possible with filter)
#   polyOrder: order of polynomial for interpolation filtering
#   filterWindow: if zero: produces unfiltered derivative; if positive, must be ODD integer {1,3,5,...} and > polyOrder; filterWindow determines the length of the filter window (e.g., to get rid of noise)
#   centralDifferentiate: if True, it uses a central differentiation for first order, unfiltered derivatives; leads to less phase shift of signal!
#**output: numpy array containing same columns, but with filtered signal and according derivatives
def FilterSensorOutput(signal, filterWindow=5, polyOrder=3, derivative=0, centralDifferentiate=True):
    data = np.copy(signal)
    if len(data.shape) < 2:
        raise ValueError("FilterSensorOutput: signal must have 2 dimensions (rows and columns)")
    nColumns = data.shape[1]
    if nColumns < 2:
        raise ValueError("FilterSensorOutput: signal must have at least two columns: first column for time and second column for data")
    
    #first column is time!
    t = signal[:,0]
    #symmetrically extend step sizes:
    derT0 = np.diff(t)
    derT1 = np.diff(t)
    derT0 = np.append(derT0,derT0[-1])
    derT1 = np.append(derT1[0], derT1)
    # print("derT0=",derT0)
    # print("derT1=",derT1)
    for i in range(nColumns-1):
        x = signal[:,i+1]
        if filterWindow == 0 and derivative != 0: #regular mode
            if derivative == 1:
                der = np.diff(x,n=derivative, append=x[-1])/derT0 #append duplicate of last entry
                if centralDifferentiate:
                    der = 0.5*(der + np.diff(x,n=derivative, prepend=x[0])/derT1)
            elif derivative == 2:
                der = np.diff(x,n=derivative, prepend=x[0], append=x[-1]) #append duplicate of last entry
                der = der/derT0**2
                if len(x)>2:
                    der[0] = der[1] #boundary values are extended
                    der[-1] = der[-2] #boundary values are extended
            else:
                raise ValueError("FilterSensorOutput: unfiltered signals only supported up to 2nd derivatives")
            data[:,i+1] = der
        elif filterWindow == 0 and derivative == 0:
            print("WARNING: FilterSensorOutput: no processing")
        else:
            if filterWindow%2 != 1:
                raise ValueError("FilterSensorOutput: filterWindow must be ODD integer and >= 1")
                
            from scipy.signal import savgol_filter
            y = savgol_filter(x, window_length=filterWindow, polyorder=polyOrder, deriv=derivative)
            if derivative > 0:
                y = y/derT0**derivative #for n-th derivative, needs to divide by stepsize^n
            data[:,i+1] = y
            
    return data

#**function: filter 1D signal (using numpy savgol filter) as well as numerical differentiation to compute derivative of signal
#**input:
#   signal: 1D numpy array 
#   samplingRate: (time increment) of signal values, needed for derivatives
#   derivative: 0=no derivative, 1=first derivative, 2=second derivative, etc. (>2 only possible with filter)
#   polyOrder: order of polynomial for interpolation filtering
#   filterWindow: if zero: produces unfiltered derivative; if positive, must be ODD integer {1,3,5,...} and > polyOrder; filterWindow determines the length of the filter window (e.g., to get rid of noise)
#   centralDifferentiate: if True, it uses a central differentiation for first order, unfiltered derivatives; leads to less phase shift of signal!
#**output: numpy array containing same columns, but with filtered signal and according derivatives
def FilterSignal(signal, samplingRate=-1, filterWindow=5, polyOrder=3, derivative=0, centralDifferentiate=True):
    data = np.copy(signal)
    
    dt = samplingRate
    x = signal

    if filterWindow == 0 and derivative != 0: #regular mode
        if derivative == 1:
            der = np.diff(x,n=derivative, append=x[-1])/dt #append duplicate of last entry
            if centralDifferentiate:
                der = 0.5*(der + np.diff(x,n=derivative, prepend=x[0])/dt)
        elif derivative == 2:
            der = np.diff(x,n=derivative, prepend=x[0], append=x[-1]) #append duplicate of last entry
            der = der/dt**2
            if len(x)>2:
                der[0] = der[1] #boundary values are extended
                der[-1] = der[-2] #boundary values are extended
        else:
            raise ValueError("FilterSignal: unfiltered signals only supported up to 2nd derivatives")
        data = der
    elif filterWindow == 0 and derivative == 0:
        print("WARNING: FilterSignal: no processing")
    else:
        if filterWindow%2 != 1:
            raise ValueError("FilterSignal: filterWindow must be ODD integer and >= 1")
            
        from scipy.signal import savgol_filter
        y = savgol_filter(x, window_length=filterWindow, polyorder=polyOrder, deriv=derivative)
        if derivative > 0:
            y = y/(dt**derivative) #for n-th derivative, needs to divide by stepsize^n
        data = y
            
    return data


#**function: computes fast-fourier-transform (FFT) resulting in frequency, magnitude and phase of signal data using numpy.fft of numpy
#**author: Stefan Holzinger
#**date: 02.04.2020
#**input: 
#   time ... time vector in SECONDS in numpy format, having constant sampling rate (not checked!)
#   data ... data vector in numpy format
#**output:
#   frequency ... frequency vector (Hz, if time is in SECONDS)   
#   magnitude ... magnitude vector
#   phase     ... phase vector (in radiant)
def ComputeFFT(time, data):
    from math import floor

    # compute sample time
    sampleTime = time[1] - time[0]
    
    # compute FFT 
    Y = np.fft.fftn(data)
    #Y = np.fft.fft(data)
    
    # number of values
    numberOfDataPoints     = len(data)  
    halfNumberOfDataPoints = floor(numberOfDataPoints/2)
    
    # generate frequency vector in (Hz)
    frequency = 1./(numberOfDataPoints*sampleTime)*np.linspace(0, halfNumberOfDataPoints+1, halfNumberOfDataPoints+1, endpoint=False)
    
    # compute amplitude vector 
    magnitude = 2.*abs( Y[0:halfNumberOfDataPoints+1] ) / numberOfDataPoints
    
    # compute phase vector in (rad)
    phase = np.angle( Y[0:halfNumberOfDataPoints+1] )
    
    return [frequency, magnitude, phase]      




#**function: Interpolate signal having time values with constant sampling rate in timeArray and according data in dataArray
#**input: 
#  time: time at which the data should be evaluated
#  dataArray: 1D numpy array containing data values to be interpolated [alternatively: 2D numpy array, rows containg the data of the according time point; use dataArrayColumnIndex to specify the column of requested data]
#  timeArray: 1D numpy array containing time values with CONSTANT SAMPLING RATE to be interpolated [alternatively: 2D numpy array, rows containg the time and data of the according time point; use timeArrayColumnIndex to specify the column representing time]; if timeArray is empty list [], dataArray is used instead!
#  rangeWarning: print warning if resulting index gets out of range
#  dataArrayColumnIndex: in case of 2D arrays, this represents the column of the requested data
#  timeArrayColumnIndex: in case of 2D arrays, this represents the column of time values
#  tolerance: this tolerance is used to check, if the timeArray has equidistant interpolation and if the found indices are correct; use e.g. 1e10 in order to ignore this tolerance
#**notes: for interpolation of data WITHOUT constant data rate, use numpy.interp(time, timeArray, dataArray) in case that timeArray and dataArray are 1D arrays
#**output: interpolated value
def GetInterpolatedSignalValue(time, dataArray, timeArray=[], dataArrayIndex = -1, timeArrayIndex = -1, rangeWarning=True, tolerance=1e-6):

    timeArrayNew = timeArray #as this should be fast, we avoid copy here; should be safe, as timeArrayNew is not modified
    if IsListOrArray(timeArrayNew) and len(timeArrayNew) == 0:
        timeArrayNew=dataArray
    if dataArray.ndim != 1 and dataArrayIndex == -1:
        raise ValueError('GetInterpolatedSignalValue: in case of 2D dataArray, dataArrayIndex must be provided!')
    if timeArrayNew.ndim != 1 and timeArrayIndex == -1:
        raise ValueError('GetInterpolatedSignalValue: in case of 2D timeArray, timeArrayIndex must be provided!')

    t0 = -1 #time of first value
    tEnd = -1 #time of last value
    if len(dataArray) > 1 and len(timeArrayNew) > 1:
        if timeArrayIndex == -1: #1D array
            t0 = timeArrayNew[0]
            tEnd = timeArrayNew[-1]
            dt = timeArrayNew[1] - t0 #sampling rate
        else: #2D array
            t0 = timeArrayNew[0,timeArrayIndex]
            tEnd = timeArrayNew[-1,timeArrayIndex]
            dt = timeArrayNew[1,timeArrayIndex] - t0
            
        if dt == 0.:
            raise ValueError('GetInterpolatedSignalValue: sample rate is zero!')
    else:
        raise ValueError('GetInterpolatedSignalValue: dataArray or timeArray does not contain sufficient rows!')

    
    index = int((time-t0) / dt)
    #print('index=', index, ', t0=', t0, ', dt=', dt)
    if (time-t0) < 0.: #no interpolation
        index = 0
        if rangeWarning:
            print('Warning: GetInterpolatedSignalValue: index returned smaller than 0; using 0 instead')
        if dataArrayIndex == -1: #1D array
            value = dataArray[0]
        else: #2D array
            value = dataArray[0,dataArrayIndex]
    elif index >= len(dataArray)-1: #no interpolation any more
        if time > tEnd and rangeWarning:
            print('Warning: GetInterpolatedSignalValue: time larger than available time data; using last row of dataArray instead')
        index = len(dataArray)-1
        if dataArrayIndex == -1: #1D array
            value = dataArray[-1]
        else: #2D array
            value = dataArray[-1,dataArrayIndex]
    else:
        
        #interpolate
        tA = dt*index + t0
        tB = tA+dt 
        #check if time index is correct:
        if timeArrayIndex == -1: #1D array
            if abs(tA-timeArrayNew[index]) > tolerance or abs(tB-timeArrayNew[index+1]) > tolerance :
                print('Warning: GetInterpolatedSignalValue: timeArray does not seem to have constant sampling rate; use larger tolerance or numpy.interp(...) instead')
        else: #2D array
            if (abs(tA-timeArrayNew[index,timeArrayIndex]) > tolerance or 
                abs(tB-timeArrayNew[index+1,timeArrayIndex]) > tolerance):
                print('Warning: GetInterpolatedSignalValue: timeArray does not seem to have constant sampling rate; use larger tolerance or numpy.interp(...) instead')
            

        if dataArrayIndex == -1: #1D array
            valueA = dataArray[index]
            valueB = dataArray[index+1]
        else: #2D array
            valueA = dataArray[index,dataArrayIndex]
            valueB = dataArray[index+1,dataArrayIndex]

        #print('tA=', tA, ', tB=', tB, ', valueA=', valueA, ', valueB=', valueB)
        value = valueA*(tB-time)/dt + valueB*(time-tA)/dt

    return value



#simple tests:
if __name__ == '__main__':
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #GetInterpolatedSignalValue
    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #create 1D signal
    t = np.linspace(1,10,40)
    x = np.sin(t)
    
    #coarsen
    t2 = np.linspace(0,11,15)
    x2 = np.array([GetInterpolatedSignalValue(ti, x, t, rangeWarning=False) for ti in t2])

    #refine
    t3 = np.linspace(0,11,90)
    x3 = np.array([GetInterpolatedSignalValue(ti, x, t, rangeWarning=False) for ti in t3])
    
    from exudyn.plot import PlotSensor
    PlotSensor(None, sensorNumbers=[np.vstack((t,x)).T, np.vstack((t2,x2)).T, np.vstack((t3,x3)).T], closeAll=True, title='1D array')
    
    #create 2D signal
    t = np.linspace(1,10,40)
    x = np.sin(t)
    y = np.cos(t)
    data = np.vstack((t,np.vstack((x,y)))).T
    
    #coarsen, data is 2D
    t2 = np.linspace(0,11,15)
    x2 = np.array([GetInterpolatedSignalValue(ti, dataArray=data, dataArrayIndex=1, timeArrayIndex=0, rangeWarning=False) for ti in t2])

    #refine, data is 2D
    t3 = np.linspace(0,11,90)
    x3 = np.array([GetInterpolatedSignalValue(ti, dataArray=data, dataArrayIndex=2, timeArrayIndex=0, rangeWarning=False) for ti in t3])

    #refine, use 'different' time and data arrays; both arrays are 2D arrays
    t4 = np.linspace(0,11,45)
    x4 = np.array([GetInterpolatedSignalValue(ti, dataArray=data, timeArray=data, dataArrayIndex=1, timeArrayIndex=0, rangeWarning=False) for ti in t4])
    
    #refine, use different time and data arrays; data is 2D, time is 1D
    t5 = np.linspace(0,11,45)
    x5 = np.array([GetInterpolatedSignalValue(ti, dataArray=data, timeArray=t, dataArrayIndex=2, rangeWarning=False) for ti in t5])

    #refine, use different time and data arrays; data is 1D, time is 2D
    t6 = np.linspace(0,11,45)
    x6 = np.array([GetInterpolatedSignalValue(ti, dataArray=y, timeArray=data, timeArrayIndex=0, rangeWarning=False) for ti in t6])
    
    PlotSensor(None, sensorNumbers=[np.vstack((t,x)).T, np.vstack((t,y)).T, 
                                    np.vstack((t2,x2)).T, np.vstack((t3,x3)).T, np.vstack((t4,x4)).T, np.vstack((t5,x5)).T, np.vstack((t6,x6)).T
                                    ], newFigure=True, title='2D array')
    
    import time
    ts = -time.time()    
    t2 = np.linspace(0,11,500000)
    x2 = np.array([GetInterpolatedSignalValue(ti, x, t, rangeWarning=False) for ti in t2])

    #about 1.4 seconds on 3GHz i7 processor
    print('5e5 x GetInterpolatedSignalValue takes', time.time()+ts, 'seconds')



    