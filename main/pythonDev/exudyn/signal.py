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


