
.. _sec-module-signalprocessing:

Module: signalProcessing
========================

The signal library supports processing of signals for import (e.g. measurement data)
and for filtering result data.

- Date:      2020-12-10 
- Notes:     This module is still under construction and should be used with care! 


.. _sec-signalprocessing-filtersensoroutput:

Function: FilterSensorOutput
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`FilterSensorOutput <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/signalProcessing.py\#L28>`__\ (\ ``signal``\ , \ ``filterWindow = 5``\ , \ ``polyOrder = 3``\ , \ ``derivative = 0``\ , \ ``centralDifferentiate = True``\ )

- | \ *function description*\ :
  | filter output of sensors (using numpy savgol filter) as well as numerical differentiation to compute derivative of signal
- | \ *input*\ :
  | \ ``signal``\ : numpy array (2D array with column-wise storage of signals, as exported by EXUDYN position, displacement, etc. sensors); first column = time, other columns = signals to operate on; note that it is assumed, that time devided in almost constant steps!
  | \ ``derivative``\ : 0=no derivative, 1=first derivative, 2=second derivative, etc. (>2 only possible with filter)
  | \ ``polyOrder``\ : order of polynomial for interpolation filtering
  | \ ``filterWindow``\ : if zero: produces unfiltered derivative; if positive, must be ODD integer {1,3,5,...} and > polyOrder; filterWindow determines the length of the filter window (e.g., to get rid of noise)
  | \ ``centralDifferentiate``\ : if True, it uses a central differentiation for first order, unfiltered derivatives; leads to less phase shift of signal!
- | \ *output*\ :
  | numpy array containing same columns, but with filtered signal and according derivatives

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TM), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TM)



----


.. _sec-signalprocessing-filtersignal:

Function: FilterSignal
^^^^^^^^^^^^^^^^^^^^^^
`FilterSignal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/signalProcessing.py\#L83>`__\ (\ ``signal``\ , \ ``samplingRate = -1``\ , \ ``filterWindow = 5``\ , \ ``polyOrder = 3``\ , \ ``derivative = 0``\ , \ ``centralDifferentiate = True``\ )

- | \ *function description*\ :
  | filter 1D signal (using numpy savgol filter) as well as numerical differentiation to compute derivative of signal
- | \ *input*\ :
  | \ ``signal``\ : 1D numpy array
  | \ ``samplingRate``\ : (time increment) of signal values, needed for derivatives
  | \ ``derivative``\ : 0=no derivative, 1=first derivative, 2=second derivative, etc. (>2 only possible with filter)
  | \ ``polyOrder``\ : order of polynomial for interpolation filtering
  | \ ``filterWindow``\ : if zero: produces unfiltered derivative; if positive, must be ODD integer {1,3,5,...} and > polyOrder; filterWindow determines the length of the filter window (e.g., to get rid of noise)
  | \ ``centralDifferentiate``\ : if True, it uses a central differentiation for first order, unfiltered derivatives; leads to less phase shift of signal!
- | \ *output*\ :
  | numpy array containing same columns, but with filtered signal and according derivatives

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TM)



----


.. _sec-signalprocessing-computefft:

Function: ComputeFFT
^^^^^^^^^^^^^^^^^^^^
`ComputeFFT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/signalProcessing.py\#L128>`__\ (\ ``time``\ , \ ``data``\ )

- | \ *function description*\ :
  | computes fast-fourier-transform (FFT) resulting in frequency, magnitude and phase of signal data using numpy.fft of numpy
- | \ *input*\ :
  | time ... time vector in SECONDS in numpy format, having constant sampling rate (not checked!)
  | data ... data vector in numpy format
- | \ *output*\ :
  | frequency ... frequency vector (Hz, if time is in SECONDS)
  | magnitude ... magnitude vector
  | phase     ... phase vector (in radiant)
- | \ *author*\ :
  | Stefan Holzinger
- | \ *date*\ :
  | 02.04.2020

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex)



----


.. _sec-signalprocessing-getinterpolatedsignalvalue:

Function: GetInterpolatedSignalValue
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetInterpolatedSignalValue <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/signalProcessing.py\#L167>`__\ (\ ``time``\ , \ ``dataArray``\ , \ ``timeArray = []``\ , \ ``dataArrayIndex = -1``\ , \ ``timeArrayIndex = -1``\ , \ ``rangeWarning = True``\ , \ ``tolerance = 1e-6``\ )

- | \ *function description*\ :
  | Interpolate signal having time values with constant sampling rate in timeArray and according data in dataArray
- | \ *input*\ :
  | \ ``time``\ : time at which the data should be evaluated
  | \ ``dataArray``\ : 1D numpy array containing data values to be interpolated [alternatively: 2D numpy array, rows containg the data of the according time point; use dataArrayColumnIndex to specify the column of requested data]
  | \ ``timeArray``\ : 1D numpy array containing time values with CONSTANT SAMPLING RATE to be interpolated [alternatively: 2D numpy array, rows containg the time and data of the according time point; use timeArrayColumnIndex to specify the column representing time]; if timeArray is empty list [], dataArray is used instead!
  | \ ``rangeWarning``\ : print warning if resulting index gets out of range
  | \ ``dataArrayColumnIndex``\ : in case of 2D arrays, this represents the column of the requested data
  | \ ``timeArrayColumnIndex``\ : in case of 2D arrays, this represents the column of time values
  | \ ``tolerance``\ : this tolerance is used to check, if the timeArray has equidistant interpolation and if the found indices are correct; use e.g. 1e10 in order to ignore this tolerance
- | \ *output*\ :
  | interpolated value
- | \ *notes*\ :
  | for interpolation of data WITHOUT constant data rate, use numpy.interp(time, timeArray, dataArray) in case that timeArray and dataArray are 1D arrays

