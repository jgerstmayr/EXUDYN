
.. _examples-tcpipclienttest:

******************
TCPIPclientTest.py
******************

You can view and download this file on Github: `TCPIPclientTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/TCPIPclientTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for connecting two Python codes via TCP/IP
   #           See file TCPIPserverTest.py for running on the other Python instance
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-11-06
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import socket
   import sys
   import time
   import struct
   packer = struct.Struct('I d d d d')
   ackPacker = struct.Struct('I')
   
   HOST='127.0.0.1'
   PORT = 65124
   # Create a TCP/IP socket
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   
   # Connect the socket to the port where the server is listening
   server_address = (HOST, PORT)
   print('connecting to %s port %s' % server_address)
   sock.connect(server_address)
   # After the connection is established, data can be sent through the socket with sendall() and received with recv(), just as in the server.
   
   
   t = 0
   try:
       
       # Send data
       # message = 'This is the message.  It will be repeated.'
       # print('sending "%s"' % message)
       # byteMessage=message.encode(encoding='utf-8')
       # sock.sendall(byteMessage)
   
       while t < 10:
           x=1.13
           y=4
           z=-3.1415
           values = (4, t, x, y, z)
           byteMessage = packer.pack(*values)
           print("send", values)
           sock.sendall(byteMessage)
           chksum = sum(byteMessage)
           ackData = ackPacker.pack(chksum)
   
           ackReceived = False
           timeout = 0
           while not ackReceived:
               data = sock.recv(len(ackData)) #recv waits until it receives the package!
               # if (len(data)):
               #     print(data, len(data), ackPacker.unpack(data)[0] ,'==', chksum)
               if len(data) == len(ackData) and ackPacker.unpack(data)[0] == chksum:
                   print('  ok (', ackPacker.unpack(data)[0], ')')
                   ackReceived = True
               else:
                   print('error in checksum; check your connection')
                   ackReceived = True
   
           time.sleep(0.5)
           t+=0.5
       
           # Look for the response
           # amount_received = 0
           # amount_expected = len(byteMessage)
           
           # while amount_received < amount_expected:
           #     data = sock.recv(16)
           #     amount_received += len(data)
           #     print('received "%s"' % data)
   
   finally:
       print('closing socket')
       sock.close()
       
       


