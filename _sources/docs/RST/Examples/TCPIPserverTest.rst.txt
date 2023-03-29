
.. _examples-tcpipservertest:

******************
TCPIPserverTest.py
******************

You can view and download this file on Github: `TCPIPserverTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/TCPIPserverTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for connecting two Python codes via TCP/IP
   #           See file TCPIPclientTest.py for running on the other Python instance
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
   
   #https://docs.python.org/3/library/struct.html
   packer = struct.Struct('I d d d d') #I=unsigned int, i=int, d=double
   ackPacker = struct.Struct('I')
   
   # Create a TCP/IP socket
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   
   HOST='127.0.0.1'
   PORT = 65124
   
   # # Bind the socket to the port
   # server_address = (HOST, PORT)
   # print('starting up on %s port %s' % server_address)
   # sock.bind(server_address)
   
   # #Calling listen() puts the socket into server mode, and accept() waits for an incoming connection.
   # # Listen for incoming connections
   # sock.listen(1)
   
   
   with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
       s.bind((HOST, PORT))
       s.listen()
       conn, addr = s.accept()
       with conn:
           print('Connected by', addr)
           while True:
               # data = conn.recv(1024) #data size in bytes
               data = conn.recv(packer.size) #data size in bytes
               
               if not data:
                   break #usually does not happen!
   
               unpacked_data = packer.unpack(data)
               print('data=',unpacked_data, ', len=',len(unpacked_data))
               
               chksum = sum(data) #checksum of packed bytes
               ackData = ackPacker.pack(int(chksum+unpacked_data[1]))
               conn.sendall(ackData)
   
   
   
   # while True:
   #     # Wait for a connection
   #     print('waiting for a connection')
   #     connection, client_address = sock.accept()
   # #accept() returns an open connection between the server and client, along with the address of the client. The connection is actually a different socket on another port (assigned by the kernel). Data is read from the connection with recv() and transmitted with sendall().
   
   # try:
   #     print('connection from', client_address)
   
   #     # Receive the data in small chunks and retransmit it
   #     while True:
   #         data = connection.recv(16)
   #         print('received "%s"' % data)
   #         if data:
   #             print(sys.stderr, 'sending data back to the client')
   #             connection.sendall(data)
   #         else:
   #             print('no more data from', client_address)
   #             break
           
   # finally:
   #     # Clean up the connection
   #     connection.close()
   


