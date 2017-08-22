#!/usr/bin/env python

import socket
import numpy as np
from cStringIO import StringIO

HOST = "169.254.123.191"
HOST = "169.254.254.143"
#HOST = "127.0.0.1"
PORT = 20000 # The same port as used by the server

class numpysocket():
    def __init__(self):
        self.current_socket = socket.socket()
        pass

    @staticmethod
    def startServer(self):
        port=PORT
        server_socket=self.current_socket
        server_socket.bind((HOST,port))
        server_socket.listen(3)
        print 'waiting for a connection...'
        client_connection,client_address=server_socket.accept()
        print 'connected to ',client_address[0]
        self.current_socket = server_socket
        return client_connection, client_address
    
    @staticmethod
    def collectImage(self, client_connection, client_address):
        server_socket = self.current_socket
        ultimate_buffer=''
        buffer_length = 0
        check = 0
        while True:
            image_length = client_connection.recv(1024)
            print "IMAGE LENGTH: ",  image_length
            if int(image_length)>0:
                client_connection.send("Length received")
                break
            else:
                client_connection.send("Failed")
                check = 1
                break
        print image_length
        while True: 
            if check == 1:
                break
            receiving_buffer = client_connection.recv(1024)
            buffer_length = buffer_length + len(receiving_buffer)
            if buffer_length >= int(image_length):
                ultimate_buffer+= receiving_buffer
                break
            ultimate_buffer+= receiving_buffer
            #buffer_length
        if check == 0:
            print buffer_length, image_length
            final_image=np.load(StringIO(ultimate_buffer))['frame']
            client_connection.send("Image received")
            #client_connection.close()
            #server_socket.close()
            print '\nframe received'
            return final_image
        else:
            print "Failed"
            return None

    @staticmethod
    def startClient(image,server_address=HOST):
        if not isinstance(image,np.ndarray):
            print 'not a valid numpy image'
            return
        client_socket=socket.socket()
        port=PORT
        try:
            client_socket.connect((server_address, port))
            print 'Connected to %s on port %s' % (server_address, port)
        except socket.error,e:
            print 'Connection to %s on port %s failed: %s' % (server_address, port, e)
            return
        f = StringIO()
        np.savez_compressed(f,frame=image)
        f.seek(0)
        out = f.read()
        client_socket.sendall(out)
        client_socket.shutdown(1)
        client_socket.close()
        print 'image sent'
        pass