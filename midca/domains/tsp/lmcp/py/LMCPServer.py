import sys, time, socket
from midca.domains.tsp.lmcp.py.lmcp import LMCPFactory

if (sys.version_info > (3, 0)):
    import socketserver as ss
else:
    import SocketServer as ss

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

myHost = ''
myPort = 11041

class LMCPHandler(ss.StreamRequestHandler):
    def handle(self):
        self.factory = LMCPFactory.LMCPFactory()
        print("Client address: %s" % (self.client_address,))
        while True:
            try:
                data = bytearray(self.request.recv(LMCPFactory.HEADER_SIZE))
                if(len(data) >= LMCPFactory.HEADER_SIZE):
                    print("header size: %d" % (len(data),) )
                    size = LMCPFactory.getSize(data)
                    print("object size: %d" % (size,))
                    data.extend(bytearray(self.request.recv(size+4))) # compensate for checksum
                    print("%d bytes received" % (len(data),))
                    recv_obj = self.factory.getObject(data)
                    print("%s received" % recv_obj.__class__)
                    if recv_obj != None:
                        print("Printing object XML...")
                        print(recv_obj.toXMLStr(""))
                    else:
                       print("Invalid object received.")
            except socket.error:
                self.stop = True
        self.request.close()

if __name__ == '__main__':
    # make a threaded server, listen/handle clients forever
    myaddr = (myHost, myPort)
    server = ss.TCPServer(myaddr, LMCPHandler)
    server.serve_forever()



    


