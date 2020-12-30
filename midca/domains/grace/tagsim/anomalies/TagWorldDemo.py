import socket

class TagWorld():
	def __init__(self,host='127.0.0.1',port=5700):
	#def __init__(self,host='127.0.0.1',port=80):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.connect((host,port))
		self.sock.settimeout(1)
		
	def runSim(self):
		self.sock.send(str.encode(str('start')))
		
	def endSim(self):
		self.sock.send(str.encode(str('quit')))
	
		
		#initial_position and final_position is a grid cell of format [x,y]
	# template to move from one cell to another
	def move_cell(self,initial_position, final_destination):
		msg ="moveTo,"+str(final_destination[0])+","+str(final_destination[1])
		self.sock.send(str.encode(str(msg)))
		#make the movement from initial_position to the final destination
		  
		#return acknowledgment once it moved to the destination grid cell
		return self.in_cell(final_destination) # boolean value; true if reached
    
	# position = [x, y]  # x and y are the coordinates of the grid cell
	def in_cell(self,position):
		msg ="inCell,"+str(position[0])+","+str(position[1])
		self.sock.send(str.encode(str(msg)))
		msg=self.sock.recv(1024)
		return msg.decode('utf-8')
	
	def get_cell(self):
		try:
			msg = "getCell"
			self.sock.send(str.encode(str(msg)))
			msg = self.sock.recv(2048)
			position = msg.decode('utf-8')
            # convert to midca coordinates
            # for example [2,2] is [1,1] in our representation
			if position:
				position = position.split(",")
				position = [str(int(pos)-1) for pos in position]
				position = ",".join(position)
			return position
		except Exception as e:
			print(e)
			
	# position = [x, y]  # x and y are the coordinates of the grid cell
	def search_and_get_tags2(self,position):
		#search the location (grid cell [x,y])
		if self.searchComplete()=='True':
			return self.get_tags()#no_of_tags
		else:
			self.search(position)
	
	def search_and_get_tags(self,position):
		#search the location (grid cell [x,y])
		self.search(position)
		complete='False'
		while complete=='False':
		    complete=self.searchComplete()
		# collect tags
		print(complete)
		return self.get_tags()#no_of_tags
	# position = [x, y]  # x and y are the coordinates of the grid cell
	
	def search(self,position):
		msg ="search,"+str(position[0])+","+str(position[1])
		self.sock.send(str.encode(str(msg)))
		#search the location (grid cell [x,y])
		# display the movement of the agent in the cell
		
	def searchComplete(self):
		self.sock.send(str.encode('searchComplete'))
		msg=self.sock.recv(1024)
		return msg.decode('utf-8')
		
	def get_tags(self):
		msg ="get_tags"
		self.sock.send(str.encode(str(msg)))
		# collect tags
		msg=self.sock.recv(1024)
		return int(msg.decode('utf-8'))
		
	def get_cell_poisson_rate(self,pos=None):
		msg ="cell_lambda"
		if type(pos) != type(None):
			msg+=","+str(pos[0])+","+str(pos[1])
		self.sock.send(str.encode(str(msg)))
		# collect tags
		msg=self.sock.recv(1024)
		return float(msg.decode('utf-8'))
		
		
if __name__ == "__main__":
        import time
        """"
        tag = TagWorld()
        tag.runSim()


        tag = TagWorld()
        tag.move_cell([0,0], [1,3])

        """


        tag = TagWorld()
        time.sleep(1)
        tag.runSim()
       
        tag.search([1,1])
        time.sleep(10)
        tag.endSim() 
        #print (tag.simtime())
        #tag.search([0, 3])
        #print tag.get_cell_poisson_rate()
        #print tag.get_tags()
        #tag.send([3,3])
        #print (msg)
        #tag.UpdateUncertainity("High", 100)
        #tag.UpdateUncertainity("Med", 50)
        #tag.UpdateSurfstatus("ascend")