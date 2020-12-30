import numpy as np
class AcousticTag():
    def __init__(self,ID,ping_delay=17,last_ping=0):
        self.last_ping=last_ping
        self.pos = np.array([0,0,0])
        self.delay=ping_delay
        self.ID=ID
        self.bin=0
		
    def pinging(self,current_time):
        if current_time-self.delay > self.last_ping:
            return True
        return False
		
    def updatePing(self,current_time):
        if current_time-self.delay > self.last_ping:
            self.last_ping+=self.delay
			
