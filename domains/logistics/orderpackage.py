
class Order:
    
    def __init__(self, id):
        self.id = id
        self.location = None
        self.destination = None
        
        
    def __str__(self):
    	return self.id
    
    def __hash__(self):
    	return hash(self.id)
    
    def __eq__(self, other):
    	try:
    		return self.id == other.id
    	except Exception:
    		return False

class warehouse: 
    
    def __init__(self, id, packages):
        self.id = id
        self.packages = packages
        self.destination = None
        
        
    def __str__(self):
        return self.id
    
    def __hash__(self):
        return hash(self.id)
    
    def __eq__(self, other):
        try:
            return self.id == other.id
        except Exception:
            return False