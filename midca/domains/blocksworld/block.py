
# single block, a square, a triangle, or a table. These are the blocks of which
# scenes are composed.
# Possible predicates involving blocks:
#   -square(x)
#   -triangle(x)
#   -table(x)
#   -clear(x)
#   -on(x,y)
class Block:
    SQUARE = 1          # constant
    TRIANGLE = 2
    TABLE = 3

    def __init__(self, type, id):
        self.type = type
        self.id = id
        self.on = None
        self.clear = True
        self.onfire = False

    # place self block onto onto block
    def place(self, onto):
        if self.on:
            self.on.clear()
        self.on = onto
        self.on.unclear()

    # set the block to not clear
    def unclear(self):
        self.isclear = False

    # set the block to clear
    def clear(self):
        self.isclear = True  
        
    def __str__(self):
    	return self.id
    
    def __hash__(self):
    	return hash(self.id)
    
    def __eq__(self, other):
    	try:
    		return self.id == other.id
    	except Exception:
    		return False
