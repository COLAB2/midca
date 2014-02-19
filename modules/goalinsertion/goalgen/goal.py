
class Goal:
    GOAL_ON = "on"
    GOAL_NO_FIRE = "notonfire"
    GOAL_APPREHEND = "apprehend"
    
    def __init__(self, goaltype, goalargs=[], priority = 0):
        self.goaltype = goaltype
        self.goalargs = goalargs
        self.priority = priority

    # returns a string representation of the goal suitable as input to FOIL
    def foil_str(self):
        s = "(" + self.goaltype
        for arg in self.goalargs:
        	s += " " + arg.id
        return s + ")"
    
    def __str__(self):
    	s = self.goaltype + "("
    	for arg in self.goalargs:
    		s += str(arg) + " "
    	if self.goalargs:
    		return s[:-1] + ") priority-" + str(round(self.priority, 2)) 
    	else:
    		return s + ") priority-" + str(round(self.priority, 2)) 
    
    def __hash__(self):
    	return hash((self.goaltype, tuple(self.goalargs)))
    
    def __lt__(self, other):
    	return self.priority > other.priority
    def __eq__(self, other):
    	try:
    		return self.goaltype == other.goaltype and self.goalargs == other.goalargs
    	except AttributeError:
    		return False
	def __ne__(self, other):
		return not self == other
	def __gt__(self, other):
		return other<self
	def __ge__(self, other):
		return not self<other
	def __le__(self, other):
		return not other<self