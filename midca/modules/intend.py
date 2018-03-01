from midca import base,midcatime
import copy,itertools,operator
import random
import time as ctime

class SimpleIntend(base.BaseModule):

    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
	# get all the goals from the root of the goal graph
        goals = goalGraph.getUnrestrictedGoals()

	if not goals:
            if verbose >= 1:
                print "No Goals in Goal graph. Intend will do nothing."
            return
		
	# take the first goal
	goals = [goals[0]]
	# add it to the current goal in memory

	# current goals as a stack
	if self.mem.get(self.mem.CURRENT_GOALS) :
		current_goals = self.mem.get(self.mem.CURRENT_GOALS)
		if not current_goals[-1] == goals:
			current_goals.append(goals)
        		self.mem.set(self.mem.CURRENT_GOALS, current_goals)
		else:
			goals = []
	else:
		self.mem.set(self.mem.CURRENT_GOALS, [goals])

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class WarehouseIntend(base.BaseModule):
    
    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return
        goals = goalGraph.getUnrestrictedGoals()
        goals_selected = []
        # special code for NBeacons, need to change for later
        exists_free_goal = False
        free_goal = None
        for g in goals:
            if 'free' in str(g):
                exists_free_goal = True
                free_goal = g
        
        # activation goals: just pick one
        
        if free_goal:
            goals_selected = [free_goal]
            self.mem.set(self.mem.CURRENT_GOALS,[goals_selected] )
        else:
            if len(goals) > 1:
                # accomplish one of the warehouse packages
                goals_selected = []
                goals_selected_warehouse = goals[0].args[2]
                for g in goals:
                    if g.args[2] == goals_selected_warehouse:
                        goals_selected.append(g)
                        
                self.mem.set(self.mem.CURRENT_GOALS,[goals_selected] )        
            else:
                goals_selected = goals
                self.mem.set(self.mem.CURRENT_GOALS, [goals_selected])
        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals_selected:
                    print goal,
                print
                
class SimpleIntend_Restaurant(base.BaseModule):

    cost_dishes = {} # cost of dishes
    #time_dishes = {} # time taken to cook dishes
    selected_goals = []
    actual_cost = 0 # actual cost
    actual_score = 0 # actual score
    variation = 0.5
    # "selection_method" , "fifo_method"
    method = "FIFO_method" # this denotes whether it is a selection method or FIFO
				# for FIFO

    def get_cost(self,goals):
	'''
	input : goals
	output : doesnt return anything but updates the cost_dishes and time_dishes variables
	from the world get the atoms and from the atoms
	get the "costofdish" and "timetomakedish" predicates
	'''
	world = self.mem.get(self.mem.STATES)[-1]
	for atom in world.atoms:
	    if atom.predicate.name == "costofdish":
		self.cost_dishes[atom.args[0].name] = int(atom.args[1].name)

    def persons_combination(self,goals):
	'''
	input: goals
	output: return the dictionay file that contains all the combinations of persons with 
		values of money less than that of in memory
	get time from the memory
	compute cost for each person from the goals
	make all the combinations of the persons interms of tuples along with the cost 
	and save it in dictionary
	return the dictionary	
	'''
	compute_person_combinations = {}
	compute_person_cost = {}
	money = self.mem.get(self.mem.MONEY)
	# goal.args[0] = name of person
	# goal.args[1] = name of the dish
	# check if the person is already present in the dictionary
	# if already present add cost of dish to the previous cost
	# else just assign cost of dish
	for goal in goals:
		if goal.args[0] in compute_person_cost:
			sum_cost = compute_person_cost[goal.args[0]] + self.cost_dishes[goal.args[1]]
		else:
			sum_cost = self.cost_dishes[goal.args[1]]
		compute_person_cost[goal.args[0]] = sum_cost	
	#print("compute_person_cost " + str(compute_person_cost))
	# get the combinations of the persons with less than Money from memory
	# iter.combinations is a predefined method which results in the combinations of number specefied
	# save it in the compute_person_combinations dictionary only if it got the total cost to be less than Money in memory
	for i in range(1,len(compute_person_cost)+1):
		for each_tuple in itertools.combinations(compute_person_cost, i):
			sum_cost = 0
			for person in each_tuple:
				sum_cost = sum_cost + compute_person_cost[person]
			if sum_cost <= money:
				compute_person_combinations[each_tuple] = sum_cost
	#print("compute_person_combinations " + str(compute_person_combinations))
	return compute_person_combinations

    def best_cost_combination(self,goals,combinations):
	'''
	input : same satisfiability(score) combination of type list
	output : a combination which has more cost
	'''
	best_combination = {}
	total_cost = 0
	cost_compute = [] # list of costs for each person in best combinations
	for each_combination in combinations:
		cost_compute[:] = []
		for each in each_combination:
			cost = 0
			for goal in goals:
				if each in goal.args:
					cost = cost + self.cost_dishes[goal.args[1]]
			cost_compute.append(cost)
		if total_cost < sum(cost_compute):
			best_combination = {}
			best_combination[each_combination] = copy.deepcopy(cost_compute)
			total_cost = sum(cost_compute)
	return best_combination

    def get_goals_p_t(self,combination,goals):
	'''
	input: goals and the combination with maximum cost
	output: dictionary of goals with p/t scores 
	'''
	goal_combination = {}
	select_goals = []
	for each_combination in combination:
		for id,each in enumerate(each_combination):
			select_goals[:]= []
			for goal in goals:
				if each in goal.args:
					select_goals.append(goal)
			select_goals_tuple = tuple(select_goals)
			goal_combination[select_goals_tuple] = [len(select_goals_tuple) ,combination[each_combination][id]]
	return goal_combination
		 	
					
				
    def compute_time_expectation(self,goals):
	'''
	input : goals
	output: expected time
	iterate through the goals, calculate the total time taken for the dishes
	'''
	time = 0
	for goal in goals:
		time = time + self.time_dishes[goal.args[1]]
	return time	

    def compute_cost_expectation(self,goals):
	'''
	input : goals
	output: expected cost
	iterate through the goals, calculate the total time taken for the dishes
	'''
	cost = 0
	for goal in goals:
		cost = cost + self.cost_dishes[goal.args[1]]
	return cost

    def compute_score_expectation(self,goals):
	'''
	input : goals
	output: expected score (satisfaction)
	'''
	return len(goals)

    def write_expected_to_memory(self,expected_score,expected_cost):
	'''
	input : expected_time , expected_score, expected_cost
	output : place all these into memory
	'''
	if not self.mem.get(self.mem.EXPECTED_COST):
		#print(expected_score)
		self.mem.set(self.mem.EXPECTED_SCORE, expected_score)
		self.mem.set(self.mem.EXPECTED_COST, expected_cost)

    def write_selected_to_memory(self,selected_goals):
	'''
	input : selected goals
	output : write the orders to the memory
	iterate through the selected goals, get each tuple and append it to mem variable
	'''
	selected_orders = self.mem.get(self.mem.SELECTED_ORDERS)
	orders = []
	if not selected_orders:	
		for goal_set in selected_goals:
			for goal in goal_set:
				orders.append(goal.args[0])
				break
		self.mem.set(self.mem.SELECTED_ORDERS,orders)	


    def calculate_combinations_best_scores(self,combination,goals):
	'''
	input : combinations of the persons with less than the money in memory
	output : return a list of combination or combinations with highest satisfybility
	'''
	person_satisfybility = {}
	combination_with_satisfyability = {}
	same_satisfaction_combinations = []
	# for each goal with a person the satisfybility increases by 1
	for goal in goals:
		if goal.args[0] in person_satisfybility:
			person_satisfybility[goal.args[0]] += 1
		else:
			person_satisfybility[goal.args[0]] = 0
	# compute the satisfybility for combinations
	for persons in combination:
	    satisfaction = 0
	    for person in persons:
		satisfaction = satisfaction + person_satisfybility[person] 
	    combination_with_satisfyability[persons] = satisfaction
	#print("combination_with_satisfyability" + str(combination_with_satisfyability))
	# get the best satisfaction count
	maximum = 0
	for i in sorted( combination_with_satisfyability, key=combination_with_satisfyability.__getitem__ ,reverse = True):		
		if maximum <= combination_with_satisfyability[i]:
			maximum = combination_with_satisfyability[i]
			same_satisfaction_combinations.append(i)
	#print(same_satisfaction_combinations)
	return same_satisfaction_combinations

    def goals_combination(self,persons_combinations,goals):
	'''
	input : persons_combinations (all the combination of persons with Money less than Money in memory)
		goals (given by user)
	output : goals_combinations (opt the combination with greater satisfybility, 
				    order the goals according to the (P/T) ratio 
				    P = satisfaction, T = Cost)
	'''
	same_length_combinations = self.calculate_combinations_best_scores(persons_combinations,goals)
	#print("same_length_combinations" + str(same_length_combinations))
	# get the combination which has best cost
	combination = self.best_cost_combination(goals,same_length_combinations)
	#print("combination" + str(combination))
	# get the goals with p,t values
	goals_combinations = self.get_goals_p_t(combination,goals)
	# sort the goals with p/t 
	sorted_goals = {}
	for each in goals_combinations:
		sorted_goals[each] = goals_combinations[each][0]/goals_combinations[each][1]
	selected_goals = sorted(sorted_goals , key=sorted_goals.get ,reverse =True)
	# print the P/T ratio , Expected time and Total expected time for persons
	total_score = 0
	total_cost = 0
	for each in selected_goals :
		for goal in each:
			p = goals_combinations[each][0]
			t = goals_combinations[each][1]
			print("")
			print ("For the Person " + goal.args[0] + ": P/T ratio (" + str(p)+"/"+str(t) + ")  = " + str(float(p)/float(t)))
			expected_score = self.compute_score_expectation(each)
			expected_cost = self.compute_cost_expectation(each)
			total_score = expected_score + total_score
			total_cost = expected_cost + total_cost
			print("\t\t   Expected Score : " + str(expected_score))
			print("\t\t   Expected Cost : " + str(expected_cost))
			break
	print("Expected Total Score : " + str(total_score))
	print("Expected Total Cost : " + str(total_cost))
	print("")
	self.write_expected_to_memory(total_score,total_cost)
	self.write_selected_to_memory(selected_goals)
	self.selected_goals = selected_goals
	return selected_goals
			
						

    def select_priority_goals(self,goals):
	'''
	input: goals
	output: return the goals that are selected by the goal selecting method
	goal selecting method : get the combinations of the goals that takes time less than that of time in memory
				opt the combination with greater satisfybility
				order the goals according to the (P/T) ratio P = satisfaction, T = Cost
				return the goals
	'''
	# get the cost and time taken to cook dishes
	self.get_cost(goals)
	#print("cost_dishes " + str(self.cost_dishes))
	# make the combination of persons that usually result in making less than or equal to Time in memory
	persons_combinations = self.persons_combination(goals)
	#print("persons_combinations" + str(persons_combinations))
	# make the combination of goals based on the maximum satisfaction(score)
	goals_combinations = self.goals_combination(persons_combinations,goals)
	for each in goals_combinations:
		if each:
		  return list(each)

    def compute_cost_goal(self,goal):
	'''
	output: calculate the cost computed for the goal 
	'''
	#print(goal)
	cost_dishes_p = ((self.variation)* (self.cost_dishes[goal.args[1]])) + self.cost_dishes[goal.args[1]]
	cost_dishes_n = (self.cost_dishes[goal.args[1]]) - ((self.variation)*(self.cost_dishes[goal.args[1]]))
	return  random.uniform(cost_dishes_p, cost_dishes_n)

    def print_goals(self,goals):
	for goal in goals:
		print(goal)

    def remove_from_selected_goals(self,goal):
	'''
	input : goal
	output : remove the goal from the selected goals
	'''
	for id,goal_set in enumerate(self.selected_goals):
		for each_goal in goal_set:
			if each_goal.args == goal.args and each_goal["predicate"] == goal["predicate"]:
				temp_list = list(self.selected_goals[id])				
				#print("temp_list" + str(temp_list))
				temp_list.remove(each_goal)
				#print(temp_list)
				self.selected_goals[id] = tuple(temp_list)
				return 
			
				 
    
    def check_goal_in_current_goals(self,goal):
	'''
	input : goal
	output : return True if the goal is in current goals from memory
		 else return false
	'''
	
	current_goals = self.mem.get(self.mem.CURRENT_GOALS)
	# if there are no current goals return False
	if not current_goals:
		return False
	#print("Current Goals : ")	
	#self.print_goals(current_goals)
	for each_goal in current_goals:
		if goal.args == each_goal.args and goal["predicate"] == each_goal["predicate"]:
			return each_goal
	return False

    def get_score(self,goal):
	'''
	input : goal
	output : return the score of the goal
	'''
	score_p = self.variation + 1
	score_n = 1 - self.variation
	return random.uniform(score_p, score_n)

    def write_executed_goal_memory(self,goal):
	'''
	input : goal
	output: get the person name from the goal and append it to memory
	'''
	completed_goals = self.mem.get(self.mem.COMPLETED_ORDERS)
	person_name = goal.args[0]
	if not completed_goals:
		self.mem.set(self.mem.COMPLETED_ORDERS, [person_name])
	else:
		completed_goals.append(person_name)
		self.mem.set(self.mem.COMPLETED_ORDERS, completed_goals)

    def check_cost(self, cost):
	'''
	input : cost
	output : return False if time is less than zero
		 return true
		 
	'''
	cost_comp =self.mem.get(self.mem.MONEY) 
	if (cost_comp - cost) < 0:
		self.mem.set(self.mem.MONEY, 0)
		self.selected_goals[:] = []
		return False
	return True

    def calculate_cost_remaining(self):
	'''
	output : return time taken and remove the goals from the selected goals
		 if the goal doesnot exist in current goal
	''' 
	world = self.mem.get(self.mem.STATES)[-1]
	goals = copy.deepcopy(self.selected_goals)
	for id,goals_set in enumerate(goals):
		for i,goal in enumerate(goals_set):
			check_goal = self.check_goal_in_current_goals(goal)
			if check_goal: 
				if world.atom_true(world.midcaGoalAsAtom(check_goal)):
					self.remove_from_selected_goals(goal)
					cost_taken = self.compute_cost_goal(goal)
					score = self.get_score(goal)
					self.actual_cost = self.actual_cost + cost_taken
					self.actual_score = self.actual_score + score
					#print("score" + str(score))
					return cost_taken,score
				return 0,0
			else:
				cost_taken = self.compute_cost_goal(goal)
				score = self.get_score(goal)
				#print("score" + str(score))
				if self.check_cost(cost_taken):
					self.write_executed_goal_memory(goal)
					self.write_actual_parameters_memory(cost_taken,score)
					self.actual_cost = 0
					self.actual_score = 0
					# without variation
					if self.variation == 0:
						self.selected_goals.pop(id)
					# with variation
					else:
						del self.selected_goals	
						self.selected_goals = []
				return cost_taken,score
			return 0,0

    def write_actual_parameters_memory(self,cost_taken,score):
	'''
	takes the actual parameters and writes them to memory
	'''
	actual_cost = copy.deepcopy(self.actual_cost)
	actual_score = copy.deepcopy(self.actual_score)
	actual_cost = actual_cost + cost_taken
	actual_score = actual_score + score	
	if self.mem.get(self.mem.ACTUAL_COST):
		actual_cost = self.mem.get(self.mem.ACTUAL_COST) + actual_cost
		actual_score = self.mem.get(self.mem.ACTUAL_SCORE) + actual_score
	#print(actual_cost)
	#print(actual_score)
	self.mem.set(self.mem.ACTUAL_COST, actual_cost)
	self.mem.set(self.mem.ACTUAL_SCORE, actual_score)

    def get_current_goals(self):
	'''
	output: if there exists any pending goals, place them into current goals
	calculate time remaining.
	'''
	print("")
	print("Not all Current goals achieved ,")
	print("Processing : "),
	self.print_goals(self.mem.get(self.mem.CURRENT_GOALS))
	print("")
	return self.mem.get(self.mem.CURRENT_GOALS)

    def check_existing_goals(self):
	'''
	check if there are current goals, if not check if there are pending goals
	output : if there exists any pending goals, place them into current goals
		calculate time remaining.
	''' 
	if not self.selected_goals == [()]:
		#print(self.selected_goals)
		money = self.mem.get(self.mem.MONEY)
		cost_taken,score = self.calculate_cost_remaining()
		if self.check_cost(cost_taken):
			self.mem.set(self.mem.MONEY, (money - cost_taken))
			print("actual_cost" +str(self.actual_cost))
			print("actual_score" + str(self.actual_score))
			print("Amount Spent for previous dish order : " + str(cost_taken))
	if self.mem.get(self.mem.CURRENT_GOALS):
		return self.get_current_goals()
	if not self.selected_goals == [()]:
		for goal_set in self.selected_goals:
			goal = list(goal_set)
			self.mem.set(self.mem.CURRENT_GOALS,goal)
			return self.mem.get(self.mem.CURRENT_GOALS)
    
    def fifo_method_goals(self,goals):
	'''
	input : goals a list which contains the goals of persons as an element
	output: return the selected goals
	'''
	total_score = 0
	total_cost = 0
	for each in goals :
		for goal in each:
			print ("For the Person " + goal.args[0] )
			expected_score = self.compute_score_expectation(each)
			expected_cost = self.compute_cost_expectation(each)
			total_score = expected_score + total_score
			total_cost = expected_cost + total_cost
			print("\t\t   Expected Score : " + str(expected_score))
			print("\t\t   Expected Cost : " + str(expected_cost))
			break
	print("Expected Total Score : " + str(total_score))
	print("Expected Total Cost : " + str(total_cost))
	print("")
	self.write_expected_to_memory(total_score,total_cost)
	self.write_selected_to_memory(goals)
	self.selected_goals = goals
	return goals

    def selection_method(self,goals):
	'''
	input: goals from the goal graph
	output : returns the selected goals by using selection method
	'''
	# select the goals based on the score,time and cost
	goals = self.select_priority_goals(goals)  
	# add it to the current goal in memory
	return goals

    def seperate_goals_fifo(self,goals):
	'''
	input : goals
	output : seperate the goals for a person from the whole goals
	'''
	selected_goals = []
	temp_dict = {}
	temp = []
	for goal in goals:
		if goal.args[0] in temp_dict:
			temp.append(goal)
		else:
			if temp:
				selected_goals.append(tuple(temp))
				temp = []
			temp.append(goal)
		temp_dict[goal.args[0]] = goal.args[1]
	if goals:
	     selected_goals.append(tuple(temp))

	cost = 0
	temp_goals = copy.deepcopy(selected_goals)
	for id,goal_set in enumerate(temp_goals):
		for goal in goal_set:
			cost = cost + self.cost_dishes[goal.args[1]]
		if cost > self.mem.get(self.mem.MONEY):
			selected_goals = selected_goals[:id]
			return selected_goals
	
	
	return selected_goals
	

    def fifo_selection(self,goals):
	'''
	input: goals from the goal graph
	output : returns the selected goals by using FIFO selection method
	'''
	# get the cost and time taken to cook dishes

	self.get_cost(goals)
	selected_goals = self.seperate_goals_fifo(goals)
	self.fifo_method_goals(selected_goals)
	for each in self.selected_goals:
		if each:
		  return list(each)
	
		
    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)
	goals = None
        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return

	# if there is no money remaining then intend does nothing
	if self.mem.get(self.mem.MONEY):
		if self.mem.get(self.mem.MONEY) <= 0:
			self.selected_goals[:] = []
			self.mem.set(self.mem.MONEY, 0)
			self.actual_score = 0
			self.actual_cost = 0
			print "Money Insufficient. Intend will do nothing."
			return 

	# for selection_method we need to check whether the remaining goals can be done with time 
	# remaining
	if self.method == "selection_method":
		money = self.mem.get(self.mem.MONEY)
		cost = 0
		if self.selected_goals:
			for goal in self.selected_goals[0]:
					cost = cost + self.cost_dishes[goal.args[1]]
			if cost > money:
				print("Remaining goals for the order " + str(self.selected_goals[0][0].args[0]) + " Cannot be done")
				self.selected_goals[:] = []
				self.actual_score = 0
				self.mem.set(self.mem.CURRENT_GOALS, None)
				print("cost" + str(self.actual_cost))
				print("score" + str(self.actual_score))
		
	# if there are current goals or pending goals then execute them
	# else folow selection
	if  self.mem.get(self.mem.CURRENT_GOALS) or self.selected_goals:
	    goals = self.check_existing_goals()

	if not goals:
		# get all the goals from the root of the goal graph
        	goals = goalGraph.getUnrestrictedGoals()
		# select the goals based on the score,time and cost
		if self.method == "selection_method":
			goals = self.selection_method(goals)
		else:
			goals = self.fifo_selection (goals)
		# add it to th current goal in memory
        	self.mem.set(self.mem.CURRENT_GOALS, goals)

	# Time remaining
	if goals:
		print("Money Remaining: " + str(self.mem.get(self.mem.MONEY)))
	else:
		self.mem.set(self.mem.CURRENT_GOALS, [])
		print("Money Remaining: " + str(self.mem.get(self.mem.MONEY)))
		self.mem.set(self.mem.MONEY, 0)
		self.selected_goals[:] = []
		self.actual_score = 0
		self.actual_cost = 0

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print



class SimpleIntend_construction_goal_transformation(base.BaseModule):

    goals_goalgraph = []
    building_list = []
    current_goals_copy = []
    score = 0

    def remove_goals_from_goalgraph(self,goal):
	for goals in self.goals_goalgraph:
		if goal.args == goals.args and goal.kwargs["predicate"] == goals.kwargs["predicate"]:
			self.goals_goalgraph.remove(goals)
	
	return
				
    def check_arguments(self,arg , building):
	goals = copy.deepcopy(self.goals_goalgraph)
	for goal in goals:
		if arg in goal.args:
			building.append(goal)
			self.remove_goals_from_goalgraph(goal)
			self.check_arguments(goal.args[0] , building)
	return


    def building_blocks(self):	
	building = []
	goals = copy.deepcopy(self.goals_goalgraph)
	for goal in goals:
		if(goal.kwargs["predicate"] == "on-table"):
			building[:] = []
			building.append(goal)
			self.remove_goals_from_goalgraph(goal)
			self.check_arguments(goal.args[0],building)
			self.building_list.append(copy.deepcopy(building))
	return 

    def calculate_score(self):
	'''
	input: get the global variable current_goals_copy
	output : update the score ; for mortar it is 2 , else 1
	'''
	executed_buildings = []
	if self.mem.get(self.mem.ACTUAL_SCORES):
		self.score = self.mem.get(self.mem.ACTUAL_SCORES)
	else:
		self.score = 0
	for goal in self.current_goals_copy :
		if goal["predicate" ] == "on":
			self.score = self.score + 1
		if goal["predicate"] == "stable-on":
			self.score = self.score + 2
		if goal["predicate"] == "on-table":
			temp = [goal.args[0] , len(self.current_goals_copy)]
			executed_buildings.append(tuple(temp))
	self.current_goals_copy[:] = []
	self.mem.set(self.mem.ACTUAL_SCORES , self.score)
	if self.mem.get(self.mem.EXECUTED_BUILDING_LIST):
		buildings = copy.deepcopy(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))
		buildings.append(executed_buildings)
	else:
		buildings = copy.deepcopy(executed_buildings)	
	self.mem.set(self.mem.EXECUTED_BUILDING_LIST, buildings)

    def selected_goals_to_memory(self):
	'''
	input : selected goals to memory variable select_building_list
	'''
	selected_buildings = []
	for each in self.building_list:
		for goal in each:
			if goal["predicate"] == "on-table":
				temp = [goal.args[0] , len(each)]
				selected_buildings.append(tuple(temp))
				break
	self.mem.set(self.mem.SELECTED_BUILDING_LIST , selected_buildings)
		
    def run(self, cycle, verbose = 2):
        trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

	goals = goalGraph.getUnrestrictedGoals()
	

	
        if not goalGraph or not goals:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
		if not self.mem.get(self.mem.TIME_CONSTRUCTION):
	    		self.goals_goalgraph[:] = []
    			self.building_list[:] = []
			if self.current_goals_copy:
    				self.current_goals_copy[:] = []
			else:
				self.current_goals_copy = []
    			self.score = 0
		else:
			if self.current_goals_copy:
				self.calculate_score()
			self.goals_goalgraph[:] = []
    			self.building_list[:] = []
    			self.current_goals_copy[:] = []
    			self.score = 0
			
            return

	if self.mem.get(self.mem.CURRENT_GOALS):
		self.current_goals_copy = copy.deepcopy(self.mem.get(self.mem.CURRENT_GOALS))
		goals = self.mem.get(self.mem.CURRENT_GOALS)
	else:
			
	        if self.current_goals_copy:
			self.calculate_score()

		if not self.building_list:
			# get all the goals from the root of the goal graph
			self.score = 0
        		goals = goalGraph.getUnrestrictedGoals()
			self.goals_goalgraph = copy.deepcopy(goals)
			self.building_blocks()
			if goals:
				self.selected_goals_to_memory()
		# take the first building
		if self.building_list:
			goals = self.building_list.pop(0)
			
		self.current_goals_copy = copy.deepcopy(self.mem.get(self.mem.CURRENT_GOALS))

		# add it to the current goal in memory
        	self.mem.set(self.mem.CURRENT_GOALS, goals)

        if trace:
            trace.add_data("GOALS",goals)

        if not goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
                print "Selecting goal(s):",
                for goal in goals:
                    print goal,
                print

class SimpleIntend_construction(base.BaseModule):

    goals_goalgraph = []
    goals_in_goalgraph = []
    building_list = []
    selected_goals = []
    t = [1 , 2.2 , 3.4 , 5.4 , 8.4 , 13.4 , 22.4 , 42.4]
    buildings_scores = {'b' : [] , 't' : [] }
    combinations_t = { }
    combinations_b = { }
    selected_buildings = []
    time = None # for changing time very 20s
    time_change = False
    count = -1
    T = 0
    new_time = None
    goals = []
    actual_scores = 0
    new = False
    cycle = 0
    p_list = []
    t_list = []

    #def build_building_from_goals(base,goals):


    def remove_goals_from_goalgraph(self,goal):
	for goals in self.goals_goalgraph:
		if goal.args == goals.args and goal.kwargs["predicate"] == goals.kwargs["predicate"]:
			self.goals_goalgraph.remove(goals)
	
	return
				
    def check_arguments(self,arg , building):
	goals = copy.deepcopy(self.goals_goalgraph)
	for goal in goals:
		if arg in goal.args:
			building.append(goal)
			self.remove_goals_from_goalgraph(goal)
			self.check_arguments(goal.args[0] , building)
	return


    def building_blocks(self):	
	building = []
	goals = copy.deepcopy(self.goals_goalgraph)
	for goal in goals:
		if(goal.kwargs["predicate"] == "on-table"):
			building[:] = []
			building.append(goal)
			self.remove_goals_from_goalgraph(goal)
			self.check_arguments(goal.args[0],building)
			self.building_list.append(copy.deepcopy(building))
			
			
	return 


    def check_current_goals_cant_be_acheived(self):
	time = 0
	world = self.mem.get(self.mem.STATES)[-1]
	current_goals = self.mem.get(self.mem.CURRENT_GOALS)
	for i,goal in enumerate(current_goals):
		achieved = world.atom_true(world.midcaGoalAsAtom(goal))
		if not achieved:
			time = self.t[i]
	if time > self.T:
		return True
	else:
		return False		

    def run(self, cycle, verbose = 2):
	self.goals_goalgraph = [] # store all goals in the goalgraph 
    	self.goals_in_goalgraph = [] # copy of self.goals_goalgraph
    	self.building_list = [] # goals categorized as building goals
	# time 
    	self.t = [1 , 2.2 , 3.4 , 5.4 , 8.4 , 13.4 , 22.4 , 42.4]
	self.t_copy = [1 , 1.2 , 1.2 , 2 , 3 , 5 , 9 , 20]
	# b and time for the buildings 
    	self.buildings_scores = {'b' : [] , 't' : [] }
    	self.combinations_t = { }
    	self.combinations_b = { }
	self.selected_goals = []
	#self.selected_buildings = []
	self.T = self.mem.get(self.mem.TIME_CONSTRUCTION)
	

	time_variation = 0.2
	time_change = 0.5
	self.implement ="selection"
	#self.implement ="stack"
        


	trace = self.mem.trace
        if trace:
            trace.add_module(cycle,self.__class__.__name__)
            trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        goalGraph = self.mem.get(self.mem.GOAL_GRAPH)

	if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
		if verbose >= 1:
        	        print "No Goals in Goal graph. Intend will do nothing."
       		        return

        if not goalGraph:
            if verbose >= 1:
                print "Goal graph not initialized. Intend will do nothing."
            return




	# to check whether the goals are acheived or not.
	world = self.mem.get(self.mem.STATES)[-1]

	# calculate time
	if not self.time:
		self.time = ctime.time()

	# calculate cycles, for every cycle greater than 10
	# randomly change the construction time
	'''
	self.cycle = self.cycle + 1
	if self.cycle >= 10:
		self.cycle = 0
		self.T = random.uniform(time_change*(self.T), self.T + time_change*(self.T))
		print("Time is changed : " + str(self.T))
		self.mem.set(self.mem.TIME_CONSTRUCTION, self.T)
	'''
	
	if self.mem.get(self.mem.CURRENT_GOALS) and self.T :
	  if not len(self.mem.get(self.mem.CURRENT_GOALS)) == 0:
		# check the acheived goals in current goals
		# if all goals acheived, temp = 0
		# add it to the executed_building_list in memory
		# i indicates how many goals acheived
		# self.count and i are same, except that when they are not equal a goal is acheived
		temp = 0
		i = 0
		for i,goal in enumerate(self.mem.get(self.mem.CURRENT_GOALS)):
			achieved = world.atom_true(world.midcaGoalAsAtom(goal))
			if not achieved:
				temp = 1
				break

		if temp == 1:
			i= i-1

		if temp == 0:
			self.count = -100
			temp_build = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
			if temp_build:
				temp_build.append(len(self.mem.get(self.mem.CURRENT_GOALS)))
				self.mem.set(self.mem.EXECUTED_BUILDING_LIST , temp_build )
				print(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))
			else:
				self.mem.set(self.mem.EXECUTED_BUILDING_LIST , [len(self.mem.get(self.mem.CURRENT_GOALS))])
				print(self.mem.get(self.mem.EXECUTED_BUILDING_LIST))



		# if zone is -1, then re- initialize self.count to 0
		# if there is no sufficient time, then give a hint to eval to remove all goals
		zone = 0
		if not (self.count == i):
			random.seed(i)
			action_score = random.uniform(1 - time_variation*1,1 + time_variation*1)	
                        action_time = random.uniform(self.t_copy[i] - time_variation*self.t_copy[i], self.t_copy[i] + time_variation*self.t_copy[i])
                        print("Actual Time Taken For Previous Action :" + str(action_time))
                        print("Expected Time Taken For Previous Action :" + str(self.t_copy[i]))
			print("Actual Score for Previous Action : " + str(action_score))

			#raw_input("enter")
			self.actual_scores = self.actual_scores + action_score
			actual_time  = self.mem.get(self.mem.ACTUAL_TIME_CONSTRUCTION)
			temp_p = round(action_score,3)
			temp_t = round(action_time,3)
			self.p_list.append(temp_p)
			self.t_list.append(temp_t)
			if actual_time:
				 	actual_time = actual_time + action_time 
				 	self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION ,actual_time )
			else:
					self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION ,action_time )
                        self.T = self.T - action_time
			if self.count == -100:
				 #print("Zone 1")
				 zone = -1

			if self.check_current_goals_cant_be_acheived():
				if self.implement:
					if self.implement is "selection": 
						self.p_list = []
						self.t_list = []
						self.mem.set(self.mem.CURRENT_GOALS , [])
						#raw_input(self.count)
						self.count = -1
						self.actual_scores = 0
						del self.selected_buildings


			if self.T < 0:
				#print("Zone 2") 

				if temp == 0:
					temp_build = self.mem.get(self.mem.EXECUTED_BUILDING_LIST)
					if temp_build:
						temp_build.pop()
						self.mem.set(self.mem.EXECUTED_BUILDING_LIST , temp_build)

				self.T = 0
				self.actual_scores = 0
				if actual_time:
					self.mem.set(self.mem.ACTUAL_TIME_CONSTRUCTION , actual_time - action_time)
				print("No Goals are selected due to less time .Abandoning Goals in next cycle.")
				self.mem.set(self.mem.CURRENT_GOALS , [])
				self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
				self.p_list = []
				self.t_list = []
				self.goals =[]
				if self.selected_buildings:
					del self.selected_buildings
				self.mem.set(self.mem.TIME_CONSTRUCTION , self.T)
				self.count = -1
				if zone == -1:
					self.count = -1
				return
			P = self.mem.get(self.mem.P)
			t = self.mem.get(self.mem.t)
				

			if P :
			    if self.p_list:
				if not self.p_list in P:
					P.append(self.p_list)
			        	self.mem.set(self.mem.P,P)
			else:
			    if self.p_list:
			    	self.mem.set(self.mem.P,self.p_list)

			if t :
			    if self.t_list:
				if not self.t_list in t:
			    		t.append(self.t_list)
			    		self.mem.set(self.mem.t,t)
			else:
			    if self.t_list:
			    	self.mem.set(self.mem.t,self.t_list)


			self.mem.set(self.mem.TIME_CONSTRUCTION , self.T)
			self.count = i
			print("scores and time")
			if zone == -1:
				if self.mem.get(self.mem.ACTUAL_SCORES):
					score = self.mem.get(self.mem.ACTUAL_SCORES) + self.actual_scores
					self.mem.set(self.mem.ACTUAL_SCORES , score)
				else:
					self.mem.set(self.mem.ACTUAL_SCORES , self.actual_scores)
				self.actual_scores = 0
				self.p_list = []
				self.t_list = []
				self.count = -1



			
	print("Time Remaining : " + str(self.T))


	
	if ( (ctime.time() - self.time) > 120) and (self.T >= 0):
		self.time = ctime.time()
		self.T = random.uniform(0.7*(self.T), self.T)
		#self.time_change = True
		print("")
		print("Time is changed & Time Remaining is: " + str(self.T)) 
		self.mem.set(self.mem.TIME_CONSTRUCTION, self.T)
		print("")	
			
	

	# get all the goals from the goal graph
        self.goals_goalgraph = goalGraph.getUnrestrictedGoals()
    
	# Just a copy of all the goals from goal graph
	self.goals_in_goalgraph = copy.deepcopy(self.goals_goalgraph )
	

	# if there are current goals to acheive, let it complete it's execution
	if  self.mem.get(self.mem.CURRENT_GOALS) :
	   if not len(self.mem.get(self.mem.CURRENT_GOALS)) == 0:
		print("Not all Current goals achieved ,")
		print("Processing" + str(self.mem.get(self.mem.CURRENT_GOALS)))
		return

	# This function gets the goals as the building goals, and writes to the complete_building_list
	self.building_blocks()
	temp_building_list = []
	if not self.mem.get(self.mem.COMPLETE_BUILDING_LIST):
		for each in self.building_list:
			temp_building_list.append(len(each))
		self.mem.set(self.mem.COMPLETE_BUILDING_LIST , temp_building_list)
	
	
        print ("Given Building Goals")
	for i in range(0,len(self.building_list)):
	        print("")
		print("Tower " + self.building_list[i][0].args[0])
		print ("[")
		for j in self.building_list[i]:
			print(j)
		print("]")
		print("")
	print("")
	

	p_t = [] # This is list variable with (P/T) scores of all buildings
	towers = []
	temp = []
	# this variable contains the buildings that are selected
	self.selected_buildings = self.building_list

	
	for i in range(0,len(self.building_list)):
		self.buildings_scores['t'].append(self.t[len(self.building_list[i])-1])
		self.buildings_scores['b'].append(len(self.building_list[i])/self.t[len(self.building_list[i])-1])
		p_t.append(len(self.building_list[i])/self.t[len(self.building_list[i])-1])

	# when there is no time limit
	# self.T is the time that you specify on Midca/examples/cogsci_demo_mortar_construction.py
	
	
	if self.implement is "stack":
		temp = []
		towers = []
		score = 0
		time = 0
		for i in range(0,len(self.building_list)):
					score = score + len(self.building_list[i])
					time = time + self.t[len(self.building_list[i])-1]
					if time < self.T:
						temp.append(self.building_list[i])
						towers.append(self.selected_buildings[i][0].args[0])
					else:
						break

		
					
		# Finally the selected buildings are the one's stored in temp
		if not temp:
			print("No Goals are selected due to less time . Removing all goals from Goal Graph During Next Evaluate Phase.")
			self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
			self.mem.set(self.mem.CURRENT_GOALS , [])
			if self.goals:
                            del self.goals
                            del self.selected_buildings
			self.count = -1
			return
		else:
			score = 0
			time = 0
			self.selected_buildings = temp
			self.selected_goals = self.selected_buildings[0]
			for i in range(0,len(self.selected_buildings)):
				score = score + len(self.selected_buildings[i])
				time = time + self.t[len(self.selected_buildings[i])-1]
			print("Selected Towers are : " + str(towers))
			print("")
			length = [len(each) for each in self.selected_buildings]
			if not self.mem.get(self.mem.SELECTED_BUILDING_LIST):
				self.mem.set(self.mem.SELECTED_BUILDING_LIST , length)
			print("Overall Score : " + str(score))
			print("Overall Time is : " + str(time))
			if not self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION):
				self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , time)
			print("")	

	elif not self.T:
		minimum_score = min(self.buildings_scores['b'])
		#print("P/T is " + str(minimum_score))
		index = self.buildings_scores['b'].index(minimum_score)
		#print("time is " + str(self.buildings_scores['t'][index]))
		#self.selected_goals = self.building_list[index]
		#self.selected_buildings.append(self.building_list[index])
		#print("Score is " + str(len(self.building_list[index])))


		p_t.sort()
		temp = []
		towers = []
		# sort the buildings according to the highest P/T scores and store it in temp
		# Get the first argument of the first building goal which will be the name of the tower
		# store all the names of the towers in towers[] variable in the order of their P/T scores

		for each in p_t:
			for i in range(0,len(self.selected_buildings)):
				if each == (len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]):
					temp.append(self.selected_buildings[i])
					towers.append(self.selected_buildings[i][0].args[0])
					
		# Finally the selected buildings are the one's stored in temp
		self.selected_buildings = temp
		self.selected_goals = self.selected_buildings[0]
		print("Selected Towers are : " + str(towers))
		print("")

	# when there is  time limit
	else:
		self.selected_buildings = []
		a = []
		for i in range(0,len(self.buildings_scores['t'])):
			a.append(i)
		for i in xrange(1,len(a)+1):
			c = list(itertools.combinations(a,i))
			for every in c:
				sum=0
				name = ""
				for each in every:
					sum = sum + self.buildings_scores['t'][each]
					name = name + str(each) + ","
				name = name[:-1]
				if not (sum > self.T):
					self.combinations_t[name] = sum
		sorted_x = sorted(self.combinations_t.items(), key=operator.itemgetter(1))

		max_length = 0
		for every in sorted_x:
			name = every[0].split(",")
			if(max_length < len(name)):
				max_length = len(name)

		for every in sorted_x:
			name = every[0].split(",")
			sum=0
			if(len(name) == max_length):
				for each in name:
					sum = sum + self.buildings_scores['b'][int(each)]
				
				self.combinations_b[every[0]] = sum

		
		sorted_x = sorted(self.combinations_b.items(), key=operator.itemgetter(1))

		if(len(sorted_x) == 0):
			print("No Goals are selected due to less time . Removing all goals from Goal Graph During Next Evaluate Phase.")
			print(" ")
			self.count = -1
			temp_goals = []
			for goals in self.goals_in_goalgraph:
				temp_goals.append(goals)
			self.mem.set(self.mem.REJECTED_GOALS,["remove all"])
			self.mem.set(self.mem.CURRENT_GOALS , [])
			if self.goals:
                            del self.goals
                            del self.selected_buildings
			return
			
			

		#print(sorted_x)
		score = 0;
		p_t = []
		towers = []
		time = 0
		for every in sorted_x[0]:
			name = every.split(",")
			for each in name:
				self.selected_buildings.append(self.building_list[int(each)])
			        p_t.append(len(self.building_list[int(each)])/self.t[len(self.building_list[int(each)])-1])
				score = score + len(self.building_list[int(each)])
				#for goal in (self.building_list[int(each)]):
				#	self.selected_goals.append(goal)
			#print("P/T is " + str(self.combinations_b[every]))
			time =  time + self.combinations_t[every]
			#print("Score is " + str(self.combinations_b[every] * self.combinations_t[every]))
			
			break

		p_t.sort()
		p_t.reverse()
		temp = []
		towers = []
		for each in p_t:
			for i in range(0,len(self.selected_buildings)):
				if each == (len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]):
				  if not self.selected_buildings[i][0].args[0] in towers:
					temp.append(self.selected_buildings[i])
					towers.append(self.selected_buildings[i][0].args[0])

					
		length = [len(each) for each in self.selected_buildings]
		if not self.mem.get(self.mem.SELECTED_BUILDING_LIST):
			self.mem.set(self.mem.SELECTED_BUILDING_LIST , length)
		self.selected_buildings = temp
		print("Selected Towers are : " + str(towers))
		print("Overall Score : " + str(score))	
		print("Overall Time is : " + str(time))
		if not self.mem.get(self.mem.EXPECTED_TIME_CONSTRUCTION):
			self.mem.set(self.mem.EXPECTED_TIME_CONSTRUCTION , time)
		print("")	


		
				
		selected_goals_mem = []
		if self.selected_buildings:	
		   #print("Selected Buildings")
		   for each in self.selected_buildings:
			for goal in each:
				selected_goals_mem.append(goal)
			#print("")
		   self.selected_goals = self.selected_buildings[0]
                   #self.mem.set(self.mem.SELECTED_GOALS, selected_goals_mem)

	'''

	for i in range(0,len(self.building_list)):
		for j in self.building_list[i]:
			print(j)
		print("")
	'''
			
	self.mem.set(self.mem.CURRENT_GOALS, copy.deepcopy(self.selected_goals))
	#print(self.mem.get(self.mem.CURRENT_GOALS))

        if trace:
            trace.add_data("GOALS",copy.deepcopy(self.selected_goals))
	    trace.add_data("GOALGRAPH",copy.deepcopy(self.mem.GOAL_GRAPH))

        if not self.selected_goals:
            if verbose >= 2:
                print "No goals selected."
        else:
            if verbose >= 2:
		print("Time is " + str(self.T))
		print("")
                print "Selecting Tower(s):",
		#print("")
		
		
		for i in range(0,len(self.selected_buildings)):
				print("")
				print("Tower " + self.selected_buildings[i][0].args[0] ),
				if(len(self.selected_buildings[i]) == 1):
					print("(" + str(len(self.selected_buildings[i])) + " Block)"),
				else:
					print("(" + str(len(self.selected_buildings[i])) + " Blocks)"),
				print(" with priority : " + str(i+1)),
				print ("["),
				#for goals in self.selected_buildings[i]:
				#	print(goals)
				#print("]")
				#print("")
				print("Time(T) :" + str(self.t[len(self.selected_buildings[i])-1]) + ","),
				print("Score(P) :" + str(len(self.selected_buildings[i])) + ","),
				print("P/T :" + str(len(self.selected_buildings[i])/self.t[len(self.selected_buildings[i])-1]) + " ]"),
		print("")

		
		self.goals = copy.deepcopy(self.selected_buildings[0])
		del self.selected_buildings[0]
			
		'''
                for goal in self.selected_goals:
                    print goal,
                print
		'''
		print("")


