from midca.domains.logistics.orderpackage import Order
from midca.worldsim import worldsim
from threading import Thread

def get_order_list(world):
	orders = []
	packages = {}
	for atom in world.atoms:
		if atom.predicate == world.predicates["obj-at"]:
			packages.update({atom.args[0].name: atom.args[1].name})
			
	for atom in world.atoms:		
		if atom.predicate == world.predicates["deliver"]: # or :
			package1 = atom.args[0].name
			order = Order(package1)
			order.destination = atom.args[1].name
			if package1 in  packages.keys():
				order.location = packages[package1]
				orders.append(order)
				
	
	return orders
		
def get_warehouse_list(orders):
	#this keeps the warehouse information. location: [list of orders]
	w = {}
	
	for order in orders:
		if order.location in w.keys():
			w[order.location].add(order)
			
		else:
			w = {order.location: [order]}
		
		
	return w