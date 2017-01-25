from MIDCA.domains.logistics.orderpackage import Order
from MIDCA.worldsim import worldsim


def get_order_list(world):
	orders = {}
	
	for atom in world.atoms:
		
		if atom.predicate == world.predicates["deliver"]: # or :
			order = Order(atom.args[0].name)
			order.destination = atom.args[1].name
			return order
		
	return ''

