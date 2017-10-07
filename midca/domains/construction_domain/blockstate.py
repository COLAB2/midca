from midca.domains.blocksworld.block import Block
from midca.worldsim import worldsim

def b(name):
	return "b" + str(name)

def get_block_list(world):
	blocks = {}
	for obj in world.objects.values():
		if obj.type.name != "BLOCK":
			continue
		block = None
		if world.atom_true(worldsim.Atom(world.predicates["table"], [obj])):
			block = Block(Block.TABLE, obj.name)
		elif world.atom_true(worldsim.Atom(world.predicates["triangle"], [obj])):
			block = Block(Block.TRIANGLE, obj.name)
		elif world.atom_true(worldsim.Atom(world.predicates["block"], [obj])):
			block = Block(Block.SQUARE, obj.name)
		if not block:
			continue
		block.clear = False
		block.on = None
		block.onfire = False
		block.hasmortar = False
		if block.type == block.TABLE:
			table = block
		blocks[obj.name] = block
	for atom in world.atoms:
		try: # some domains won't use stable-on, so if we get a KeyError just ignore
			if atom.predicate == world.predicates["stable-on"]:
				blocks[atom.args[0].name].on = blocks[atom.args[1].name]
		except KeyError:
			pass
		
		if atom.predicate == world.predicates["on"]: # or :
			blocks[atom.args[0].name].on = blocks[atom.args[1].name] 
		elif atom.predicate == world.predicates["on-table"]:
			blocks[atom.args[0].name].on = table
		elif atom.predicate == world.predicates["clear"]:
			blocks[atom.args[0].name].clear = True
		elif atom.predicate == world.predicates["onfire"]:
			blocks[atom.args[0].name].onfire = True
		elif "hasmortar" in world.predicates.keys() and atom.predicate == world.predicates["hasmortar"]:
			blocks[atom.args[0].name].hasmortar = True
	return sorted(blocks.values(), key = lambda x: x.id)

