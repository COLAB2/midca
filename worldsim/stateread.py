import worldsim as plan, domainread as domain_read

#Note: this algorithm does not handle many potential user errors.
def _apply_state(world, lines):
	lineNum = 1
	for line in lines:
		if "#" in line:
			line = line[:line.index("#")] #comments
		if "(" in line:
			if ")" not in line:
				raise Exception("Line " + str(lineNum) + ": Declarations must be contained on single line: " + line)
			call = line[:line.index("(")].strip()
			argnames = line[line.index("(") + 1:line.index(")")].split(",")
			for i in range(len(argnames)):
				argnames[i] = argnames[i].strip()
			if call.startswith("!"):
				negate = True
				call = call[1:]
			else:
				negate = False
			if call in world.predicates:
				args = []
				for name in argnames:
					if not name:
						continue
					if name not in world.objects:
						
						raise Exception("Line " + str(lineNum) + ": Object - " + name + " DNE " + line)
					args.append(world.objects[name])
				atom = world.predicates[call].instantiate(args)
				#if len(args) > 0:
				#	print("just instantiated atom: "+str(atom)+" args[0]: "+str(args[0]))
				if negate:
					world.remove_atom(atom)
				else:
					world.add_atom(atom)
			elif call in world.types:
				name = argnames[0]
				if negate:
					if not world.remove_object(name):
						raise Exception("Line " + str(lineNum) + ": Tried to remove object " + name + " but there is no such object - " + line)
				else:
					world.add_object(world.types[call].instantiate(name))
			elif len(line) > 0:
				raise Exception("Line " + str(lineNum) + ": invalid command " + line)
		elif line.startswith("!"):
			name = line[1:]
			if not world.remove_object(name):
				raise Exception("Line " + str(lineNum) + ": Tried to remove object " + name + " but there is no such object - " + line)
		elif line.strip() != "":
			raise Exception("Line " + str(lineNum) + ": invalid command - " + line)
		lineNum += 1

def apply_state_str(world, s):
	lines = s.split("\n")
	_apply_state(world, lines)

def apply_state_file(world, filename):
	lines = open(filename).readlines()
	_apply_state(world, lines)				

def set_state(world, filename):
	world.atoms = []
	world.objects = {}
	apply_state_file(world, filename)


