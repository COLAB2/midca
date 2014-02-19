import worldsim as plan, domainread as domain_read

def apply_state_file(world, filename):
	lines = open(filename).readlines()
	lineNum = 0
	for line in lines:
		if "#" in line:
			line = line[:line.index("#")] #commnents
		if "(" in line:
			if ")" not in line:
				raise Exception("Line " + str(lineNum) + ": Declarations must be contained on single line")
			call = line[:line.index("(")].strip()
			argnames = line[line.index("(") + 1:line.index(")")].split(",")
			for i in range(len(argnames)):
				argnames[i] = argnames[i].strip()
			if call in world.predicates:
				args = []
				for name in argnames:
					if not name:
						continue
					if name not in world.objects:
						raise Exception("Line " + str(lineNum) + ": Object " + name + " DNE")
					args.append(world.objects[name])
				world.add_atom(world.predicates[call].instantiate(args))
			elif call in world.types:
				name = argnames[0]
				world.add_object(world.types[call].instantiate(name))
			elif len(line) > 0:
				raise Exception("Line " + str(lineNum) + ": invalid command")
		lineNum += 1
				

def set_state(world, filename):
	world.atoms = []
	world.objects = {}
	apply_state_file(world, filename)


