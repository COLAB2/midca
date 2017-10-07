import worldsim

types = {"obj": worldsim.Type("obj", [])}
objects = {}
predicates = {}
atoms = []
operators = {}
cltree = {"rootnode": "" , "allnodes" : [] , "checked" : [] } 
obtree = {"rootnode": "" , "allnodes" : [] , "checked" : [] }

def type(name, parentnames = ["obj"]):
	temp = [name]
	if not parentnames == ["obj"]:
		temp.append(parentnames)
	if isinstance(parentnames, basestring):
		parentnames = [parentnames]
	parents = []
	for parent in parentnames:
		if parent not in types:
			raise Exception("parent type DNE.")
		parents.append(types[parent])
	types[name] = worldsim.Type(name, parents)	
	otree = worldsim.ObjectTree(obtree['rootnode'] , 
				    obtree['allnodes'], 
				    obtree['checked'] , 
				    temp)
	obtree['rootnode'] = otree.rootnode
	obtree['allnodes'] = otree.allnodes
	obtree['checked'] = otree.checked


def ptype(*args):
	'''
	Create a class hierarchy tree and get the result into cltree,
        which is a global variable inorder to store previous nodes of tree.
	'''
	temp = list(args)
	tree = worldsim.Tree(cltree['rootnode'] , cltree['allnodes'], cltree['checked'] , temp)
	cltree['rootnode'] = tree.rootnode
	cltree['allnodes'] = tree.allnodes
	cltree['checked'] = tree.checked


def instance(name, typename):
	if typename not in types:
		raise Exception("object type DNE.")
	objects[name] = types[typename].instantiate(name)

def predicate(name, argnames, argtypenames = []):
	argtypes = []
	for typename in argtypenames:
		if typename not in types:
			raise Exception("object type DNE.")
		argtypes.append(types[typename])
	predicates[name] = worldsim.Predicate(name, argnames, argtypes)

def statement(predicatename, argnames):
	if predicatename not in predicates:
		raise Exception("predciate DNE.")
	args = []
	for argname in argnames:
		if argname not in objects:
			raise Exception("object DNE.")
		args.append(objects[argname])
	atoms.append(predicates[predicatename].instantiate(args))

class Cond:
	
	def __init__(self, predicate, argnames, positive):
		self.predicate = predicate
		self.argnames = argnames
		self.positive = positive

def condition(predicatename, args = [], negate = False):
	if predicatename not in predicates:
		raise Exception("predicate "+str(predicatename)+" DNE.")
	return Cond(predicates[predicatename], args, not negate)
	
#args is a dict: {argname: argtypename}
def operator(name, args = [], preconditions = [], results = []):
	objnames = []
	argtypes = {}
	for argname, argtype in args:
		objnames.append(argname)	
		if argtype not in types:
			raise Exception("object type DNE.")
		argtypes[argname] = types[argtype]
	prepredicates = []
	preobjnames = []
	preobjtypes = []
	prePositive = []
	for condition in preconditions:
		for argname in condition.argnames:
			if argname not in objnames:
				raise Exception("condition argument not listed as an object for this operator")
		prepredicates.append(condition.predicate)
		preobjnames.append(condition.argnames)
		objtypes = []
		for objname in preobjnames[-1]:
			objtypes.append(argtypes[objname])
		preobjtypes.append(objtypes)
		prePositive.append(condition.positive)
	postpredicates = []
	postobjnames = []
	postobjtypes = []
	postPositive = []
	for condition in results:
		for argname in condition.argnames:
			if argname not in objnames:
				raise Exception("condition argument not listed as an object for this operator")
		postpredicates.append(condition.predicate)
		postobjnames.append(condition.argnames)
		objtypes = []
		for objname in postobjnames[-1]:
			objtypes.append(argtypes[objname])
		postobjtypes.append(objtypes)
		postPositive.append(condition.positive)
	operators[name] = worldsim.Operator(name, objnames, prepredicates, preobjnames, preobjtypes, prePositive, postpredicates, postobjnames, postobjtypes, postPositive)

def preprocess(text):
	i = 0
	corrected = ""
	endChars = [",", ")", "]"]
	startChars = ["\n", " ", "\t", "[", "(", ","]
	while True:
		newI = len(text)
		for char in endChars:		
			if char in text[i:]:
				newI = min(text.index(char, i), newI)
		newI
		if newI == len(text):
			break
		backI = newI - 1
		while text[backI] not in startChars and backI > i - 1:
			backI -= 1
		if text[backI] not in startChars:
			corrected += text[i:newI + 1]
			i = newI + 1
			continue
		if text[backI] != "[" or newI != backI + 1:
			corrected += text[i:backI + 1] + "\"" + text[backI + 1:newI] + "\"" + text[newI]
		else:
			corrected += "[]"
		i = newI + 1
	return corrected.replace("\"False\"", "False").replace("\"True\"", "True") 

def load_domain(filename):
	f = open(filename)
	exec preprocess(f.read())
	f.close()
	world = worldsim.World(operators.values(), predicates.values(), atoms, types, objects.values(),cltree,obtree)
	return world

def load_domain_str(str):
	exec preprocess(str)
	world = worldsim.World(operators.values(), predicates.values(), atoms, types, objects.values())
	return world

def operator_no_side_effect(name, args = [], preconditions = [], results = []):
	'''
	Just like operator function above, except doesn't save the operator into the global operators, instead
	returns the operator
	'''
	objnames = []
	argtypes = {}
	for argname, argtype in args:
		objnames.append(argname)	
		if argtype not in types:
			raise Exception("object type DNE.")
		argtypes[argname] = types[argtype]
	prepredicates = []
	preobjnames = []
	preobjtypes = []
	prePositive = []
	for condition in preconditions:
		for argname in condition.argnames:
			if argname not in objnames:
				raise Exception("condition argument not listed as an object for this operator")
		prepredicates.append(condition.predicate)
		preobjnames.append(condition.argnames)
		objtypes = []
		for objname in preobjnames[-1]:
			objtypes.append(argtypes[objname])
		preobjtypes.append(objtypes)
		prePositive.append(condition.positive)
	postpredicates = []
	postobjnames = []
	postobjtypes = []
	postPositive = []
	for condition in results:
		for argname in condition.argnames:
			if argname not in objnames:
				raise Exception("condition argument not listed as an object for this operator")
		postpredicates.append(condition.predicate)
		postobjnames.append(condition.argnames)
		objtypes = []
		for objname in postobjnames[-1]:
			objtypes.append(argtypes[objname])
		postobjtypes.append(objtypes)
		postPositive.append(condition.positive)
	return worldsim.Operator(name, objnames, prepredicates, preobjnames, preobjtypes, prePositive, postpredicates, postobjnames, postobjtypes, postPositive)

def load_operator_str(op_str):
	return eval(preprocess(op_str))

def to_shop2_domain(world, name):
	strs = ["(in-package :shop2)\n\n"]
	strs.append("(defdomain " + name)

def parenthetical(text):
	return text.strip()[0] == "(" and text.strip()[-1] == ")"

def get_tokens(text):
	if parenthetical(text):
		text = text.strip()[1:-1]
	i = 0
	begin = i
	open = 0
	tokens = []
	while i < len(text):
		nextOpen = len(text)
		nextClosed = len(text)
		if "(" in text[i:]:
			nextOpen = text.index("(", i)
		if ")" in text[i:nextOpen]:
			nextClosed = text.index(")", i)
		if nextOpen < nextClosed: #open paren happens first
			if open == 0:
				tokens.append(text[begin:nextOpen])
				begin = nextOpen
			i = nextOpen + 1
			open += 1
		elif nextClosed < nextOpen: #close paren happens first
			open -= 1
			if open == 0:
				tokens.append(text[begin:nextClosed + 1])
				begin = nextClosed + 1
			i = nextClosed + 1
		else:
			i = len(text) #end
		if i == len(text):
			tokens.append(text[begin:])
	tokens = [token for token in tokens if token.strip() != ""]
	if len(tokens) == 1:
		return tokens[0]
	return tokens

def get_outer_tokens(text):
	return get_tokens("(" + text + ")")

def tokenize(outertoken):
	tokens = get_tokens(outertoken)
	if isinstance(tokens, basestring):
		tokens = tokens.split(" ")
		for token in range(len(tokens)):
			tokens[token] = tokens[token].strip()
		return [token for token in tokens if token != ""]
	tokenization = []
	for token in tokens:
		tokenization.append(tokenize(token))
	return tokenization

def remove_comments(text, commentChar):
	i = 0
	newtext = ""
	while commentChar in text[i:] and i < len(text):
		newtext += text[i:text.index(commentChar, i)]
		i = text.index(commentChar, i)
		if "\n" in text[i:]:
			i = text.index("\n", i)
		else:
			i = len(text)
	return newtext + text[i:]
				
def load_shop2(operators):
	for operator in operators:
		name = operator[1][0]
		args = operator[1][0:]
		
'''	
str = open("./blocks-htn.lisp").read()
no_comments = remove_comments(str, ";")
tokens = []
operators = []
for token in get_outer_tokens(no_comments):
	tokens.append(tokenize(token))
for token in tokens[1][1]:
	if token[0][0] == ":operator":
		print token

two = get_outer_tokens(no_comments)[1]
x = get_tokens(two)
y = x[1]
z = get_tokens(y)
print len(z)
print z[16]

for token in tokens:
	print len(token)
	print token.__class__
'''
