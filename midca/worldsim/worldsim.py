import copy
import numbers

func_val_dict = {}


class Obj:

    def __init__(self, name, type):
        self.name = name
        self.type = type

    def is_a(self, type):
        return self.type.is_a_(type)

    def ancestors(self):
        return self.type.ancestors()

    def __str__(self):
        return self.name


class Function:
    def __init__(self, name, argnames, argtypes):
        self.name = name
        self.argnames = argnames
        self.argtypes = argtypes

    def __str__(self):
        return "(" + self.name + " " + self.argnames.__str__() + ")"

    def instantiate(self, args, val=[]):

        if val:
            return Atom(self, args, val)
        else:

            return Atom(self, args)


class Predicate_function:
    def __init__(self, op, args):
        self.name = op
        self.op = op
        self.args = args
        self.argnames = args[0].argnames if args else []
        self.argtypes = args[0].argtypes if args else []
        self.val = args[1] if args else []

    def __str__(self):
        return "(" + (self.op if self.op else " ") + " " + self.args.__str__() + ") "

    def instantiate(self, args):
        # (= (tool-in-hand) (tool-id ?tool) 10)
        if isinstance(self.args[1], numbers.Number):
            return self.args[0].instantiate(args[0]), self.args[1]

        else:

            arg = self.args[1].instantiate(args[1])


            return self.args[0].instantiate(args[0]), arg


class Constant:
    def __init__(self, val):
        self.val = val

    def __str__(self):
        return str(self.val)

    def __eq__(self, other):
        return isinstance(other, Constant) and self.val == other.val


class Type:

    def __init__(self, name, parents=[]):
        self.parents = parents
        self.name = name

    # optimize (i.e. save ancestor set) if necessary
    def ancestors(self):
        ancestors = {}
        for parent in self.parents:
            ancestors[parent] = True
            for ancestor in parent.ancestors():
                ancestors[ancestor] = True
        return ancestors

    def is_a_(self, type):
        return type == self or type in self.parents

    def instantiate(self, name):
        return Obj(name, self)

    def __str__(self):
        return "Type: " + self.name


class Atom:

    def __init__(self, predicate, args, val=None):
        if len(predicate.argnames) != len(args):

            raise Exception("Wrong number of args for " + predicate.name + " " + args.__str__())
        i = 0
        if predicate.argtypes:
            for arg in args:
                if not arg.is_a(predicate.argtypes[i]):
                    raise Exception("Instantiating argument " + predicate.argnames[
                        i] + " with " + arg.name + ", which is the wrong type of object")
                i += 1
        self.predicate = None
        self.func = None

        if type(predicate) is Predicate:
            self.predicate = predicate
            self.args = args
            self.hash = hash(predicate.name + str(list(map(str, args))))  # little expensive because of map, but only
        # happens at initialization
        else:
            self.func = predicate
            self.args = args
            self.val = val
            self.hash = hash(predicate.name + str(list(map(str, args))))

    def __getitem__(self, item):
        if item in self.predicate.argnames:
            return self.args[self.predicate.argnames.index(item)]
        try:
            return self.args[item]
        except Exception:
            raise Exception("value must be name or index of argument")

    def __str__(self):
        if self.predicate:
            s = self.predicate.name + "("
        else:
            s = self.func.name + "("
        for arg in self.args:
            s += arg.name + ", "
        if self.args:
            s = s[:-2]
        return s + ")"

    def __hash__(self):
        return self.hash

    def __eq__(self, other):
        return self.hash == other.hash

    def __ne__(self, other):
        return self.hash != other.hash


class Predicate:

    def __init__(self, name, argnames, argtypes=[]):
        if argtypes and len(argnames) != len(argtypes):
            raise Exception("argnames list must be same length as argtypes")
        self.name = name
        self.argnames = argnames
        self.argtypes = argtypes

    def instantiate(self, args):
        return Atom(self, args)

    def __str__(self):
        s = "Predicate: " + self.name + "("
        i = 0
        for arg in self.argnames:
            s += arg
            if self.argtypes:
                s += "[" + self.argtypes[i].name + "]"
            s += ", "
            i += 1
        if self.argnames:
            s = s[:-2]
        return s + ")"


class Action:

    def __init__(self, operator, preconds, prePos, results, postPos, funcPre=[], funcRes=[], prefuncpos=[],
                 postfuncpos=[]):
        self.operator = operator
        self.preconds = preconds
        self.prePos = prePos
        self.results = results
        self.postPos = postPos
        self.funcRes = funcRes
        self.funcPre = funcPre
        self.preFuncPos = prefuncpos
        self.postFuncPos = postfuncpos

    def set_args(self, args):
        '''
        TODO - make this an arg in the constructor
        '''
        self.args = args

    def __str__(self):
        s = "Action - " + self.operator.name + ":\nPreconditions: ["
        i = 0
        for condition in self.preconds:
            if not self.prePos[i]:
                s += "Not "
            s += str(condition) + " ; "
            i += 1
        if self.preconds:
            s = s[:-3]
        s += "]\nResults: ["
        i = 0
        for condition in self.results:
            if not self.postPos[i]:
                s += "Not "
            s += str(condition) + " ; "
            i += 1
        if self.results:
            s = s[:-3] + "]"
        return s


class Condition:

    def __init__(self, arg1, argtypes, op=None, arg2=None):
        self.arg1 = arg1
        self.argtypes = argtypes
        self.op = op
        self.arg2 = arg2  # this is an atom or number

    def instantiate(self, args):
        if self.argtypes:
            i = 0
            for arg in args:
                if type(self.argtypes[i]) is list:

                    for at in self.argtypes[i]:
                        if not arg.is_a(at):
                            raise Exception("Trying to instantiate " + arg.name + " as a " + at.name)
                else:
                    if not arg.is_a(self.argtypes[i]):
                        raise Exception("Trying to instantiate " + arg.name + " as a " + self.argtypes[i].name)
                i += 1

        if self.arg1.func:

            if type(self.arg2) is Atom:
                return self.arg1.func.instantiate([]), self.arg2.func.instantiate([args[-1]])

            return self.arg1.func.instantiate(args), self.arg2

        if self.arg1.predicate:
            return self.arg1.predicate.instantiate(args)

    def fits(self, args):
        if self.argtypes:
            i = 0
            for arg in args:
                if not arg.is_a(self.argtypes[i]):
                    return False
                i += 1
        if self.atom.predicate.argtypes:
            i = 0
            for arg in args:
                if not arg.is_a(self.atom.predicate.argtypes[i]):
                    return False
                i += 1
        for i in range(len(self.atom.args)):
            for i2 in range(i + 1, len(self.atom.args)):
                if self.atom[i] == self.atom[i2] and args[i] != args[i2]:
                    return False
        return True

    def __str__(self):
        if self.op and self.val:
            return str(self.op) + " " + str(self.atom) + " " + str(self.val)
        return str(self.atom)


class Operator:

    def __init__(self, name, objnames, prepredicates, preobjnames, preobjtypes, prePositive, postpredicates,
                 postobjnames, postobjtypes, postPositive, prefunc=[], prefuncnames=[], prefunctypes=[], postfunc=[],
                 postfuncnames=[], postfunctypes=[], prefuncpos=[], postfuncpos=[]):
        self.name = name
        self.objnames = objnames
        self.preconditions = {}
        self.precondorder = []
        self.prePos = prePositive
        self.types = preobjtypes + postobjtypes
        self.prefunc = prefunc
        self.postfunc = postfunc
        self.prefuncnames = prefuncnames
        self.postfuncnames = postfuncnames
        self.isevent = True if name.startswith("event") else False
        self.results = {}
        self.resultorder = []
        self.postPos = postPositive
        self.prefuncpos = prefuncpos
        self.postfunpos = postfuncpos

        for pred in range(len(prepredicates)):
            args = []
            usednames = []

            names = preobjnames[pred]
            types = preobjtypes[pred]
            for arg in range(len(names)):
                if names[arg] in usednames:
                    args.append(args[names.index(names[arg])])
                else:
                    args.append(types[arg].instantiate(names[arg]))
                usednames.append(names[arg])

            cond = Condition(prepredicates[pred].instantiate(args), types)

            self.precondorder.append(cond)
            self.preconditions[cond] = names

        for pred in range(len(prefunc)):
            args = []
            usednames = []

            names = prefuncnames[pred] if prefuncnames else []
            types = prefunctypes[pred] if prefuncnames else []

            for arg in range(len(names)):
                if names[arg] in usednames:
                    args.append(args[names.index(names[arg])])
                else:
                    temp = []
                    for aa in range(len(names[arg])):
                        temp.append(types[arg][aa].instantiate(names[arg][aa]))
                    args.append(temp)
                usednames.append(names[arg])

            # it is two args related to one func
            (arg1, arg2) = prefunc[pred].instantiate(args)

            cond = Condition(arg1, types, prefunc[pred].op, arg2)

            self.precondorder.append(cond)
            self.preconditions[cond] = names

        for pred in range(len(postfunc)):
            args = []
            usednames = []

            names = postfuncnames[pred] if postfuncnames else []
            types = postfunctypes[pred] if postfunctypes else []

            for arg in range(len(names)):
                if names[arg] in usednames:
                    args.append(args[names.index(names[arg])])
                else:
                    temp = []
                    for aa in range(len(names[arg])):
                        temp.append(types[arg][aa].instantiate(names[arg][aa]))
                    args.append(temp)
                usednames.append(names[arg])

            (arg1, arg2) = postfunc[pred].instantiate(args)
            # args2 can be a number or a func
            cond = Condition(arg1, types, postfunc[pred].op, arg2)

            self.resultorder.append(cond)
            self.results[cond] = names
        # print(postobjnames)
        for pred in range(len(postpredicates)):
            args = []
            usednames = []
            names = postobjnames[pred]
            types = postobjtypes[pred]
            for arg in range(len(names)):
                if names[arg] in usednames:
                    args.append(args[names.index(names[arg])])
                else:
                    args.append(types[arg].instantiate(names[arg]))
                usednames.append(names[arg])

            cond = Condition(postpredicates[pred].instantiate(args), types)

            self.resultorder.append(cond)
            self.results[cond] = names

    def instantiate(self, args, verbose=2):
        if len(args) != len(self.objnames):
            raise Exception("wrong number of arguments")
        objdict = {}
        for i in range(len(args)):
            objdict[self.objnames[i]] = args[i]

        preconditions = []
        func_preconditions = []
        for condition in self.precondorder:
            names = self.preconditions[condition]
            args = []
            # TODO: Zohreh; it is hard coded here for constant objects. should be modified.
            for name in names:
                if not (type(name) == list):
                    if name and name in objdict.keys():
                        args.append(objdict[name])
                    elif name:
                        for t in self.types:
                            for x in t:
                                if x.name == "resource":
                                    resourceType = x
                        args.append(Obj(name, resourceType))
                else:
                    for n in name:
                        if n in objdict.keys():
                            args.append(objdict[n])
                        elif n:
                            for t in self.types:
                                for x in t:
                                    if x.name == "resource":
                                        resourceType = x
                            args.append(Obj(n, resourceType))

            if condition.op:

                # if not isinstance(condition.val, numbers.Number):
                #     new_val = condition.val.instantiate(args)
                #     func_preconditions.append((condition.instantiate([]), condition.op, new_val))
                # else:
                arg1, arg2 = condition.instantiate(args)
                func_preconditions.append((arg1, arg2, condition.op))

            else:
                preconditions.append(condition.instantiate(args))

        results = []
        func_results = []
        for condition in self.resultorder:
            names = self.results[condition]
            args = []
            for name in names:
                if not (type(name) == list):
                    if name and name in objdict.keys():
                        args.append(objdict[name])
                    elif name:
                        for t in self.types:
                            for x in t:
                                if x.name == "resource":
                                    resourceType = x
                        args.append(Obj(name, resourceType))
                else:
                    for n in name:
                        if n in objdict.keys():
                            args.append(objdict[n])
                        elif n:
                            for t in self.types:
                                for x in t:
                                    if x.name == "resource":
                                        resourceType = x
                            args.append(Obj(n, resourceType))

            if condition.op:

                # if not isinstance(condition.val, numbers.Number):
                #     new_val = condition.val.instantiate(args)
                #     func_results.append((condition.instantiate([]), condition.op, new_val))
                # else:
                arg1, arg2 = condition.instantiate(args)

                func_results.append((arg1, arg2, condition.op))
            else:
                results.append(condition.instantiate(args))

        result_action = Action(self, preconditions, self.prePos, results, self.postPos, func_preconditions,
                               func_results, self.prefuncpos, self.postfunpos)
        result_action.set_args(args)
        return result_action

    def resource_type(self):
        for t in self.types:
            for x in t:
                if x.name == "resource":
                    return x

    def __str__(self):
        s = "Operator - " + self.name + "("
        for name in self.objnames:
            s += name + ", "
        if self.objnames:
            s = s[:-2]
        # s += ")\nPreconditions: ["
        i = 0
        # for condition in self.preconditions:
        #     # print "precondition is "+str(condition)
        #     if not self.prePos[i]:
        #         s += "Not "
        #     s += str(condition) + " ; "
        #     i += 1
        # if self.preconditions:
        #     s = s[:-3]
        # s += "]\nPostconditions: ["
        i = 0
        # for condition in self.results:
        #     print ("postcondition is "+str(condition))
        #     for j in range(len(self.postPos)):
        #         print(self.postPos[j])
        #
        #     if self.postPos and not self.postPos[i]:
        #         s += "Not "
        #     s += str(condition) + " ; "
        #     i += 1
        # if self.results:
        #     s = s[:-3] + "]"
        return s


class Tree:
    '''
    Class heirarchy tree which contains insert and print functions
    '''

    def __init__(self, rootnode, allnodes, checked, args):
        '''
        initialize the rootnode , allnodes and checked .
        rootnode is the main node that contains the trace of all nodes.
        checked is something which is usefull in goaltransforms ,
        to check whether the result is not as same as what to be transformed.
        '''
        self.rootnode = rootnode
        self.allnodes = allnodes
        self.checked = checked
        if not args == 0:
            self.insert(args)

    def printall(self, space, printed, root):
        '''
        Print the predicates in the heirarchy of spaces.
        for example rootnode should contain no space while the following children
        should contain a tab space and so on.
        '''
        if (len(root.parents) == 0) and (not (len(printed) == 0)):
            return 0
        if not root.predicate in printed:
            printed.append(root.predicate)
            print((space + root.predicate))
            space = space[0:4] + space
        else:
            space = space[0:-4]
        if len(root.children) == 0:
            self.printall(space, printed, root.parents.pop())
        else:
            for s in root.children:
                self.printall(space, printed, s)

    def printtree(self):
        '''
        Start from the root node and print all the predicates by parsing through the children.
        '''
        root = copy.deepcopy(self.rootnode)
        printed = list()
        space = "  "
        print("--------- Class Hierarchy Predicate Tree ----------")
        self.printall(space, printed, root)
        print("")
        print("--------- Class Hierarchy Predicate Tree Ends ----------")
        print("")

    def insert(self, content):
        '''
        create a node for each new argument and call it predicate ,
        if there is no root node initialize root node
        else check the node to add a branch for and add a branch to it.
        '''
        for i in range(0, len(content)):
            newnode = Node()
            temp_string = content
            newnode = newnode.insert(temp_string[len(temp_string) - 1])

        if not (self.rootnode):
            self.rootnode = newnode
            self.allnodes.append(newnode)

        for i in range(0, len(temp_string) - 1):
            newnode = Node()
            newnode = newnode.insert(temp_string[i])
            newnode.insertbranch(self.check_in_all_tree_nodes(temp_string[len(temp_string) - 1]), self)
            self.allnodes.append(newnode)

    def check_in_all_tree_nodes(self, predicate):
        for a in range(0, len(self.allnodes)):
            if self.allnodes[a].predicate == predicate:
                return self.allnodes[a]


class ObjectTree:
    '''
    Class heirarchy tree which contains insert and print functions
    '''

    def __init__(self, rootnode, allnodes, checked, args):
        self.rootnode = rootnode
        self.allnodes = allnodes
        self.checked = checked
        if not args == 0:
            self.objectinsert(args)

    def printall(self, space, printed, root):
        '''
        Print the predicates in the heirarchy of spaces. for example rootnode
        should contain no space while the following children
        should contain a tab space and so on.
        '''

        if (len(root.parents) == 0) and (not (len(printed) == 0)):
            return 0

        if not root.predicate in printed:
            printed.append(root.predicate)
            print((space + root.predicate))
            space = space[0:4] + space
        else:

            space = space[0:-4]

        if len(root.children) == 0:
            self.printall(space, printed, root.parents.pop())
        else:
            for s in root.children:
                self.printall(space, printed, s)

    def printtree(self):
        '''
        Start from the root node and print all the predicates by parsing through the children.
        '''
        root = copy.deepcopy(self.rootnode)
        printed = list()
        space = "  "
        print("--------- Class Hierarchy Object Tree ----------")
        self.printall(space, printed, root)
        print("")
        print("--------- Class Hierarchy Object Tree Ends ----------")
        print("")

    def objectinsert(self, content):
        '''
        create a node for each new argument and call it predicate ,
        if there is no root node initialize root node
        else check the node to add a branch for and add a branch to it.
        '''
        for i in range(0, len(content)):
            temp_string = content
            newnode = Node()
            newnode = newnode.insert(temp_string[len(temp_string) - 1])

        if not (self.rootnode):
            self.rootnode = newnode
            self.allnodes.append(newnode)

        for i in range(0, len(temp_string) - 1):
            newnode = Node()
            newnode = newnode.insert(temp_string[i])
            newnode.insertbranch(self.check_in_all_object_nodes(temp_string[len(temp_string) - 1]), self)
            self.allnodes.append(newnode)

    def check_in_all_object_nodes(self, predicate):
        for a in range(0, len(self.allnodes)):
            if self.allnodes[a].predicate == predicate:
                return self.allnodes[a]


class Node:
    '''
    Node for creating a tree which contains predicate as name ,
    parents and children as data members.
    '''

    def __init__(self):
        self.predicate = ""
        self.parents = list()
        self.children = list()

    def insert(self, predicate):
        self.predicate = predicate
        return self

    def insertbranch(self, s, tree):
        self.parents.append(s)
        s.children.append(self)
        for a in range(0, len(tree.allnodes)):
            if tree.allnodes[a].predicate is s.predicate:
                tree.allnodes[a] = s
        return self


class World:

    def __init__(self, operators, predicates, atoms, types, objects=[], functions=[], cltree=[], obtree=[]):
        self.operators = {}
        self.types = types
        self.cltree = cltree
        self.obtree = obtree
        for operator in operators:
            self.operators[operator.name] = operator
        self.predicates = {}
        self.functions = {}
        for predicate in predicates:
            self.predicates[predicate.name] = predicate
        for func in functions:
            self.functions[func.name] = func
        self.objects = {}
        for atom in atoms:
            for arg in atom.args:
                self.objects[arg.name] = arg
        for object in objects:
            self.objects[object.name] = object
        self.atoms = set(atoms)

    def get_atoms(self, filters=[]):
        '''
        Will return atoms if the filter (string) is within any pred or arg names
        If more than one filter string, then all must match (conjunctive filters)
        '''

        # if no filters, return everything
        if len(filters) == 0:
            return self.atoms

        # record which filters have matched
        filter_matches = {k: False for k in filters}
        # print("filter matches = "+str(filter_matches))
        relevant_atoms = []
        if len(filters) > 0:
            for atom in self.atoms:
                atom_parts = [atom.predicate] + atom.args
                for filter_str in filters:  # assuming there shouldn't be more than 3-4 filters
                    for part in atom_parts:  # assuming shouldn't be atoms with more than 4-5 parts
                        # print "type(part) == "+str(part.name)
                        if (not filter_matches[filter_str]) and filter_str in part.name:
                            filter_matches[filter_str] = True

                if not (False in list(filter_matches.values())):  # check to see they are all True
                    relevant_atoms.append(atom)
                # print("Just added "+str(atom)+" to relevant atoms")
                # reset filter matches
                filter_matches = {k: False for k in filters}

        return relevant_atoms

    def get_objects_names_by_type(self, typename):
        objs = []
        for each_obj in list(self.objects.keys()):
            if (str(self.obj_type(each_obj)) == str(typename)):
                objs.append(each_obj)
        return objs

    def diff(self, otherWorld):
        '''
        Given another world, return the differences in atoms.
        Return value is a tuple where the first element is the atoms in this world not in the given world
        Second element is the atoms not in this world but in given world
        '''
        atoms_not_in_other = []
        atoms_not_in_self = []
        # go through all atoms in this world, check to see if they are in other world
        for atom in self.atoms:
            if not otherWorld.atom_true(atom):
                atoms_not_in_other.append(atom)

        # go through all atoms in given world, check to see if they are in this world
        for atom in otherWorld.atoms:
            if not self.atom_true(atom):
                atoms_not_in_self.append(atom)

        return (atoms_not_in_self, atoms_not_in_other)

    def equal(self, otherWorld):
        diff_result = self.diff(otherWorld)
        # print "diff_result inside equal() : "+str(map(str,diff_result[0]))+","+str(map(str,diff_result[1]))
        return diff_result == ([], [])

    def fast_equal(self, otherworld):
        pass

    def copy(self):
        # safety check, in case a list is passed for atoms instead of a set
        if not type(self.atoms) is set:
            self.atoms = set(self.atoms)

        return World(list(self.operators.values()), list(self.predicates.values()), self.atoms.copy(),
                     self.types.copy(),
                     list(self.objects.values()), list(self.functions.values()))

    def is_true(self, predname, argnames=[]):
        for atom in self.atoms:
            if atom.predicate and atom.predicate.name == predname:
                if len(atom.args) == len(argnames):
                    namesCorrect = True
                    for i in range(len(atom.args)):
                        if atom.args[i].name != argnames[i]:
                            namesCorrect = False
                    if namesCorrect:
                        return True
        return False

    def atom_func_true(self, atom, val, op):
        a = next((x for x in self.atoms if x.func and x.func == atom.func and x.args == atom.args), None)
        if a:
            if not isinstance(val, numbers.Number):

                func_2 = next((x for x in self.atoms if x.func and x.func == val.func and x.args == val.args), None)

                val = func_2.val

            new_val = a.val

            # print(a.val)

            if not a.val:
                for f in func_val_dict:
                    if f.func.name == a.func.name:
                        # print("found one: " + str(f))
                        if f.args and f.args == atom.args:
                            new_val = func_val_dict[f]

                        if not f.args:
                            new_val = func_val_dict[f]

            if op == ">":
                return float(new_val) > float(val)

            if op == "<":
                return float(new_val) < float(val)

            if op == "=":
                return float(new_val) == float(val)

        # print("return false")
        return False

    def atom_true(self, atom):
        # this is very fast, because atom objects have hashes and self.atoms is a set, not a list
        return atom in self.atoms

    def atom_val_true(self, atom, verbose=2):
        if verbose >= 2: print(atom)
        # a = next((x for x in self.atoms if x.func and x.func == atom.func), None)
        a = self.get_val_func(atom)
        # TODO: I only assume greater operator here; it needs to be changes for <, =
        if verbose >=2: print(a.val)
        if a.val is None:
            a.val = 0
        return int(a.val) > int(atom.val)

    def add_atom(self, atom):
        self.atoms.add(atom)

    def add_fact(self, predname, argnames=[]):
        if not self.is_true(predname, argnames):
            self.add_atom(Atom(self.predicates[predname], [self.objects[name] for name in argnames]))

    def remove_atom(self, atom):
        self.atoms.remove(atom)

    def remove_fact(self, predname, argnames=[]):
        toRemove = None
        for atom in self.atoms:
            if atom.predicate.name == predname:
                if len(atom.args) == len(argnames):
                    namesCorrect = True
                    for i in range(len(atom.args)):
                        if atom.args[i].name != argnames[i]:
                            namesCorrect = False
                    if namesCorrect:
                        toRemove = atom
                        break
        if toRemove:
            self.remove_atom(toRemove)

    def add_object(self, object):
        self.objects[object.name] = object

    def add_object_by_type(self, name, type):
        if type in self.types:
            self.add_object(self.types[type].instantiate(name))
        else:
            raise Exception("Type " + str(type) + " DNE.")

    def obj_type(self, objname):
        return self.objects[objname].type.name

    def remove_object(self, object):
        if object in self.objects:
            actualObject = self.objects[object]
            del (self.objects[object])
            self.atoms = [atom for atom in self.atoms if actualObject not in atom.args]
            return True
        else:
            for key in self.objects:
                if self.objects[key] == object:
                    del (self.objects[key])
                    self.atoms = [atom for atom in self.atoms if object not in atom.args]
                    return True
        return False

    def get_possible_objects(self, predicate, arg):
        return list(self.objects.values())  # not, obviously, a good implementation

    def get_objects_by_type(self, some_type):
        if type(some_type) is str:
            if some_type not in self.get_types():
                raise Exception("Trying to get object of type " + str(some_type) + " but not a valid type")
        else:
            if some_type.name not in self.get_types():
                raise Exception("Trying to get object of type " + str(some_type) + " but not a valid type")

        objs = []
        for obj in list(self.objects.values()):
            if obj.is_a(some_type):
                objs.append(obj)

        return objs

    def get_types(self):
        return self.types

    def is_applicable(self, action):
        for i in range(len(action.preconds)):
            if action.prePos[i] and not self.atom_true(action.preconds[i]):
                # print("nor true + " + str( action.preconds[i]))
                return False
            if not action.prePos[i] and self.atom_true(action.preconds[i]):
                # print("nor true + " + str(action.preconds[i]))
                return False

        # todo: Zohreh; fix the bug later here
        try:
            for i in range(len(action.preFuncPos)):
                (atom, arg2, op) = action.funcPre[i]
                if action.preFuncPos[i] and not self.atom_func_true(atom, arg2, op):
                    print(action.preFuncPos[i])
                    # print("not true")
                    return False

                if not action.preFuncPos[i] and self.atom_func_true(atom, arg2, op):
                    print(action.preFuncPos[i])
                    # print("not false")
                    return False

                # if True and not self.atom_func_true(atom, arg2, op):
                #     print("not true")
                #     return False
        except Exception as e:
            print("ERROR: " + str(e))

        return True

    # convenience method for operating with MIDCA
    def midca_action_applicable(self, midcaAction):
        try:
            operator = self.operators[midcaAction.op]
            args = [self.objects[arg] for arg in midcaAction.args]
        except KeyError:
            return False

        # print(operator)
        # print("is going to be instantiated")

        action = operator.instantiate(args)

        return self.is_applicable(action)

    def get_val_func(self, atom):
        for f in self.atoms:
            if f.func and f.func.name == atom.func.name:
                for a in range(len(f.args)):
                    if str(f.args[a].name).strip() == str(atom.args[a].name).strip():
                        return f

                if not f.args:
                   return f

        return None

    def apply(self, simAction, verbose=2):

        for (atom, val, op) in simAction.funcRes:
            # func = next((x for x in self.atoms if x.func and x.func == atom.func  and x.args == atom.args), None)
            # print("atom" + str(atom))
            func = self.get_val_func(atom)
            #todo: Zohreh -- change this later
            if func.val is None:
                func.val = 0

            if not isinstance(val, numbers.Number):
                func_2 = next((x for x in self.atoms if x.func and x.func == val.func and x.args == val.args), None)

                val = func_2.val
                if val == None:
                    val = 0

            if op == "decrease":
                func.val = float(func.val) - float(val)

            elif op == "increase":
                # print("func" + str(func))
                func.val = float(func.val) + float(val)
                # print("val" + str(func.val))

            elif op == "assign":
                func.val = float(val)

            # for f in func_val_dict:
            #     if f.func.name == func.func.name:
            #         print("Update the value for " + str(f))
            #         for a in range(len(f.args)):
            #             if str(f.args[a].name).strip() == str(func.args[a].name).strip():
            #                 func_val_dict[f] = func.val
            #         if not f.args:
            #             func_val_dict[f] = func.val

        for i in range(len(simAction.results)):

            if simAction.postPos[i]:  ## it is an atom
                # print("adding_atom " + str(simAction.results[i]))
                self.add_atom(simAction.results[i])
            else:
                # print("removing_atom "+str(simAction.results[i]))
                self.remove_atom(simAction.results[i])


    def apply_named_action(self, opName, argNames, verbose=2):

        args = []
        for name in argNames:
            if name not in self.objects:
                raise Exception("Object " + name + " DNE")
            args.append(self.objects[name])
        if opName not in self.operators:
            raise Exception("Operator " + opName + " DNE")
        simAction = self.operators[opName].instantiate(args)

        if not self.is_applicable(simAction):
            raise Exception("Preconditions not met.")

        self.apply(simAction)

    # TODO: I need to modify this for an actual event
    '''
    
    '''

    def apply_event(self, opName, argNames=[], verbose=2):
        try:
            func = self.functions["player-current-health"]
            a = next((x for x in self.atoms if x.func == func), None)
            a.val = a.val - 5

            if opName == "thing-at-map":
                pred = self.predicates[opName]

                if (argNames[0] or argNames[1]) not in self.objects:
                    raise Exception(": Object - " + argNames[0] + " DNE ")

                newatom = Atom(pred, [self.objects[argNames[0]], self.objects[argNames[1]]])
                self.add_atom(newatom)
                if verbose >= 2:
                    print(str(argNames[0]) + "is simulated to be spawned at " + str(argNames[1]))

            if opName == "zombie_damage":
                pred = self.predicates["monster-at"]
                # (zombie-at zombie m0_1)
                newatom = Atom(pred, ["zombie", "m1_2"])
                self.add_atom(newatom)

                if verbose >= 2:
                    print("player's current health is down to " + str(a.val))

            if opName == "arrow_damage":
                pred = self.predicates["thing-at-map"]
                # (obj-at arrow m0_1)
                name = "arrow"
                loc = "m1_2"
                args = []
                if (name or loc) not in self.objects:
                    raise Exception(": Object - " + name + " DNE ")
                args.append(self.objects[name])

                args.append(self.objects[loc])

                newatom = Atom(pred, args)
                self.add_atom(newatom)

                print("player's current health is down to " + str(a.val) + " and arrows are nearby")
                print()

        except Exception as e:
            print("ERROR: " + str(e))

    # convenience method for operating with MIDCA
    def apply_midca_action(self, midcaAction):
        opname = midcaAction.op
        argnames = [str(arg) for arg in midcaAction.args]

        self.apply_named_action(opname, argnames)

    # interprets a MIDCA goal as a predicate statement. Expects the predicate name to be either in kwargs under 'predicate' or 'Predicate', or in args[0]. This is complicated mainly due to error handling.
    def midcaGoalAsAtom(self, goal):
        try:
            if 'predicate' in goal.kwargs:
                predName = str(goal['predicate'])
                val = None
            elif 'func' in goal.kwargs:
                predName = str(goal['func'])
                val = str(goal['val'])
        except KeyError:
            try:
                predName = str(goal['Predicate'])
            except KeyError:
                try:
                    predName = str(goal[0])
                except KeyError:
                    raise ValueError(
                        "Trying to interpret " + str(goal) + " as a predicate atom, but cannot find a predicate name.")
        try:
            if 'predicate' in goal.kwargs:
                predicate = self.predicates[predName]
            elif 'func' in goal.kwargs:
                predicate = self.functions[predName]
        except KeyError:
            raise ValueError("Predicate/Function " + predName + " not in domain.")

        args = []  # args for new atom
        # check if predicate took first spot in arg list
        if goal.args and goal.args[0] != "predicate":
            nextArgI = 0
        else:
            nextArgI = 1
        for i in range(len(predicate.argnames)):
            if nextArgI < len(goal.args):
                try:
                    args.append(self.objects[str(goal.args[nextArgI])])
                    nextArgI += 1
                except KeyError:
                    raise ValueError("Object " + str(goal.args[nextArgI]) + " not found; goal " + str(
                        goal) + " does not encode a valid predicate representation.")
            else:
                if predicate.argnames[i] in goal.kwargs:
                    try:
                        value = goal.kwargs[predicate.argnames[i]]
                        args.append(self.objects[str(value)])
                    except KeyError:
                        raise ValueError("Object " + str(value) + " not found; goal " + str(
                            goal) + " does not encode a valid predicate representation.")
                else:
                    raise ValueError("Trying to interpret " + str(
                        goal) + " as a predicate atom, but cannot find a value for argument " + predicate.argnames[i])
        assert len(args) == len(predicate.argnames)  # sanity check
        try:
            return Atom(predicate, args, val)
        except Exception:
            raise ValueError(str(predicate) + str(args) + " does not seem to be a valid state")

    def plan_correct(self, plan):
        testWorld = copy.deepcopy(self)
        for action in plan.get_remaining_steps():
            if not testWorld.midca_action_applicable(action):
                return False
            testWorld.apply_midca_action(action)
        return True

    def goals_achieved(self, plan, goalSet):
        testWorld = copy.deepcopy(self)
        achievedGoals = set()
        for action in plan.get_remaining_steps():
            if not testWorld.midca_action_applicable(action):
                print("it is not applicable")
                break
            else:
                testWorld.apply_midca_action(action)

        for goal in goalSet:
            achieved = testWorld.atom_true(self.midcaGoalAsAtom(goal))
            if 'negate' in goal and goal['negate']:
                achieved = not achieved
            if achieved:
                achievedGoals.add(goal)
            else:
                print(self.midcaGoalAsAtom(goal))
        return achievedGoals

    def goals_achieved_now(self, goalSet):
        '''
        Same as goals_achieved but no plan, uses this world
        '''
        for goal in goalSet:
            achieved = self.atom_true(self.midcaGoalAsAtom(goal))
            if 'negate' in goal and goal['negate']:
                achieved = not achieved
            if not achieved:
                return False

        return True

    def plan_goals_achieved(self, plan):
        return self.goals_achieved(plan, plan.goals)

    def get_operators(self):
        return self.operators

    def remove_operator(self, opname):
        if opname in list(self.operators.keys()):
            del self.operators[opname]
            return True
        return False

    def __str__(self):
        s = "[\n"
        for name in sorted(self.objects.keys()):
            object = self.objects[name]
            s += object.name + " (" + object.type.name + ") : "
            for atom in self.atoms:
                if object in atom.args:
                    s += str(atom) + " and "
            if s[-2:] == "d ":
                s = s[:-5] + "\n"
            else:
                s = s[:-3] + "\n"
        return s + "]\n"
