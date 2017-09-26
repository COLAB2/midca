import copy
import inspect, os

class Tree:
        def __init__(self):
                self.rootnode = ""
                self.allnodes = list()
                self.checked = list()

        def printall(self,space,printed,root):

			if (len(root.parents) == 0) and (not( len(printed) == 0)):
				return 0

			if not root.predicate in printed:
				printed.append(root.predicate)
				print(space + root.predicate)
				space =  space[0:4] + space
			else:
				space = space[0:-4]


			if len(root.children) == 0:
				self.printall(space, printed,root.parents.pop())
			else:
				for s in root.children:
					self.printall(space, printed,s)

			
        
        def printtree(self):
                root = copy.deepcopy(Tree.rootnode)
                printed = list()
                space = "  "
                print("--------- Class Hierarchy Predicate Tree ----------")
                self.printall(space,printed,root)
                print("")
                print("--------- Class Hierarchy Predicate Tree Ends ----------")
                print("")


class ObjectTree:
        rootnode = ""
        allnodes = list()
	checked = []

        def printall(self,space,printed,root):

			if (len(root.parents) == 0) and (not( len(printed) == 0)):
				return 0

			if not root.predicate in printed:
				printed.append(root.predicate)
				print(space + root.predicate)
				space =  space[0:4] + space
			else:
				space = space[0:-4]


			if len(root.children) == 0:
				self.printall(space, printed,root.parents.pop())
			else:
				for s in root.children:
					self.printall(space, printed,s)

			
        
        def printtree(self):
                root = copy.deepcopy(ObjectTree.rootnode)
                printed = list()
                space = "  "
                print("--------- Class Hierarchy Object Tree ----------")
                self.printall(space,printed,root)
                print("")
                print("--------- Class Hierarchy Object Tree Ends ----------")
                print("")

class Node:
        def __init__(self):
                self.predicate = ""
                self.parents = list()
                self.children = list()

        def insert(self,predicate):
                self.predicate = predicate
                return self
                       
                                       
        def insertbranch(self,s):
                self.parents.append(s)
                s.children.append(self)
                for a in range(0,len(Tree.allnodes)):
                        if Tree.allnodes[a].predicate is s.predicate:
                                Tree.allnodes[a] = s
#                Tree.allnodes.append(self)                               
                return self

        
                        
'''
def implement(tree):
        tree = Tree()
        branches = list()
        revbranches = list()
        while True:
                predicate = raw_input('Enter predicate: ')
                newnode = Node()
                if not(Tree.rootnode):
                        branches.append(newnode.insert(predicate))
                        Tree.rootnode= newnode
                        Tree.allnodes.append(newnode)
                        
                else:
                		if len(revbranches) == 0:
                			for s in branches:
                				Tree.rootnode = s
                				break
                		else:
                			for s in revbranches:
                				Tree.rootnode = s
                				break
                		temp = branches.pop()
                		revbranches.append(temp)
                		print('added a branch to ' + temp.predicate)
                		newnode = newnode.insert(predicate)
                		branches.append(newnode.insertbranch(temp))
                		tree.printtree()
                		print('Press c to add children to ' + newnode.predicate + ": " )
                		key = raw_input()
                		while not key is "c":
                                        if len(revbranches) == 1:
                                                store = revbranches.pop()
                                                print("cannot move further up forcedly adding a branch to " + store.predicate)
                                                del(branches[:])
                                                branches.append(store)
                                                break
                                        
                                        print("Moving a branch up ")
                                        nodestore = revbranches.pop()
                                        print('Press c to add children to ' + nodestore.predicate + ": " )
                                        del(branches[:])
                                        branches.append(nodestore)
                                        key = raw_input()

                                
                                      
                print(' press "x" to exit ')
                key = raw_input()
                if key is "x":
                        if len(revbranches) == 0:
                                Tree.rootnode = branches.pop()
                        else:
                            while len(revbranches) > 1:
                                    revbranches.pop()
                            Tree.rootnode = revbranches.pop()
                                    
                            
                        
                        return 0

def insertion(content):
	count = 0
	tree = Tree()
        branches = list()
        revbranches = list()
	newnode = Node()
        if not(Tree.rootnode):
                 branches.append(newnode.insert(content[0].replace("\n" ,"")))
                 Tree.rootnode= newnode
                 Tree.allnodes.append(newnode)
	for i in range(1,len(content)):
		predicate = content[i].replace("\n","")
		if(predicate == ""):
			return 
		tabspace_count = len(predicate) - len(predicate.lstrip('\t'));
		for a in range(0,tabspace_count ):
			predicate = predicate.lstrip('\t')
		if (tabspace_count > count):
			count = tabspace_count
			newnode = Node()
			newnode = newnode.insert(predicate)
			temp = branches[len(branches)-1]
			branches.append(newnode.insertbranch(temp))
			Tree.allnodes.append(newnode)
#			tree.printtree()
			
		else:
#			print("hi")
#			raw_input("enter")
			for j in range(0,((count - tabspace_count )+1)):
				branches.pop()
			count = tabspace_count
			newnode = Node()
			newnode = newnode.insert(predicate)
			temp = branches[len(branches)-1]
			branches.append(newnode.insertbranch(temp))
			Tree.allnodes.append(newnode)
#			tree.printtree()
'''
def check_in_all_tree_nodes(predicate):
#	print(predicate)
	for a in range(0,len(Tree.allnodes)):
#		print(Tree.allnodes[a].predicate)
		if Tree.allnodes[a].predicate == predicate:
			#print(Tree.allnodes[a].predicate)	
			return Tree.allnodes[a]

def check_in_all_object_nodes(predicate):
#	print(predicate)
	for a in range(0,len(ObjectTree.allnodes)):
#		print(Tree.allnodes[a].predicate)
		if ObjectTree.allnodes[a].predicate == predicate:
			#print(Tree.allnodes[a].predicate)	
			return ObjectTree.allnodes[a]	


			 
def objectinsert(content):
		temp_string = content
		objecttree = ObjectTree()
		print(temp_string)
		newnode = Node()
		newnode = newnode.insert(temp_string[len(temp_string)-1])
		
		if not(ObjectTree.rootnode):
                 	 ObjectTree.rootnode= newnode
                 	 ObjectTree.allnodes.append(newnode)
		
		for i in range(0,len(temp_string)-1):
			#print(temp_string[len(temp_string)-1])
			newnode = Node()
			newnode = newnode.insert(temp_string[i])
			newnode.insertbranch(check_in_all_object_nodes(temp_string[len(temp_string)-1]))
			ObjectTree.allnodes.append(newnode)
			#objecttree.printtree()		
	
	
'''
def objectinsertion(content):
	count = 0
	objecttree = ObjectTree()
        branches = list()
        revbranches = list()
	newnode = Node()
        if not(ObjectTree.rootnode):
                 branches.append(newnode.insert(content[0].replace("\n" ,"")))
                 ObjectTree.rootnode= newnode
                 ObjectTree.allnodes.append(newnode)
	for i in range(1,len(content)):
		predicate = content[i].replace("\n","")
		if(predicate == ""):
			return 
		tabspace_count = len(predicate) - len(predicate.lstrip('\t'));
		for a in range(0,tabspace_count ):
			predicate = predicate.lstrip('\t')
		if (tabspace_count > count):
			count = tabspace_count
			newnode = Node()
			newnode = newnode.insert(predicate)
			temp = branches[len(branches)-1]
			branches.append(newnode.insertbranch(temp))
			ObjectTree.allnodes.append(newnode)
#			objecttree.printtree()
			
		else:
#			print("hi")
#			raw_input("enter")
			for j in range(0,((count - tabspace_count )+1)):
				branches.pop()
			count = tabspace_count
			newnode = Node()
			newnode = newnode.insert(predicate)
			temp = branches[len(branches)-1]
			branches.append(newnode.insertbranch(temp))
			ObjectTree.allnodes.append(newnode)
#			objecttree.printtree()
'''

def implement(fname,tree,objecttree):
	#fname = MIDCA_ROOT + "/worldsim/tree/class.tree"
	content_class_heirarchy = []
	content_object_heirarchy = []
	with open(fname) as f:
    		content = f.readlines()
#		if("\n" in content[0]):
#			print(content[0].replace("\n",""))
		for i in range(0,len(content)):
			if "ptype" in content[i]:
				content_class_heirarchy.append(content[i])
			elif "type" in content[i]:
				content_object_heirarchy.append(content[i])
		#print(content_class_heirarchy[0:])
		#print(content_class_heirarchy)
		#print(content_object_heirarchy)
		insert(content_class_heirarchy[0:])
		#tree.printtree()
		objectinsert(content_object_heirarchy[0:])
		#objecttree.printtree()
    		

if __name__ == "__main__":
    main()










