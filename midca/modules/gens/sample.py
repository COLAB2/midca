import cl,goaltransform



def main():
    #print("Enter Class Hierarchy in Tree Structure: ")
    tree = cl.Tree()
    objecttree = cl.ObjectTree()
    fname = "/home/sravya/Documents/midca/worldsim/domains/shopping.sim"
    cl.implement(fname,tree,objecttree)
    tree.printtree()
    objecttree.printtree()
    #goal = raw_input('Enter your desired Goal:')

    
    while True:
        goal = raw_input('Enter your desired Goal:')
        #identity
        identity_res = goaltransform.identity(goal,1)
        print(identity_res)

        print("--------- Genralization Phase ----------")

        res = goaltransform.generalization(goal,1)
        print(res)


        print("--------- Specailization Phase ----------")
        res = goaltransform.specailization(goal,1)
        print(res)


        print("--------- Choose Phase ----------")
        res = goaltransform.choose(goal)
        print("Choose Output Final " + res)

        print("--------- Abstraction Phase ----------")
        res = goaltransform.abstraction(goal,1)
        print("Abstraction Output Final " + res)

        print("--------- Concretion Phase ----------")
        res = goaltransform.concretion(goal,1)
        print("Concretion Output Final " + res)


        print(' press "x" to exit ')
        key = raw_input()
        if key is "x":
                        return 0
	
	

if __name__ == "__main__":
    main()
