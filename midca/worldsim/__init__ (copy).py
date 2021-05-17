#MIDCA_ROOT = thisDir + "/../"
domainfile = "worldsim/domain_new.pddl"
#EVENT_FILE = MIDCA_ROOT + "worldsim/domain_new.pddl"
problemfile =  "worldsim/p01.pddl"
def parseDomainAndProblem(domainfile, problemfile):
    print("Parsing domain", domainfile)
    dtree = readAndParseFile(domainfile)
    domain = dtree.domain()
    if domain is not None:
        dom = parseDomain(domain)
    else:
        raise Exception("No domain defined in " + domainfile)

    print("Parsing problem", problemfile)
    ptree = readAndParseFile(problemfile)
    problem = ptree.problem()
    if problem is not None:
        prob = parseProblem(problem)
    else:
        raise Exception("No problem defined in " + problemfile)

    return (dom, prob)

def parseDomain(domain):
    # Get name
    domainname = domain.domainName().name().getText()

    reqs = []
    for r in domain.requireDef().REQUIRE_KEY():
        reqs.append(r.getText())

    # Get types
    if domain.typesDef() is not None:
        types = parseTypeNameList(domain.typesDef().typedNameList())
    else:
        types = TypedArgList([])

    # Get constants
    if domain.constantsDef() is not None:
        constants = parseTypeNameList(domain.constantsDef().typedNameList())
    else:
        constants = TypedArgList([])

    # Get functions
    functions = []
    if domain.functionsDef() is not None:
        for func in domain.functionsDef().functionList().atomicFunctionSkeleton():
            functions.append(
                Function(func.functionSymbol().name().getText(), parseTypeVariableList(func.typedVariableList())))

    # Get predicates
    predicates = []
    if domain.predicatesDef() is not None:
        for pred in domain.predicatesDef().atomicFormulaSkeleton():
            predicates.append(
                Predicate(pred.predicate().name().getText(), parseTypeVariableList(pred.typedVariableList())))

    # Get actions and durative actions
    durative_actions = []
    actions = []
    # derived = []
    for action in domain.structureDef():
        if action.actionDef() is not None:
            actions.append(parseAction(action.actionDef()))
        elif action.durativeActionDef() is not None:
            durative_actions.append(parseDurativeAction(action.durativeActionDef()))
        # elif action.derivedDef() is not None:
        #    derived.append(
        #                   [parseTypeVariableList(action.derivedDef().typedVariableList()), parseGoalDescription(action.derivedDef().goalDesc())]
        #                   )

    d = Domain(domainname, reqs, types, constants, predicates, functions, actions, durative_actions)
    return d

(dom, prob) = pddl.parseDomainAndProblem(domainfile, problemfile)
