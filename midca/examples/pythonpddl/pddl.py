#!/usr/bin/env python
# -*- coding: utf-8 -*-

from antlr4 import *
from pythonpddl import pddlLexer
from pythonpddl import pddlParser


import itertools
import sys



class TypedArg:
    """ represents an argument (possibly typed)"""
    def __init__(self,arg_name, arg_type = None):
        self.arg_name = arg_name
        self.arg_type = arg_type

    def asPDDL(self):
        if self.arg_type is None:
            return self.arg_name
        else:
            return self.arg_name + " - " + self.arg_type

class TypedArgList:
    """ represents a list of arguments (possibly with types)"""
    def __init__(self, args):
        self.args = args
#        self.complete_missing_types()
#
#    def complete_missing_types(self):
#        last_type = None
#        for arg in reversed(self.args):
#            if arg.arg_type is not None:
#                last_type = arg.arg_type
#            else:
#                arg.arg_type = last_type
   
    def asPDDL(self):
        return " ".join(map(lambda x: x.asPDDL(), self.args))


def parseTypeVariableList(tvl):
    args = []

    arg_name = ""
    arg_type = "<NONE>"
    
    for arg in tvl.singleTypeVarList():
        arg_type = arg.r_type().getText()
        for arg_context in arg.VARIABLE():
            arg_name = arg_context.getText()
            args.append(TypedArg(arg_name, arg_type))
    for arg_context in tvl.VARIABLE():
        arg_name = arg_context.getText()
        args.append(TypedArg(arg_name))
        
    return TypedArgList(args)


def parseTypeNameList(tnl):
    args = []

    arg_name = ""
    arg_type = "<NONE>"
    
    for arg in tnl.singleTypeNameList():
        arg_type = arg.r_type().getText()
        for arg_context in arg.name():
            arg_name = arg_context.getText()
            args.append(TypedArg(arg_name, arg_type))
    for arg_context in tnl.name():
        arg_name = arg_context.getText()
        args.append(TypedArg(arg_name))
        
    return TypedArgList(args)

class Function:
    """ represents a function"""
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def asPDDL(self):
        return "(" + self.name + " " + self.args.asPDDL() + ")"

class Predicate:
    """ represents a predicate"""
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def asPDDL(self):
        return "(" + self.name + " " + self.args.asPDDL() + ")"

def parsePredicate(pred):
    return Predicate(pred.predicate().name().getText(), parseTypeVariableList(pred.typedVariableList()))


class Formula:
    """ represented a goal description (atom / negated atom / and / or)"""
    def __init__(self, subformulas, op = None, is_effect = False, is_numeric = False):
        self.subformulas = subformulas
        self.op = op
        self.is_effect = is_effect
        self.is_numeric = is_numeric
   
    def get_predicates(self, positive):
        """ returns positive or negative predicates in this goal description"""
        if self.op is None and positive:
            assert len(self.subformulas) == 1
            return [self.subformulas[0]]
        elif self.op == "not" and not positive:
            assert len(self.subformulas) == 1
            return [self.subformulas[0]]
        elif self.op == "and":
            l = []
            for s in self.subformulas:
                l = l + s.get_predicates(positive)
            return l
        elif self.op == "or":
            raise Exception("Don't know how to handle disjunctive condition " + str(self.subformulas))
        return []

    def asPDDL(self):
        if self.op is None:
            assert len(self.subformulas) == 1
            return self.subformulas[0].asPDDL()
        elif self.op == "not":
            assert len(self.subformulas) == 1
            return "(not " + self.subformulas[0].asPDDL() + ")"
        elif self.op in ["and",'>', '<', '=', '>=', '<=', 'increase', 'decrease', 'assign', 'scale-up', 'scale-down']:
            return "(" + self.op + " " + " ".join(map(lambda x: x.asPDDL(), self.subformulas)) + ")"
        elif self.op == "or":
            raise Exception("Don't know how to handle disjunctive condition " + str(self.subformulas))
        else:
            raise Exception("Don't know how to handle op " + self.op)

def parseGoalDescription(gd, is_effect=False):
    """ parses a goal description. Returns a Formula"""
    if gd.atomicTermFormula() is not None:
        name = gd.atomicTermFormula().predicate().name().getText()
        terms = []
        for t in gd.atomicTermFormula().term():
            if t.VARIABLE() is not None:
                terms.append(TypedArg(t.VARIABLE().getText()))
            elif t.name() is not None:
                terms.append(TypedArg(t.name().getText()))
            else:
                raise Exception("Can't handle term " + gd.getText())

        op = None
        if gd.getChildCount() > 1:
            # This hack is meant to take care of negative effects
            op = gd.getChild(1).getText()
        return Formula([Predicate(name, TypedArgList(terms))], op, is_effect=is_effect)
    elif gd.fComp() is not None:
        op = gd.fComp().binaryComp().getText()
        fexp1 = parseFExp(gd.fComp().fExp()[0])
        fexp2 = parseFExp(gd.fComp().fExp()[1])
        return Formula([fexp1, fexp2], op, is_effect=is_effect)
    else:
        op = gd.getChild(1).getText()
        preds = []
        for p in gd.goalDesc():
            preds.append(parseGoalDescription(p))
        return Formula(preds, op, is_effect=is_effect)

class TimedFormula:
    """ represents a timed goal description"""
    def __init__(self, timespecifier, gd):
        self.timespecifier = timespecifier
        self.formula = gd

    def asPDDL(self):
        if self.timespecifier == "start":
            return "(at start " + self.formula.asPDDL() + ")"
        elif self.timespecifier == "end":
            return "(at end " + self.formula.asPDDL() + ")"
        elif self.timespecifier == "all":
            return "(over all " + self.formula.asPDDL() + ")"
        else:            
            return "(at " + str(self.timespecifier) + " " + self.formula.asPDDL() + ")"

def parseTimedGoalDescription(timedGD):
    gd = parseGoalDescription(timedGD.goalDesc())
    timespecifier = None
    if timedGD.interval() is not None:
        timespecifier = timedGD.interval().getText()
    elif timedGD.timeSpecifier() is not None:
        timespecifier = timedGD.timeSpecifier().getText()
    return TimedFormula(timespecifier, gd)



class PrefTimedGoalDescription:
    """ represents a timed goal description, possibly with a preference"""
    def __init__(self, timedgd, prefname = None):
        self.timedgd = timedgd
        self.prefname = prefname
        assert self.prefname is not None

    def asPDDL(self):
        return "(preference " + self.prefname + " " + self.timedgd.asPDDL() + ")"

def parsePrefTimedGoalDescription(prefTimedGD):
    timedGD = parseTimedGoalDescription(prefTimedGD.timedGD())
    name = prefTimedGD.name()
    if name is not None:
        raise Exception("Can't handle preferences " + prefTimedGD.getText())
        return PrefTimedGoalDescription(name, timedGD)
    else:
        return timedGD


def parseCEffect(ceff):
    if ceff.condEffect() is not None:
        raise Exception("Can't handle conditional effect " + ceff.getText())
    elif ceff.typedVariableList() is not None:
        raise Exception("Can't handle quantified effect " + ceff.getText())
    else:
        assert ceff.pEffect() is not None
        return parsePEffect(ceff.pEffect())

def parsePEffect(peff):
    if peff.assignOp() is not None:
        op = peff.assignOp().getText()
        head = parseFHead(peff.fHead())
        exp = parseFExp(peff.fExp())
        return Formula([head, exp], op, is_effect=True, is_numeric=True)        
    else:
        return parseGoalDescription(peff, is_effect=True)        

def parseTimedEffect(timedEffect):
    timespecifier = timedEffect.timeSpecifier().getText()
    if timedEffect.cEffect() is not None:
        ceff = parseCEffect(timedEffect.cEffect())
        return TimedFormula(timespecifier, ceff)
    else:
        raise Exception("Don't know how to handle effect " + timedEffect.getText())

def parseDaEffect(daEffect):
    if daEffect.timedEffect() is not None:
        te = parseTimedEffect(daEffect.timedEffect())
        return [te]
    else:
        op = daEffect.getChild(1).getText()
        assert op == 'and'
        effs = []
        for p in daEffect.daEffect():
            effs = effs + parseDaEffect(p)
        return effs


class Action:
    """ represents a (non-durative) action"""
    def __init__(self, name, parameters, pre, eff):
        self.name = name
        self.parameters = parameters
        self.pre = pre                  #precondition formula
        self.eff = eff                  #list of effects

    def get_pre(self, positive):        
        return self.pre.get_predicates(positive)

    def get_eff(self, positive):
        l = []
        for x in self.eff:
            l = l + x.get_predicates(positive)
        return l
            

    def asPDDL(self):
        ret = ""
        ret = ret + "(:action " + self.name + "\n"
        ret = ret + "\t:parameters (" + self.parameters.asPDDL() + ")\n"
        ret = ret + "\t:precondition " + self.pre.asPDDL() + "\n"
        ret = ret + "\t:effect (and "  + " ".join(map(lambda x: x.asPDDL(), self.eff)) + ")\n"
        ret = ret + ")"
        return ret

def parseAction(act):
    name = act.actionSymbol().getText()
    parameters = parseTypeVariableList(act.typedVariableList())
    
    body = act.actionDefBody()

    action_cond = []
    pre = parseGoalDescription(body.precondition().goalDesc())

    effs = list(map(lambda x: parseCEffect(x), body.effect().cEffect()))
            
    return Action(name, parameters, pre, effs)


class DurativeAction:
    """ represents a durative action"""
    def __init__(self, name, parameters, duration_lb, duration_ub, cond, eff):
        self.name = name
        self.parameters = parameters
        self.duration_lb = duration_lb
        self.duration_ub = duration_ub
        self.cond = cond               # list of conditions
        self.eff = eff                 # list of effects

    def get_cond(self, timespecifier, positive):
        l = []
        for x in self.cond:
            if x.timespecifier == timespecifier:
                l = l + x.formula.get_predicates(positive)
        return l

    def get_eff(self, timespecifier, positive):
        l = []
        for x in self.eff:
            if x.timespecifier == timespecifier:
                l = l + x.formula.get_predicates(positive)
        return l
            

    def asPDDL(self):
        ret = ""
        ret = ret + "(:durative-action " + self.name + "\n"
        ret = ret + "\t:parameters (" + self.parameters.asPDDL() + ")\n"
        ret = ret + "\t:duration "
        if self.duration_lb == self.duration_ub:
            ret = ret + "(= ?duration " + self.duration_lb.asPDDL() + ")\n"
        else:
            ret = ret + "(and (<= ?duration " + self.duration_ub.asPDDL() + ") (>= ?duration " + self.duration_lb.asPDDL() + "))\n"
        ret = ret + "\t:condition (and "  + " ".join(map(lambda x: x.asPDDL(), self.cond)) + ")\n"
        ret = ret + "\t:effect (and "  + " ".join(map(lambda x: x.asPDDL(), self.eff)) + ")\n"
        ret = ret + ")"
        return ret


class FHead:
    """ represents a functional symbol and terms, e.g.,  (f a b c)"""
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def asPDDL(self):
        return "(" + self.name + " " + self.args.asPDDL() + ")"

def parseFHead(fhead):
    terms = []
    for t in fhead.term():
        if t.VARIABLE() is not None:
            terms.append(TypedArg(t.VARIABLE().getText()))
        elif t.name() is not None:
            terms.append(TypedArg(t.name().getText()))
        else:
            raise Exception("Can't handle term " + fhead.getText())
    return FHead(fhead.functionSymbol().name().getText(), TypedArgList(terms))

class ConstantNumber:
    """ represents a constant number"""
    def __init__(self, val):
        self.val = val

    def asPDDL(self):
        return str(self.val)

    def __eq__(self, other):
        return isinstance(other, ConstantNumber) and self.val == other.val
    
class TotalTime:
    """ represents (total-time)"""
    def __init__(self):
        pass
    
    def asPDDL(self):
        return "total-time"

    def __eq__(self, other):
        return isinstance(other, TotalTime)    

def parseConstantNumber(number):
    return ConstantNumber(float(number.getText()))


class FExpression:
    """ represents a functional / numeric expression"""
    def __init__(self, op, subexps):
        self.op = op
        self.subexps = subexps     

    def asPDDL(self):
        #if self.op == '-':
        #    assert len(self.subexps) == 1
        #    return "(-" + self.subexps[0].asPDDL() + ")"
        #else:
        return "(" + self.op + " " + " ".join(map(lambda x: x.asPDDL(), self.subexps))  + ")"

def parseFExp(fexp):
    if fexp.NUMBER() is not None:
        return parseConstantNumber(fexp.NUMBER())
    elif fexp.fHead() is not None:
        return parseFHead(fexp.fHead())
    else:
        op = None
        fexp1 = parseFExp(fexp.fExp())        
        if fexp.binaryOp() is not None:
            op = fexp.binaryOp().getText()
            fexp2 = parseFExp(fexp.fExp2().fExp())
            return FExpression(op, [fexp1, fexp2])
        else:
            op = "-"
            return FExpression(op, [fexp1])
        return 
    
def parseMetricFExp(fexp):
    if fexp.NUMBER() is not None:
        return parseConstantNumber(fexp.NUMBER())
    elif fexp.functionSymbol() is not None:             
        return FHead(fexp.functionSymbol().name().getText(), TypedArgList(list(map(lambda x: TypedArg(x.NAME().getText()), fexp.name()))))         
    elif fexp.getText() == 'total-time':
        return TotalTime()    
    else:
        op = None
        subexps = list(map(parseMetricFExp, fexp.metricFExp()))                    
        if fexp.binaryOp() is not None:
            op = fexp.binaryOp().getText()
        else:
            assert fexp.getChildCount() > 1
            op = fexp.getChild(1).getText()
        return FExpression(op, subexps)    


def parseSimpleDurationConstraint(sdc):
    op = sdc.durOp().getText()
    

    if sdc.durValue().NUMBER() is not None:
        val = parseConstantNumber(sdc.durValue().NUMBER())
    elif sdc.durValue().fExp() is not None:
        val = parseFExp(sdc.durValue().fExp())
    return (op, val)

def parseDurativeAction(da):
    name = da.actionSymbol().getText()
    parameters = parseTypeVariableList(da.typedVariableList())
    
    body = da.daDefBody()

    duration = body.durationConstraint().simpleDurationConstraint()
    duration_lb = None
    duration_ub = None
    if duration is not None:
        if len(duration) == 1:
            d = parseSimpleDurationConstraint(duration[0])
            assert d[0] == '='
            duration_lb = d[1]
            duration_ub = d[1]
        else:
            assert len(duration) == 2
            d1 = parseSimpleDurationConstraint(duration[0])
            d2 = parseSimpleDurationConstraint(duration[1])
            if d1[0] == '<=':
                assert d2[0] == '>='
                duration_lb = d2[1]
                duration_ub = d1[1]
            elif d1[0] == '>=':
                assert d2[0] == '<='
                duration_lb = d1[1]
                duration_ub = d2[1]
            else:
                raise Exception("Can't parse duration " + duration.getText())
    
    

    action_cond = []
    cond = body.daGD()
    if cond.typedVariableList() is not None:
        raise Exception("Can't handle forall " + cond.getText())
    elif cond.prefTimedGD() is not None:
        action_cond.append(parsePrefTimedGoalDescription(cond.prefTimedGD()))
    elif cond.daGD() is not None:
        for x in cond.daGD():
            action_cond.append(parsePrefTimedGoalDescription(x.prefTimedGD()))

    effs = parseDaEffect(body.daEffect())
            
    return DurativeAction(name, parameters, duration_lb, duration_ub, action_cond, effs)


class Domain:
    """ represents a PDDL domain"""
    def __init__(self, name, reqs, types, constants, predicates, functions, actions, durative_actions):
        self.name = name
        self.reqs = reqs
        self.types = types
        self.constants = constants
        self.predicates = predicates
        self.functions = functions
        self.actions = actions
        self.durative_actions = durative_actions

    def asPDDL(self):
        ret = ""
        ret = ret + "(define (domain " + self.name + ")\n"
        ret = ret + "\t(:requirements " + " ".join(self.reqs) + ")\n"
        ret = ret + "\t(:types " + self.types.asPDDL() + ")\n"
        ret = ret + "\t(:constants " + self.constants.asPDDL() + ")\n"
        
        if len(self.functions) > 0:
            ret = ret + "\t(:functions\n"
            for func in self.functions:
                ret = ret + "\t\t" + func.asPDDL() + "\n"
            ret = ret + "\t)\n"
        
        if len(self.predicates) > 0:
            ret = ret + "\t(:predicates\n"
            for pred in self.predicates:
                ret = ret + "\t\t" + pred.asPDDL() + "\n"
            ret = ret + "\t)\n"


        for a in self.actions:   
            ret = ret + a.asPDDL() + "\n"


        for da in self.durative_actions:   
            ret = ret + da.asPDDL() + "\n"

        ret = ret + ")"
        return ret
        

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
            functions.append(Function(func.functionSymbol().name().getText(), parseTypeVariableList(func.typedVariableList())))

    # Get predicates
    predicates = []
    if domain.predicatesDef() is not None:
        for pred in domain.predicatesDef().atomicFormulaSkeleton():
            predicates.append(Predicate(pred.predicate().name().getText(), parseTypeVariableList(pred.typedVariableList())))

    # Get actions and durative actions
    durative_actions = []
    actions = []
    #derived = []
    for action in domain.structureDef():
        if action.actionDef() is not None:
            actions.append(parseAction(action.actionDef()))
        elif action.durativeActionDef() is not None:
            durative_actions.append(parseDurativeAction(action.durativeActionDef()))
        #elif action.derivedDef() is not None:
        #    derived.append(
        #                   [parseTypeVariableList(action.derivedDef().typedVariableList()), parseGoalDescription(action.derivedDef().goalDesc())]
        #                   )

    
    d = Domain(domainname, reqs, types, constants, predicates, functions, actions, durative_actions)
    return d
        
class Metric:
    """ represents a metric/optimization objective"""
    def __init__(self, objective, fexp):
        self.objective = objective
        self.fexp = fexp
        
    def asPDDL(self):
        return "(:metric " + self.objective + " " + self.fexp.asPDDL() + ")"
    
class Problem:
    """ represents a PDDL problem"""
    def __init__(self, name, domainname, objects, initialstate, goal, metric=None):
        self.name = name
        self.domainname = domainname
        self.objects = objects
        self.initialstate = initialstate
        self.goal = goal
        self.metric = metric

    def asPDDL(self):
        ret = ""
        ret = ret + "(define (problem " + self.name + ")\n"
        ret = ret + "\t(:domain " + self.domainname + ")\n"
        ret = ret + "\t(:objects " + self.objects.asPDDL() + ")\n"
        ret = ret + "\t(:init \n"
        for initel in self.initialstate:
            ret = ret + "\t\t" + initel.asPDDL() + "\n"
        ret = ret + "\t)\n"
        ret = ret + "\t(:goal " +  self.goal.asPDDL() + ")\n"
        if self.metric is not None:
            ret = ret + "\t" + self.metric.asPDDL() + "\n"
        ret = ret + ")"
        return ret
        
def parseNameLiteral(nameLiteral):
    name = nameLiteral.atomicNameFormula().predicate().name().getText()
    terms = []
    for t in nameLiteral.atomicNameFormula().name():
        terms.append(TypedArg(t.NAME().getText()))
    op = None
    if nameLiteral.getChildCount() > 1 and nameLiteral.getChild(1).getText() == 'not':
        op = "not"
    return Formula([Predicate(name, TypedArgList(terms))], op)

    
def parseInitStateElement(initel):
    if initel.getChildCount() > 1 and initel.nameLiteral() and initel.getChild(1).getText() == 'at':
        time = float(initel.NUMBER().getText())       
        return TimedFormula(time, parseNameLiteral(initel.nameLiteral()))
    elif initel.nameLiteral() is not None: 
        return parseNameLiteral(initel.nameLiteral())
    elif initel.fHead() is not None:
        fhead = parseFHead(initel.fHead())
        val = parseConstantNumber(initel.NUMBER())
        return FExpression("=", [fhead, val])
    else:
        raise Exception("Don't know how to handle initial element " + initel.getText())


def parseProblem(problem):
    name = problem.problemDecl().name().getText()
    domain = problem.problemDomain().name().getText()
    if problem.objectDecl() is not None:
        objects = parseTypeNameList(problem.objectDecl().typedNameList())
    else:
        objects = TypedArgList([])

    init = []
    for initel in problem.init().initEl():
        init.append(parseInitStateElement(initel))

    goal = parseGoalDescription(problem.goal().goalDesc())
        
    
    metric = None
    if problem.metricSpec() is not None:
        met = problem.metricSpec()
        optimization = met.optimization().getText()
        fexp = parseMetricFExp(met.metricFExp())
        metric = Metric(optimization, fexp)
        print(metric.asPDDL())

    return Problem(name, domain, objects, init, goal, metric)

def readAndParseFile(file):
    inp = FileStream(file)
    lexer = pddlLexer.pddlLexer(inp)
    stream = CommonTokenStream(lexer)
    parser = pddlParser.pddlParser(stream)
    return parser

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

