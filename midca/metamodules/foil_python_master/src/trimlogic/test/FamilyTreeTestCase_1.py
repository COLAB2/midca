import logging
import unittest
from midca.metamodules.foil_python_master.src.trimlogic.test.helper import FoilTestCase
from midca.metamodules.foil_python_master.src.trimlogic.predicate import KnowledgeBase
from midca.metamodules.foil_python_master.src.trimlogic.predicate import RuleBasedPredicate
from midca.metamodules.foil_python_master.src.trimlogic.predicate import VariableFactory, UniqueVariableFactory, AtomFactory
from midca.metamodules.foil_python_master.src.trimlogic.foil import TrainingSet, find_gainful_and_determinate_literals, construct_clause_recursive, foil, find_partial_ordering_of_terms, determine_param_orderings
from midca.metamodules.foil_python_master.src.trimlogic.algorithm import fol_bc_ask
from midca.metamodules.foil_python_master.src.trimlogic.term import Atom,Position,Time
global native,invasive,spray,no_native,no_invasive

rule_file = "/home/zahid/Music/midca/midca/metamodules/foil-python-master/src/trimlogic/test/rule.txt"

class FamilyMemberFactory:
  
  def __getattr__(self, name):
    return FamilyMember(name)

class PositionFactory:
  
  def __getattr__(self, name):
    return Position(name)

class TimeFactory:
  
  def __getattr__(self, name):
    return Time(name)

class Position(Atom):
  
  def __init__(self, name):
    Atom.__init__(self, name)
  def __call__(self,name):
    pass
  
  def set_ordering(self, ordering):
    self._ordering = ordering
    def compare(x,y):
        return cmp(self._ordering.index(x), self._ordering.index(y))
    Position.__cmp__ = compare
    
  set_ordering = classmethod(set_ordering)

class Time(Atom):
  
  def __init__(self, name):
    Atom.__init__(self, name)
  def __call__(self,name):
    pass
  
  def set_ordering(self, ordering):
    self._ordering = ordering
    def compare(x,y):
          return cmp(self._ordering.index(x), self._ordering.index(y))
    Time.__cmp__ = compare
    
  set_ordering = classmethod(set_ordering)


class FamilyMember(Atom):
  
  def __init__(self, name):
    Atom.__init__(self, name)
  def __call__(self,name):
    pass
  
  def set_ordering(self, ordering):
    self._ordering = ordering
    def compare(x,y):
      return cmp(self._ordering.index(x), self._ordering.index(y))
    FamilyMember.__cmp__ = compare
    
  set_ordering = classmethod(set_ordering)


class FamilyTreeTestCase(FoilTestCase):
  
  def setUp(self):
    pass
    
  def loadFamilyTree1(self):
    """
    Frank ________ Rebecca             George ________ Jannet
           |     |                             |     |
          Abe   Alan ______ Joan  Jill ______ Bob    Tim ____ Tammy
                        |             |                   |
                       Sean ________ Jan       Tipsy ___ Tom
                               |                      |
                              Jane ____ Ian         Thomas ___ Debrah
                                    |                       |
                                   Ann                     Billy
    """
    v, a = self.v, self.a
    p,t = self.p,self.t
    #print(type(p))
    #ordering = [a.native(0_0),a.native(0_1),a.native(1_0),a.native(1_1),a.invasive(0_0),a.invasive(0_1),a.invasive(1_0),a.invasive(1_1),a.spray(0_0),a.spray(0_1),a.spray(1_0),a.spray(1_1),a.no_native(0_0),a.no_native(0_1),a.no_native(1_0),a.no_native(1_1),a.no_invasive(0_0),a.no_invasive(0_1),a.no_invasive(1_0),a.no_invasive(1_1)]
    
    '''ordering_1 = [p.p(0,0),p.p(0,1),p.p(0,2),p.p(0,3),p.p(0,4),p.p(0,5),p.p(1,0),p.p(1,1),p.p(1,2),p.p(1,3),p.p(1,4),p.p(1,5),p.p(2,0),p.p(2,1),p.p(2,2),p.p(2,3),p.p(2,4),p.p(2,5),p.p(3,0),p.p(3,1),p.p(3,2),p.p(3,3),p.p(3,4),p.p(3,5),p.p(4,0),p.p(4,1),p.p(4,2),p.p(4,3),p.p(4,4),p.p(4,5),p.p(5,0),p.p(5,1),p.p(5,2),p.p(5,3),p.p(5,4),p.p(5,5)]'''
    ordering_1 = [p.p0_0,p.p0_1,p.p0_2,p.p0_3,p.p0_4,p.p0_5,p.p0_6,p.p0_7,p.p0_8,p.p0_9,p.p1_0,p.p1_1,p.p1_2,p.p1_3,p.p1_4,p.p1_5,p.p1_6,p.p1_7,p.p1_8,p.p1_9,p.p2_0,p.p2_1,p.p2_2,p.p2_3,p.p2_4,p.p2_5,p.p2_6,p.p2_7,p.p2_8,p.p2_9,p.p3_0,p.p3_1,p.p3_2,p.p3_3,p.p3_4,p.p3_5,p.p3_6,p.p3_7,p.p3_8,p.p3_9,p.p4_0,p.p4_1,p.p4_2,p.p4_3,p.p4_4,p.p4_5,p.p4_6,p.p4_7,p.p4_8,p.p4_9,p.p5_0,p.p5_1,p.p5_2,p.p5_3,p.p5_4,p.p5_5,p.p5_6,p.p5_7,p.p5_8,p.p5_9,p.p6_0,p.p6_1,p.p6_2,p.p6_3,p.p6_4,p.p6_5,p.p6_6,p.p6_7,p.p6_8,p.p6_9,
                  p.p7_0,p.p7_1,p.p7_2,p.p7_3,p.p7_4,p.p7_5,p.p7_6,p.p7_7,p.p7_8,p.p7_9,p.p8_0,p.p8_1,p.p8_2,p.p8_3,p.p8_4,p.p8_5,p.p8_6,p.p8_7,p.p8_8,p.p8_9,p.p9_0,p.p9_1,p.p9_2,p.p9_3,p.p9_4,p.p9_5,p.p9_6,p.p9_7,p.p9_8,p.p9_9]
    
    Position.set_ordering(ordering_1)

    ordering_2 = [t.t1,t.t2,t.t3,t.t4,t.t5,t.t6,t.t7]
    Time.set_ordering(ordering_2)

    #self.native.rules=[]
    #self.adjacent.rules=[]
    self.ISADJ_south.rules=[]
    self.ISADJ_north.rules=[]
    self.ISADJ_east.rules=[]
    self.ISADJ_west.rules=[]
    self.ISADJ_noes.rules=[]
    self.ISADJ_nows.rules=[]
    self.ISADJ_soes.rules=[]
    self.ISADJ_sows.rules=[]
    self.plant.rules=[]
    #self.invasive.rules=[]
    self.ISADJ_time.rules=[]
    self.spray.rules=[]
    #Fact.set_ordering(ordering)
    #Position.set_ordering(ordering)
    #self.father.rules = []
    #self.native.param_orderings = None
    #self.adjacent.param_orderings = None
    self.ISADJ_south.param_orderings = None
    self.ISADJ_north.param_orderings = None
    self.ISADJ_east.param_orderings = None
    self.ISADJ_west.param_orderings = None
    self.ISADJ_noes.param_orderings = None
    self.ISADJ_soes.param_orderings = None
    self.ISADJ_nows.param_orderings = None
    self.ISADJ_sows.param_orderings = None
    self.plant.param_orderings = None
    #self.invasive.param_orderings = None
    self.spray.param_orderings = None
    self.ISADJ_time.param_orderings= None
    #map(self.native.add_rule,[(p.p1_0, t.t1), (p.p1_1, t.t1), (p.p3_3, t.t1), (p.p1_0, t.t2), (p.p1_1, t.t2), (p.p3_3, t.t2), (p.p1_0, t.t3), (p.p1_1, t.t3)])
    map(self.plant.add_rule,[(p.p2_3,t.t1),(p.p1_3,t.t1),(p.p2_2,t.t1),(p.p4_4,t.t1),(p.p2_3,t.t2),(p.p1_3,t.t2),(p.p4_4,t.t2),(p.p1_2,t.t1),(p.p4_3,t.t1),(p.p2_1,t.t1),(p.p1_2,t.t2),(p.p4_3,t.t2)])
    map(self.ISADJ_time.add_rule,[( t.t1,t.t2), (t.t2,t.t3),(t.t3,t.t4), (t.t4,t.t5),(t.t5,t.t6),(t.t6,t.t7)])
    #map(self.adjacent.add_rule,[(p.p0_0,p.p0_1),(p.p0_0,p.p1_0),(p.p0_0,p.p1_1),(p.p1_0,p.p0_0),(p.p1_0,p.p0_1),(p.p1_0,p.p1_1),(p.p1_0,p.p2_1),(p.p1_0,p.p2_2),(p.p2_0,p.p1_0),(p.p2_0,p.p1_1),(p.p2_0,p.p2_1),(p.p2_0,p.p3_1),(p.p2_0,p.p3_0),(p.p3_0,p.p2_0),(p.p3_0,p.p2_1),(p.p3_0,p.p3_1),(p.p3_0,p.p4_1),(p.p3_0,p.p4_0),(p.p4_0,p.p3_0),(p.p4_0,p.p3_1),(p.p4_0,p.p4_1),(p.p0_1,p.p0_0),(p.p0_1,p.p0_2),(p.p0_1,p.p1_2),(p.p0_1,p.p1_1),(p.p0_1,p.p1_0),(p.p1_1,p.p0_0),(p.p1_1,p.p0_1),(p.p1_1,p.p0_2),(p.p1_1,p.p1_2),(p.p1_1,p.p1_0),(p.p1_1,p.p2_2),(p.p1_1,p.p2_1),(p.p1_1,p.p2_0),(p.p2_1,p.p1_0),(p.p2_1,p.p1_1),(p.p2_1,p.p1_2),(p.p2_1,p.p2_2),(p.p2_1,p.p2_0),(p.p2_1,p.p3_0),(p.p2_1,p.p3_1),(p.p2_1,p.p3_2),(p.p3_1,p.p2_0),(p.p3_1,p.p2_1),(p.p3_1,p.p2_2),(p.p3_1,p.p3_2),(p.p3_1,p.p3_0),(p.p3_1,p.p4_0),(p.p3_1,p.p4_1),(p.p3_1,p.p4_2),(p.p4_1,p.p3_0),(p.p4_1,p.p3_1),(p.p4_1,p.p3_2),(p.p4_1,p.p4_0),(p.p4_1,p.p4_2),(p.p0_2,p.p0_3),(p.p0_2,p.p0_1),(p.p0_2,p.p1_3),(p.p0_2,p.p1_2),(p.p0_2,p.p1_1),(p.p1_2,p.p0_1),(p.p1_2,p.p0_2),(p.p1_2,p.p0_3),(p.p1_2,p.p1_1),(p.p1_2,p.p1_3),(p.p1_2,p.p2_1),(p.p1_2,p.p2_2),(p.p1_2,p.p2_3),(p.p2_2,p.p1_1),(p.p2_2,p.p1_2),(p.p2_2,p.p1_3),(p.p2_2,p.p2_1),(p.p2_2,p.p2_3),(p.p2_2,p.p3_1),(p.p2_2,p.p3_2),(p.p2_2,p.p3_3),(p.p3_2,p.p2_1),(p.p3_2,p.p2_2),(p.p3_2,p.p2_3),(p.p3_2,p.p3_1),(p.p3_2,p.p3_3),(p.p3_2,p.p4_1),(p.p3_2,p.p4_2),(p.p3_2,p.p4_3),(p.p4_2,p.p3_1),(p.p4_2,p.p3_2),(p.p4_2,p.p3_3),(p.p4_2,p.p4_1),(p.p4_2,p.p4_3),(p.p0_3,p.p0_2),(p.p0_3,p.p0_4),(p.p0_3,p.p1_2),(p.p0_3,p.p1_3),(p.p0_3,p.p1_3),(p.p1_3,p.p0_2),(p.p1_3,p.p0_3),(p.p1_3,p.p0_4),(p.p1_3,p.p1_2),(p.p1_3,p.p1_4),(p.p1_3,p.p2_2),(p.p1_3,p.p2_3),(p.p1_3,p.p2_4),(p.p2_3,p.p1_2),(p.p2_3,p.p1_3),(p.p2_3,p.p1_4),(p.p2_3,p.p2_2),(p.p2_3,p.p2_4),(p.p2_3,p.p3_2),(p.p2_3,p.p3_3),(p.p2_3,p.p3_4),(p.p3_3,p.p2_2),(p.p3_3,p.p2_3),(p.p3_3,p.p2_4),(p.p3_3,p.p3_2),(p.p3_3,p.p3_4),(p.p3_3,p.p4_2),(p.p3_3,p.p4_3),(p.p3_3,p.p4_4),(p.p4_3,p.p3_2),(p.p4_3,p.p3_3),(p.p4_3,p.p3_4),(p.p4_3,p.p4_2),(p.p4_3,p.p4_4),(p.p0_4,p.p0_3),(p.p0_4,p.p1_3),(p.p0_4,p.p1_4),(p.p1_4,p.p0_3),(p.p1_4,p.p0_4),(p.p1_4,p.p1_3),(p.p1_4,p.p2_3),(p.p1_4,p.p2_4),(p.p2_4,p.p1_3),(p.p2_4,p.p1_4),(p.p2_4,p.p2_3),(p.p2_4,p.p3_3),(p.p2_4,p.p3_4),(p.p3_4,p.p2_3),(p.p3_4,p.p2_4),(p.p3_4,p.p3_3),(p.p3_4,p.p4_3),(p.p3_4,p.p4_4),(p.p4_4,p.p3_3),(p.p4_4,p.p3_4),(p.p4_4,p.p4_3)])
    map(self.ISADJ_south.add_rule,
        [(p.p0_1, p.p0_0), (p.p0_2, p.p0_1), (p.p0_3, p.p0_2), (p.p0_4, p.p0_3), (p.p0_5, p.p0_4),(p.p0_6, p.p0_5),(p.p0_7, p.p0_6),(p.p0_8, p.p0_7),(p.p0_9, p.p0_8), (p.p1_1, p.p1_0),
         (p.p1_2, p.p1_1), (p.p1_3, p.p1_2), (p.p1_4, p.p1_3), (p.p1_5, p.p1_4),(p.p1_6, p.p1_5),(p.p1_7, p.p1_6),(p.p1_8, p.p1_7),(p.p1_9, p.p1_8), (p.p2_1, p.p2_0), (p.p2_2, p.p2_1),
         (p.p2_3, p.p2_2), (p.p2_4, p.p2_3), (p.p2_5, p.p2_4),(p.p2_6, p.p2_5),(p.p2_7, p.p2_6),(p.p2_8, p.p2_7),(p.p2_9, p.p2_8), (p.p3_1, p.p3_0), (p.p3_2, p.p3_1), (p.p3_3, p.p3_2),
         (p.p3_4, p.p3_3), (p.p3_5, p.p3_4),(p.p3_6, p.p3_5),(p.p3_7, p.p3_6),(p.p3_8, p.p3_7),(p.p3_9, p.p3_8), (p.p4_1, p.p4_0), (p.p4_2, p.p4_1), (p.p4_3, p.p4_2), (p.p4_4, p.p4_3),
         (p.p4_5, p.p4_4),(p.p4_6, p.p4_5),(p.p4_7, p.p4_6),(p.p4_8, p.p4_7),(p.p4_9, p.p4_8), (p.p5_1, p.p5_0), (p.p5_2, p.p5_1), (p.p5_3, p.p5_2), (p.p5_4, p.p5_3), (p.p5_5, p.p5_4),(p.p5_6, p.p5_5),(p.p5_7, p.p5_6),(p.p5_8, p.p5_7),(p.p5_9, p.p5_8),
         (p.p6_1, p.p6_0), (p.p6_2, p.p6_1), (p.p6_3, p.p6_2), (p.p6_4, p.p6_3), (p.p6_5, p.p6_4),(p.p6_6, p.p6_5),(p.p6_7, p.p6_6),(p.p6_8, p.p6_7),(p.p6_9, p.p6_8),
         (p.p7_1, p.p7_0), (p.p7_2, p.p7_1), (p.p7_3, p.p7_2), (p.p7_4, p.p7_3), (p.p7_5, p.p7_4),(p.p7_6, p.p7_5),(p.p7_7, p.p7_6),(p.p7_8, p.p7_7),(p.p7_9, p.p7_8),
         (p.p8_1, p.p8_0), (p.p8_2, p.p8_1), (p.p8_3, p.p8_2), (p.p8_4, p.p8_3), (p.p8_5, p.p8_4),(p.p8_6, p.p8_5),(p.p8_7, p.p8_6),(p.p8_8, p.p8_7),(p.p8_9, p.p8_8),
         (p.p9_1, p.p9_0), (p.p9_2, p.p9_1), (p.p9_3, p.p9_2), (p.p9_4, p.p9_3), (p.p9_5, p.p9_4),(p.p9_6, p.p9_5),(p.p9_7, p.p9_6),(p.p9_8, p.p9_7),(p.p9_9, p.p9_8)])

    map(self.ISADJ_north.add_rule,
        [(p.p0_0, p.p0_1), (p.p0_1, p.p0_2), (p.p0_2, p.p0_3), (p.p0_3, p.p0_4), (p.p0_4, p.p0_5),(p.p0_5, p.p0_6),(p.p0_6, p.p0_7),(p.p0_7, p.p0_8),(p.p0_8, p.p0_9), (p.p1_0, p.p1_1),
         (p.p1_1, p.p1_2), (p.p1_2, p.p1_3), (p.p1_3, p.p1_4), (p.p1_4, p.p1_5),(p.p1_5, p.p1_6),(p.p1_6, p.p1_7),(p.p1_7, p.p1_8),(p.p1_8, p.p1_9), (p.p2_0, p.p2_1), (p.p2_1, p.p2_2),
         (p.p2_2, p.p2_3), (p.p2_3, p.p2_4), (p.p2_4, p.p2_5),(p.p2_5, p.p2_6),(p.p2_6, p.p2_7),(p.p2_7, p.p2_8),(p.p2_8, p.p2_9), (p.p3_0, p.p3_1), (p.p3_1, p.p3_2), (p.p3_2, p.p3_3),
         (p.p3_3, p.p3_4), (p.p3_4, p.p3_5),(p.p3_5, p.p3_6),(p.p3_6, p.p3_7),(p.p3_7, p.p3_8),(p.p3_8, p.p3_9), (p.p4_0, p.p4_1), (p.p4_1, p.p4_2), (p.p4_2, p.p4_3), (p.p4_3, p.p4_4),
         (p.p4_4, p.p4_5),(p.p4_5, p.p4_6),(p.p4_6, p.p4_7),(p.p4_7, p.p4_8),(p.p4_8, p.p4_9), (p.p5_0, p.p5_1), (p.p5_1, p.p5_2), (p.p5_2, p.p5_3), (p.p5_3, p.p5_4), (p.p5_4, p.p5_5),(p.p5_5, p.p5_6),(p.p5_6, p.p5_7),(p.p5_7, p.p5_8),(p.p5_8, p.p5_9),
         (p.p6_0, p.p6_1), (p.p6_1, p.p6_2), (p.p6_2, p.p6_3), (p.p6_3, p.p6_4), (p.p6_4, p.p6_5),(p.p6_5, p.p6_6),(p.p6_6, p.p6_7),(p.p6_7, p.p6_8),(p.p6_8, p.p6_9),
         (p.p7_0, p.p7_1), (p.p7_1, p.p7_2), (p.p7_2, p.p7_3), (p.p7_3, p.p7_4), (p.p7_4, p.p7_5),(p.p7_5, p.p7_6),(p.p7_6, p.p7_7),(p.p7_7, p.p7_8),(p.p7_8, p.p7_9),
         (p.p8_0, p.p8_1), (p.p8_1, p.p8_2), (p.p8_2, p.p8_3), (p.p8_3, p.p8_4), (p.p8_4, p.p8_5),(p.p8_5, p.p8_6),(p.p8_6, p.p8_7),(p.p8_7, p.p8_8),(p.p8_8, p.p8_9),
         (p.p9_0, p.p9_1), (p.p9_1, p.p9_2), (p.p9_2, p.p9_3), (p.p9_3, p.p9_4), (p.p9_4, p.p9_5),(p.p9_5, p.p9_6),(p.p9_6, p.p9_7),(p.p9_7, p.p9_8),(p.p9_8, p.p9_9)])
    # map(self.ISADJ_north.add_rule,[(p.p0_1, p.p0_0),(p.p0_2, p.p0_1),(p.p0_3, p.p0_2),(p.p0_4, p.p0_3),(p.p0_5, p.p0_4),(p.p1_1, p.p1_0),(p.p1_2, p.p1_1),(p.p1_3, p.p1_2),(p.p1_4, p.p1_3),(p.p1_5,p.p1_4),(p.p2_1,p.p2_0),(p.p2_2,p.p2_1),(p.p2_3,p.p2_2),(p.p2_4,p.p2_3),(p.p2_5,p.p2_4),(p.p3_1,p.p3_0),(p.p3_2,p.p3_1),(p.p3_3,p.p3_2),(p.p3_4,p.p3_3),(p.p3_5,p.p3_4),(p.p4_1,p.p4_0),(p.p4_2,p.p4_1),(p.p4_3,p.p4_2),(p.p4_4,p.p4_3),(p.p4_5,p.p4_4),(p.p5_1,p.p5_0),(p.p5_2,p.p5_1),(p.p5_3,p.p5_2),(p.p5_4,p.p5_3),(p.p5_5,p.p5_4)])
    map(self.ISADJ_east.add_rule,
        [(p.p0_0, p.p1_0), (p.p1_0, p.p2_0), (p.p2_0, p.p3_0), (p.p3_0, p.p4_0), (p.p4_0, p.p5_0),(p.p5_0, p.p6_0),(p.p6_0, p.p7_0),(p.p7_0, p.p8_0),(p.p8_0, p.p9_0), (p.p0_1, p.p1_1),
         (p.p1_1, p.p2_1), (p.p2_1, p.p3_1), (p.p3_1, p.p4_1), (p.p4_1, p.p5_1),(p.p5_1, p.p6_1),(p.p6_1, p.p7_1),(p.p7_1, p.p8_1),(p.p8_1, p.p9_1), (p.p0_2, p.p1_2), (p.p1_2, p.p2_2),
         (p.p2_2, p.p3_2), (p.p3_2, p.p4_2), (p.p4_2, p.p5_2),(p.p5_2, p.p6_2),(p.p6_2, p.p7_2),(p.p7_2, p.p8_2),(p.p8_2, p.p9_2), (p.p0_3, p.p1_3), (p.p1_3, p.p2_3), (p.p2_3, p.p3_3),
         (p.p3_3, p.p4_3), (p.p4_3, p.p5_3),(p.p5_3, p.p6_3),(p.p6_3, p.p7_3),(p.p7_3, p.p8_3),(p.p8_3, p.p9_3), (p.p0_4, p.p1_4), (p.p1_4, p.p2_4), (p.p2_4, p.p3_4), (p.p3_4, p.p4_4),
         (p.p4_4, p.p5_4),(p.p5_4, p.p6_4),(p.p6_4, p.p7_4),(p.p7_4, p.p8_4),(p.p8_4, p.p9_4), (p.p0_5, p.p1_5), (p.p1_5, p.p2_5), (p.p2_5, p.p3_5), (p.p3_5, p.p4_5), (p.p4_5, p.p5_5),(p.p5_5, p.p6_5),(p.p6_5, p.p7_5),(p.p7_5, p.p8_5),(p.p8_5, p.p9_5),
         (p.p0_6, p.p1_6), (p.p1_6, p.p2_6), (p.p2_6, p.p3_6), (p.p3_6, p.p4_6), (p.p4_6, p.p5_6),(p.p5_6, p.p6_6),(p.p6_6, p.p7_6),(p.p7_6, p.p8_6),(p.p8_6, p.p9_6),
         (p.p0_7, p.p1_7), (p.p1_7, p.p2_7), (p.p2_7, p.p3_7), (p.p3_7, p.p4_7), (p.p4_7, p.p5_7),(p.p5_7, p.p6_7),(p.p6_7, p.p7_7),(p.p7_7, p.p8_7),(p.p8_7, p.p9_7),
         (p.p0_8, p.p1_8), (p.p1_8, p.p2_8), (p.p2_8, p.p3_8), (p.p3_8, p.p4_8), (p.p4_8, p.p5_8),(p.p5_8, p.p6_8),(p.p6_8, p.p7_8),(p.p7_8, p.p8_8),(p.p8_8, p.p9_8),
         (p.p0_9, p.p1_9), (p.p1_9, p.p2_9), (p.p2_9, p.p3_9), (p.p3_9, p.p4_9), (p.p4_9, p.p5_9),(p.p5_9, p.p6_9),(p.p6_9, p.p7_9),(p.p7_9, p.p8_9),(p.p8_9, p.p9_9)])
    map(self.ISADJ_noes.add_rule,
        [(p.p0_0, p.p1_1), (p.p1_0, p.p2_1), (p.p2_0, p.p3_1), (p.p3_0, p.p4_1), (p.p4_0, p.p5_1),(p.p5_0, p.p6_1),(p.p6_0, p.p7_1),(p.p7_0, p.p8_1),(p.p8_0, p.p9_1), (p.p0_1, p.p1_2),
         (p.p1_1, p.p2_2), (p.p2_1, p.p3_2), (p.p3_1, p.p4_2), (p.p4_1, p.p5_2),(p.p5_1, p.p6_2),(p.p6_1, p.p7_2),(p.p7_1, p.p8_2),(p.p8_1, p.p9_2), (p.p0_2, p.p1_3), (p.p1_2, p.p2_3),
         (p.p2_2, p.p3_3), (p.p3_2, p.p4_3), (p.p4_2, p.p5_3),(p.p5_2, p.p6_3),(p.p6_2, p.p7_3),(p.p7_2, p.p8_3),(p.p8_2, p.p9_3), (p.p0_3, p.p1_4), (p.p1_3, p.p2_4), (p.p2_3, p.p3_4),
         (p.p3_3, p.p4_4), (p.p4_3, p.p5_4),(p.p5_3, p.p6_4),(p.p6_3, p.p7_4),(p.p7_3, p.p8_4),(p.p8_3, p.p9_4), (p.p0_4, p.p1_5), (p.p1_4, p.p2_5), (p.p2_4, p.p3_5), (p.p3_4, p.p4_5),
         (p.p4_4, p.p5_5),(p.p5_4, p.p6_5),(p.p6_4, p.p7_5),(p.p7_4, p.p8_5),(p.p8_4, p.p9_5),
         (p.p0_5, p.p1_6), (p.p1_5, p.p2_6), (p.p2_5, p.p3_6), (p.p3_5, p.p4_6),(p.p4_5, p.p5_6),(p.p5_5, p.p6_6),(p.p6_5, p.p7_6),(p.p7_5, p.p8_6),(p.p8_5, p.p9_6),
         (p.p0_6, p.p1_7), (p.p1_6, p.p2_7), (p.p2_6, p.p3_7), (p.p3_6, p.p4_7),(p.p4_6, p.p5_7),(p.p5_6, p.p6_7),(p.p6_6, p.p7_7),(p.p7_6, p.p8_7),(p.p8_6, p.p9_7),
         (p.p0_7, p.p1_8), (p.p1_7, p.p2_8), (p.p2_7, p.p3_8), (p.p3_7, p.p4_8),(p.p4_7, p.p5_8),(p.p5_7, p.p6_8),(p.p6_7, p.p7_8),(p.p7_7, p.p8_8),(p.p8_7, p.p9_8),
         (p.p0_8, p.p1_9), (p.p1_8, p.p2_9), (p.p2_8, p.p3_9), (p.p3_8, p.p4_9),(p.p4_8, p.p5_9),(p.p5_8, p.p6_9),(p.p6_8, p.p7_9),(p.p7_8, p.p8_9),(p.p8_8, p.p9_9)])
    map(self.ISADJ_soes.add_rule,
        [(p.p0_1, p.p1_0),
         (p.p1_1, p.p2_0), (p.p2_1, p.p3_0), (p.p3_1, p.p4_0), (p.p4_1, p.p5_0),(p.p5_1, p.p6_0),(p.p6_1, p.p7_0),(p.p7_1, p.p8_0),(p.p8_1, p.p9_0), (p.p0_2, p.p1_1), (p.p1_2, p.p2_1),
         (p.p2_2, p.p3_1), (p.p3_2, p.p4_1), (p.p4_2, p.p5_1),(p.p5_2, p.p6_1),(p.p6_2, p.p7_1),(p.p7_2, p.p8_1),(p.p8_2, p.p9_1), (p.p0_3, p.p1_2), (p.p1_3, p.p2_2), (p.p2_3, p.p3_2),
         (p.p3_3, p.p4_2), (p.p4_3, p.p5_2),(p.p5_3, p.p6_2),(p.p6_3, p.p7_2),(p.p7_3, p.p8_2),(p.p8_3, p.p9_2), (p.p0_4, p.p1_3), (p.p1_4, p.p2_3), (p.p2_4, p.p3_3), (p.p3_4, p.p4_3),
         (p.p4_4, p.p5_3),(p.p5_4, p.p6_3),(p.p6_4, p.p7_3),(p.p7_4, p.p8_3),(p.p8_4, p.p9_3), (p.p0_5, p.p1_4), (p.p1_5, p.p2_4), (p.p2_5, p.p3_4), (p.p3_5, p.p4_4), (p.p4_5, p.p5_4),(p.p5_5, p.p6_4),(p.p6_5, p.p7_4),(p.p7_5, p.p8_4),(p.p8_5, p.p9_4),
         (p.p0_6, p.p1_5), (p.p1_6, p.p2_5), (p.p2_6, p.p3_5), (p.p3_6, p.p4_5), (p.p4_6, p.p5_5),(p.p5_6, p.p6_5),(p.p6_6, p.p7_5),(p.p7_6, p.p8_5),(p.p8_6, p.p9_5),
         (p.p0_7, p.p1_6), (p.p1_7, p.p2_6), (p.p2_7, p.p3_6), (p.p3_7, p.p4_6), (p.p4_7, p.p5_6),(p.p5_7, p.p6_6),(p.p6_7, p.p7_6),(p.p7_7, p.p8_6),(p.p8_7, p.p9_6),
         (p.p0_8, p.p1_7), (p.p1_8, p.p2_7), (p.p2_8, p.p3_7), (p.p3_8, p.p4_7), (p.p4_8, p.p5_7),(p.p5_8, p.p6_7),(p.p6_8, p.p7_7),(p.p7_8, p.p8_7),(p.p8_8, p.p9_7),
         (p.p0_9, p.p1_8), (p.p1_9, p.p2_8), (p.p2_9, p.p3_8), (p.p3_9, p.p4_8), (p.p4_9, p.p5_8),(p.p5_9, p.p6_8),(p.p6_9, p.p7_8),(p.p7_9, p.p8_8),(p.p8_9, p.p9_8)])
    map(self.ISADJ_nows.add_rule,
        [(p.p1_0, p.p0_1), (p.p2_0, p.p1_1), (p.p3_0, p.p2_1), (p.p4_0, p.p3_1),(p.p5_0, p.p4_1),(p.p6_0, p.p5_1),(p.p7_0, p.p6_1),(p.p8_0, p.p7_1),(p.p9_0, p.p8_1),
         (p.p1_1, p.p0_2), (p.p2_1, p.p1_2), (p.p3_1, p.p2_2), (p.p4_1, p.p3_2),(p.p5_1, p.p4_2),(p.p6_1, p.p5_2),(p.p7_1, p.p6_2),(p.p8_1, p.p7_2),(p.p9_1, p.p8_2), (p.p1_2, p.p0_3),
         (p.p2_2, p.p1_3), (p.p3_2, p.p2_3), (p.p4_2, p.p3_3),(p.p5_2, p.p4_3),(p.p6_2, p.p5_3),(p.p7_2, p.p6_3),(p.p8_2, p.p7_3),(p.p9_2, p.p8_3), (p.p1_3, p.p0_4), (p.p2_3, p.p1_4),
         (p.p3_3, p.p2_4), (p.p4_3, p.p3_4), (p.p5_3, p.p4_4), (p.p6_3, p.p5_4),(p.p7_3, p.p6_4),(p.p8_3, p.p7_4),(p.p9_3, p.p8_4),(p.p1_4, p.p0_5), (p.p2_4, p.p1_5), (p.p3_4, p.p2_5),
         (p.p4_4, p.p3_5), (p.p5_4, p.p4_5),(p.p6_4, p.p5_5),(p.p7_4, p.p6_5),(p.p8_4, p.p7_5),(p.p9_4, p.p8_5),
         (p.p1_5, p.p0_6), (p.p2_5, p.p1_6), (p.p3_5, p.p2_6),(p.p4_5, p.p3_6), (p.p5_5, p.p4_6),(p.p6_5, p.p5_6),(p.p7_5, p.p6_6),(p.p8_5, p.p7_6),(p.p9_5, p.p8_6),
         (p.p1_6, p.p0_7), (p.p2_6, p.p1_7), (p.p3_6, p.p2_7),(p.p4_6, p.p3_7), (p.p5_6, p.p4_7),(p.p6_6, p.p5_7),(p.p7_6, p.p6_7),(p.p8_6, p.p7_7),(p.p9_6, p.p8_7),
         (p.p1_7, p.p0_8), (p.p2_7, p.p1_8), (p.p3_7, p.p2_8),(p.p4_7, p.p3_8), (p.p5_7, p.p4_8),(p.p6_7, p.p5_8),(p.p7_7, p.p6_8),(p.p8_7, p.p7_8),(p.p9_7, p.p8_8),
         (p.p1_8, p.p0_9), (p.p2_8, p.p1_9), (p.p3_8, p.p2_9),(p.p4_8, p.p3_9), (p.p5_8, p.p4_9),(p.p6_8, p.p5_9),(p.p7_8, p.p6_9),(p.p8_8, p.p7_9),(p.p9_8, p.p8_9)])
    map(self.ISADJ_sows.add_rule,
        [ (p.p1_1, p.p0_0),
         (p.p1_2, p.p0_1), (p.p1_3, p.p0_2), (p.p1_4, p.p0_3), (p.p1_5, p.p0_4),(p.p1_6, p.p0_5),(p.p1_7, p.p0_6),(p.p1_8, p.p0_7),(p.p1_9, p.p0_8), (p.p2_1, p.p1_0), (p.p2_2, p.p1_1),
         (p.p2_3, p.p1_2), (p.p2_4, p.p1_3), (p.p2_5, p.p1_4),(p.p2_6, p.p1_5),(p.p2_7, p.p1_6),(p.p2_8, p.p1_7),(p.p2_9, p.p1_8), (p.p3_1, p.p2_0), (p.p3_2, p.p2_1), (p.p3_3, p.p2_2),
         (p.p3_4, p.p2_3), (p.p3_5, p.p2_4),(p.p3_6, p.p2_5),(p.p3_7, p.p2_6),(p.p3_8, p.p2_7),(p.p3_9, p.p2_8), (p.p4_1, p.p3_0), (p.p4_2, p.p3_1), (p.p4_3, p.p3_2), (p.p4_4, p.p3_3),
         (p.p4_5, p.p3_4),(p.p4_6, p.p3_5),(p.p4_7, p.p3_6),(p.p4_8, p.p3_7),(p.p4_9, p.p3_8), (p.p5_1, p.p4_0), (p.p5_2, p.p4_1), (p.p5_3, p.p4_2), (p.p5_4, p.p4_3), (p.p5_5, p.p4_4),(p.p5_6, p.p4_5),(p.p5_7, p.p4_6),(p.p5_8, p.p4_7),(p.p5_9, p.p4_8),
          (p.p6_1, p.p5_0), (p.p6_2, p.p5_1), (p.p6_3, p.p5_2), (p.p6_4, p.p5_3), (p.p6_5, p.p5_4),(p.p6_6, p.p5_5),(p.p6_7, p.p5_6),(p.p6_8, p.p5_7),(p.p6_9, p.p5_8),
          (p.p7_1, p.p6_0), (p.p7_2, p.p6_1), (p.p7_3, p.p6_2), (p.p7_4, p.p6_3), (p.p7_5, p.p6_4),(p.p7_6, p.p6_5),(p.p7_7, p.p6_6),(p.p7_8, p.p6_7),(p.p7_9, p.p6_8),
          (p.p8_1, p.p7_0), (p.p8_2, p.p7_1), (p.p8_3, p.p7_2), (p.p8_4, p.p7_3), (p.p8_5, p.p7_4),(p.p8_6, p.p7_5),(p.p8_7, p.p7_6),(p.p8_8, p.p7_7),(p.p8_9, p.p7_8),
          (p.p9_1, p.p8_0), (p.p9_2, p.p8_1), (p.p9_3, p.p8_2), (p.p9_4, p.p8_3), (p.p9_5, p.p8_4),(p.p9_6, p.p8_5),(p.p9_7, p.p8_6),(p.p9_8, p.p8_7),(p.p9_9, p.p8_8)])

    map(self.ISADJ_west.add_rule,
       [(p.p1_0, p.p0_0),(p.p2_0, p.p1_0),(p.p3_0, p.p2_0),(p.p4_0, p.p3_0),(p.p5_0, p.p4_0),(p.p6_0, p.p5_0),(p.p7_0, p.p6_0),(p.p8_0, p.p7_0),(p.p9_0, p.p8_0),(p.p1_1, p.p0_1),
        (p.p2_1, p.p1_1),(p.p3_1, p.p2_1),(p.p4_1, p.p3_1),(p.p5_1, p.p4_1),(p.p6_1, p.p5_1),(p.p7_1, p.p6_1),(p.p8_1, p.p7_1),(p.p9_1, p.p8_1),(p.p1_2, p.p0_2),(p.p2_2, p.p1_2),(p.p3_2, p.p2_2),
        (p.p4_2, p.p3_2),(p.p5_2, p.p4_2),(p.p6_2, p.p5_2),(p.p7_2, p.p6_2),(p.p8_2, p.p7_2),(p.p9_2, p.p8_2),(p.p1_3, p.p0_3),(p.p2_3, p.p1_3),(p.p3_3, p.p2_3),(p.p4_3, p.p3_3),(p.p5_3, p.p4_3),
        (p.p6_3, p.p5_3),(p.p7_3, p.p6_3),(p.p8_3, p.p7_3),(p.p9_3, p.p8_3),(p.p1_4, p.p0_4),(p.p2_4, p.p1_4),(p.p3_4, p.p2_4),(p.p4_4, p.p3_4),(p.p5_4, p.p4_4),(p.p6_4, p.p5_4),(p.p7_4, p.p6_4),(p.p8_4, p.p7_4),(p.p9_4, p.p8_4),
        (p.p1_5, p.p0_5),(p.p2_5, p.p1_5),(p.p3_5, p.p2_5),(p.p4_5, p.p3_5),(p.p5_5, p.p4_5),(p.p6_5, p.p5_5),(p.p7_5, p.p6_5),(p.p8_5, p.p7_5),(p.p9_5, p.p8_5),
        (p.p1_6, p.p0_6),(p.p2_6, p.p1_6),(p.p3_6, p.p2_6),(p.p4_6, p.p3_6),(p.p5_6, p.p4_6),(p.p6_6, p.p5_6),(p.p7_6, p.p6_6),(p.p8_6, p.p7_6),(p.p9_6, p.p8_6),
        (p.p1_7, p.p0_7),(p.p2_7, p.p1_7),(p.p3_7, p.p2_7),(p.p4_7, p.p3_7),(p.p5_7, p.p4_7),(p.p6_7, p.p5_7),(p.p7_7, p.p6_7),(p.p8_7, p.p7_7),(p.p9_7, p.p8_7),
        (p.p1_8, p.p0_8),(p.p2_8, p.p1_8),(p.p3_8, p.p2_8),(p.p4_8, p.p3_8),(p.p5_8, p.p4_8),(p.p6_8, p.p5_8),(p.p7_8, p.p6_8),(p.p8_8, p.p7_8),(p.p9_8, p.p8_8),
        (p.p1_9, p.p0_9),(p.p2_9, p.p1_9),(p.p3_9, p.p2_9),(p.p4_9, p.p3_9),(p.p5_9, p.p4_9),(p.p6_9, p.p5_9),(p.p7_9, p.p6_9),(p.p8_9, p.p7_9),(p.p9_9, p.p8_9),])

    map(self.spray.add_rule,[(p.p2_1,t.t1)])
  
  def setUp(self):
    self.v, self.a = VariableFactory(), FamilyMemberFactory()
    self.p,self.t = PositionFactory(), TimeFactory()
    self.kb = KnowledgeBase()
    v, a = self.v, self.a
    p,t = self.p,self.t
    #self.native = RuleBasedPredicate('native', (Position,Time))
    self.plant = RuleBasedPredicate('plant',(Position,Time))
    #self.adjacent = RuleBasedPredicate('adjacent', (Position,Position))
    self.ISADJ_south = RuleBasedPredicate('ISADJ_south', (Position, Position))
    self.ISADJ_north = RuleBasedPredicate('ISADJ_north', (Position, Position))
    self.ISADJ_east = RuleBasedPredicate('ISADJ_east', (Position, Position))
    self.ISADJ_west = RuleBasedPredicate('ISADJ_west', (Position, Position))
    self.ISADJ_noes = RuleBasedPredicate('ISADJ_noes', (Position, Position))
    self.ISADJ_soes = RuleBasedPredicate('ISADJ_soes', (Position, Position))
    self.ISADJ_nows = RuleBasedPredicate('ISADJ_nows', (Position, Position))
    self.ISADJ_sows = RuleBasedPredicate('ISADJ_sows', (Position, Position))
    #self.invasive = RuleBasedPredicate('invasive', (Position,Time))
    self.ISADJ_time = RuleBasedPredicate('ISADJ_time', (Time,Time))
    self.spray = RuleBasedPredicate('spray', (Position,Time))
    #self.mother = RuleBasedPredicate('mother', (FamilyMember, FamilyMember))
    self.kb.add_all([self.ISADJ_time,self.ISADJ_east,self.ISADJ_south,self.ISADJ_north,self.ISADJ_west,self.ISADJ_noes,self.ISADJ_nows,self.ISADJ_soes,self.ISADJ_sows,self.spray])
  
  def testAncestor(self):
    v, a = self.v, self.a
    p,t = self.p,self.t
    self.loadFamilyTree1()
    #discrepancy_native = RuleBasedPredicate('discrepancy_native',  (Position, Time))
    #discrepancy_not_native = RuleBasedPredicate('discrepancy_not_native',  (Position, Time))
    #discrepancy = RuleBasedPredicate('discrepancy_invasive',  (Position, Time))
    #discrepancy_not_invasive = RuleBasedPredicate('discrepancy_not_invasive', (Position, Time))
    spray_actual = RuleBasedPredicate('spray_actual', (Position, Time))
    #self.kb.add(discrepancy_not_native)
    #self.kb.add(discrepancy_not_invasive)
    self.kb.add(spray_actual)
    positive_tuples=[[p.p2_2,t.t2]]
    negative_tuples=[[p.p2_3,t.t1],[p.p1_3,t.t1],[p.p2_2,t.t1],[p.p4_4,t.t1],[p.p2_3,t.t2],[p.p1_3,t.t2],[p.p4_4,t.t2],[p.p1_2,t.t1],[p.p4_3,t.t1],[p.p2_1,t.t1],[p.p1_2,t.t2],[p.p4_3,t.t2],[p.p2_1,t.t2]]




    #self.assertFollows(ancestor(a.bob, a.jane))
    #self.assertNotFollows(ancestor(a.bob, a.ian))
    '''self.kb.remove(discrepancy_not_invasive)
    self.print_rules(discrepancy_not_invasive)'''
    foil(spray_actual, positive_tuples, negative_tuples, self.kb, ordering=None)
    f = open(rule_file, 'w')

    self.kb.remove(spray_actual)
    self.print_rules(spray_actual)
    for rule in spray_actual.rules:
      f.write(str(rule)[33:])
      f.write("\n")

    f.close()



if __name__ == "__main__":
    unittest.main()
    
