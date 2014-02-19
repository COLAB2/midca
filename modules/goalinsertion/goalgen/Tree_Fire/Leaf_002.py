from Leaf import Leaf
import random

from utils.block import Block
from modules.goalinsertion.goalgen.goal import Goal

class Leaf2(Leaf):

    # no goal associated with absence of fire
    def evaluate(self, blockset):
        return None
