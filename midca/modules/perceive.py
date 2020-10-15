from midca.modules._robot_world import world_repr
from midca.worldsim import domainread, stateread
from midca import rosrun, midcatime, base
import copy
import os
import socket
try:
    # baxter robot requirements
    from midca.examples import ObjectDetector
    from bzrlib.config import LocationStore
except:
    pass

class ROSObserver:

    def init(self, world, mem):
        self.mem = mem
        self.mem.set(self.mem.STATE, world_repr.SimpleWorld())

    def store_history(self,world,history,blocks):
        '''
        store the history of last 5 state changes
        '''
        if blocks:
            a = {}
            for each in blocks:
                positions= world.all_pos(each)
                a[each] = positions.pop().position

            if a:
                history = history.append(a)

            if not history:
                history = []

            if len(history) > 5:
                history = history[:5]

            history.reverse()
            return history
        return None




    def check_with_history(self,world,history,detectionEvents):
        '''
        store the past 5 change in events for the robot to remember things
        '''
        blocks = set()
        for each in detectionEvents:
            blocks.add(each.id)
        if not history:
            history = []
            self.store_history(world,history,blocks)
        else:
            if not len(blocks) == len(history[len(history) -1]):
                history = self.store_history(world,history,blocks)
        return history

    def run(self, cycle, verbose = 2):
        #self.ObserveWorld()
        detectionEvents = self.mem.get_and_clear(self.mem.ROS_OBJS_DETECTED)
        detecttionBlockState = self.mem.get_and_clear(self.mem.ROS_OBJS_STATE)
        utteranceEvents = self.mem.get_and_clear(self.mem.ROS_WORDS_HEARD)
        feedback = self.mem.get_and_clear(self.mem.ROS_FEEDBACK)
        world = self.mem.get_and_lock(self.mem.STATE)
        history = self.mem.get_and_lock(self.mem.STATE_HISTORY)

        if not detectionEvents:
            detectionEvents = []
        if not detecttionBlockState:
            detecttionBlockState = []
        if not utteranceEvents:
            utteranceEvents = []
        if not feedback:
            feedback = []
        for event in detectionEvents:
            event.time = midcatime.now()
            world.sighting(event)
        for blockstate in detecttionBlockState:
            blockstate.time = midcatime.now()
            world.position(blockstate)
        for event in utteranceEvents:
            event.time = midcatime.now()
            world.utterance(event)
        for msg in feedback:
            d = rosrun.msg_as_dict(msg)
            d['received_at'] = float(midcatime.now())
            self.mem.add(self.mem.FEEDBACK, d)

        # if there are any change in events remember
        history = self.check_with_history(world,history,detecttionBlockState)
        self.mem.unlock(self.mem.STATE_HISTORY)
        if history:
            if len(history) > 5:
                history = history[:5]
            self.mem.set(self.mem.STATE_HISTORY , history)
        self.mem.unlock(self.mem.STATE)


        if verbose > 1:
            print "World observed:", len(detectionEvents), "new detection event(s),", len(utteranceEvents), "utterance(s) and", len(feedback), "feedback msg(s)"

class AsyncGraceObserver(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''
    def __init__(self, simulator):
        #import simulator
        self.sim = simulator

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world
        self.mem.set(self.mem.INTERFACE, self.sim)



    #perfect observation
    def observe(self):
        return self.world.copy()

    def RandomFishTagdata(self, subarea="subarea-3", loc="Tx3y3"):
        """

        :return: a random fishtag number if the action is collectdata
        """
        import random
        actions = self.mem.get(self.mem.ACTIONS)
        if actions and actions[-1]:
            [action] = actions[-1]
            if action.op == "collectdata":
                if action.args[2] == subarea and action.args[3] == loc:
                    return [str(random.randint(81, 100)), action.args[2], action.args[3]]
                else:
                    return [str(random.randint(1, 80)), action.args[2], action.args[3]]
        return [[], [], []]

    def get_agent_position(self):
        """
        create an object for Tagworld and call get_pos
        :return: the position in Tile format
        """

        while(True):
            tag = self.sim.TagWorld()
            position = tag.get_cell()
            if position:
                [x, y] = position.split(",")
                if int(x) < 0:
                    x = "0"
                if int(y) < 0:
                    y = "0"
                position = "Tx" + x + "y" + y
                return position

    def add_prev_tag(self, tag_data, position):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        for atom in self.world.atoms:
            if atom.predicate.name == "uniqueTagCount" and \
                        atom.args[2].name == position:

                    print (atom)
                    return int(atom.args[1].name) + int(tag_data)

        return tag_data

    def add_est_prev_tag(self, tag_data, position):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        for atom in self.world.atoms:
            if atom.predicate.name == "estuniqueTagCount" and \
                        atom.args[2].name == position:

                    print (atom)
                    return float(atom.args[1].name) + float(tag_data)

        return tag_data

    def parse_tile(self, input):
        output = ""
        y_index = input.index('y')
        output = [int(input[2:y_index]), int(input[y_index + 1:])]
        return output

    def delete_est_tag_atoms(self, positions):
        world = self.world.copy()
        for atom in world.atoms:
            if atom.predicate.name == "estuniqueTagCount" and \
                 atom.args[2].name in positions:
                self.world.atoms.remove(atom)

    def get_adjacent_position_tags(self, position, states):
        """
        create an object for Tagworld and call get_cell_poisson_rate
        :return: the position in Tile format
        """
        positions = []
        display_tag_data = []
        for atom in self.world.atoms:
            if (atom.predicate.name == "adjacent-south" or \
                    atom.predicate.name == "adjacent-north" or \
                    atom.predicate.name == "adjacent-east" or \
                    atom.predicate.name == "adjacent-west") and \
                    atom.args[0].name == position:
                adj_position_tile = atom.args[1].name
                adj_pos = self.parse_tile(position)
                tag = self.sim.TagWorld()
                tag_data = tag.getAdjacent(adj_pos)
                display_tag_data = tag_data
                if atom.predicate.name == "adjacent-south":
                    tag_data = tag_data[0]

                elif atom.predicate.name == "adjacent-north":
                    tag_data = tag_data[1]

                elif atom.predicate.name == "adjacent-east":
                    tag_data = tag_data[2]

                elif atom.predicate.name == "adjacent-west":
                    tag_data = tag_data[3]

                arguments = ", ".join([str(tag_data), adj_position_tile])
                states += "NUM(" + str(tag_data) + ")\n"
                states += "estuniqueTagCount(grace, " + arguments + ")\n"
                positions.append(adj_position_tile)
                del tag

        if positions:
            try:
                self.delete_est_tag_atoms(positions)
                self.display_est_tag(display_tag_data, position)
            except Exception as e:
                print(e)
                pass

        return states

    def remove_corresponding_atoms(self, filter):
        atoms = self.world.get_atoms(filter)
        for atom in atoms:
            self.world.atoms.remove(atom)

    def updateCertainity(self, mode, radius):
        tag = self.sim.TagWorld()
        tag.UpdateUncertainity(mode, radius)

    def blocRadius(self, position, states):
        """
        :param states:
        :return:
        """
        scale = 15
        alreadyCalCertainity = self.world.get_atoms(filters = ["QblocRadius", "grace", position])

        if not alreadyCalCertainity:
                get_previous_rad_atom = self.world.get_atoms(filters = ["priorblocRadius",
                                                                        "grace"])
                prior_radius = 1
                if get_previous_rad_atom:
                    prior_radius = float(get_previous_rad_atom[0].args[1].name)
                    import math
                    prior_radius = math.sqrt(prior_radius**2 + 4)
                    states += "NUM(" + str(prior_radius) + ")\n"
                    states += "priorblocRadius(grace," + str(prior_radius) + ")\n"
                    self.remove_corresponding_atoms(["priorblocRadius", "grace"])
                    print ("The prior Radius is : " + str(prior_radius))
                else:
                    states += "NUM(" + str(prior_radius) + ")\n"
                    states += "priorblocRadius(grace," + str(prior_radius) + ")\n"
                    self.remove_corresponding_atoms(["priorblocRadius", "grace"])
                    print ("The prior Radius is : " + str(prior_radius))


                if prior_radius > 6:
                    states += "QblocRadius(grace," + position + ", LOW)\n"
                    self.remove_corresponding_atoms(["QblocRadius", "grace", position])
                    self.remove_corresponding_atoms(["certainblocRadius", "grace", position])
                    self.updateCertainity("LOW", prior_radius*scale)

                elif prior_radius <= 6 and prior_radius > 2.5:
                    states += "QblocRadius(grace," + position + ", MED)\n"
                    self.remove_corresponding_atoms(["QblocRadius", "grace", position])
                    self.remove_corresponding_atoms(["certainblocRadius", "grace", position])
                    self.updateCertainity("MED", prior_radius*scale)

                else:
                    states += "QblocRadius(grace," + position + ", HIGH)\n"
                    states += "certainblocRadius(grace," + position + ")\n"
                    self.remove_corresponding_atoms(["certainblocRadius", "grace", position])
                    self.remove_corresponding_atoms(["QblocRadius", "grace", position])
                    self.updateCertainity("HIGH", prior_radius*scale)

        action = self.mem.get(self.mem.ACTIONS)
        if action:
            if action[-1][0].op.startswith("ascend"):
                states += "QblocRadius(grace," + position + ", HIGH)\n"
                states += "NUM(" + str(1) + ")\n"
                states += "priorblocRadius(grace," + str(1) + ")\n"
                states += "certainblocRadius(grace," + position + ")\n"
                self.updateCertainity("HIGH",  scale)
                self.remove_corresponding_atoms(["priorblocRadius", "grace"])
                self.remove_corresponding_atoms(["certainblocRadius", "grace"])
                self.remove_corresponding_atoms(["QblocRadius", "grace"])

        return states

    def get_fish_tags(self, position):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        tag = self.sim.TagWorld()
        tag_data = None
        action = self.mem.get(self.mem.ACTIONS)
        if action:
            if action[-1][0].op == "collectdata":
                tag_data = tag.get_tags(position)
        return tag_data

    def get_fish_tags_wo_action(self, position):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """
        #self.mem.set(self.mem.GETDATA, True)
        if self.mem.get(self.mem.GETDATA):
            self.mem.set(self.mem.GETDATA, False)
            #time.sleep(0.1)
            tag = self.sim.TagWorld()
            tag_data = None
            tag_data = tag.get_tags(position)
            return tag_data
        else:
            return False

    def get_mode(self):
        tag = self.sim.TagWorld()
        mode = tag.get_mode()
        return mode

    def add_prev_tag(self, tag_data, position):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        for atom in self.world.atoms:
            if atom.predicate.name == "uniqueTagCount" and \
                        atom.args[2].name == position:

                    print (atom)
                    return int(atom.args[1].name) + int(tag_data)

        return tag_data

    def drawlistinlists(self, frame, numbered_borders=False):
        result = ""
        if numbered_borders:
            result += "   "
            for i in range(len(frame)):
                if i >= 10:
                    result += str(i)[1] + " "
                else:
                    result += str(i) + " "
            result += '\n'
        r = 0
        for row in frame:
            if numbered_borders:
                if r < 10:
                    result += str(r) + "  "  # two spaces to make it look nicer
                else:
                    result += str(r) + " "  # only one space
            for val in row:
                result += val + " "
            result += "\n"
            r += 1
        # remove trailing newline
        result = result[:len(result) - 2]
        return result

    def display_tag(self, tag_data=None, position=None, dim=5):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        grid = self.mem.get(self.mem.GRID)

        if not grid:
            grid = []
            for c in range(dim):
                row = []
                for r in range(dim):
                    row.append(".")
                grid.append(row)
            self.mem.set(self.mem.GRID, grid)

        else:
            if tag_data and position:
                output = []
                y_index = position.index('y')
                output = [int(position[2:y_index]), int(position[y_index + 1:])]
                if  grid[4-output[1]][output[0]]:
                    grid[4-output[1]][output[0]] = str(tag_data)
                self.mem.set(self.mem.GRID, grid)

        #display
        print ("---------------Tag Data------------\n")
        print (self.drawlistinlists(grid))
        print ("-----------------------------------\n")

    def display_est_tag(self, tag_data=None, position=None, dim=5):
        """
        create an object for Tagworld and call get_pos
        :return: the integer value of fish tags detected
        """

        grid = self.mem.get(self.mem.ESTGRID)

        if not grid:
            grid = []
            for c in range(dim):
                row = []
                for r in range(dim):
                    row.append(str([0.0, 0.0, 0.0, 0.0]))
                grid.append(row)
            self.mem.set(self.mem.ESTGRID, grid)

        else:

            if tag_data and position:
                output = []
                y_index = position.index('y')
                output = [int(position[2:y_index]), int(position[y_index + 1:])]
                if  grid[4-output[1]][output[0]]:
                    grid[4-output[1]][output[0]] = str(tag_data)
                self.mem.set(self.mem.ESTGRID, grid)

        #display
        print ("---------------Estimated Tag Data------------\n")
        print (self.drawlistinlists(grid))
        print ("-----------------------------------\n")


    def run(self, cycle, verbose = 2):

        world = self.observe()
        if not world:
            raise Exception("World observation failed.")

        states = ""

        position = self.get_agent_position()

        if position:
            states += "agent-at(grace," + position + ")\n"

        #calculate atoms related to bloc radius
        #states = self.blocRadius(position, states)
        states += "QblocRadius(grace," + position + ", HIGH)\n"

        # get the tag data
        tag_data = self.get_fish_tags_wo_action(self.parse_tile(position))

        #get the operational mode of the grace
        mode = self.get_mode()
        if mode:
            # first remove the previous state
            self.remove_corresponding_atoms(["agent-mode", "grace"])
            # update the state
            states += "MODE("+ mode + ")\n"
            states += "agent-mode(grace," + mode +")\n"

            # grace is not free as it is not operating at regular mode
            if not mode == "m1":
                self.remove_corresponding_atoms(["free", "grace"])


        if tag_data:
            self.display_tag(tag_data, position)
            arguments = ", ".join([str(tag_data), position])
            states += "NUM(" + str(tag_data) + ")\n"
            states += "uniqueTagCount(grace, " + arguments + ")\n"

            #get adjacent position
            states = self.get_adjacent_position_tags(position, states)

            print (states)

        else:
            self.display_tag()
            self.display_est_tag()

        if tag_data > 235:
            states += "hotspot-detected(grace, " + position + ")\n"
            #import sys
            #sys.exit()

        if states:
            print ("  States Added  ")
            print (states)
            if position and tag_data:
                # remove agent location
                atoms = copy.deepcopy(world.atoms)
                for atom in atoms:
                    if atom.predicate.name == "agent-at":
                        world.atoms.remove(atom)
                        self.world.atoms.remove(atom)
                    elif atom.predicate.name == "uniqueTagCount" and \
                        atom.args[2].name == position:
                        world.atoms.remove(atom)
                        self.world.atoms.remove(atom)
                    elif atom.predicate.name == "estuniqueTagCount" and \
                        atom.args[2].name == position:
                        world.atoms.remove(atom)
                        self.world.atoms.remove(atom)

            elif position:
                atoms = copy.deepcopy(world.atoms)
                for atom in atoms:
                    if atom.predicate.name == "agent-at":
                        world.atoms.remove(atom)
                        self.world.atoms.remove(atom)

            stateread.apply_state_str(self.world, states)
            stateread.apply_state_str(world, states)

        self.mem.add(self.mem.STATES, world)

        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[2:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print "World observed."

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))

class PerfectObserver(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    #perfect observation
    def observe(self):
        return self.world.copy()

    def run(self, cycle, verbose = 2):
        world = self.observe()
        if not world:
            raise Exception("World observation failed.")
        self.mem.add(self.mem.STATES, world)

        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print "World observed."

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))

class PerfectObserverWithThief(base.BaseModule):

    '''
    MIDCA Module which copies a complete world state. It is designed to interact with the
    built-in MIDCA world simulator. To extend this to work with other representations,
    modify the observe method so that it returns an object representing the current known
    world state.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world

    #perfect observation
    def observe(self):
        return self.world.copy()

    def run(self, cycle, verbose = 2):
        world = self.observe()
        thisDir = os.path.dirname(os.path.realpath(__file__))
        thief_file = thisDir + "/theif.txt"
        theft_items=[]

#         with open(thief_file) as f:
#               lines = f.readlines()
#               for line in lines:
#                       theft_items.append(line.split(" "))
#
        if not world:
            raise Exception("World observation failed.")

#         self.mem.add(self.mem.STATES, world)

        for item in theft_items:

            for atom in world.atoms:
                if atom.predicate.name == item[0] and atom.args[0].name == item[1]:
                    world.atoms.remove(atom)
                    print("removed:" + atom.args[0].name)
                    break

        self.mem.add(self.mem.STATES, world)

        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            #print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print "World observed."

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD",copy.deepcopy(world))



class MAReport:

    namecounts = {"report": 0}

    def __init__(self):
        self.actions = []
        self.finalstate = None

    def str_dict(self, item, numtabs = 1, skipfirsttab = True):
        if isinstance(item, dict):
            s = ""
            first = True
            for key in sorted(item.keys()):
                if not first or not skipfirsttab:
                    s += "\t" * numtabs
                else:
                    s += " "
                s += "(" + str(key) + " " + self.str_dict(item[key], numtabs + 1) + ")\n"
                first = False
            s = s[:-1]
        else:
            s = str(item)
        return s

    def action_str(self, action):
        if action[0] in self.namecounts:
            self.namecounts[action[0]] += 1
        else:
            self.namecounts[action[0]] = 1
        s = "(" + str(action[0]) + "." + str(self.namecounts[action[0]]) + "\n"
        valuepairs = {}
        if action[0] in ("stack", "unstack", "pickup", "putdown", "apprehend", "putoutfire"):
            valuepairs["actor"] = {"value": "person.0"}
        elif action[0] == "burns":
            valuepairs["actor"] = {"value": "nature"}
        valuepairs["object"] = {"value": str(action[1]).replace(" ", "_")}
        if action[0] in ("stack", "unstack"):
            valuepairs["recipient"] = {"value": str(action[2]).replace(" ", "_")}
        return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"

    def atom_pairs(self, atom):
        valuepairs = {}
        for i in range(len(atom.args)):
            if i == 0:
                valuepairs["domain"] = {"value": atom.args[i].name.replace(" ", "_")}
            elif i == 1:
                valuepairs["co-domain"] = {"value": atom.args[i].name.replace(" ", "_")}
        return valuepairs

    def state_str(self, world):
        s = "(state\n"
        valuepairs = {}
        for atom in world.atoms:
            if atom.predicate.name in self.namecounts:
                self.namecounts[atom.predicate.name] += 1
            else:
                self.namecounts[atom.predicate.name] = 1
            valuepairs[atom.predicate.name + "." + str(self.namecounts[atom.predicate.name])] = self.atom_pairs(atom)
        return s + self.str_dict(valuepairs, skipfirsttab = False) + ")"


    def __str__(self):
        #if not self.actions:
        #    return "incomplete"
        # if there is no state do not send report to meta aqua
        if not self.finalstate:
            return "incomplete"
        else:
            self.namecounts["report"] += 1
            s = "(" + "report." + str(self.namecounts["report"]) + "\n("
            for action in self.actions:
                s += "\t(\n"
                s += self.action_str(action)
                s += "\n\"\")\n("
                s += self.state_str(self.finalstate)
                s += "\n\"\")\n"
            return s + "))"

'''
ma = MAReport()
import domainread, stateread
world = domainread.load_domain("./domain.sim")
stateread.apply_state_file(world, "./defstate.sim")
ma.finalstate = world
ma.actions.append(["unstack", "block1", "block2"])
ma.actions.append(["catchfire", "block1"])
print ma
'''

class MAReporter(base.BaseModule):

    '''
    MIDCA module that sends a report on the world and actions to the
    Meta-AQUA story understanding system. This requires Meta-AQUA to be
    running or it will not work. Also depends on the basic observation
    module.
    '''

    def __init__(self, writePort):
        self.writeS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.writeS.connect(("localhost", writePort))

    def get_lit_blocks(self, world):
        res = []
        for objectname in world.objects:
            if world.is_true("onfire", [objectname]) and \
            world.objects[objectname].type.name == "BLOCK" and \
            objectname != "table":
                res.append(objectname)
        return res


    def run(self, cycle, verbose = 2):
        world = None
        lastWorld = None
        try:
            world = self.mem.get(self.mem.STATES)[-1]
            lastWorld = self.mem.get(self.mem.STATES)[-2]
        except (TypeError,IndexError):
            pass
        if not world:
            return #no report if not world observed
        report = MAReport()
        report.finalstate = world
        try:
            actions = self.mem.get(self.mem.ACTIONS)[-1]
        except (TypeError, IndexError):
            actions = []
        blocksPutOut = []
        for action in actions:
            action.args = list(action.args)
            report.actions.append([action.op] + action.args)
            if action.op == "putoutfire":
                blocksPutOut.append(action.args[0])
        if lastWorld:
            lastBurning = self.get_lit_blocks(lastWorld)
            burning = self.get_lit_blocks(world)
            for block in burning:
                if block not in lastBurning or block in blocksPutOut:
                    report.actions.append(["burns", block])
        #report is finished, send to Meta-AQUA
                #report contains actions and state,
                #for every action there will be the state attached to it
        if verbose >= 1:
            print "Sending report to Meta-AQUA",
            if verbose >= 2:
                print ":\n", report
        if not str(report ) == "incomplete":
            self.writeS.send(str(report))

    def __del__(self):
        '''
            close sockets on deletion. Also send 'Done' message to Meta-AQUA.
        '''
        try:
            self.writeS.send(self.endMsg)
        finally:
            self.writeS.shutdown(socket.SHUT_RDWR)
