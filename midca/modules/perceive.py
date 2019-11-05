from midca.modules._robot_world import world_repr
from midca.worldsim import domainread, stateread
from midca import rosrun, midcatime, base
import copy
import os
import zmq
import socket,time


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


class MoosObserver(base.BaseModule):
    '''
    MIDCA Module which interacts with moos to get the current states
    of the vehicle in the moos.
    '''

    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world
        context = zmq.Context()
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber_mine = context.socket(zmq.SUB)

        self.subscriber = context.socket(zmq.SUB)
        self.subscriber_mine_removal = context.socket(zmq.SUB)

        self.subscriber.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_mine.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_mine_removal.setsockopt(zmq.SUBSCRIBE, '')

        self.subscriber.setsockopt(zmq.RCVTIMEO, 1)
        self.subscriber_mine.setsockopt(zmq.RCVTIMEO, 1)
        self.subscriber_mine_removal.setsockopt(zmq.RCVTIMEO, 1)

        self.subscriber.setsockopt(zmq.CONFLATE, 1)
        #self.subscriber_mine.setsockopt(zmq.CONFLATE, 1)

        self.subscriber.connect("tcp://127.0.0.1:5563")

        self.subscriber_mine.connect("tcp://127.0.0.1:5564")

        self.subscriber_mine_removal.connect("tcp://127.0.0.1:5580")

        self.removed_mines = set()
    # perfect observation
    def observe(self):
        return self.world.copy()

    def run(self, cycle, verbose=2):
        '''
        Read from the subscriber in the format "X:float,Y:float,SPEED:float"
        '''
        world = self.observe()
        if not world:
            raise Exception("World observation failed.")

        self.mem.add(self.mem.STATES, world)

        x = -1
        y = -1
        speed = -1
        direction = -1
        mine_x = -1
        mine_y = -1
        mine_label = -1
        states = ""
        removed_mine = ""

        '''
        The following code gets the current X,Y,Speed and updates the location of uuv.
        i.e., if the vehicle is in qroute or green area 1 or green area 2.
        the else part is to remove the state after the vehicle leaves the specific location
        '''

        try:
            current_position = self.subscriber.recv()
            print (current_position)
            x,y,speed,direction = current_position.split(",")
            x = float(x.split(":")[1])
            y = float(y.split(":")[1])
            speed = float(speed.split(":")[1])
            direction = float(direction.split(":")[1])
            print (x)
            print (y)
            print(direction)
        # for mine
            try:
                mines_checked = []
                # for removed mine
                try:
                    removed_mine = self.subscriber_mine_removal.recv()
                    self.removed_mines.add(removed_mine)
                except:
                    if verbose > 2:
                        print ("No mines removed ")

                mine_report = self.subscriber_mine.recv()
                mine_x,mine_y,mine_label = mine_report.split(":")[1].split(",")
                mine_x = float(mine_x.split("=")[1])
                mine_y = float(mine_y.split("=")[1])
                mine_label = mine_label.split("=")[1]


                # ignore already checked mines
                for atom in self.world.atoms:
                    if atom.predicate.name == "hazard_checked":
                        mines_checked.append(atom.args[0].name)

                if "mine"+mine_label in mines_checked or \
                   "mine"+mine_label in self.removed_mines:
                    raise Exception("Mine previously checked.")
                '''
                # for mine at qroute,ga1,ga2 or not
                if mine_x >=5 and mine_x <=35 and mine_y >=-94 and mine_y<=-65:
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",ga1)\n"
                elif mine_x >=133 and mine_x <=164 and mine_y >=-94 and mine_y<=-65:
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",ga2)\n"
                elif mine_y >=-98 and mine_y<=-48:
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",qroute)\n"
                else:
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",transit)\n"
                '''
                # for mine at GA1 and GA2
                if (mine_x>=-3 and mine_x<=44) and (mine_y>=-102 and mine_y<=-56):
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",ga1)\n"
                    self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "ga1"])

                elif (mine_x>=124 and mine_x<=175) and (mine_y>=-102 and mine_y<=-56):
                    states+= "HAZARD(mine" + mine_label + ")\n"
                    states+="hazard_at_location(mine" + mine_label + ",ga2)\n"
                    self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "ga2"])

                else:
                        # for mine at qroute or not
                    if mine_y >=-98 and mine_y<=-48:
                        states+= "HAZARD(mine" + mine_label + ")\n"
                        states+="hazard_at_location(mine" + mine_label + ",qroute)\n"
                        self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "qroute"])
                    else:
                        states+= "HAZARD(mine" + mine_label + ")\n"
                        states+="hazard_at_location(mine" + mine_label + ",transit)\n"
                        self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "transit"])

                path_mines = self.mem.get(self.mem.MINE_LOCATION)
                remus_location = self.mem.get(self.mem.REMUS_LOCATION)
                if not path_mines:
                    path_mines = {}

                # for mine in the pathway
                if (mine_x-5 < x) and (mine_x + 5 > x ) and direction > 100:
                        states+="hazard_at_pathway(mine" + mine_label+")\n"
                        path_mines["mine"+mine_label] = {"X": mine_x , "Y": mine_y}

                if (mine_x-5 > x) and (mine_x + 5 < x ) and direction < 0:
                        states+="hazard_at_pathway(mine" + mine_label+")\n"
                        path_mines["mine"+mine_label] = {"X": mine_x , "Y": mine_y}

                if (mine_y-5 < y) and (mine_y + 5 > y ) and direction < 100:
                        states+="hazard_at_pathway(mine" + mine_label+")\n"
                        path_mines["mine"+mine_label] = {"X": mine_x , "Y": mine_y}
                '''
                if (mine_y-5 > y) and (mine_y + 5 < y ):
                        states+="hazard_at_pathway(mine" + mine_label+")\n"
                        path_mines["mine"+mine_label] = {"X": mine_x , "Y": mine_y}
                '''
                path_mines["mine" + mine_label] = {"X": mine_x, "Y": mine_y}
                self.mem.set(self.mem.MINE_LOCATION, path_mines)


            except:
                print ("Mine Report not received")
                pass





            if y >=-98 and y<=-48:
                states+="at_location(remus,qroute)\n"
            else:
                if (x >= -13 and y >= -35) and (x <= 14 and y <= -15):
                    states+="at_location(remus,transit1)\n"

                elif (x >= 133 and y >= -30) and (x <= 173 and y <= -12):
                    states+="at_location(remus,transit2)\n"

                else:
                    states+="at_location(remus,transit)\n"

            if (x >= 50 and y >= -80) and (x <= 64 and y <= -70):
                states+="at_location(remus,qroute_transit)\n"

            if (x > 28 and x<= 37) and (y > -72 and y<= -61) :
            #if (x == 28) and (y == -62) :
                states+="at_location(remus,ga1)\n"

            if (x > 154 and x<= 163) and (y > -75 and y <=-63):
                states+="at_location(remus,ga2)\n"

            if x>165 and y > -6:
                states+="at_location(remus,home)\n"

            way_points = self.mem.get(self.mem.WAY_POINTS)

            if way_points:

                '''
                way_point = way_points[-1]
                if (x > (way_point[0] - 5) and x < (way_point[0] + 5)) and (y > (way_point[1] - 5) and y < (way_point[1] + 5)) :
                    states += "at_location(remus,way_point)\n"
                '''

                if way_points["id"] == "line":
                    if (x > (63 - 5) and x < (63 + 5)) and (y > (-96 - 5) and y < (-96 + 5)):
                        states += "at_location(remus,way_point)\n"

                if way_points["id"] == "mission_failed":
                    if (x > (216 - 5) and x < (216 + 5)) and (y > (-63 - 5) and y < (-63 + 5)):
                        states += "at_location(remus,way_point)\n"

                else:
                    if "at_location(remus, way_point)" in states:
                        states += "!at_location(remus,way_point)\n"
            else:
                if "at_location(remus, way_point)" in states:
                    states += "!at_location(remus,way_point)\n"

            self.mem.set(self.mem.REMUS_LOCATION, {"X": x, "Y": y, "speed": speed, "direction": speed})

        except:
            print ("states not received")
            pass

        # remove all states related to at_location
        atoms = copy.deepcopy(self.world.atoms)
        for atom in atoms:
            if atom.predicate.name == "at_location":
                    self.world.atoms.remove(atom)

        # this is to update the world into memory
        if not states == "":
            if verbose >= 1:
                print(states)
            stateread.apply_state_str(self.world, states)
            self.mem.add(self.mem.STATES, self.world)


        states = self.mem.get(self.mem.STATES)
        if len(states) > 400:
            # print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print "World observed."

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(self.world))

class MoosObserverWithFishingVessels(base.BaseModule):
    '''
    MIDCA Module which interacts with moos to get the current states
    of the vehicle in the moos.
    '''


    def init(self, world, mem):
        base.BaseModule.init(self, mem)
        if not world:
            raise ValueError("world is None!")
        self.world = world
        context = zmq.Context()

        # connection for remus
        self.subscriber_remus = context.socket(zmq.SUB)
        self.subscriber_remus.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_remus.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_remus.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_remus.bind("tcp://127.0.0.1:5563")

        #connection to know about mines detected
        self.subscriber_mine = context.socket(zmq.SUB)
        self.subscriber_mine.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_mine.setsockopt(zmq.RCVTIMEO, 5)
        self.subscriber_mine.bind("tcp://127.0.0.1:5564")

        #connection to know about fisher vessels

        # Zeromq connection with the fisher1 for perception
        self.subscriber_fisher1 = context.socket(zmq.SUB)
        self.subscriber_fisher1.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_fisher1.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_fisher1.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_fisher1.bind("tcp://127.0.0.1:5593")

        # Zeromq connection with the fisher2 for perception
        self.subscriber_fisher2 = context.socket(zmq.SUB)
        self.subscriber_fisher2.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_fisher2.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_fisher2.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_fisher2.bind("tcp://127.0.0.1:5596")

        # Zeromq connection with the fisher3 for perception
        self.subscriber_fisher3 = context.socket(zmq.SUB)
        self.subscriber_fisher3.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_fisher3.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_fisher3.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_fisher3.bind("tcp://127.0.0.1:5599")

        # Zeromq connection with the fisher4 for perception
        self.subscriber_fisher4 = context.socket(zmq.SUB)
        self.subscriber_fisher4.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_fisher4.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_fisher4.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_fisher4.bind("tcp://127.0.0.1:5590")

        # Zeromq connection with the friendly vessels for action
        self.subscriber_friendly_vessels = []
        for i in range(0,10):
            connection = context.socket(zmq.SUB)
            connection.setsockopt(zmq.SUBSCRIBE, '')
            connection.setsockopt(zmq.RCVTIMEO, -1)
            connection.setsockopt(zmq.CONFLATE, 1)
            connection.bind("tcp://127.0.0.1:651"+str(i))
            self.subscriber_friendly_vessels.append(connection)
            time.sleep(0.1)

        self.removed_mines = set()
    # perfect observation
    def observe(self):
        return self.world.copy()

    def get_the_mine_location(self, mine_report, states):
        """

        :param msg: the mine location
        :param states: predicate, argument format to create world
        :return: the states
        """

        # for mine
        mines_checked = []
        # for removed mine

        mine_x,mine_y,mine_label = mine_report.split(":")[1].split(",")
        mine_x = float(mine_x.split("=")[1])
        mine_y = float(mine_y.split("=")[1])
        mine_label = mine_label.split("=")[1]

        # ignore already checked mines
        for atom in self.world.atoms:
            if atom.predicate.name == "hazard_checked":
                mines_checked.append(atom.args[0].name)

        if "mine"+mine_label in mines_checked:
            raise Exception("Mine previously checked.")

        # for mine at GA1 and GA2
        if (mine_x>=-3 and mine_x<=44) and (mine_y>=-102 and mine_y<=-56):
            states+= "HAZARD(mine" + mine_label + ")\n"
            states+="hazard_at_location(mine" + mine_label + ",ga1)\n"
            self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "ga1"])

        elif (mine_x>=124 and mine_x<=175) and (mine_y>=-102 and mine_y<=-56):
            states+= "HAZARD(mine" + mine_label + ")\n"
            states+="hazard_at_location(mine" + mine_label + ",ga2)\n"
            self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "ga2"])

        elif mine_y >=-98 and mine_y<=-48:
            states+= "HAZARD(mine" + mine_label + ")\n"
            states+="hazard_at_location(mine" + mine_label + ",qroute)\n"
            self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "qroute"])

        elif y >=-166 and y<=-226:
            states+= "HAZARD(mine" + mine_label + ")\n"
            states+="hazard_at_location(mine" + mine_label + ",qroute1)\n"
            self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "qroute1"])

        else:
            states+= "HAZARD(mine" + mine_label + ")\n"
            states+="hazard_at_location(mine" + mine_label + ",transit)\n"
            self.mem.set (self.mem.CURRENT_HAZARD, [mine_label , "transit"])


        return states

    def get_ship_status(self, ship_status, states, vehicle):
        """

        :param ship_status: message about the active status of the ships
        :param states: predicate, argument format to create world
        :return: the states
        """

        x,y,speed,direction = ship_status.split(",")
        x = float(x.split(":")[1])
        y = float(y.split(":")[1])
        speed = float(speed.split(":")[1])
        direction = float(direction.split(":")[1])
        #print ("Vehicle Name : {}".format(vehicle))
        #print ("X : {}".format(x))
        #print ("Y : {}".format(y))

        #should check if the vehicle is in Q-route1, Q-route2, GA1, GA2
        if x ==-1700.00 and y == -1500.00:
            states+="wrecked(" + vehicle + ")\n"

        return states

    def get_the_location(self, current_position, states, vehicle):
        """

        :param states: contain predicate, argument format to create the world
        :param vehicle: the name of the vehicle; ex : remus, fisher1 ... 4
        :return: states
        """
        x,y,speed,direction = current_position.split(",")
        x = float(x.split(":")[1])
        y = float(y.split(":")[1])
        speed = float(speed.split(":")[1])
        direction = float(direction.split(":")[1])
        #print ("Vehicle Name : {}".format(vehicle))
        #print ("X : {}".format(x))
        #print ("Y : {}".format(y))

        # update agent and fisher location
        if vehicle == "remus":
            self.mem.set(self.mem.AGENT_LOCATION, [x,y])

        if vehicle == "fisher4":
            self.mem.set(self.mem.ENEMY_LOCATION, [x,y])

        #should check if the vehicle is in Q-route1, Q-route2, GA1, GA2
        if y >=-98 and y<=-48:
                states+="at_location(" + vehicle + ",qroute)\n"

                if (x >= 45 and y >= -85) and (x <= 68 and y <= -65):
                    states+="at_location(" + vehicle + ",qroute_transit)\n"

                elif (x > 23 and x<= 42) and (y > -67 and y<= -56) :
                        #if (x == 28) and (y == -62) :
                        states+="at_location(" + vehicle + ",ga1)\n"

                elif (x > 150 and x<= 160) and (y > -70 and y <=-60):
                        states+="at_location(" + vehicle + ",ga2)\n"

        elif y >=-166 and y<=-226:
                states+="at_location(" + vehicle + ",qroute1)\n"

        elif (x >= -13 and y >= -35) and (x <= 14 and y <= -15):
                states+="at_location(" + vehicle + ",transit1)\n"

        elif (x >= 133 and y >= -30) and (x <= 173 and y <= -12):
                states+="at_location(" + vehicle + ",transit2)\n"

        elif x>165 and y > -6:
                states+="at_location(" + vehicle + ",home)\n"

        else:
            states+="at_location(" + vehicle + ",transit)\n"

        return states

    def run(self, cycle, verbose=2):
        '''
        Read from the subscriber in the format "X:float,Y:float,SPEED:float"
        '''
        world = self.observe()
        if not world:
            raise Exception("World observation failed.")

        states = ""

        '''
        The following code gets the current X,Y,Speed and updates the location of uuv.
        i.e., if the vehicle is in qroute or green area 1 or green area 2.
        the else part is to remove the state after the vehicle leaves the specific location
        '''

        try:
            current_position = self.subscriber_remus.recv()
            states += self.get_the_location(current_position, states, "remus")
        except:
            print ("Remus location not recieved")


        try:
            current_position = self.subscriber_fisher1.recv()
            states += self.get_the_location(current_position, states, "fisher1")

            current_position = self.subscriber_fisher2.recv()
            states += self.get_the_location(current_position, states, "fisher2")

            current_position = self.subscriber_fisher3.recv()
            states += self.get_the_location(current_position, states, "fisher3")

            current_position = self.subscriber_fisher4.recv()
            states += self.get_the_location(current_position, states, "fisher4")

        except :
            print ("Fishing vessels location not recieved")

        try:
            mine_report = self.subscriber_mine.recv()
            states += self.get_the_mine_location(mine_report,states)

        except:
                print ("Mine Report not received")
                pass

        try:
            for index, ship_subscribe in enumerate(self.subscriber_friendly_vessels):
                ship_status = ship_subscribe.recv()
                states += self.get_ship_status(ship_status,states,"ship"+str(index))
        except Exception as e:
                print (e)
                print ("Ships details not recieved")
                pass

        # remove all states related to at_location
        atoms = copy.deepcopy(self.world.atoms)
        for atom in atoms:
            if atom.predicate.name == "at_location":
                    self.world.atoms.remove(atom)

        # this is to update the world into memory
        if not states == "":
            if verbose >= 1:
                #print(states)
                pass
            stateread.apply_state_str(self.world, states)
            self.mem.add(self.mem.STATES, self.world)


        states = self.mem.get(self.mem.STATES)
        if states and len(states) > 400:
            # print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print "World observed."

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(self.world))




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
        thisDir =  "C:/Users/Zohreh/git/midca/modules/_plan/jShop"
        thief_file = thisDir + "/theif.txt"
        theft_items=[]

        with open(thief_file) as f:
            lines = f.readlines()
            for line in lines:
                theft_items.append(line.split(" "))

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

        if action[0] in ("survey"):
            valuepairs["actor"] = {"value": str(action[1]).replace(" ", "_")}
            valuepairs["location"] = { "value":  str(action[2]).replace(" ", "_") }
            del valuepairs["object"]

        if action[0] in ("remove" , "ignore", "hazard-detection", "is-a-problem"):
            valuepairs["actor"] = {"value": str(action[3]).replace(" ", "_")}
            object = str(action[1]).replace(" ", "_")
            object = ''.join([i for i in object if not i.isdigit()])
            valuepairs["object"] = {"value": object}
            valuepairs["location"] =  {"value" :  str(action[2]).replace(" ", "_") }

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
        time.sleep(0.1)

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

        # mine that is currently detected
        current_mine = self.mem.get (self.mem.CURRENT_HAZARD)
        mine_to_explain = ""
        if current_mine:
            mine_to_explain = "mine" + current_mine[0]
        for atom in world.atoms:
            if (atom.predicate.name) == "hazard_at_location" and atom.args[0].name == mine_to_explain:
                report.actions.append( ["hazard-detection", atom.args[0].name, atom.args[1].name,"remus"] )
                #if atom.args[1].name == "qroute":
                #    report.actions.append(["is-a-problem", atom.args[0].name, atom.args[1].name, "remus"])

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
