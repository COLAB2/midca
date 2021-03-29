from midca.modules._robot_world import world_repr
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
            print("World observed:", len(detectionEvents), "new detection event(s),", len(utteranceEvents), "utterance(s) and", len(feedback), "feedback msg(s)")
