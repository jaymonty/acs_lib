"""
    Aerial Combat Swarms Swarm State object

    Maintains a group of ACS_UAVState objects.

    Author: Michael Day
    Date: June 2015

"""

from acs_lib import acs_uav_state
from ap_lib import acs_messages

class ACS_SwarmState(object):
    ''' 
    Maintain state information about all known UAVs. 
    Swarms are further subdivided into subswarms based on their
    subswarm ID, availabe as the "subswarm" member variable in UAVState 
    '''

    def __init__(self):
        #Dictionary of UAVState classes that hold each UAV's state
        #This dictionary constitutes the Swarm.
        self.uav_states = {}

    def process_msg(self, msg):
        if isinstance(msg, acs_messages.FlightStatus): 
            self.update_uav_preprocess_msg(msg.msg_src, msg)
            self.update_uav_state(msg.msg_src, msg)
        elif isinstance(msg, acs_messages.Pose):
            self.update_uav_preprocess_msg(msg.msg_src, msg)
            self.update_uav_pose(msg.msg_src, msg)

        #Currently not sending any info or throwing exceptions
        #on unrecognized messages.

    def update_uav_preprocess_msg(self, id, msg):
        if id not in self.uav_states:
            self.uav_states[id] = acs_uav_state.ACS_UAVState(id)

        #TODO: process header

    def update_uav_state(self, id, msg):
        #print("%d %s %d %d" % (msg.msg_src, name, msg.armed, msg.mode))

        name = msg.name

        #TODO: remove this workaround when we switch everthing to Python3:
        name = name[2:name.find("\\x00")]

        self.uav_states[id].update_status(msg.msg_secs, name, msg.mode, msg.batt_rem, msg.batt_vcc, msg.ok_gps, msg.swarm_state, msg.msg_sub, msg.ctl_mode, msg.swarm_behavior)

    def update_uav_pose(self, id, msg):
        quat = (msg.q_x, msg.q_y, msg.q_z, msg.q_w)
        self.uav_states[id].update_pose(msg.msg_secs, msg.lat, msg.lon, msg.alt, quat)

    def get_uav_ids(self):
        ids = []
        for id in self.uav_states.keys():
            ids.append(id)

        return ids
