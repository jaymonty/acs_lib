"""
    Aerial Combat Swarms Swarm State object

    Maintains a group of ACS_UAVState objects.

    Author: Michael Day
    Date: June 2015

"""

from acs_lib import acs_uav_state

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
 
    def update_uav_preprocess_msg(self, id, msg):
        if id not in self.uav_states:
            self.uav_states[id] = acs_uav_state.ACS_UAVState(id)

        #TODO: process header

    def update_uav_state(self, id, msg):
        self.update_uav_preprocess_msg(id, msg)
    
        #TODO: verify this is a FlightStatus message
        
        name = msg.name

        #TODO: remove this workaround when we switch everthing to Python3:
        name = name[2:name.find("\\x00")]

        self.uav_states[id].update_status(msg.msg_secs, name, msg.mode, msg.batt_rem, msg.batt_vcc, msg.ok_gps, msg.swarm_state, msg.msg_sub, msg.ctl_mode, msg.swarm_behavior)

    def update_uav_pose(self, id, msg):
        self.update_uav_preprocess_msg(id, msg)

        quat = (msg.q_x, msg.q_y, msg.q_z, msg.q_w)
        self.uav_states[id].update_pose(msg.msg_secs, msg.lat, msg.lon, msg.alt, quat)

    def get_uav_ids(self):
        ids = []
        for id in self.uav_states.keys():
            ids.append(id)

        return ids
