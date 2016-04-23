"""
    Aerial Combat Swarms Swarm State object

    Maintains a group of ACS_UAVState objects.

    Author: Michael Day
    Date: June 2015

"""
#
#Written in 2015 by the Advanced Robotic Systems Engineering Laboratory at the
#U.S. Naval Postgraduate School, Monterey, California, USA.
#
#Pursuant to 17 USC 105, this work is not subject to copyright in the
#United States and is in the public domain.
#
#THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
#REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
#AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
#INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
#LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
#OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
#PERFORMANCE OF THIS SOFTWARE.
#
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
        elif isinstance(msg, acs_messages.PrevMsgAP):
            self.update_uav_preprocess_msg(msg.msg_src, msg)
            self.update_uav_ap_msgs(msg.msg_src, msg)
        elif isinstance(msg, acs_messages.WaypointMsg):
            self.update_uav_preprocess_msg(msg.msg_src, msg)
            self.update_waypoint_from_ap(msg.msg_src, msg)
        elif isinstance(msg, acs_messages.ParamAPMsg):
            self.update_uav_preprocess_msg(msg.msg_src, msg)
            self.update_uav_ap_param(msg.msg_src, msg)

        #Currently not sending any info or throwing exceptions
        #on unrecognized messages.

    def update_uav_preprocess_msg(self, id, msg):
        if id not in self.uav_states:
            self.uav_states[id] = acs_uav_state.ACS_UAVState(id)

    def update_uav_state(self, id, msg):
        #print("%d %s %d %d" % (msg.msg_src, name, msg.armed, msg.mode))

        self.uav_states[id].update_status(msg)

    def update_uav_pose(self, id, msg):
        quat = (msg.q_x, msg.q_y, msg.q_z, msg.q_w)
        self.uav_states[id].update_pose(msg.msg_secs, msg.lat, msg.lon, msg.alt, quat)

    def update_waypoint_from_ap(self, id, msg):
        self.uav_states[id].update_wp_from_ap(msg)
   
    def update_uav_ap_param(self, id, msg):
        self.uav_states[id].update_ap_param(msg.param_name, msg.param_value)

    def update_uav_ap_msgs(self, id, msg):
        self.uav_states[id].update_ap_msgs(msg)

    def get_uav_ids(self):
        ids = []
        for id in self.uav_states.keys():
            ids.append(id)

        return ids
