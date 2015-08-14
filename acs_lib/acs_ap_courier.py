"""
ACS_AP_Courier class

This class is for fetching messages from the payload that were previously 
forwarded from the autopilot. 

Note that this class is designed to be a member of a uav_state object,
and as such ONLY processing INCOMING messages from the paylod.  ONLY
the GCS should make requests for AP messages: that minimizes network/radio
traffic as the GCS typically only asks for messages from one UAV at a time.

Author: Michael Day
Date: Aug 2015
"""
from threading import RLock

class ACS_AP_Courier(object):

    def __init__(self, id):
        self.__id = id

        #messages sent from autopilot, indexed by sequence number:
        self.__ap_msgs = {}
        self.__ap_msg_min_seq = 0 #minimum seq number (tells the payload: 
                                  #"I have already received up to and including
                                  #this sequence number")
        self.__ap_msg_max_seq = 0 #maximum seq number available as reported
                                  #by the payload
        self.__ap_msg_win_size = 20 #only ever asking for this number of msgs

        self.__ap_msg_lock = RLock()

    def update_ap_msgs(self, msg):
        with self.__ap_msg_lock:
            if msg.seq not in self.__ap_msgs:
                self.__ap_msgs[msg.seq] = msg.msg

            if self.__ap_msg_max_seq < msg.final_seq:
                self.__ap_msg_max_seq = msg.final_seq

            #Now determine where to set the bottom of the request window----    
            #Have we missed enough messages that we're past the window?
            if (self.__ap_msg_max_seq - self.__ap_msg_min_seq >
                    self.__ap_msg_win_size):
                self.__ap_msg_min_seq = \
                    self.__ap_msg_max_seq - self.__ap_msg_win_size
            else: #See if we've filled up the bottom of the window:
                new_min = self.__ap_msg_min_seq
                for i in range(self.__ap_msg_min_seq + 1,
                               self.__ap_msg_max_seq + 1):
                    if i in self.__ap_msgs:
                        new_min = i
                    else:
                        break
                self.__ap_msg_min_seq = new_min
                
    def get_ap_msg_min_seq(self):
        with self.__ap_msg_lock:
            return self.__ap_msg_min_seq

    def get_ap_msg_max_seq(self):
        with self.__ap_msg_lock:
            return self.__ap_msg_max_seq

    def get_ap_msg_win_size(self):
        return self.__ap_msg_win_size

    def get_num_ap_msgs_received(self):
        with self.__ap_msg_lock:
            return len(self.__ap_msgs)

    def get_ap_msgs_as_list(self):
        msg_list = []
        with self.__ap_msg_lock:
            for k, v in self.__ap_msgs.items():
                msg_list.append(v)

        return msg_list

