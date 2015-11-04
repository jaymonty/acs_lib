"""
    Aerial Combat Swarms UAV State object.

    Encapsulates state information for a UAV.

    Author: Michael Day
    Date: Mar 2015
    Revised: June 2015 to be more easily accessible outside Swarm Commander
"""

import time
from threading import RLock

import ap_lib.ap_enumerations as enums

from acs_lib.acs_enums import Health
from acs_lib.acs_ap_courier import ACS_AP_Courier

class ACS_UAVState(object):
    '''
    Contains state information for a single UAV.
    '''

    def __init__(self, id):
        #status variables
        self.__id = id
        self.__name = ""
        self.__mode = -1
        self.__new_mode = False  # Set to True when the autopilot mode changes
        self.__batt_rem = 0
        self.__batt_vcc = 0
        
        self.__mis_cur = 0

        self.__fence_state = 2 #0=not_breached, 1=breached, 2=disabled
        self.__swarm_state = 0
        self.__subswarm = 0
        self.__ctl_mode = 0
        self.__swarm_behavior = 0
        self.__last_status_update = 0.0 #last time this class updated status
        self.__last_status_ts = 0.0     #last status message timestamp

        self.__airspeed = 0.0
        #part of status, not pose?
        self.__alt_rel = 0.0

        #pose variables
        self.__lat = 0.0
        self.__lon = 0.0
        self.__alt = 0.0
        self.__quat = (0.0, 0.0, 0.0, 0.0)
        self.__last_pose_update = 0.0   #last time SwarmCommander updated pose
        self.__last_pose_ts = 0.0       #last pose status timestamp

        #bitmask to capture health state
        self.__health_state = Health.HEALTHY

        #fetches and stores autopilot messages when necessary
        self.ap_courier = ACS_AP_Courier(self.__id)
                
        #threading locks;
        self.__status_lock = RLock()
        self.__pose_lock = RLock()        

    def get_name(self):
        with self.__status_lock:
            return self.__name

    def get_mode(self):
        with self.__status_lock:
            return self.__mode

    def is_new_mode(self):
        with self.__status_lock:
            return self.__new_mode

    def get_mode_str(self):
        current_mode = 'ERROR'
        with self.__status_lock:
            try:
                current_mode = enums.MODE_STRINGS[self.__mode]
            except:
                pass #failed to get mode string, assume bad index

        return current_mode

    def get_batt_rem(self):
        with self.__status_lock:
            return self.__batt_rem

    def get_batt_vcc(self):
        with self.__status_lock:
            return self.__batt_vcc

    def get_gps_ok(self):
        with self.__status_lock:
            return (self.__health_state & Health.GPS != 0)

    def get_armed(self):
        with self.__status_lock:
            return (self.__health_state & Health.DISAMRED == 0) 

    def get_curr_wp(self):
        with self.__status_lock:
            return self.__mis_cur

    def get_swarm_state(self):
        with self.__status_lock:
            return self.__swarm_state

    def get_swarm_state_str(self):
        ret_val = 'ERROR'
        with self.__status_lock:
            try:
                ret_val = enums.STATE_STRINGS[self.__swarm_state]
            except:
                pass # failed to get state string, assume bad index

        return ret_val

    def get_swarm_state_id_from_str(self, state_str):
        ret_val = -1
        try:
            ret_val = enums.STATE_VALUES[state_str]
        except:
            pass #failed to get state id, assume bad index

        return ret_val
    
    def get_subswarm(self):
        with self.__status_lock:
            return self.__subswarm

    def get_ctl_mode(self):
        with self.__status_lock:
            return self.__ctl_mode

    def get_ctl_mode_str(self):
        ret_val = 'ERROR'
        with self.__status_lock:
            try:
                ret_val = enums.CTL_MODES[self.__ctl_mode]
            except:
                pass # failed to get ctl mode string, assume bad index

        return ret_val

    def get_swarm_behavior(self):
        with self.__status_lock:
            return self.__swarm_behavior

    def get_swarm_behavior_str(self):
        ret_val = 'ERROR'
        with self.__status_lock:
            try:
                ret_val = enums.SWARM_BHVRS[self.__swarm_behavior]
            except:
                pass #failed to get string, assume bad index

        return ret_val

    def get_last_status_update(self):
        with self.__status_lock:
            return self.__last_status_update

    def get_last_status_ts(self):
        with self.__status_lock:
            return self.__last_status_ts

    def get_lat(self):
        with self.__pose_lock:
            return self.__lat

    def get_lon(self):
        with self.__pose_lock:
            return self.__lon

    def get_alt(self):
        with self.__pose_lock:
            return self.__alt

    def get_alt_rel(self):
        with self.__status_lock: #Weird that this is part of status vice pose
            return self.__alt_rel

    def get_airspeed(self):
        with self.__status_lock: 
            return self.__airspeed

    def get_quat(self):
        with self.__pose_lock: 
            return self.__quat

    def get_last_pose_update(self):
        with self.__pose_lock: 
            return self.__last_pose_update

    def get_last_pose_ts(self):
        with self.__pose_lock: 
            return self.__last_pose_ts

    #Returns bitmask of health state 
    def get_health_state(self):
        now = time.time()   
        
        with self.__status_lock: 
            #Link 
            if now - self.__last_status_update > 10.0:
                #print("Link red alert!\n")
                self.__health_state |= Health.LINK_RED
            elif now - self.__last_status_update > 5.0:
                #print("Link yellow alert!\n")
                self.__health_state |= Health.LINK_YELLOW 

            return self.__health_state

    def status_str(self):
        stat_str = ""
        stat_str += "\tName:     " + self.get_name() + "\n"
        stat_str += "\tMode:     " + self.get_mode_str() + "\n"
        stat_str += "\tBatt:     " + str(self.get_batt_rem()) + "\n"
        stat_str += "\tBatt V:   " + str(self.get_batt_vcc()) + "\n"
        stat_str += "\tGPS OK?   " + str(self.get_gps_ok()) + "\n"
        stat_str += "\tCtl Mode: " + str(self.get_ctl_mode_str()) + "\n"
        stat_str += "\tSubswarm: " + str(self.get_subswarm())
        stat_str += "\tSwarm State: " + self.get_swarm_state_str() 
        stat_str += "\tSwarm Behav: " + self.get_swarm_behavior_str() + "\n"

        return stat_str

    #There are no single-variable setters in this class ON PURPOSE.
    #I want variables to be set by network message only and all at once.
    
    def set_health_state(self, msg):
        with self.__status_lock: 
            self.__health_state = Health.HEALTHY
    
            if (self.__mode == enums.MANUAL):
                self.__health_state |= Health.MODE_RED
            elif (self.__mode != enums.AUTO):
                #print("Mode alert!\n")
                self.__health_state |= Health.MODE_YELLOW

            if (self.__batt_rem < 20 or self.__batt_vcc < 10.6):
                #print("Batt red alert!\n")
                self.__health_state |= Health.BATT_RED
            elif (self.__batt_rem < 40 or self.__batt_vcc < 10.8):
                #print("Batt yellow alert!\n")
                self.__health_state |= Health.BATT_YELLOW
      
            if msg.ok_gps == 0:
                #print("GPS alert!\n")
                self.__health_state |= Health.GPS

            if msg.armed == 0:
                self.__health_state |= Health.DISARMED
        
            #As currently implemented in ArduPlane, AHRS gives too many 
            #false positives.
            #if msg.ok_ahrs == 0:
            #    self.__health_state |= Health.AHRS

            #TODO: Need a way to indicate gyro health

            if msg.ok_as == 0:
                self.__health_state |= Health.ARSPD_CAL

            #TODO: verify this is a bool, not a float representing airspeed
            #self.__as_calib = msg.airspeed

            if msg.ok_ins == 0:
                self.__health_state |= Health.INS

            if msg.ok_mag == 0:
                self.__health_state |= Health.MAG

            if msg.ok_wp == 0:
                self.__health_state |= Health.WP_WRONG
         
            #TODO: reactivate for field (during FX23)
            #AND: make work in SITL (post event)
            #if msg.ok_prm == 0:
            #    self.__health_state |= Health.PRM_WRONG

            #TODO: make ok_fen work for SITL
            if msg.ok_fen == 0:
                self.__health_state |= Health.FENCE_WRONG

            if msg.fence_state == 1:
                self.__health_state |= Health.FENCE_BREACH
            if msg.fence_state == 2:
                self.__health_state |= Health.FENCE_DISABLED
        
            if msg.ok_ral == 0:
                self.__health_state |= Health.RAL_WRONG

            if msg.ready == 0:
                self.__health_state |= Health.NOT_FLT_READY

    def update_status(self, msg):
        with self.__status_lock: 
            #only want this message if it is newer than the previous one
            # -- need to look at message time stamp
            if (self.__last_status_ts > msg.msg_secs):
                return

        name = msg.name
        #TODO: remove this workaround when we switch everthing to Python3:
        name = name[2:name.find("\\x00")]

        with self.__status_lock:
            self.__name = name
            if self.__mode != msg.mode:
                self.__new_mode = True
            else:
                self.__new_mode = False
            self.__mode = msg.mode
            self.__batt_rem = msg.batt_rem
            self.__batt_vcc = msg.batt_vcc/1000.0 # Msg format is Volts * 1000
        
            self.__mis_cur = msg.mis_cur

            self.__alt_rel = msg.alt_rel / 1000.0
            self.__airspeed = msg.airspeed

            self.__fence_state = msg.fence_state

            self.__swarm_state = msg.swarm_state
            self.__subswarm = msg.msg_sub
            self.__ctl_mode = msg.ctl_mode
            self.__swarm_behavior = msg.swarm_behavior
        
            self.__last_status_ts = msg.msg_secs
            self.__last_status_update = time.time()
        
            self.set_health_state(msg)

    def update_pose(self, msg_timestamp, lat, lon, alt, quat):
        with self.__pose_lock: 
            #only want this message if it is newer than the previous one
            # -- need to look at message time stamp
            if (self.__last_pose_ts > msg_timestamp):
                return

            self.__lat = lat
            self.__lon = lon
            self.__alt = alt
            self.__quat = quat

            self.__last_pose_ts = msg_timestamp
            self.__last_pose_update = time.time()

    def update_ap_msgs(self, msg):
        self.ap_courier.update_ap_msgs(msg)
        
