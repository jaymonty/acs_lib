"""
    Aerial Combat Swarms (ACS) ground network interface class
    Authors: Michael Day, Duane Davis, Mike Clement
    Date: Oct 2014
    Modified: June 2015 to make easier to access outside Swarm Commander.
"""
from ap_lib.acs_socket import Socket
from ap_lib import acs_messages

from atcommander import ATCommandSet

import glob, subprocess, threading, time

class ACS_NetworkGround(object):
    def __init__(self,device,port,sock_id):

        self.__port = port
        self.__device = device
        self.__my_ip = None
        self.__bcast_ip = None

        self.__acs_socket_id = sock_id

        self.__msg_rcvd_cb = None

        self.__heartbeat_enabled = True
        self.__heartbeat_count = 0
        self.__heartbeat_rate = 2.0 #Hz
                
        self.init_comm_threads()

        self.__mavproxy_watchers = {}
        self.__watch_mavproxy = True
        self.__mp_watcher_t = threading.Thread(target=self.mavproxy_watcher_thread)
        self.__mp_watcher_t.daemon = True
        self.__mp_watcher_t.start()

    def set_message_received_callback(self, cb):
        ''' Register callback for message receipt '''
        self.__msg_rcvd_cb = cb

    def mavproxy_watcher_thread(self):
        while self.__watch_mavproxy:
            entries_to_remove = []
            for id, proc in self.__mavproxy_watchers.items():
                if proc.poll() is not None: #has the process closed?
                    #shut down slave
                    self.close_mavproxy_slave(id) 

                    #mark entry for removal from dictionary
                    entries_to_remove.append(id)

            #remove any marked dictionary entries
            for id in entries_to_remove:
                del self.__mavproxy_watchers[id]

            #don't need to poll very often for local closed MAVProxies
            time.sleep(3)

    def init_comm_threads(self):
        self.__hb_t = None
        self.__t = None

        self.__stop_heartbeat_thread = False
        self.__hb_t = threading.Thread(target=self.heartbeat_thread)
        self.__hb_t.daemon = True
        self.__hb_t.start()

        #read socket
        self.__sock = None
        self.open_socket()
        self.__time_to_stop = False
        self.__t = threading.Thread(target=self.read_socket)
        self.__t.daemon = True
        self.__t.start()

    def get_heartbeat_count(self):
        return self.__heartbeat_count

    def set_heartbeat_enabled(self, do_enable = True):
        self.__heartbeat_enabled = do_enable

    def get_heartbeat_enabled(self):
        return self.__heartbeat_enabled

    def heartbeat_thread(self):
        try:
            sock = Socket(self.__acs_socket_id, self.__port, self.__device,
                    None, None, send_only=True)
        except Exception:
            print("Couldn't start up the ACS ground heartbeat.")
            return
    
        message = acs_messages.Heartbeat()
        message.msg_dst = Socket.ID_BCAST_ALL
        message.msg_secs = 0
        message.msg_nsecs = 0
        message.counter = self.__heartbeat_count

        while not self.__stop_heartbeat_thread:
            if self.__heartbeat_enabled:
                sock.send(message)
                self.__heartbeat_count += 1
                message.counter += self.__heartbeat_count

            time.sleep(1.0 / self.__heartbeat_rate)

    def open_socket(self):
        self.__my_ip = None
        self.__bcast_ip = None

        try:
            self.__sock = Socket(self.__acs_socket_id, self.__port,
                    self.__device, self.__my_ip, self.__bcast_ip)
        except Exception as e:
            print("Couldn't start up socket on interface", self.__device)
            return

        # NOTE: The next two lines are *definitely* not the most pythonic
        #  (shouldn't just grab class data members)
        self.__my_ip = self.__sock._ip
        self.__bcast_ip = self.__sock._bcast

    def read_socket(self):
        while not self.__time_to_stop:
            #give up the CPU for a bit
            #(0.002 secs -> about 500 Hz read rate in the worst case:
            #50 planes * 10 msgs/sec = 500 Hz
            time.sleep(0.002)

            #There is a chance somebody is trying to reopen the
            #socket in a different thread:
            if self.__sock == None:
                continue

            msg = self.__sock.recv()

            if msg is False:      # Saw a message, but not one we can use
                continue          # Check for more messages in queue
            if msg is None:       # No messages available in queue
                continue
        
            if (self.__msg_rcvd_cb is not None):
                self.__msg_rcvd_cb(msg)

    def unload(self):
        ''' Called when ACS Network modoule is unloaded'''
        #cleanup open sockets, shut down threads etc.

        self.__time_to_stop = True

    def send_message_to(self, id, message):
        message.msg_dst = id
        cur_time = time.clock()
        message.msg_secs = int(cur_time)
        message.msg_nsecs = int(1e9 * (cur_time - int(cur_time)))
        
        res = None
        try:
            res = self.__sock.send(message)
        except Exception as ex:
            print (ex.args)
        return res

    #this method with arm or disarm (set arm_state to False to disarm)
    def arm_throttle_for(self, plane_id, arm_state=True):
        msg = acs_messages.Arm()
        msg.enable = arm_state
        msg.msg_fl_rel = True

        self.send_message_to(plane_id, msg)

    def change_mode_for(self, id, mode):
        message = acs_messages.Mode()
        message.mode = mode
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def set_controller_for(self, id, controller):
        message = acs_messages.SetController()
        message.controller = controller
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def set_waypoint_goto_for(self, id, wp_id):
        message = acs_messages.WaypointGoto()
        message.index = wp_id
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def set_subswarm_for(self, id, subswarm):
        message = acs_messages.SetSubswarm()
        message.subswarm = subswarm
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_behavior_for(self, id, behavior):
        message = acs_messages.SwarmBehavior()
        message.swarm_behavior = behavior
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def suspend_swarm_behavior_for(self, id):
        message = acs_messages.SuspendSwarmBehavior()
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def pause_swarm_behavior_for(self, id, pause):
        message = acs_messages.PauseSwarmBehavior()
        message.behavior_pause = pause
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_egress_for(self, id):
        message = acs_messages.SwarmEgress()
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_follow_for(self, id, distance, angle, stack_formation):
        message = acs_messages.SwarmFollow()
        message.distance = distance
        message.angle = angle
        message.stack_formation = stack_formation
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_sequence_land_for(self, id, ldg_wpt):
        message = acs_messages.SwarmSequenceLand()
        message.ldg_wpt = ldg_wpt
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_search_for(self, id, searchAreaLength, searchAreaWidth, lat, lon, masterSearcherID, searchAlgoEnum):
        message = acs_messages.SwarmSearch()
        message.searchAreaLength = searchAreaLength
        message.searchAreaWidth = searchAreaWidth
        message.lat = lat
        message.lon = lon
        message.masterSearcherID = masterSearcherID
        message.searchAlgoEnum = searchAlgoEnum
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def swarm_state_for(self, id, state):
        message = acs_messages.SwarmState()
        message.swarm_state = state
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def set_autopilot_heartbeat_for(self, id, enable=True):
        message = acs_messages.PayloadHeartbeat()
        message.enable = enable
        
	#only set the "fl_rel" flag for messages that _must_ be reliable:
        message.msg_fl_rel = False
        
        self.send_message_to(id, message)

    def send_mission_config_for(self, id, alt, stack, tkoff_mode):
        mc = acs_messages.MissionConfig()
        mc.msg_dst = id
        mc.msg_secs = 0
        mc.msg_nsecs = 0
        mc.std_alt = alt
        mc.stack_num = stack
        mc.takeoff_active = tkoff_mode
        self.send_message_to(id, mc)

    def send_ap_msgs_request_for(self, id, n, since_seq):
        msg = acs_messages.ReqPrevNMsgsAP()
        msg.msg_dst = id
        msg.n = n
        msg.since_seq = since_seq
        self.send_message_to(id, msg)

    # Might want to depricate this eventually (direct controller setup and
    # invocation via network message bypasses the swarm_manager node and
    # has potential race conditions that might lead to unsafe situations.
    # Functionality in SwarmCommander will be eliminated before April event.
    def set_follower_params_for(self, id, leader_id, follow_range, \
                                offset_angle, alt_mode, ctrl_alt, seq_num):
        message = acs_messages.FollowerSetup()
        message.leader_id = leader_id
        message.follow_range = follow_range
        message.offset_angle = offset_angle
        message.alt_mode = alt_mode
        message.control_alt = ctrl_alt
        message.seq = seq_num
        message.msg_fl_rel = True

        self.send_message_to(id, message)

    def setup_mavlink_slave_ch(self, target_id, port, chan, enable=True):
        ''' Open/close a slave mavlink channel to the aircraft '''
        ss = acs_messages.SlaveSetup()
        ss.msg_dst = int(target_id)
        ss.msg_secs = 0
        ss.msg_nsecs = 0
        ss.channel = chan
        ss.enable = enable
        ss.msg_fl_rel = True
    
        self.__sock.send(ss)

    def enable_slave(self, target_id, port, chan):
        self.setup_mavlink_slave_ch(target_id, port, chan, True)

    def disable_slave(self, target_id, port, chan):
        self.setup_mavlink_slave_ch(target_id, port, chan, False)

    def set_device(self, device_name):
        #shut off heartbeat thread
        self.__stop_heartbeat_thread = True

        #shut off read thread and socket
        self.__time_to_stop = True
        self.__sock = None

        #pause a sec for thread(s) to cleanup
        time.sleep(1)
        
        self.__device = device_name

        #start socket on new device
        #self.open_socket()

        #restart heartbeat thread
        self.init_comm_threads()

    def get_device(self):
        return self.__device

    def get_port(self):
        return self.__port

    def get_acs_id(self):
        return self.__acs_socket_id

    def slave_port_and_master_str(self, plane_id):
         #pick an aircraft-unique port
        slave_port = 15554 + int(plane_id)
        master_str = "udp:%s:%u" % (self.__my_ip, slave_port)

        return (slave_port, master_str)
    
    def close_mavproxy_slave(self, plane_id):
        (slave_port, master_str) = self.slave_port_and_master_str(plane_id)
        self.disable_slave(plane_id, slave_port, master_str)

    def open_mavproxy_wifi(self, plane_id):
        if plane_id in self.__mavproxy_watchers:
            print("Mavproxy appears to already be open for", str(plane_id))
            return

        (slave_port, mavproxy_master) = self.slave_port_and_master_str(plane_id)

        self.enable_slave(plane_id, slave_port, mavproxy_master) 

        # Start up a MAVProxy instance and connect to slave channel
        proc = subprocess.Popen( ["/usr/bin/xterm", "-e", "mavproxy.py --baudrate 57600 --master " + mavproxy_master + " --speech --aircraft uav_" +  plane_id] )

        #the mavproxy watcher thread needs to be aware of this new process,
        #so the slave channel can be closed when MAVProxy closes:
        self.__mavproxy_watchers[plane_id] = proc

    #allow outside class to stop a slow or hung radio config attempt
    def abort_radio_config(self):
        self.__abort_SiK = True

    #Throws Exception if unsuccessful configuring telem radio,
    def open_mavproxy_SiK(self, plane_id):
        radios = glob.glob("/dev/serial/by-id/usb-FTDI*")
        if len(radios) < 1:
            raise Exception("No telem radios found.")
            return

        self.__abort_SiK = False

        #just use the first radio
        atc = ATCommandSet(radios[0])

        #calculate freq band based on id
        minfreq = (plane_id % 3) * 9000 + 902000
        maxfreq = (plane_id % 3) * 9000 + 910000

        # Enumerate commands by function call
        commands = [lambda: atc.leave_command_mode_force(),
                    lambda: atc.unstick(),
                    lambda: atc.enter_command_mode(),
                    lambda: atc.set_param(ATCommandSet.PARAM_NETID,
                                          plane_id),
                    lambda: atc.set_param(ATCommandSet.PARAM_MIN_FREQ,
                                          minfreq),
                    lambda: atc.set_param(ATCommandSet.PARAM_MAX_FREQ,
                                          maxfreq),
                    lambda: atc.write_params(),
                    lambda: atc.reboot(),
                    lambda: atc.leave_command_mode()]

        retries_left = 10
        command = commands.pop(0)
        while retries_left > 0 and self.__abort_SiK is not True:
            try:
                # Always give a little time for the radio to settle
                time.sleep(0.1)
                res = command()
                if res == False:
                    # Some commands return True/False, others return None
                    # Only False is bad
                    raise Exception("command failed")
                if len(commands) == 0:
                    # If no commands left, we're done
                    break
                command = commands.pop(0)
            except Exception as ex:
                print("Radio Config exception: " + str(ex) + ", retrying ...")
                retries_left -= 1 

        if retries_left <= 0:
            raise Exception("Failed to config radio.")
            return
        
        #wait a moment for the radio to finish config
        time.sleep(0.5)
        
        if self.__abort_SiK is True:
            raise Exception("User aborted radio config.")
            return

        #Fire up MAVProxy
        subprocess.Popen( \
            "xterm -e mavproxy.py --speech --console --map --master %s --baudrate 57600" % \
            radios[0],
            shell=True,
            cwd="/tmp")

