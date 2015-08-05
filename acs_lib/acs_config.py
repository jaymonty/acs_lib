"""
ACS_Config Class

For management of common configurations parameters (network device, port,
window sizes, etc) for ACS utililities such as Swarm Commander and
the Health Monitor.

Author: Michael Day
Date: Aug 5, 2006
"""

import configparser

class ACS_Config(object):
    def __init__(self, config_file_path):
        self.__config_file = config_file_path
        self.__config = configparser.ConfigParser()
        
        if self.__config.read(self.__config_file) == []:
            print("Unable to parse config file:", self.__config_file)
            #Add known sections for later use:
            self.__config.add_section('NETWORK')

    def get_network_device(self):
        return self.__config.get('NETWORK','device',fallback=None)

    def get_acs_id(self):
        return self.__config.getint('NETWORK','ACS_id',fallback=None)

    def get_network_port(self):
        return self.__config.getint('NETWORK','port',fallback=None)

    def set_network_device(self, dev):
        self.__config.set('NETWORK','device',dev)

    def set_acs_id(self, id):
        self.__config.set('NETWORK','ACS_id',str(id))

    def set_network_port(self, port):
        self.__config.set('NETWORK','port',str(port))

    def save(self, new_file_path=None):
        path = self.__config_file
        if new_file_path is not None:
            path = new_file_path

        with open(path,'w') as configfile:
            self.__config.write(configfile)
