"""
ACS_Config Class

For management of common configurations parameters (network device, port,
window sizes, etc) for ACS utililities such as Swarm Commander and
the Health Monitor.

Author: Michael Day
Date: Aug 5, 2006
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
import configparser

class ACS_Config(object):
    def __init__(self, config_file_path):
        self.__config_file = config_file_path
        self.__config = configparser.ConfigParser()
        
        if self.__config.read(self.__config_file) == []:
            print("No config file, ", self.__config_file, 
            " starting with defaults." )
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
