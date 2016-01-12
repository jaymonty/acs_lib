"""
ACS Speech class

Current just a wrapper for espeak, but this Speech abstraction layer should
facilitate extending beyound espeak in the future, should we choose to do so.

Author: Michael Day
Date: Aug 2015
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
from espeak import espeak

class ACS_Speech(object):
    def __init__(self):
        self.__gender = 'F'

        #preserve default even when someone forgets to call initialize():
        espeak.set_voice('female2')

    def initialize(self, gender='F'):
        self.set_gender(gender)

    def set_gender(self, gender):
        gender = gender.upper()
        
        if gender[0] == 'M':
            espeak.set_voice('male3')
        else:
            espeak.set_voice('female2')

    def say(self, text):
        espeak.synth(text) 

