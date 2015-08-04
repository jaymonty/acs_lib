"""
ACS Speech class

Current just a wrapper for espeak, but this Speech abstraction layer should
facilitate extending beyound espeak in the future, should we choose to do so.

Author: Michael Day
Date: Aug 2015
"""

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

