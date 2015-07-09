"""
    Global enumerations defined for the ACS lib.

    Michael Day
    July 2015
"""

from enum import Enum
from enum import IntEnum

class Health(IntEnum):
    HEALTHY =       0b0000000000000000
    GPS =           0b0000000000000001
    LINK_YELLOW =   0b0000000000000010
    LINK_RED =      0b0000000000000100
    MODE =          0b0000000000001000
    BATT_RED =      0b0000000000010000
    BATT_YELLOW =   0b0000000000100000
    DISARMED =      0b0000000001000000
    #TODO: Motor, payload, fence etc.  see failure mode diagram

