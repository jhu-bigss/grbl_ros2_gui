# -*- coding: UTF-8 -*-

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                         '
' Copyright 2018-2021 Gauthier Brière (gauthier.briere "at" gmail.com)    '
'                                                                         '
' This file is part of cn5X++                                             '
'                                                                         '
' cn5X++ is free software: you can redistribute it and/or modify it       '
' under the terms of the GNU General Public License as published by       '
' the Free Software Foundation, either version 3 of the License, or       '
' (at your option) any later version.                                     '
'                                                                         '
' cn5X++ is distributed in the hope that it will be useful, but           '
' WITHOUT ANY WARRANTY; without even the implied warranty of              '
' MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           '
' GNU General Public License for more details.                            '
'                                                                         '
' You should have received a copy of the GNU General Public License       '
' along with this program.  If not, see <http://www.gnu.org/licenses/>.   '
'                                                                         '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

from enum import Enum
from PyQt5 import QtGui

ORG_NAME           = "fra589"
APP_NAME           = "cn5X++"
APP_VERSION_STRING = "0.8.3"
APP_VERSION_DATE   = "20210208"

DEFAULT_NB_AXIS    = 5 # Laisser 3 permet de gerer un Grbl original a 3 axes
DEFAULT_AXIS_NAMES = ['X', 'Y', 'Z', 'A', 'B']

COM_DEFAULT_BAUD_RATE = 115200
SERIAL_READ_TIMEOUT   = 250      # ms
GRBL_QUERY_DELAY      =  75      # ms, default 13.3 Hz

DEFAULT_JOG_SPEED     = 1000

class logSeverity(Enum):
  info    = 0
  warning = 1
  error =   2

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                         '
' Definition des commandes de GRBL                                        '
'                                                                         '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

''' Grbl '$' Commands '''
CMD_GRBL_HELP                  = "$"
CMD_GRBL_GET_SETTINGS          = "$$"
CMD_GRBL_GET_GCODE_PARAMATERS  = "$#"
CMD_GRBL_GET_GCODE_STATE       = "$G"
CMD_GRBL_GET_BUILD_INFO        = "$I"
CMD_GRBL_GET_STARTUP_BLOCKS    = "$N"
CMD_GRBL_SET_STARTUP_BLOCK0    = "$N0="
CMD_GRBL_SET_STARTUP_BLOCK1    = "$N1="
CMD_GRBL_TOGGLE_CHECK_MODE     = "$C"
CMD_GRBL_KILL_ALARM_LOCK       = "$X"
CMD_GRBL_RUN_HOME_CYCLE        = "$H"
CMD_GRBL_JOG                   = "$J="
CMD_GRBL_RESET_SETTINGS        = "$RST=$"
CMD_GRBL_RESET_OFFSETS         = "$RST=#"
CMD_GRBL_RESET_ALL_EEPROM      = "$RST=*"
CMD_GRBL_SLEEP                 = "$SLP"

''' Grbl v1.1 Realtime commands '''
REAL_TIME_SOFT_RESET           = chr(0x18) # Ctrl+X
REAL_TIME_REPORT_QUERY         = '?'
REAL_TIME_CYCLE_START_RESUME   = '~'
REAL_TIME_FEED_HOLD            = '!'

''' Extended-ASCII Realtime Commands '''
REAL_TIME_SAFETY_DOOR          = chr(0x84)
REAL_TIME_JOG_CANCEL           = chr(0x85)
REAL_TIME_FEED_100_POURCENT    = chr(0x90)
REAL_TIME_FEED_PLUS_10         = chr(0x91)
REAL_TIME_FEED_MOINS_10        = chr(0x92)
REAL_TIME_FEED_PLUS_1          = chr(0x93)
REAL_TIME_FEED_MOINS_1         = chr(0x94)
REAL_TIME_RAPID_100_POURCENT   = chr(0x95)
REAL_TIME_RAPID_50_POURCENT    = chr(0x96)
REAL_TIME_RAPID_25_POURCENT    = chr(0x97)
REAL_TIME_SPINDLE_100_POURCENT = chr(0x99)
REAL_TIME_SPINDLE_PLUS_10      = chr(0x9A)
REAL_TIME_SPINDLE_MOINS_10     = chr(0x9B)
REAL_TIME_SPINDLE_PLUS_1       = chr(0x9C)
REAL_TIME_SPINDLE_MOINS_1      = chr(0x9D)
REAL_TIME_TOGGLE_SPINDLE_STOP  = chr(0x9E)
REAL_TIME_TOGGLE_FLOOD_COOLANT = chr(0xA0)
REAL_TIME_TOGGLE_MIST_COOLANT  = chr(0xA1)

GRBL_STATUS_IDLE  = 'Idle'
GRBL_STATUS_RUN   = 'Run'
GRBL_STATUS_HOLD0 = 'Hold:0'
GRBL_STATUS_HOLD1 = 'Hold:1'
GRBL_STATUS_JOG   = 'Jog'
GRBL_STATUS_ALARM = 'Alarm'
GRBL_STATUS_DOOR0 = 'Door:0'
GRBL_STATUS_DOOR1 = 'Door:1'
GRBL_STATUS_DOOR2 = 'Door:2'
GRBL_STATUS_DOOR3 = 'Door:3'
GRBL_STATUS_CHECK = 'Check'
GRBL_STATUS_HOME  = 'Home'
GRBL_STATUS_SLEEP = 'Sleep'

TXT_COLOR_GREEN  = QtGui.QColor(0, 92, 0)
TXT_COLOR_ORANGE = QtGui.QColor(255, 127, 0)
TXT_COLOR_RED    = QtGui.QColor(92, 0, 0)
TXT_COLOR_BLUE   = QtGui.QColor(0, 0, 92)

COM_FLAG_NO_FLAG  = 0
COM_FLAG_NO_OK    = 1
COM_FLAG_NO_ERROR = 2

''' qtabMain indexes '''
CN5X_TAB_MAIN     = 0
CN5X_TAB_PROBE_XY = 1
CN5X_TAB_PROBE_Z  = 2

''' qtabConsole indexes '''
CN5X_TAB_GRBL     = 0
CN5X_TAB_FILE     = 1
CN5X_TAB_LOG      = 2
CN5X_TAB_DEBUG    = 3

''' Probe defaults parameters '''
DEFAULT_TOOL_DIAMATER                   = 3
DEFAULT_PROBE_DISTANCE                  = 15
DEFAULT_CLEARANCE_XY                    = 5
DEFAULT_CLEARANCE_Z                     = 5
DEFAULT_PROBE_FEED_RATE                 = 25
DEFAULT_PROBE_SEEK                      = True
DEFAULT_PROBE_SEEK_RATE                 = 75
DEFAULT_PROBE_PULL_OFF_DISTANCE_XY      = 3
DEFAULT_PROBE_PULL_OFF_DISTANCE_Z       = 3
DEFAULT_PROBE_MOVE_AFTER_Z              = True
DEFAULT_PROBE_MOVE_AFTER_XY             = True
DEFAULT_PROBE_GO_2_POINT_AFTER_XY       = True
DEFAULT_PROBE_GO_2_POINT_AFTER_Z        = True
DEFAULT_PROBE_RETRACT_AFTER_XY          = False
DEFAULT_PROBE_RETRACT_AFTER_Z           = False
DEFAULT_PROBE_RETRACT_DISTANCE_AFTER_XY = 5
DEFAULT_PROBE_RETRACT_DISTANCE_AFTER_Z  = 5
DEFAULT_PROBE_ORIGINE_G54_Z             = True
DEFAULT_PROBE_ORIGINE_G92_Z             = False
DEFAULT_PROBE_ORIGINE_OFFSET_Z          = 0.0
DEFAULT_TOOLSENSOR_POSITION_X           = -5
DEFAULT_TOOLSENSOR_POSITION_Y           = -5

''' Valeurs renvoyées par grblDecode.waitForGrblReply() et grblDecode.waitForGrblProbe() '''
SIG_OK    = 0
SIG_ERROR = 2
SIG_ALARM = 4
SIG_PROBE = 8

''' Menu help probe '''
MENU_SINGLE_AXIS    = 0
MENU_INSIDE_CORNER  = 1
MENU_OUTSIDE_CORNER = 2
MENU_INSIDE_CENTER  = 3
MENU_OUTSIDE_CENTER = 4











