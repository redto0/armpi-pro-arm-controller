#!/usr/bin/env python3
import os
import sys
import time
import RPi.GPIO as GPIO
from BusServoCmd import *
from smbus2 import SMBus, i2c_msg

# Hiwonder Technology Raspberry Pi Expansion Board SDK #
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

def setBusServoID(oldid, newid):
    """
    Configure servo ID (factory default is 1)
    :param oldid: Original ID (factory default is 1)
    :param newid: New ID
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID(id=None):
    """
    Read serial servo ID
    :param id: Default is None (only one servo allowed on bus if None)
    :return: Servo ID
    """
    while True:
        if id is None:  # Only one servo allowed on bus
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # Get response
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time):
    """
    Drive serial servo to specified position
    :param id: Servo ID to drive
    :param pulse: Target position (0-1000)
    :param use_time: Movement duration in ms
    """
    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo(id=None):
    '''
    Stop servo movement
    :param id: Servo ID (None stops all servos)
    :return: None
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation(id, d=0):
    """
    Adjust servo deviation
    :param id: Servo ID
    :param d: Deviation value (-125 to 125)
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def saveBusServoDeviation(id):
    """
    Save deviation setting (persistent after power loss)
    :param id: Servo ID
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

time_out = 50
def getBusServoDeviation(id):
    '''
    Read deviation value
    :param id: Servo ID
    :return: Deviation value or None if timeout
    '''
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit(id, low, high):
    '''
    Set servo angle limits
    :param id: Servo ID
    :param low: Minimum angle (0-1000)
    :param high: Maximum angle (0-1000)
    :return: None
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    '''
    Read servo angle limits
    :param id: Servo ID
    :return: Tuple (low, high) angle limits
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoVinLimit(id, low, high):
    '''
    Set servo voltage limits
    :param id: Servo ID
    :param low: Minimum voltage (mV)
    :param high: Maximum voltage (mV)
    :return: None
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):
    '''
    Read servo voltage limits
    :param id: Servo ID
    :return: Tuple (low, high) voltage limits
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp):
    '''
    Set servo maximum temperature limit
    :param id: Servo ID
    :param m_temp: Maximum temperature (°C)
    :return: None
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    '''
    Read servo temperature limit
    :param id: Servo ID
    :return: Temperature limit (°C)
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    '''
    Read current servo position
    :param id: Servo ID
    :return: Current position (0-1000)
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    '''
    Read servo temperature
    :param id: Servo ID
    :return: Current temperature (°C)
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    Read servo voltage
    :param id: Servo ID
    :return: Current voltage (mV)
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def restBusServoPulse(oldid):
    # Reset servo deviation and center position (500)
    serial_servo_set_deviation(oldid, 0)    # Clear deviation
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # Center position

## Power off servo (unload)
def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

## Read servo power status
def getBusServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg