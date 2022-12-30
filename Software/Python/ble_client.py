# -*- coding: utf-8 -*-
"""
BLE client module for communication with gloves.

Created on Thu Apr 30 17:33:47 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""


import asyncio
import nest_asyncio
nest_asyncio.apply()
from bleak import discover
import struct
from madgwick import Madgwick
#from mahony import Mahony
#from kalman import Kalman
import numpy as np

import datetime
import pandas as pd
from itertools import chain

#COLLECTION ONLY: fcols = [['time', 'voltage']]
#COLLECTION ONLY: fcols.append(list(chain.from_iterable([['%02d_ax' % i, '%02d_ay' % i, '%02d_az' % i, '%02d_gx' % i, '%02d_gy' % i, '%02d_gz' % i, '%02d_mx' % i, '%02d_my' % i, '%02d_mz' % i, '%02d_roll' % i, '%02d_pitch' % i, '%02d_yaw' % i] for i in range(15)])))
#COLLECTION ONLY: fcols.append(['temperature', 'pressure'])
#COLLECTION ONLY: col_names = list(chain.from_iterable(fcols))
#COLLECTION ONLY: collected_data  = pd.DataFrame(columns = col_names)

class AngleSensor:
    MAG_OFFSETS = np.array([9.83, 4.42, -6.97])

    # Soft iron error compensation matrix
    MAG_SOFTIRON_MATRIX = np.array([ [  0.586,  0.006,  0.001 ],
                                     [  0.006,  0.601, -0.002 ],
                                     [  0.001,  -0.002,  2.835 ] ])

    MAG_FIELD_STRENGTH = 56.33
    
    MAG_MGAUSS_TO_UT = 0.1
    SENSORS_GRAVITY_STANDARD = 9.80665

    def __init__(self, index):
        self.sensor_id = index
        self.LSM6DSL_sensitivity_A = 0.0
        self.LSM6DSL_sensitivity_G = 0.0
        self.LIS2MDL_sensitivity_M = 0.0
        self.data_time_diff = 0
        self.fusion = Madgwick()
        self.last_data_time = 0
        self.last_print_time = 0
    
    def process_data(self, time, data):
        self.data_time_diff = time - self.last_data_time
        self.last_data_time = time
        
        #BLEClient.print("Timediff: %d ms" % self.data_time_diff)
        #BLEClient.print("Data [", len(data) ,"]:", data)
        
        #int16s = struct.unpack('<h', data)
        int16s = np.array([int.from_bytes(data[i:i+2], byteorder="little", signed=True) for i in range(0, len(data), 2)])
        
        accelerometer = tuple(int16s[0:3] * self.LSM6DSL_sensitivity_A / 1000 * AngleSensor.SENSORS_GRAVITY_STANDARD)
        gyroscope = tuple(int16s[3:6] * self.LSM6DSL_sensitivity_G / 1000)
        magnetometer = tuple([0.0, 0.0, 0.0])
        if len(int16s) == 9:
            magnetometer = tuple(AngleSensor.MAG_SOFTIRON_MATRIX.dot(AngleSensor.MAG_MGAUSS_TO_UT * int16s[6:9] * self.LIS2MDL_sensitivity_M - AngleSensor.MAG_OFFSETS))
            
        try:
            self.fusion.update(accelerometer, gyroscope, magnetometer, self.data_time_diff/1000)
        except Exception as e:
            BLEClient.print("Fusion update error: %s" % str(e))
    
        '''if self.sensor_id == BLEClient.NUM_ANGLE_SENSORS - 1 and time - self.last_print_time > 100:
            #BLEClient.print("#%02d | %06u ms | a: [%04d %04d %04d] | g: [%04d %04d %04d] | m: [%04d %04d %04d]" % (self.sensor_id, time, int16s[0], int16s[1], int16s[2], int16s[3], int16s[4], int16s[5], int16s[6], int16s[7], int16s[8]), debug=True)
            #BLEClient.print("#%02d | %06u ms | a: [%04d %04d %04d] | g: [%04d %04d %04d]" % (self.sensor_id, time, int16s[0], int16s[1], int16s[2], int16s[3], int16s[4], int16s[5]), debug=True)
            a = self.get_angles()
            #BLEClient.print("#%02d | %06u ms | a: [%03.2f %03.2f %03.2f]" % (self.sensor_id, time, a[0], a[1], a[2]), debug=True)
            BLEClient.print("#%02d | %06u ms | a: [%04d %04d %04d] | g: [%04d %04d %04d] | m: [%04d %04d %04d] | °: [%03.2f %03.2f %03.2f]" % (self.sensor_id, time, int16s[0], int16s[1], int16s[2], int16s[3], int16s[4], int16s[5], int16s[6], int16s[7], int16s[8], a[0], a[1], a[2]), debug=True)
            self.last_print_time = time'''
        
        return list(chain.from_iterable([list(accelerometer), list(gyroscope), list(magnetometer)]))
    
    def set_sensitivities(self, a, g, m):
        self.LSM6DSL_sensitivity_A = a
        self.LSM6DSL_sensitivity_G = g
        self.LIS2MDL_sensitivity_M = m
    
    def get_angles(self):
        return np.rad2deg(self.fusion.computeAngles())
        #return np.rad2deg(self.fusion.quaternion.to_euler_angles())
        

async def find_device(loop):
    target_address = None
    attempt = 0
    BLEClient.print("Searching BLE devices...")
    while BLEClient.RUNNING and target_address is None and attempt < BLEClient.SEARCH_ATTEMPTS:
        dev = await discover()
        for i in range(0,len(dev)):
            BLEClient.print("Found BLE device: '%s'" % dev[i].name)
            if dev[i].name == BLEClient.TARGET_NAME:
                  target_address = dev[i].address
                  break
            attempt += 1
            BLEClient.print("(searching again)")
            await asyncio.sleep(0.1, loop=loop)
    return target_address


def notification_handler(sender, data):
    '''BLEClient.print("Received (", len(data), "): ", end="")
    for b in data:
        BLEClient.print("0x%02x " % b, end="")
    BLEClient.print()'''
    #COLLECTION ONLY: collected_data_packet = list()
    
    mempos = 0
    p = 0
    time = struct.unpack('<I', data[p:p+4])[0]
    #COLLECTION ONLY: collected_data_packet.append(time)
    BLEClient.SHARED_MEM[mempos] = time
    p += 4
    mempos += 1
    
    BLEClient.SHARED_MEM[mempos] = struct.unpack('<f', data[p:p+4])[0]
    #COLLECTION ONLY: collected_data_packet.append(BLEClient.SHARED_MEM[mempos])
    mempos += 1
    p += 4
    
    #BLEClient.print("\rTime: %d ms; Voltage: %.3f V" % (time, BLEClient.SHARED_DICT["voltage"]), debug=True)
    
    for i in range(BLEClient.NUM_ANGLE_SENSORS - 1):
        agm = BLEClient.ANGLE_SENSORS[i].process_data(time, data[p:p+12])
        #COLLECTION ONLY: collected_data_packet.extend(agm)
        a = BLEClient.ANGLE_SENSORS[i].get_angles()
        BLEClient.SHARED_MEM[mempos] = -a[0]
        BLEClient.SHARED_MEM[mempos+1] = a[2]
        BLEClient.SHARED_MEM[mempos+2] = -a[1]
        #COLLECTION ONLY: collected_data_packet.extend(a)
        mempos += 3
        #BLEClient.print("Sensor %d angles: %.2f %.2f %.2f" % (i, a[0], a[1], a[2]), debug=True)
        p += 12
    
    agm = BLEClient.ANGLE_SENSORS[-1].process_data(time, data[p:p+12] + data[p+20:p+26])
    #COLLECTION ONLY: collected_data_packet.extend(agm)
    a = BLEClient.ANGLE_SENSORS[-1].get_angles()
    BLEClient.SHARED_MEM[mempos] = -a[0]
    BLEClient.SHARED_MEM[mempos+1] = a[2]
    BLEClient.SHARED_MEM[mempos+2] = -a[1]
    #COLLECTION ONLY: collected_data_packet.extend(a)
    mempos += 3
    #BLEClient.print("Hand sensor angles: %.2f %.2f %.2f" % (a[0], a[1], a[2]), debug=True)
    p += 12
    
    BLEClient.SHARED_MEM[mempos], BLEClient.SHARED_MEM[mempos+1] = struct.unpack('<ff', data[p:p+8])
    #COLLECTION ONLY: collected_data_packet.append(BLEClient.SHARED_MEM[mempos])
    #COLLECTION ONLY: collected_data_packet.append(BLEClient.SHARED_MEM[mempos+1])
    mempos += 2
    
    #BLEClient.print("Temp: %.1f °C; Press: %.2f hPa" % (BLEClient.SHARED_DICT["temperature"], BLEClient.SHARED_DICT["pressure"]), debug=True)
    #BLEClient.print("Written %d values into float array" % mempos, debug=True)
    
    #COLLECTION ONLY: collected_data.loc[len(#COLLECTION ONLY: collected_data)] = #COLLECTION ONLY: collected_data_packet
    print("Collected %d rows..." % mempos) #COLLECTION ONLY: len(collected_data)
    if mempos == 100000: #COLLECTION ONLY: len(collected_data)
        print("Collecting done!")
        BLEClient.RUNNING = False
    

async def device_connect(address, loop):
    from bleak import BleakClient, BleakError
    
    while BLEClient.RUNNING:
        BLEClient.print("Connecting to BLE device ... ")
        try:
            async with BleakClient(address, loop=loop) as client:
                x = await client.is_connected()
                BLEClient.print("Connected: {0}".format(x))
                
                await read_sensor_conf(client);
                await read_sensor_data(client);
                
                BLEClient.print("Disconnecting")
                await client.disconnect()
                
                break
        except BleakError as e:
            BLEClient.print("BleakError: %s" % str(e))
            continue

async def read_sensor_data(client):
    BLEClient.print("Starting notifications")
    await client.start_notify(BLEClient.CHAR_DATA_UUID, notification_handler)
    while BLEClient.RUNNING:
        pass
    
                
    #COLLECTION ONLY: collected_data.to_csv("measurement_" + str(datetime.datetime.now().date()) + ".csv")
                
    BLEClient.print("Stopping notifications")
    await client.stop_notify(BLEClient.CHAR_DATA_UUID)

async def read_sensor_conf(client):
    BLEClient.print("Reading configuration ... ")
    data = await client.read_gatt_char(BLEClient.CHAR_CONF_UUID)
    
    '''BLEClient.print("Received (", len(data), "): ", end="")
    for b in data:
        BLEClient.print("0x%02x " % b, end="")
    BLEClient.print()'''
    
    i = 0
    a = 0
    g = 0
    m = 0
    while i < len(data):
        sensor_id = data[i]
        i += 1
        while i < len(data):
            if data[i] == ord('A'):
                a = struct.unpack('<f', data[i+1:i+5])[0]
                i += 5
            elif data[i] == ord('G'):
                g = struct.unpack('<f', data[i+1:i+5])[0]
                i += 5
            elif data[i] == ord('M'):
                m = struct.unpack('<f', data[i+1:i+5])[0]
                i += 5
            else:
                break
            
        if sensor_id < BLEClient.NUM_ANGLE_SENSORS:
            BLEClient.print("Setting sensitivity of sensor %d to a=%.2f, g=%.2f" % (sensor_id, a, g))
            BLEClient.ANGLE_SENSORS[sensor_id].set_sensitivities(a, g, 0)
        else:
            BLEClient.print("Setting sensitivity of hand sensor to a=%.2f, g=%.2f, m=%.2f" % (a, g, m))
            BLEClient.ANGLE_SENSORS[-1].set_sensitivities(a, g, m)

            
class BLEClient():
    TARGET_NAME = "Smart Glove"
    CHAR_DATA_UUID = "1bc5d5a5-0200-36ac-e111-010000000000"
    CHAR_CONF_UUID = "1bc5d5a5-0200-56bc-e111-010000000000"
    SEARCH_ATTEMPTS = 20
    NUM_ANGLE_SENSORS = 15
    ANGLE_SENSORS = [AngleSensor(i) for i in range(NUM_ANGLE_SENSORS)]
    RUNNING = True
    LOGGER = None
    SHARED_MEM = None
    
    def __init__(self, logger, shared_mem):
        BLEClient.LOGGER = logger
        BLEClient.SHARED_MEM = shared_mem
         
        loop = asyncio.get_event_loop()
        target_address = loop.run_until_complete(find_device(loop))
        BLEClient.print("Searching ended!")
        
        if target_address is None:
            BLEClient.print("Device %s was not found" % BLEClient.TARGET_NAME)
        else:
            BLEClient.print("Device %s was found, address = %s" % (BLEClient.TARGET_NAME, repr(target_address)))
        
            loop = asyncio.get_event_loop()
            loop.run_until_complete(device_connect(target_address, loop))
            
    @staticmethod
    def print(s, debug=False):
        if BLEClient.LOGGER is None or debug:
            print(s)
        else:
            BLEClient.LOGGER.send(s)
            

if __name__ == "__main__":
    BLEClient(None, [0.0] * (15*3+4))
    