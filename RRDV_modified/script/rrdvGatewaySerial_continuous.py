import csv
import serial
import sys
import time

import OpenHdlc
import struct
import numpy as np

TRIG_ALL  = True
TRIG_PAIR = False

RANGE_ALL = [0x1F, 0x00, 0x00, 0x00]
RANGE_1_2 = [0x03, 0x00, 0x00, 0x00]
RANGE_1_3 = [0x05, 0x00, 0x00, 0x00]
RANGE_1_4 = [0x09, 0x00, 0x00, 0x00]
RANGE_1_5 = [0x11, 0x00, 0x00, 0x00]
RANGE_2_3 = [0x06, 0x00, 0x00, 0x00]
RANGE_2_4 = [0x0A, 0x00, 0x00, 0x00]
RANGE_2_5 = [0x12, 0x00, 0x00, 0x00]
RANGE_3_4 = [0x0C, 0x00, 0x00, 0x00]
RANGE_3_5 = [0x14, 0x00, 0x00, 0x00]
RANGE_4_5 = [0x18, 0x00, 0x00, 0x00]

NUMBER_OF_ROBOTS = 5 # we could support up to 32

AVG =[0,0,0,0,0]

# Open com port
ser = serial.Serial('COM10', 1000000, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

number_of_measurements = 100
line_count             = 1

_hdlc             = OpenHdlc.OpenHdlc() 

hdlcBusyReceiving = False
last_rx_byte_int  =  0x00 

in_buf                    = []
range_measurements_unpack = []
robot_measurements        = []
robot_ids                 = []
timestamp                 = []

experiment_type = input("Enter the type of experiment wanted (range all or range pair) - type ra or rp: ")

if experiment_type == 'ra':
    TRIG_ALL     = True
    TRIG_PAIR    = False
    trigger_data = RANGE_ALL
    hdlc_data_w  = _hdlc.hdlcify(trigger_data)
    
elif experiment_type == 'rp':
    TRIG_ALL     = False
    TRIG_PAIR    = True
    
else :
    print("wrong input")
    quit()

num_cmd_sent = 0

while (1):
    while line_count <= number_of_measurements:   

        if TRIG_ALL == False and TRIG_PAIR == True:
            num_cmd_sent += 1
                
            if num_cmd_sent == 1:
                trigger_data = RANGE_1_2
            elif num_cmd_sent == 2:
                trigger_data = RANGE_1_3
            elif num_cmd_sent == 3:
                trigger_data = RANGE_1_4
            elif num_cmd_sent == 4:
                trigger_data = RANGE_1_5
            elif num_cmd_sent == 5:
                trigger_data = RANGE_2_3
            elif num_cmd_sent == 6:
                trigger_data = RANGE_2_4
            elif num_cmd_sent == 7:
                trigger_data = RANGE_2_5
            elif num_cmd_sent == 8:
                trigger_data = RANGE_3_4
            elif num_cmd_sent == 9:
                trigger_data = RANGE_3_5
            elif num_cmd_sent == 10:
                trigger_data = RANGE_4_5 
                # restart the count
                num_cmd_sent = 0            
                
            # create HDLC frame for the trigger command
            hdlc_data_w  = _hdlc.hdlcify(trigger_data)     
        
        # send trigger command
        ser.write(hdlc_data_w)
        
        # get the current time in ms every time we send a trigger command
        t           = time.time()
        time_now_ms = int(t*1000)
        
        # clear the flag for HDLC frame reception
        hdlcFrameReceived = False
        
        # trigger the US ranging and get the HDLC frame
        while hdlcFrameReceived == False:
            rx_byte = ser.read(1)        
                
            rx_byte_int = int.from_bytes(rx_byte, "big")
            
            if rx_byte_int != _hdlc.HDLC_FLAG and last_rx_byte_int == _hdlc.HDLC_FLAG and hdlcBusyReceiving == False: 
                in_buf.append(_hdlc.HDLC_FLAG)  
                in_buf.append(rx_byte_int)            
                hdlcBusyReceiving = True                     
            elif rx_byte_int != _hdlc.HDLC_FLAG and hdlcBusyReceiving == True:
                in_buf.append(rx_byte_int)
            elif rx_byte_int == _hdlc.HDLC_FLAG and hdlcBusyReceiving == True:
                in_buf.append(_hdlc.HDLC_FLAG)
                hdlcBusyReceiving = False
                hdlcFrameReceived = True   
                                        
            last_rx_byte_int = rx_byte_int

        # unpack the measurements of each robot

        dehdlc_data       = _hdlc.dehdlcify(in_buf)          
        for i in range(0, len(dehdlc_data), 5):
            robot_ids.append(dehdlc_data[i])
            range_measurements_unpack = dehdlc_data[i+1:i+5]             
            range_us          = int.from_bytes(range_measurements_unpack, byteorder="little")    
            range_cm          = range_us/58        
            robot_measurements.append(range_cm)
            timestamp.append(time_now_ms)

                
        #print(f"Measurement {line_count} done")

        # Increment the line count, and stop the loop
        # once we have 100 lines
        line_count += 1
        in_buf      = []

    line_count = 1

    #Filtering ( Z-score method )
    filtered_robot_measurements_1 = []
    filtered_robot_measurements_2 = []
    threshold = 1

    elements_indice_pair = robot_measurements[::2]
    elements_indice_impair = robot_measurements[1::2]

    mean1 = np.mean(elements_indice_pair)
    std1 = np.std(elements_indice_pair)
    mean2 = np.mean(elements_indice_impair)
    std2 = np.std(elements_indice_impair)

    for measurement in elements_indice_pair:
        z_score = (measurement - mean1)/std1
        if np.abs(z_score) < threshold:
            filtered_robot_measurements_1.append(measurement)
    for measurement in elements_indice_impair:
        z_score = (measurement - mean2)/std2
        if np.abs(z_score) < threshold:
            filtered_robot_measurements_2.append(measurement)
    
    #Average on the filtered measurements
    filtered_robot_measurements_1 = sum(filtered_robot_measurements_1)/len(filtered_robot_measurements_1)
    filtered_robot_measurements_2 = sum(filtered_robot_measurements_2)/len(filtered_robot_measurements_2)

    #Calculations
    DURATION_1 = 2*filtered_robot_measurements_1/0.017 
    DURATION_2 = 2*filtered_robot_measurements_2/0.017
    print(f"Duration 1 is {DURATION_1} us")
    print(f"Duration 2 is {DURATION_2} us")
    print(f"Difference des durées is {abs(DURATION_2-DURATION_1)} us")

    #Example for 2 robots with a distance of 101.6 cm
    AIR_SPEED = (1.016/2)*1000000*(abs((1/DURATION_1)-(1/DURATION_2)))     
    print(f"Air speed is {AIR_SPEED} m/s")

    AIR_SPEED_FORMULA = 2.7229*AIR_SPEED - 0.812
    print(f"Air speed formula is {AIR_SPEED_FORMULA} m/s")
