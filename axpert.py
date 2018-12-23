#! /usr/bin/python

# Axpert Inverter control script

# Read values from inverter, sends values to emonCMS,
# read electric low or high tarif from emonCMS and setting charger and mode to hold batteries fully charged
# controls grid charging current to meet circuit braker maximum alloweble grid current(power)
# calculation of CRC is done by XMODEM mode, but in firmware is wierd mistake in POP02 command, so exception of calculation is done in serial_command(command) function
# real PL2303 = big trouble in my setup, cheap chinese converter some times disconnecting, workaround is at the end of serial_command(command) function
# differenc between SBU(POP02) and Solar First (POP01): in state POP01 inverter works only if PV_voltage <> 0 !!! SBU mode works during night


import urllib2
import serial, time, sys, string
import sqlite3
import json
import urllib
import httplib
import datetime
import calendar
import os
import re
import crcmod
#import datetime as dt
from binascii import unhexlify



# Domain you want to post to: localhost would be an emoncms installation on your own laptop
# this could be changed to emoncms.org to post to emoncms.org or your own server
server = "localhost"


# Location of emoncms in your server, the standard setup is to place it in a folder called emoncms
# To post to emoncms.org change this to blank: ""
emoncmspath = "emoncms"

# Write apikey of emoncms account
apikey = "XXXXXXXXXXXXXXXXXXXXXXXXXX"

# Node id youd like the emontx to appear as
nodeid = 0

#Axpert Commands and examples
#QPI            # Device protocol ID inquiry
#QID            # The device serial number inquiry
#QVFW           # Main CPU Firmware version inquiry
#QVFW2          # Another CPU Firmware version inquiry
#QFLAG          # Device flag status inquiry
#QPIGS          # Device general status parameters inquiry
                # GridVoltage, GridFrequency, OutputVoltage, OutputFrequency, OutputApparentPower, OutputActivePower, OutputLoadPercent, BusVoltage, BatteryVoltage, BatteryChargingCurrent, BatteryCapacity, InverterHeatSinkTemperature, PV-InputCurrentForBattery, PV-InputVoltage, BatteryVoltageFromSCC, BatteryDischargeCurrent, DeviceStatus,
#QMOD           # Device mode inquiry P: PowerOnMode, S: StandbyMode, L: LineMode, B: BatteryMode, F: FaultMode, H: PowerSavingMode
#QPIWS          # Device warning status inquiry: Reserved, InverterFault, BusOver, BusUnder, BusSoftFail, LineFail, OPVShort, InverterVoltageTooLow, InverterVoltageTooHIGH, OverTemperature, FanLocked, BatteryVoltageHigh, BatteryLowAlarm, Reserved, ButteryUnderShutdown, Reserved, OverLoad, EEPROMFault, InverterSoftFail, SelfTestFail, OPDCVoltageOver, BatOpen, CurrentSensorFail, BatteryShort, PowerLimit, PVVoltageHigh, MPPTOverloadFault, MPPTOverloadWarning, BatteryTooLowToCharge, Reserved, Reserved
#QDI            # The default setting value information
#QMCHGCR        # Enquiry selectable value about max charging current
#QMUCHGCR       # Enquiry selectable value about max utility charging current
#QBOOT          # Enquiry DSP has bootstrap or not
#QOPM           # Enquiry output mode
#QPIRI          # Device rating information inquiry - nefunguje
#QPGS0          # Parallel information inquiry
                # TheParallelNumber, SerialNumber, WorkMode, FaultCode, GridVoltage, GridFrequency, OutputVoltage, OutputFrequency, OutputAparentPower, OutputActivePower, LoadPercentage, BatteryVoltage, BatteryChargingCurrent, BatteryCapacity, PV-InputVoltage, TotalChargingCurrent, Total-AC-OutputApparentPower, Total-AC-OutputActivePower, Total-AC-OutputPercentage, InverterStatus, OutputMode, ChargerSourcePriority, MaxChargeCurrent, MaxChargerRange, Max-AC-ChargerCurrent, PV-InputCurrentForBattery, BatteryDischargeCurrent
#PEXXX          # Setting some status enable
#PDXXX          # Setting some status disable
#PF             # Setting control parameter to default value
#FXX            # Setting device output rating frequency
#POP02          # set to SBU
#POP01          # set to Solar First
#POP00          # Set to UTILITY
#PBCVXX_X       # Set battery re-charge voltage
#PBDVXX_X       # Set battery re-discharge voltage
#PCP00          # Setting device charger priority: Utility First
#PCP01          # Setting device charger priority: Solar First
#PCP02          # Setting device charger priority: Solar and Utility
#PGRXX          # Setting device grid working range
#PBTXX          # Setting battery type
#PSDVXX_X       # Setting battery cut-off voltage
#PCVVXX_X       # Setting battery C.V. charging voltage
#PBFTXX_X       # Setting battery float charging voltage
#PPVOCKCX       # Setting PV OK condition
#PSPBX          # Setting solar power balance
#MCHGC0XX       # Setting max charging Current          M XX
#MUCHGC002      # Setting utility max charging current  0 02
#MUCHGC010      # Setting utility max charging current  0 10
#MUCHGC020      # Setting utility max charging current  0 20
#MUCHGC030      # Setting utility max charging current  0 30
#POPMMX         # Set output mode       M 0:single, 1: parrallel, 2: PH1, 3: PH2, 4: PH3

#notworking
#PPCP000        # Setting parallel device charger priority: UtilityFirst - notworking
#PPCP001        # Setting parallel device charger priority: SolarFirst - notworking
#PPCP002        # Setting parallel device charger priority: OnlySolarCharging - notworking

# most of the code is tripped down from the original and extra commands added to upload the maximum amount of data to emoncms.

ser = serial.Serial()
ser.port = "/dev/ttyUSB1"
ser.baudrate = 2400
ser.bytesize = serial.EIGHTBITS     #number of bits per bytes
ser.parity = serial.PARITY_NONE     #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE  #number of stop bits
ser.xonxoff = False                 #disable software flow control
ser.timeout = 1                     #non-block read
ser.rtscts = False                  #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False                  #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2                #timeout for write

try:
    ser.open()

except Exception, e:
    print "error open serial port: " + str(e)
    exit()




def get_data():
    #collect data from axpert inverter
    mode = 0
    if ser.isOpen():
        try:
            data = "{"
            response = serial_command("QPGS0")
            if "NAKss" in response:
                time.sleep(2)
                return ""
            else:
                response.rstrip()
                nums = response.split(' ', 99)
## Get Inverter Watts from data
		data += "Test:" + nums[15]
		data += ",INVERTER_Total_VA:" + nums[16]
		data += ",INVERTER_Total_W:" + nums[17]
		data += ",INVERTER_Total_LOAD:" + nums[18]
		data += ",INVERTER_MAX_CHARGE_RANGE:" + nums[23]
		data += ",INVERTER_MAX_CHARGE:" + nums[22]
		data += ",INVERTER_CHARGER_SOURCE:" + nums[21]
## Log Grid data if inverter is switched to grid mode		
		if nums[2] == "L":
		    data += ",Gridmode:1"
		    data += ",MAINS_WATT:" + nums[9]
                else:
                    data += ",Gridmode:0"
		    data += ",MAINS_WATT:0"
## Check if Panels is Active or not 
		if nums[2] == "B":
                    data += ",Solarmode:1"
                else:
                    data += ",Solarmode:0"



## end of custom code _ Paul 

            response = serial_command("QPIGS")
            if "NAKss" in response:
                time.sleep(0.5)
                data = ""
                return ""
            response.rstrip()
            nums = response.split(' ', 99)
	    #Calculations to be sent to Emon
	    #set time
            w = time.strftime("%H.%M")
	    #Calculations
	    powerf = float(nums[5])/float(nums[4])
	    x = float(nums[8])*float(nums[12])
	    y = float(nums[9])+float(nums[15])
	    z = float(nums[5])/float(nums[2])
 ## end of custom code _ Paul            
            
	    # Data that comes from the Inverter:		
	    data += ",MAINS_V:" + nums[0]
            data += ",MAINS_FREQ:" + nums[1]
            data += ",INVERTER_VOLTS:" + nums[2]
            data += ",INVERTER_FREQ:" + nums[3]
            data += ",INVERTER_VA:" + nums[4]
            data += ",INVERTER_WATTS:" + nums[5]
            data += ",LOAD_PERCENTAGE:" + nums[6]
            data += ",DC_BUS_VOLTS:" + nums[7]
            data += ",BATTERY_VOLT:" + nums[8]
            data += ",BATT_CHARGE_AMPS:" + nums[9]
            data += ",BATT_SOC_STATE:" + nums[10]
            data += ",HEATSYNC_TEMP:" + nums[11]
            data += ",PANEL_AMPS:" + nums[12]
            data += ",PANEL_VOLTAGE:" + nums[13]
            data += ",SCC_DC_VOLTS:" + nums[14]
            data += ",BATT_DISCHARGE_AMPS:" + nums[15]
            #data += ",STATUS:" + nums[16]
	    #Calculated values sent to emon
			data += ",24_HOUR_TIME:" +  str(w)
			data += ",PANEL_WATTS_OLD:" +  str(x)
			data += ",BATTERY_CURRENT:" +  str(y)
			data += ",INVERTER_CURRENT:" +  str(z)
			data += ",INVERTER_POWER_FACTOR:" + str(powerf)
		#Blank Values to Emon for custom inputs
			data += ",USER_1:0"
			data += ",USER_2:0"
			data += ",USER_3:0"
			data += ",USER_4:0"
			data += ",USER_5:0" 
## end of custom code _ Paul         
		#Split Day and Night Use from inverter watts and solar watts
	    if x > 0 :
	    	data += ",INV_Night_Use:0"
	    	data += ",INV_Day_Use:" + nums[5]
            else:
            	data += ",INV_Night_Use:" + nums[5]
	   	data += ",INV_Day_Use:0"
## end of custom code _ Paul 
## New Command for custom Q1 Command _ Paul
            response = serial_command("Q1")
            if "NAKss" in response:
                time.sleep(2)
                return ""
            else:
                response.rstrip()
                nums = response.split(' ', 99)
## Get Data
                data += ",V_Axpert_Firmware:" + nums[0]
                data += ",V_BUS_Firmware:" + nums[1]
                data += ",V_PWM_Temp:" + nums[5]
                data += ",V_INVERTER_Temp:" + nums[6]
                data += ",V_INVERTER_Batt_Temp:" + nums[7]
                data += ",V_INVERTER_Transformer_Temp:" + nums[8]
                data += ",PANEL_WATTS:" + nums[13]
                data += ",V_INVERTER_Charge_Status:" + str(re.findall("\d+", nums[16]))

## New Command for QPIRI Command _ Paul
            response = serial_command("QPIRI")
            if "NAKss" in response:
                time.sleep(2)
                return ""
            else:
                response.rstrip()
                nums = response.split(' ', 99)
## Get Data
            data += ",xb_Grid_Rating_Volatge:" + nums[0]
            data += ",xc_Grid_Rating_Current:" + nums[1]
            data += ",xd_Inverter_Rating_Volatge:" + nums[2]
            data += ",xe_Inverter_Rating_Freq:" + nums[3]
            data += ",xf_Inverter_Rating_Current:" + nums[4]
            data += ",xh_Inverter_Rating_VA:" + nums[5]
            data += ",xi_Inverter_Rating_W:" + nums[6]
			data += ",xj_Inverter_Batt_Rate_V:" + nums[7]
			data += ",xk_Inverter_Batt_Recharge_V:" + nums[8]
			data += ",xl_Inverter_Batt_Under_V:" + nums[9]
			data += ",xm_Inverter_Batt_Bulk_V:" + nums[10]
			data += ",xn_Inverter_Batt_Flout_V:" + nums[11]
	        data += ",xo_Inverter_Batt_Type:" + nums[12]
	        data += ",xp_Inverter_Grid_Charge:" + nums[13]
	        data += ",xq_Inverter_Max_Charge:" + nums[14]
	        data += ",xo_Inverter_Change_Mode:" + nums[15]
	        data += ",xp_Inverter_Output_Source:" + nums[16]
	        data += ",xs_Inverter_Charge_Source:" + nums[17]
	        data += ",xt_Inverter_Type:" + nums[19]
	        data += ",xv_Inverter_Battery_re_Discharge_v:" + nums[22]
	        data += ",xx_Inverter_PV_Power_Balance:" + nums[23]
# check for battery type Code did not work and still need to be fine tuned 
		#if nums[13] == "AGM":
		 #   data += ",x_Inverter_AGM:1"
		  #  data += ",x_Inverter_Flooded:0"
		# elif:
               # if nums[13] == "Flooded":
		#    data += ",x_Inverter_Flooded:1"
	          #  data += ",x_Inverter_AGM:0"




        except Exception, e:
            print "error parsing inverter data...: " + str(e)
            return ""

    else:
        ser.close()
        data = ""
        print "cannot use serial port ..."
        return ""
    return data



def send_data(data):
    # Send data to emoncms server
    try:
        conn = httplib.HTTPConnection(server)
        conn.request("GET", "/"+emoncmspath+"/input/post.json?&node="+str(nodeid)+"&json="+data+"&apikey="+apikey)
        response = conn.getresponse()
        conn.close()

    except Exception as e:
        print "error sending to emoncms...: " + str(e)
        return [0]
    return [1]

# Run Serial Command
def serial_command(command):
    try:
        ser.flushInput()            #flush input buffer, discarding all its contents
        ser.flushOutput()           #flush output buffer, aborting current output and discard all that is in buffer
        xmodem_crc_func = crcmod.predefined.mkCrcFun('xmodem')
        if command == "POP02":          # wierd mistake in Axpert firmware - correct CRC is: 0xE2 0x0A
            command_crc = '\x50\x4f\x50\x30\x32\xe2\x0b\x0d'
        else:
            command_crc = command + unhexlify(hex(xmodem_crc_func(command)).replace('0x','',1)) + '\x0d'
        ser.write(command_crc)
        response = ser.readline()
        print command
        print response
        return response

    except Exception, e:
        print "error reading inverter...: " + str(e)
        ser.close()
        time.sleep(20)  # Problem with some USB-Serial adapters, they are some times disconnecting, 20 second helps to reconnect at same ttySxx
        data = ""
        ser.open()
        time.sleep(0.5)
        return [0]

    return [0]

def main():
    while True:
        time.sleep(0.1)
        data = get_data()
##        charge_current = set_charge_current ()
        if not data == "":
            send = send_data(data)

if __name__ == '__main__':
    main()

