import subprocess
import os,time
import logging
import threading
import RPi.GPIO as GPIO
import threading
import requests
import json
import serial
#import pickle
import logging
import json

os.chdir("/nishaan/")


logging.basicConfig(filename='nishaanapp.log', level=logging.INFO, format='%(levelname)s %(asctime)s %(process)s %(pathname)s %(lineno)d  - %(message)s ')

nishaanConfig = {}
uartSel = None
sel0 = None
sel1 = None
gsmPower = None
gpsPower = None
gsmReset = None
gpsReset = None
lullaby  = None

Tbtmon = None
Thcitool_lescan = None
foundDevice = {}
sendData = {}
epochTime = None
scanGPS = False
scanGSM = False
simRegistered = False


####################
# This module reads the config file and sets all the required 
# parameters for the scan.
####################
def loadAppConfig():
    try:
        with open('nishaanconfig.json') as json_data_file:
            global nishaanConfig
            nishaanConfig = json.load(json_data_file)
	logging.info("Configuration loaded successfully")
    except IOError as e:
        print e
        logging.critical(e)
    except ValueError as e:
        print e
        logging.critical(e)

####################
# This module does the  
# parameters for the scan.
####################
def gpioSetup():
	global uartSel, sel0, sel1, gsmPower, gpsPower, gsmReset, gpsReset, lullaby
	try:

		# GPIO Pin config
		#uartSel  = nishaanConfig['gpioconf']['uartsel'] #26
		sel0  = nishaanConfig['gpioconf']['sel0'] #10
		sel1  = nishaanConfig['gpioconf']['sel1'] #13
		gsmPower = nishaanConfig['gpioconf']['gsmpower'] #11
		gpsPower = nishaanConfig['gpioconf']['gpspower'] #20
		gsmReset = nishaanConfig['gpioconf']['gsmreset'] #5
		gpsReset = nishaanConfig['gpioconf']['gpsreset'] #21
		lullaby  = nishaanConfig['gpioconf']['lullaby'] #22

		# GPIO Direction config

		GPIO.setmode(GPIO.BCM)

		#GPIO.setup(uartSel , GPIO.OUT)  # UART Select
		GPIO.setup(sel0 , GPIO.OUT)  # UART Select
		GPIO.setup(sel1 , GPIO.OUT)  # UART Select

		GPIO.setup(gsmPower, GPIO.OUT)  # GSM POWER
		GPIO.setup(gpsPower, GPIO.OUT)  # GPS POWER

		GPIO.setup(gsmReset, GPIO.OUT)  # GSM RESET
		GPIO.setup(gpsReset, GPIO.OUT)  # GPS RESET

		GPIO.setup(lullaby , GPIO.OUT)  # lullaby

		GPIO.output(gsmPower, True) # initialize GSM GPIO 
		GPIO.output(gpsPower, True) # initialize GPS GPIO
		logging.info("GPIO setup Done")
	except Exception as e:
		logging.critical("Exception occurred during GPIO setup:"+str(e.message)+", "+str(e.args))

def gpioClear():
	try:
		#GPIO.output(uartSel,False) # True to Select MCU communication on MUX
		GPIO.output(sel0,False) 
		GPIO.output(sel1,False) 
		time.sleep(1)
		ser = serial.Serial("/dev/ttyS0",9600,timeout=1)

		ser.write("sleep:")    #cell lac and cid
		off = ser.readline()
		off = off.replace('\r','')
		off = off.replace('\n',' ')
		print off
		ser.close()

		GPIO.output(gsmPower, True)
		GPIO.cleanup()
		print "End of script"
		logging.info("GPIO clear succeeded")
	except Exception as e:
		logging.critical("GPIO clear failed:"+str(e.message)+", "+str(e.args))

###############################
#---------------------------
#| sel0 | sel1 | Component |
#------+------+------------|
#|  0   |  0   |   ----    |
#------+------+------------|
#|  0   |  1   |   DHT     |
#------+------+------------|
#|  1   |  0   |   GSM     |
#------+------+------------|
#|  1   |  1   |   GPS     |
#-----------------------_---
##############################
def selectDHT():
    #GPIO.output(uartSel,False) # True to Select MCU communication on MUX
    GPIO.output(sel0,False) 
    GPIO.output(sel1,True) 
    time.sleep(2)

def selectGSM():
    #GPIO.output(uartSel,True) # True to Select GSM on MUX as per new h/w changes
    GPIO.output(sel0,True) 
    GPIO.output(sel1,False) 
    time.sleep(1)

def selectGPS():
    #GPIO.output(uartSel,True) # True to Select GPS on MUX
    GPIO.output(sel0,True) 
    GPIO.output(sel1,True) 
    time.sleep(1)

def devID():
	cmd = "cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2"
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	serial = None
	while True:
		line = p.stdout.readline()
		line = line.strip()
		return  line

def CMD(cmd):
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	result = []
	while True:
		line = p.stdout.readline()
		result.append(line.strip())
		if line == '' and p.poll() != None:
			break
	#print "CMD RESULT: ",result
	return result

def btmon():
    global foundDevice
    print "======================================== S T A R T I N G - btmon "
    # notify("STARTING_BTMON")
    cmd = "btmon" 
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,preexec_fn=os.setsid)
    # print os.getpgid(p.pid)

    addr = None
    rssi = None
    advance = False
    foundDevice = {}
    while True:
        line = p.stdout.readline()
        line = line.strip()
        #print line
        if 'HCI Event' in line:
            while True:
                line = p.stdout.readline()
                line = line.strip()
        	#print line
                if "Address:" in line:
                    addr = line.split()[1]
		    #print addr
                if "RSSI" in line:
                    rssi = line.split()[1]
		    #print rssi
                    break
                if "HCI Event" in line:
                    advance = True

            foundDevice[addr] = rssi
    
        if line == '' and p.poll() != None:
            break

def hcitool_lescan():
    print "======================================== S T A R T I N G - hcitool lescan "
    # notify("STARTING_BTMON")
    cmd = "hcitool lescan --duplicates" 
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,preexec_fn=os.setsid)
    # print os.getpgid(p.pid)
    while True:
        line = p.stdout.readline()
        line = line.strip()
        # print line
        if line == '' and p.poll() != None:
            break

def BLEScan(scanFor = 10,noOFScans = 1):
    logging.info('=======================BLE START====================================')
    global bleResult
    global foundDevice
    scannedData = {}
    for x in range(0, noOFScans):
        CMD('hciconfig hci0 down')
        CMD('hciconfig hci0 up')
        Tbtmon = threading.Thread(target=btmon)
        Tbtmon.start()
        # print "Started"
        Thcitool_lescan = threading.Thread(target=hcitool_lescan)
        Thcitool_lescan.start()
        time.sleep(scanFor)
        CMD('killall hcitool')
        CMD('killall btmon')
	#CMD('hciconfig hci0 down')
        scannedData['scan'+str(x+1)] = foundDevice
    print '=======================BLE START===================================='
    logging.info(str(scannedData))
    bleResult = scannedData
    print '=======================BLE END===================================='
    logging.info('=======================BLE END====================================')

def GPSScan(seconds = 15):
    global gpsResult, gpsPower, gpsReset
    logging.info('=======================GPS START====================================')

    selectGPS()
    GPIO.output(gpsPower, False) # turn on GPS
    # Reset GPS, High Pulse 010

    #GPIO.output(gpsReset, False) 
    #GPIO.output(gpsReset, True) 
    #GPIO.output(gpsReset, False) 

    ser = serial.Serial("/dev/serial0",9600,timeout=1)
    timer = int(round(time.time()))
    scannedData = []
    while True:
        timeNow = int(round(time.time()))
        # print "timeNow ::"+str(timeNow)
        if(timeNow - timer > seconds):
            break
        ser.flush()
        rcv = ser.read(100000)
        sentences = {}
        if 'GPRMC' in rcv:
            gprmc = rcv.split('\r\n')
            for word in gprmc:
                if 'GPRMC' in word:
                    sentences['GPRMC'] = word
                    # data = word.split(',')
                    # if data[2] == 'A':
                    #     pass
                    #     # print word
                if 'GPGGA' in word:
                    sentences['GPGGA'] = word
                    # data = word.split(',')
                    # if data[6] == '1' or '2':
                    #     pass
                        # print word
        if sentences:
            scannedData.append(sentences)
    ser.close()

    GPIO.output(gpsPower, True) # turn off GPS
    print '=======================GPS START===================================='
    logging.info(str(scannedData))
    gpsResult = scannedData
    print '=======================GPS END===================================='
    logging.info('=======================GPS END====================================')


def GSMScan(noOFScans = 1):
    global gsmResult, gsmPower, gsmReset
    global simRegistered
    logging.info('=======================GSM START====================================')
    selectGSM()	

    
    # CMD('poff rnet')
    ser = serial.Serial("/dev/ttyS0",9600,timeout=1)
    #ser.flush()
    #ser.close()
    #ser.open()
    scannedData = {}
    for x in range(0, noOFScans):
        scan = []
        ser.write("AT"+"\r\n")     #modem ok
        AT = ser.read(200)
        #print AT
        AT = AT.replace('\r','')
        AT = AT.replace('\n',' ')
        scan.append(AT.strip())
        #print AT

        ser.write("AT+CSMINS?"+"\r\n")   #lac, cid, mnc, mcc, rxlev and more
        CSMINS = ser.read(200)
        #print CSMINS
        CSMINS = CSMINS.replace('\r','')
        CSMINS = CSMINS.replace('\n',' ')
        scan.append(CSMINS.strip())
        #print CSMINS
        time.sleep(0.5)

        ser.write("AT+CGREG?"+"\r\n")    #cell lac and cid
        CGREG = ser.read(200)
        #print CGREG
        CGREG = CGREG.replace('\r','')
        CGREG = CGREG.replace('\n',' ')
        # scan.append(CGREG.strip())
        #print CGREG

	retry = 0
	while(retry < 4):
            ser.write("AT+CREG?"+"\r\n")    #cell lac and cid
            CREG = ser.read(200)
            #print CREG
            CREG = CREG.replace('\r','')
            CREG = CREG.replace('\n',' ')
            status = int(CREG.split(',')[1].split(' ')[0])
            if status == 1:
            	simRegistered = True
		break;
	    else:
	    	time.sleep(2)
		retry += 1
		logging.info("SIM registration retry:"+str(retry))
	
        scan.append(CREG.strip())
        #print CREG

        ser.write("AT+CENG=3"+"\r\n")   #lac, cid, mnc, mcc, rxlev and more
        CENG = ser.read(200)
        #print CENG
        CENG = CENG.replace('\r','')
        CENG = CENG.replace('\n',' ')
        scan.append(CENG.strip())
        #print CENG
        time.sleep(0.5)
        scannedData['scan'+str(x+1)] = scan

        ser.write("AT+COPS=?"+"\r\n")   #lac, cid, mnc, mcc, rxlev and more
        ser.flush()
        _COPS = ''
        while True:
            COPS = ser.read(10000)
            print COPS
            if COPS:
                COPS = COPS.replace('\r','')
                COPS = COPS.replace('\n',' ')
                _COPS = _COPS + COPS
                print COPS
                if 'OK' in COPS :
                    scan.append(_COPS.strip())
                    break

        time.sleep(0.5)

        ser.write("AT+CSQ"+"\r\n")   #lac, cid, mnc, mcc, rxlev and more
        CSQ = ser.read(200)
        #print CSQ
        CSQ = CSQ.replace('\r','')
        CSQ = CSQ.replace('\n',' ')
        scan.append(CSQ.strip())
        #print CSQ
        time.sleep(0.5)

        ser.write("AT+CENG?"+"\r\n")
        CENG = ser.read(100000)
        #print CENG
        CENG = CENG.replace('\r','')
        CENG = CENG.replace('\n',' ')
        scan.append(CENG.strip())
        #print CENG
        #print scan
        scannedData['scan'+str(x+1)] = scan
    ser.close()
    ser.close()
    print '=======================GSM START===================================='
    logging.info(str(scannedData))
    gsmResult = scannedData
    print '=======================GSM END===================================='
    logging.info('=======================GSM END====================================')


def DHTScan():
    global dhtResult, sendData
    global epochTime
    scannedData = []
    
    logging.info('=======================DHT START====================================')

    selectDHT()

    ser = serial.Serial("/dev/ttyS0",9600,timeout=1)

    time.sleep(0.5)

    ser.write("time:")     # Get epoch time from MCU
    #time.sleep(0.5)
    epoch = ser.read(200)
    epoch = epoch.replace('\r','')
    epoch = epoch.replace('\n','')
    print epoch
    epochTime = epoch.split(':')[1].strip()
    scannedData.append(epoch.strip())

    time.sleep(0.5)
         
    ser.write("size:")     # Get the byte size to read for DHT values
    size_ = ser.read(200)
    size_ = size_.replace('\r','')
    size_ = size_.replace('\n','')
    print "SIZE:",size_
    size_ = int(size_.split(':')[1].strip())
    
    time.sleep(0.5)
    
    ser.write("dht:")    # Get DHT values
    dht = ser.read(size_)

    print "########################################################"
    print "DHT DdhtA = " 
    dht = dht.replace('\r','')
    dht = dht.replace('\n',' ')
    dht = dht.strip().split(':')[1].split(';')
    dhtResult = dht[:-1]
    print "########################################################"
    scannedData.append(dht)
    print dhtResult
    logging.info(str(dhtResult))
    print "Communication Done!!!!!!!!!!"
    
    time.sleep(0.5)
    
    ser.write("ok:")    # Send acknowledgement
    dhtAck = ser.read(200)
    ser.close()
    dhtAck = dhtAck.replace('\r','')
    dhtAck = dhtAck.replace('\n',' ')
    scannedData.append(dhtAck.strip())
    
    print '=======================DHT END===================================='
    logging.info('=======================DHT END====================================')

def scan():
	
	scanDuration = 0
	noOfScans = 0
	GPIO.output(gsmReset, True)
	GPIO.output(gsmPower, False)

	# Scan for DHT data
	scanDuration = int(nishaanConfig['tasks']['dht']['duration'])
	noOfScans = int(nishaanConfig['tasks']['dht']['scans'])
	if(scanDuration):
		DHTScan()
	else:
		logging.info("DHT scan disabled")
	
	# Scan for GPS devices
	scanDuration = int(nishaanConfig['tasks']['gps']['duration'])
	noOfScans = int(nishaanConfig['tasks']['gps']['scans'])
	if(scanDuration):
		GPSScan(scanDuration)
		if(scanDuration < 10):
			time.sleep(10-scanDuration)
	else:
		logging.info("GPS scan disabled")
		
	# Scan for GSM devices
	scanDuration = int(nishaanConfig['tasks']['gsm']['duration'])
	noOfScans = int(nishaanConfig['tasks']['gsm']['scans'])
	if(scanDuration):
		GSMScan()
		GPIO.output(gsmReset, True)
		time.sleep(0.1)
		GPIO.output(gsmReset, False)
		time.sleep(0.5)
		GPIO.output(gsmReset, True)
	else:
		logging.info("GSM disabled")

	# Scan for BLE devices
	scanDuration = int(nishaanConfig['tasks']['ble']['duration'])
	noOfScans = int(nishaanConfig['tasks']['ble']['scans'])
	if(scanDuration):
		BLEScan(scanDuration, noOfScans)
	else:
		logging.info("BLE scan disabled")
	
	
def checkPPPInterface():
    cmd = 'ifconfig ppp0'
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = False
    while True:
        line = p.stdout.readline()
        line = line.strip()
        if 'ppp' in line:
            result = True
        if line == '' and p.poll() != None:
            break
    print "checkPPPInterface :: "+str(result)    
    logging.info("checkPPPInterface :: "+str(result))
    return result


def checkServerAvailability():
    cmd = "ping -Ippp0 www.google.com -c 4"
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    while True:
        line = p.stdout.readline()
        print line.strip()
        if "packets transmitted" in line:
            quality = line.split(',')
            loss =  [s for s in quality if "loss" in s]
            loss = loss[0].strip()
            percent = loss.strip()

            if percent is not "100%":
                print "======================================== INTERNET PRESENT"
                print "checkServerAvailability :: True"
                logging.info("checkServerAvailability :: True")
                return True

            if percent is "100%":
                print "======================================== NO INTERNET"

        if "unknown host google.com" in line:
            print "======================================== NO INTERNET"

        if "connect: Network is unreachable" in line:
            print "======================================== INTERNET ERROR"

        if line == '' and p.poll() != None:
            break
    print "checkServerAvailability :: False"
    logging.info("checkServerAvailability :: False")
    return False

if __name__ == "__main__":

	try :
		global gsmResult,bleResult,dhtResult,gpsResult

		logging.info("Script Started")
		loadAppConfig()
		gpioSetup()

		bleResult = None
		gsmResult = None
		gpsResult = None
		dhtResult = None
		sendData = {}    

		startTime = int(round(time.time()))

		scan()

		#sendData['devID'] = devID()
    		sendData['devID'] = "00000000ddaba86b"
		sendData['ble'] = bleResult
		sendData['gps'] = gpsResult
		sendData['gsm'] = gsmResult
		sendData['dht'] = dhtResult
		sendData['timeStamp'] = epochTime

		counter = nishaanConfig['cloud']['counter'] + 1
		sendData['wakeCounter'] = counter

		#print nishaanConfig['cloud']['counter'] 

		sendData['totalScanTime'] = int(round(time.time())) - startTime
		print sendData

		with open('/nishaan/data/'+sendData['timeStamp']+'.json','wb') as fp:
			json.dump(sendData, fp)

		os.system("sync")

		nishaanConfig['cloud']['counter'] = counter

		print 'simRegistered :: '+str(simRegistered)

		if simRegistered:
			tppd = int(CMD("ps -ef | grep '[pppd] call rnet' | wc -l")[0])
			if not tppd:
				print "-------Connecting to Internet"
				os.system(" /usr/sbin/pppd call rnet")
				time.sleep(5)
			if checkPPPInterface():
				if checkServerAvailability():
					print " Connected to server "
					data =  os.listdir("/nishaan/data")
					
					for file in data:
						print "===========  "+file+" starts ============="
						with open('/nishaan/data/'+file,'r') as fp:
							data = json.load(fp)
							content = json.dumps(data)

						headers = {'content-type': 'application/json'}
						url = nishaanConfig['cloud']['host']+'?dir='+sendData['devID']
						myResponse = requests.post(url, data=content,headers=headers)

						if myResponse.content == '200':
							os.remove('/nishaan/data/'+file)
							print "Data Sent SuccessFully"
							logging.info(file+": Data Sent SuccessFully")
							#print content
						else:
							print "Error :: ",myResponse.content
							logging.error(str(myResponse.content))
						print "===========  "+file+" ends ============="
	except Exception as e:
		print "Exception occurred: ", e.message, e.args
		logging.error("Exception occurred: "+str(e.message)+", "+str(e.args))
	finally:
		CMD('poff rnet')
		time.sleep(1)

		#Update config file with latest info
		with open('nishaanconfig.json', 'w') as json_data_file:
			json.dump(nishaanConfig, json_data_file)
		os.system("sync")

		print "=*-=*-=*-=*-=*-=*-=*- Done =*-=*-=*-=*-=*-=*-=*-"
		endTime = int(round(time.time()))
		totalTime = endTime - startTime
		print "totalTime :: "+str(totalTime)
		logging.info("total Time :: "+str(totalTime))

		gpioClear()
		logging.info("End of script")
