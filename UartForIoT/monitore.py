import time
import json
from commands import \
    readPin1, readPin2, readPin3, readPin4,\
    readPin5, readPin6,\
    readTempSensor1, readTempSensor2,\
    batteryVoltage1, batteryVoltage2

oldPin1=None
oldPin2=None
oldPin3=None
oldPin4=None
oldPin5=None
oldPin6=None
oldTempSensor1=None
oldTempSensor2=None
oldBattery1=None
oldBattery2=None

def monitore(toTcp, fromSerial):
    global oldPin1
    global oldPin2
    global oldPin3
    global oldPin4
    global oldPin5
    global oldPin6
    global oldTempSensor1
    global oldTempSensor2
    global oldBattery1
    global oldBattery2
    print ("Entering monitoring events ...")

    try:
        while True:
            data={}

            m=readTempSensor1(fromSerial)
            if (m!=None):
                if (oldTempSensor1!=m):
                    oldTempSensor1=m
                    data['tempSensor1']=oldTempSensor1.decode('utf-8')

            m=readTempSensor2(fromSerial)
            if (m!=None):
                if (oldTempSensor2!=m):
                    oldTempSensor2=m
                    data['tempSensor2']=oldTempSensor2.decode('utf-8')

            m=readPin1(fromSerial)
            if (m!=None):
                if (oldPin1!=m):
                    oldPin1=m
                    if (oldPin1):
                        data['pin1']='1'
                    else:
                        data['pin1']='0'

            m=readPin2(fromSerial)
            if (m!=None):
                if (oldPin2!=m):
                    oldPin2=m
                    if (oldPin2):
                        data['pin2']='1'
                    else:
                        data['pin2']='0'

            m=readPin3(fromSerial)
            if (m!=None):
                if (oldPin3!=m):
                    oldPin3=m
                    if (oldPin3):
                        data['pin3']='1'
                    else:
                        data['pin3']='0'

            m=readPin4(fromSerial)
            if (m!=None):
                if (oldPin4!=m):
                    oldPin4=m
                    if (oldPin4):
                        data['pin4']='1'
                    else:
                        data['pin4']='0'
    
            m=readPin5(fromSerial)
            if (m!=None):
                if (oldPin5!=m):
                    oldPin5=m
                    if (oldPin5):
                        data['pin5']='1'
                    else:
                        data['pin5']='0'

            m=readPin6(fromSerial)
            if (m!=None):
                if (oldPin6!=m):
                    oldPin6=m
                    if (oldPin6):
                        data['pin6']='1'
                    else:
                        data['pin6']='0'

            m=batteryVoltage1(fromSerial)
            if (m!=None):
                if (oldBattery1!=m):
                    oldBattery1=m
                    data['batteryVoltage1']=oldBattery1.decode('utf-8')

            m=batteryVoltage2(fromSerial)
            if (m!=None):
                if (oldBattery2!=m):
                    oldBattery2=m
                    data['batteryVoltage2']=oldBattery2.decode('utf-8')
            
            if (len(data)):
                tx=json.dumps(data).encode('utf-8')
                toTcp.send(bytes(tx))
                print(tx)
                time.sleep(1.5)
            else:
                time.sleep(.5)
    except:
        pass
