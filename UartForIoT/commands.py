
def isOk(val):
    if (val==None):
        return None

    if (len(val)==0):
        return None

    if (val[0]==0x06) and (val[len(val)-1]==0x04):
        return val[1:-1]

    return None

def readTempSensor1(ser):
    ser.write(b'\x4e\x01')
    return isOk(ser.read(200))

def readTempSensor2(ser):
    ser.write(b'\x4e\x02')
    return isOk(ser.read(200))

def readPins(ser):
    ser.write(b'\x70')
    x=ser.read(200)
    if (x!=None):
        if (len(x)!=0):
            if (x[0]!=0x06):
                return None
    return int(x[1])

def readPin1(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x04)==0x04
    return None

def readPin2(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x08)==0x08
    return None

def readPin3(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x10)==0x10
    return None

def readPin4(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x20)==0x20
    return None

def readPin5(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x40)==0x40
    return None

def readPin6(ser):
    x=readPins(ser)
    if (x!=None):
        return (x&0x80)==0x80
    return None

def tempSensor1(ser):
    ser.write(b'\x4e\x01')
    return isOk(ser.read(200))

def tempSensor2(ser):
    ser.write(b'\x4e\x02')
    return isOk(ser.read(200))

def batteryVoltage1(ser):
    ser.write(b'\x4e\x04')
    return isOk(ser.read(200))

def batteryVoltage2(ser):
    ser.write(b'\x4e\x08')
    return isOk(ser.read(200))
