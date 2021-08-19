import serial
from totcp import ToTcp
from monitore import monitore
#sum aug 15 22:55:41 -03 2021 
def main():
    """Main file application
    """

    print(serial.__file__)
    ser=serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )
    address='192.168.1.78'
    #address='localhost'
    port=10000

    sender=ToTcp(address, port)

    monitore(sender, ser)

    sender.close()
    ser.close()

if __name__ == "__main__":
    main()
