import serial
from totcp import ToTcp
from monitore import monitore

def main():
    """Função principal da aplicação.
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

    sender=ToTcp('localhost', 10000)

    monitore(sender, ser)

    sender.close()
    ser.close()

if __name__ == "__main__":
    main()