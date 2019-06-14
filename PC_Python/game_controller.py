import serial
import argparse
import time
import logging
import pyvjoy  # Windows apenas
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume


class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1, 'B': 2, 'Analog': 3, 'PAxisX': 4, 'NAxisX': 5, 'PAxisY': 6, 'NAxisY': 7, 'Right': 8, 'Up': 9, 'Left': 10, 'Down': 11, 'C' : 12, 'D' : 13 , 'E' : 14, 'F' : 15, 'G' : 16}


devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
print('oi')
vol = 5.0

class SerialControllerInterface:

    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'
        self.vol = 5.0
        

   

    def update(self):
        # Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Receivelllld INCOMING: {}".format(self.incoming))

        data = self.ser.read(15)
        logging.debug("Received DATA: {}".format(data))
        data_format = (bytearray(data)).decode('ascii')
        #print(data_format)
        um, dois, A, B, cinco, seis, sete, oito, nove, dez, onze, doze, treze, X, Y = data_format

        #vol = 5.0
        
        # Joystick

        if nove == '1':
            logging.info("Sending L")
            self.j.set_button(self.mapping.button['Left'], 1)
            print("Left")
        elif nove == '0':
            self.j.set_button(self.mapping.button['Left'], 0)

        if dez == '1':
            logging.info("Sending D")
            self.j.set_button(self.mapping.button['Down'], 1)
            print("Down")
        elif dez == '0':
            self.j.set_button(self.mapping.button['Down'], 0)

        if onze == '1':
            logging.info("Sending R")
            self.j.set_button(self.mapping.button['Right'], 1)
            print("Right")
        elif onze == '0':
            self.j.set_button(self.mapping.button['Right'], 0)

        if doze == '1':
            logging.info("Sending U")
            self.j.set_button(self.mapping.button['Up'], 1)
            print("Up")
        elif doze == '0':
            self.j.set_button(self.mapping.button['Up'], 0)

        if X == 'P':
            logging.info("Sending X")
            self.j.set_button(self.mapping.button['PAxisX'], 1)
            self.j.set_button(self.mapping.button['NAxisX'], 0)

            print("Analogico X positivo")
        elif X == 'N':
            logging.info("Sending X")
            self.j.set_button(self.mapping.button['NAxisX'], 1)
            self.j.set_button(self.mapping.button['PAxisX'], 0)

            print("Analogico X negativo")
        elif X == '0':
            self.j.set_button(self.mapping.button['PAxisX'], 0)
            self.j.set_button(self.mapping.button['NAxisX'], 0)

        if Y == 'P':
            logging.info("Sending Y")
            self.j.set_button(self.mapping.button['PAxisY'], 1)
            self.j.set_button(self.mapping.button['NAxisY'], 0)

            print("Analogico Y positivo")
        elif Y == 'N':
            logging.info("Sending Y")
            self.j.set_button(self.mapping.button['NAxisY'], 1)
            self.j.set_button(self.mapping.button['PAxisY'], 0)

            print("Analogico Y negativo")
        elif Y == '0':
            self.j.set_button(self.mapping.button['PAxisY'], 0)
            self.j.set_button(self.mapping.button['NAxisY'], 0)

        if um == '1':
            logging.info("Sending A")
            self.j.set_button(self.mapping.button['Analog'], 1)
            print("Analog")
        elif um == '0':
            self.j.set_button(self.mapping.button['Analog'], 0)

        if dois == '1':
            logging.info("Sending A")
            self.j.set_button(self.mapping.button['C'], 1)
            print("C")
        elif dois == '0':
            self.j.set_button(self.mapping.button['C'], 0)

        if A == '1':
            logging.info("Sending A")
            self.j.set_button(self.mapping.button['A'], 1)
            print("A")
        elif A == '0':
            self.j.set_button(self.mapping.button['A'], 0)

        if B == '1':
            logging.info("Sending B")
            self.j.set_button(self.mapping.button['B'], 1)
            print("B")
        elif B == '0':
            self.j.set_button(self.mapping.button['B'], 0)
        
        if cinco == '1':
            logging.info("Sending A")
            self.j.set_button(self.mapping.button['D'], 1)
            print("D")
        elif cinco == '0':
            self.j.set_button(self.mapping.button['D'], 0)

        if seis == '1':
            logging.info("Sending E")
            self.j.set_button(self.mapping.button['E'], 1)
            print("E")
            print("Aumenta")
            if(self.vol < 10):
                self.vol +=1.5
            volume.SetMasterVolumeLevel(-self.vol, None)
           
        elif seis == '0':
            self.j.set_button(self.mapping.button['E'], 0)

        if sete == '1':
            logging.info("Sending F")
            self.j.set_button(self.mapping.button['F'], 1)
            print("Diminui")
            if(self.vol >=1.5):
                self.vol -=1.5
            volume.SetMasterVolumeLevel(-self.vol, None)
            
        elif sete == '0':
            self.j.set_button(self.mapping.button['F'], 0)

        if oito == '1':
            logging.info("Sending F")
            self.j.set_button(self.mapping.button['G'], 1)
            print("Macro")
            
        elif oito == '0':
            self.j.set_button(self.mapping.button['G'], 0)
        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(
        args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(
            port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
