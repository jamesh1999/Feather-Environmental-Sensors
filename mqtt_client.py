#!/usr/bin/python
import serial, json, argparse
import paho.mqtt.client as mqtt



MQTT_IP = "localhost"
MQTT_PORT = 1883
SERIAL_PORT = "/dev/ttyACM1"
INTERFACE_ID = 0



class InterfaceClient(object):
    
    LABELS = ["temp", "humidity", "pressure", "thermocouple"]

    def __init__(self, mqttIP, mqttPort, serialPort, interfaceID):

        self.client = mqtt.Client()
        self.client.connect(mqttIP, mqttPort)

        self.serial = serial.Serial(port = serialPort, baudrate = 115200)

        #Register interface as active & set up will
        self.interfaceID = interfaceID
        self.client.will_set("interface/{0}".format(self.interfaceID), "false")
        self.client.publish("interface/{0}".format(self.interfaceID), "true")

    def __del__(self):
        self.client.publish("interface/{0}".format(self.interfaceID), "false")
        self.client.loop()
        self.client.disconnect()

    def update(self):
        """Processes and sends one packet or terminates at end of buffer"""

        bufferedPacket = []
        packet = False

        while True:
            data = self.serial.read()
            
            if not packet:
                #Encountered a packet
                if data == '#':
                    packet = True
                #End of buffer
                elif data == "":
                    return
            else:
                #Split lines
                if data == '\n':
                    bufferedPacket.append("")
                #Ignore CR
                elif data != '\r':
                    bufferedPacket[-1] += data

            if len(bufferedPacket) == 6:

                obj = {}
                for keyValue in zip(self.LABELS, bufferedPacket[1:5]):
                    if keyValue[1][4:] == "nan": continue;
                    try:
                        obj[keyValue[0]] = float(keyValue[1][4:])
                    except ValueError:
                        pass

                self.client.publish("sensor/{0}".format(bufferedPacket[0]), json.dumps(obj))

                #Return after packet publish
                return;

    def loop(self):
        self.update()
        self.client.loop()
        


def parseArgs():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--mqtt_ip', default = MQTT_IP, help = "Set the IP address of the MQTT server")
    parser.add_argument('--mqtt_port', default = MQTT_PORT, help = "Set the port of the MQTT server")
    parser.add_argument('--serial_port', default = SERIAL_PORT, help = "Set the serial port to read from")
    parser.add_argument('--interface_id', default = INTERFACE_ID, help = "Set the ID for this interface to use")

    return parser.parse_args()



if __name__ == "__main__":
    args = parseArgs()
    
    interface = InterfaceClient(args.mqtt_ip, args.mqtt_port, args.serial_port, args.interface_id)

    try:
        while True:
            interface.loop()
    except KeyboardInterrupt, serial.serialutil.SerialException:
        pass

