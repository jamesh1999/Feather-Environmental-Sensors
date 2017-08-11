import json, argparse, datetime
import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient



MQTT_IP = "localhost"
MQTT_PORT = 1883
INFLUX_IP = "192.168.0.1"
INFLUX_PORT = 8086
INFLUX_DATABASE = "sensor_mesh"
INFLUX_WRITE_SIZE = 10



class Logger(object):
    
    def __init__(self, mqttIP, mqttPort, influxIP, influxPort, influxDatabase):
        self.__db = InfluxDBClient(influxIP, influxPort, database = influxDatabase)
        self.__db.create_database(influxDatabase)
        
        self.__buffer = []
        
        self.__mqttIP = mqttIP
        self.__mqttPort = mqttPort

        self.__client = mqtt.Client()
        self.__client.on_connect = self.onConnect
        self.__client.on_message = self.onMessage
        self.__client.on_disconnect = self.onDisconnect

        self.connect()

    def connect(self):
        self.__client.connect(self.__mqttIP, self.__mqttPort)

    def loop(self):
        self.__client.loop()

    def writeData(self, datapoint):
        datapoint["time"] = datetime.datetime.utcnow()
        self.__buffer.append(datapoint)

        if len(self.__buffer) >= INFLUX_WRITE_SIZE:
            self.__db.write_points(self.__buffer)
            self.__buffer = []

    def onConnect(self, client, userdata, flags, rc):
        pass

    def onMessage(self, client, userdata, msg):
        pass

    def onDisconnect(self, client, userdata, rc):
        print mqtt.error_string(rc)
        print "Disconnected"
        self.connect()

class SensorLogger(Logger):

    def onConnect(self, client, userdata, flags, rc):
        client.subscribe("sensor/#")

    def onMessage(self, client, userdata, msg):
        obj = json.loads(msg.payload)
        if obj == {}: return
        data = {
                "measurement" : msg.topic.split('/')[1],
                 "fields" : obj
               }
        self.writeData(data)

class InterfaceLogger(Logger):

    def onConnect(self, client, userdata, flags, rc):
        client.subscribe("interface/#")

    def onMessage(self, client, userdata, msg):
        value = msg.payload == "true"
        data = {
                "measurement" : msg.topic.split('/')[1],
                "fields" : {"enabled" : value}
               }
        self.writeData(data)



def parseArgs():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--mqtt_ip', default = MQTT_IP, help = "Set the IP address of the MQTT server")
    parser.add_argument('--mqtt_port', default = MQTT_PORT, help = "Set the port of the MQTT server")
    parser.add_argument('--influx_ip', default = INFLUX_IP, help = "Set the IP addresss of the influx database")
    parser.add_argument('--influx_port', default = INFLUX_PORT, help = "Set the port of the influx database")
    parser.add_argument('--influx_database', default = INFLUX_DATABASE, help = "Set the database name of the influx database")

    return parser.parse_args()



if __name__ == "__main__":
    args = parseArgs()

    sensor = SensorLogger(args.mqtt_ip, args.mqtt_port, args.influx_ip, args.influx_port, args.influx_database)
    interface = InterfaceLogger(args.mqtt_ip, args.mqtt_port, args.influx_ip, args.influx_port, args.influx_database)

    try:
        while True:
            sensor.loop()
            interface.loop()
    except KeyboardInterrupt:
        pass
