import paho.mqtt.client as mqtt
import serial
import time

MQTT_SERVER = "mqtt.ohstem.vn"
MQTT_PORT = "1883"
MQTT_USERNAME = "lxa_dashboard_iot"
MQTT_PASSWORD = ""
MQTT_TOPIC_SUB = ["/feeds/V3", "/feeds/V4", "/feeds/V5", "/feeds/V6"]  


class MQTTgateway:
    def __init__(self, host, port, username, password, topic) -> None:
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        self.data_server = {}
        self.data_mcu = {}
        self.ser = None
        self.mess = ""
        self.auto = "0"
        self.temp = ""
        self.limit_temp = ""
        self.time_on = ""
        self.check_send = ""
        self.time_now = None
        self.mode = 0
        self.set_time = ""
        self.check_mode = ""

    def start(self):    
        def controlLedTemp(client):
            if self.check_mode == "" and self.limit_temp != "":
                print("Setup led mode 1")
                self.check_mode = "mode1"
            if self.mode == 1:
                if float(self.temp) >= float(self.limit_temp):
                    if self.check_send != "sent_on_limit":
                        client.publish("lxa_dashboard_iot/feeds/V3", "1")
                        self.check_send = "sent_on_limit"
                        print("Turn on led by temperature limit mode")
                else:
                    if self.check_send == "sent_on_limit":
                        client.publish("lxa_dashboard_iot/feeds/V3", "0")
                        self.check_send = "sent_off_limit"
                        self.mode = 0
                        self.limit_temp = ""
                        self.check_mode == ""
                        print("Turn off led by temperature limit mode")

        def controlLedTime(client):
            if self.check_mode == "" and self.time_on != "":
                print("Setup led mode 2")
                self.check_mode = "mode2"
            if self.mode == 2:
                if (time.time() - self.time_now) <= float(self.time_on) * 60:
                    if self.check_send != "sent_on_time":
                        client.publish("lxa_dashboard_iot/feeds/V3", "1")
                        self.check_send = "sent_on_time"
                        print("Turn on led by time mode")
                else:
                    if self.check_send == "sent_on_time":
                        client.publish("lxa_dashboard_iot/feeds/V3", "0")
                        self.check_send = "sent_off_time"
                        self.mode = 0
                        self.time_on = ""
                        self.check_mode == ""
                        print("Turn off led by time mode")
        
        def controlLedMix(client):
            if self.check_mode == "" and (self.time_on != "" and self.limit_temp != ""):
                print("Setup led mode 3")
                self.check_mode = "mode3"
            if self.mode == 3:
                if float(self.temp) >= float(self.limit_temp):
                    if self.set_time != "setted":
                        self.time_now = time.time()
                        self.set_time = "setted"
                    if (time.time() - self.time_now) <= float(self.time_on) * 60:
                        if self.check_send != "sent_on_mix":
                            client.publish("lxa_dashboard_iot/feeds/V3", "1")
                            self.check_send = "sent_on_mix"
                            print("Turn on led by mix mode")
                    else:
                        if self.check_send == "sent_on_mix":
                            client.publish("lxa_dashboard_iot/feeds/V3", "0")
                            self.check_send != "sent_off_mix"
                            client.publish("lxa_dashboard_iot/feeds/V4", "0")
                            self.mode = 0
                            self.time_on = ""
                            self.limit_temp = ""
                            self.check_mode == ""
                            print("Turn off led by mix mode")    
                else:
                    if self.check_send == "sent_on_mix":
                        client.publish("lxa_dashboard_iot/feeds/V3", "0")
                        self.check_send != "sent_off_mix"
                        client.publish("lxa_dashboard_iot/feeds/V4", "0")
                        self.mode = 0
                        self.check_mode == ""
                        print("Turn off led by mix mode") 

        def controlMCU(client, data, topic):
            if topic == "lxa_dashboard_iot/feeds/V3":
                if data == "1":
                    writeData("!LED:1#")
                else:
                    writeData("!LED:0#")
            if topic == "lxa_dashboard_iot/feeds/V4":
                if data == "1":
                    self.auto = "1"
                    print("Turn on auto mode")
                else:
                    self.limit_temp = ""
                    self.time_on = ""
                    self.check_send = ""
                    self.time_now = None
                    self.mode = 0
                    self.set_time = ""
                    self.check_mode = ""
                    print("Turn off auto mode")
            if topic == "lxa_dashboard_iot/feeds/V5":
                splitData = data.split("°C")
                self.limit_temp = ""
                self.limit_temp = splitData[0]
                self.mode = 0
                self.mode += 1
                print("Setup temperature limit: " + self.limit_temp)
            if topic == "lxa_dashboard_iot/feeds/V6":
                splitData = data.split("phút")
                self.time_on = ""
                self.time_on = splitData[0]
                self.time_now = time.time()
                if self.mode != 1: 
                    self.mode = 0
                self.mode += 2
                print("Setup time on led: " + self.time_on)

        def mqtt_connected(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to server successfully!!")
                for topic_name in self.topic:
                    client.subscribe(self.username + topic_name)
            else:
                print("Connection failed with error code " + str(rc))

        def mqtt_subscribed(client, userdata, mid, granted_qos):
            print("Subscribed to topic: " + self.username + str(mid))

        def mqtt_recv_message(client, userdata, message):
            print("Topic: " + message.topic + " sent data: " + message.payload.decode("utf-8"))
            self.data_server = message.payload.decode("utf-8")
            self.topic = message.topic
            controlMCU(client, self.data_server, self.topic)           
             
        def processData(client):
            self.data_mcu = self.data_mcu.replace("!", "")
            self.data_mcu = self.data_mcu.replace("#", "")
            splitData = self.data_mcu.split(":")
            print(splitData)
            if splitData[0] == "TEMP" and splitData[2] == "HUMI":
                self.temp = splitData[1]
                client.publish(self.username + "/feeds/V1", splitData[1])
                client.publish(self.username + "/feeds/V2", splitData[3])

        def readSerial(client):
            bytesToRead = self.ser.inWaiting()  
            if bytesToRead > 0:
                self.mess = self.mess + self.ser.read(bytesToRead).decode("UTF-8")  
                while "#" in self.mess and "!" in self.mess:
                    start = self.mess.find("!")
                    end = self.mess.find("#")
                    self.data_mcu = self.mess[start:end + 1]
                    processData(client)
                    if end == len(self.mess):
                        self.mess = ""
                    else:
                        self.mess = self.mess[end + 1:]

        def writeData(data):
            self.ser.write(str(data).encode())  

        self.ser = serial.Serial(port="COM11", baudrate=115200)
        mqttClient = mqtt.Client()
        mqttClient.username_pw_set(self.username, self.password)
        mqttClient.on_connect = mqtt_connected
        mqttClient.on_subscribe = mqtt_subscribed
        mqttClient.on_message = mqtt_recv_message

        try:
            mqttClient.connect(self.host, int(self.port), 60)
            mqttClient.loop_start()
            while True:
                readSerial(mqttClient)
                if self.auto == "1":
                    controlLedTemp(mqttClient) 
                    controlLedTime(mqttClient)
                    controlLedMix(mqttClient)

        except KeyboardInterrupt:
            print("Interrupted")
        except Exception as e:
            print("An error occurred:", str(e))


client = MQTTgateway(MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, MQTT_TOPIC_SUB)
client.start()
