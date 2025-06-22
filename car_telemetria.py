import can
import paho.mqtt.client as mqtt
import json

broker_ip = "192.168.0.100"  # IP do box, enviar pra notebook
broker_port = 1883
mqtt_topic = "telemetria"
can_interface = "can0"
bitrate = 500000

mqtt_client = mqtt.Client()
mqtt_client.connect(broker_ip, broker_port)
print(f"Conectado ao broker MQTT em {broker_ip}:{broker_port}")

print(f"leitura da interface {can_interface}")
bus = can.interface.Bus(channel=can_interface, bustype='socketcan')

try:
    while True:
        msg = bus.recv()
        #para filtrar por ID
        '''
        if msg.arbitration_id == 0x18FF1515:
            dados = {
            ...
        '''
        dados = {
                "arbitration_id": msg.arbitration_id,
                "data": list(msg.data),
                "timestamp": msg.timestamp
            }

        mqtt_client.publish(mqtt_topic, json.dumps(dados)) #manda como string JSON
        print("Enviado:", dados)

except KeyboardInterrupt:
    print("\nParou")
