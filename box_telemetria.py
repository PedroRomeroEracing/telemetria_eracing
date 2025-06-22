import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    print("Conectado ao broker")
    client.subscribe("telemetria")

def on_message(client, userdata, msg):
    dados = json.loads(msg.payload) #converte de string para dicionário
    # "Mensagem recebida:{'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}"
    print("Mensagem recebida:", dados)
def tratamento_mensagem(dados):
    # filtrar por id
    id = dados['arbitration_id']
    '''
    if id == 0x123:
    '''
    hora = time.ctime(dados['timestamp'])
    byte_1 = dados['data'][0]
    byte_2 = dados['data'][1]
    byte_3 = dados['data'][2]
    byte_4 = dados['data'][3]
    byte_5 = dados['data'][4]
    byte_6 = dados['data'][5]
    byte_7 = dados['data'][6]
    byte_8 = dados['data'][7]

client = mqtt.Client()
client.connect("172.20.10.2", 1883)  #IP do broker, proprio notebook para se escutar
client.subscribe("telemetria")
client.on_message = on_message
client.loop_forever()

