import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    print("Conectado ao broker")
    client.subscribe("telemetria")

def on_message(client, userdata, msg):
    dados = json.loads(msg.payload) #converte de string para dicionário
    # "Mensagem recebida:{'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}"
    #print("Mensagem recebida:", dados)
    tratamento_mensagem(dados)

def tratamento_mensagem(dados): #dados é um dicionário com as mensagens recebidas
    # filtrar por id
    id = dados['arbitration_id']
    hora = time.ctime(dados['timestamp'])
    
    # Device status of the MOBILE 0
    if id == 0x18FF00EA:
        lista_bytes = []
        for i in range(len(dados['data'])): # para cada byte na data
            lista_bytes.append(dados['data'][i])    #adiciona os bytes na lista
            #print(f"Byte {i+1}: {dados['data'][i]}") #teste 
        print(lista_bytes)
        lista_bytes = []
        lista_bits = []
        for byte in dados['data']: # para cada byte
            bits_invertido = bin(byte)[2:].zfill(8) #00001000 que é uma str
            bits = bits_invertido[::-1]
            lista_bits.append(bits)
            bits_invertido = ''
            bits = ''

        print(f"ID: {id}, Hora: {hora}, Bytes: {dados['data']}") # printa aqui para controle do que chega
        DeviceSatate_M0 = lista_bits[0][:2]
        ErrorLamp_M0 = lista_bits[0][2:4]
        DeviceNunmber_M0 = lista_bits[0][4:8]
        Clamp15_Status_M0 = lista_bits[1][:2]
        PreCharge_M0 = lista_bits[1][2:4]
        ErrorCode_M0 = ''.join(lista_bits[2:4])
        act_DCBusVoltage_M0 = lista_bits[4]
        act_DCBusPower_M0 = ''.join(lista_bits[5:7])
        act_DeviceTemperature_M0 = lista_bits[7]
        print(
            f"DeviceSatate_M0: {DeviceSatate_M0}, ErrorLamp_M0: {ErrorLamp_M0}, "
            f"DeviceNunmber_M0: {DeviceNunmber_M0}, Clamp15_Status_M0: {Clamp15_Status_M0}, "
            f"PreCharge_M0: {PreCharge_M0}, ErrorCode_M0: {ErrorCode_M0}, act_DCBusVoltage_M0: {int(act_DCBusVoltage_M0)*0.25}, "
            f"act_DCBusPower_M0: {int(act_DCBusPower_M0)*200}, act_DeviceTemperature_M0: {act_DeviceTemperature_M0}"
        )
    '''
    lista_bytes = []
    for i in range(len(dados['data'])): # para cada byte na data
        lista_bytes.append(dados['data'][i])    #adiciona os bytes na lista
        #print(f"Byte {i+1}: {dados['data'][i]}") #teste 
    print(lista_bytes)
    lista_bytes = []
    print(f"ID: {id}, Hora: {hora}, Bytes: {byte_1}, {byte_2}, {byte_3}, {byte_4}, {byte_5}, {byte_6}, {byte_7}, {byte_8}")
    '''
    
client = mqtt.Client()
client.connect("172.20.10.2", 1883)  #IP do broker, proprio notebook para se escutar
client.subscribe("telemetria")
client.on_message = on_message
client.loop_forever()

