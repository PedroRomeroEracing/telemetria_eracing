import paho.mqtt.client as mqtt
import json
import time
import pandas as pd

planilha_VCU = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - VCU.csv',
    header=None, skip_blank_lines=True, comment='/'
)

dados = {'data': [21, 5, 1, 0, 0, 1, 1, 0]}

def extrai_planilha(id_hexadecimal, dados, planilha):
    idxs = planilha.index[planilha[1] == id_hexadecimal].tolist() # procura o id hexadecimal na planilha
    if not idxs: # se não encontrar o id na planilha
        print(f"ID {id_hexadecimal} não encontrado na tabela.")
        return
    idx_inicio = idxs[0] + 1 # pega a linha seguinte ao id
    idx_fim = idx_inicio # inicializa o índice de fim
    while idx_fim < len(planilha) and pd.notna(planilha.iloc[idx_fim, 1]): # enquanto a coluna 1 (nome do campo) não estiver vazia ou NaN
        idx_fim += 1 # incrementa o índice de fim
    sinais = planilha.iloc[idx_inicio:idx_fim] # pega as linhas entre o início e o fim

    resultado = {}
    for _, row in sinais.iterrows(): # para cada linha da tabela
        nome = str(row[1]).strip() # nome do campo
        campo = row[2] # campo da tabela
        mult = float(row[6]) if not pd.isna(row[6]) else 1 # multiplicador, se não for NaN, senão é 1
        desc = row[9] if len(row) > 9 else "" # descrição, se existir
        valor_bruto = extrai_valor(dados['data'], campo) # extrai o valor bruto dos dados
        if valor_bruto is not None: # se o valor bruto não for None
            valor = valor_bruto * mult # multiplica pelo multiplicador
            resultado[nome] = {
                "valor": valor,
                "descricao": desc
            }
    print(f"Resultado para ID {id_hexadecimal}: {resultado}")

id_hexadecimal = f'0x{id:08X}' #volta para hexa para o pandas ler na planilha
filtro_id_VCU = planilha_VCU[planilha_VCU[1] == id_hexadecimal] # retorna a linha da planilha que tem o id hexadecimal
#filtro_id_BMS = planilha_BMS[planilha_BMS[1] == id_hexadecimal]
if filtro_id_VCU.empty: #se não encontrar o id na planilha VCU
    extrai_planilha(id_hexadecimal, dados['data'], planilha_VCU)

'''
dados = {'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}
print(dados['data'][0])
print(len(dados['data']))
lista_bytes = []
for i in range(len(dados['data'])): # para cada byte na data
    lista_bytes.append(dados['data'][i])    #adiciona os bytes na lista
    #print(f"Byte {i+1}: {dados['data'][i]}") #teste 
print(lista_bytes)
lista_bytes = []
# transformação em 8 bits
for byte in dados['data']:
    print(bin(byte)[2:].zfill(8))
    print(type((bin(byte)[2:].zfill(8))))


    lista_bytes = []
    for i in range(len(dados['data'])): # para cada byte na data
        lista_bytes.append(dados['data'][i])    #adiciona os bytes na lista
        #print(f"Byte {i+1}: {dados['data'][i]}") #teste 
    print(lista_bytes)
    lista_bytes = []
    print(f"ID: {id}, Hora: {hora}, Bytes: {byte_1}, {byte_2}, {byte_3}, {byte_4}, {byte_5}, {byte_6}, {byte_7}, {byte_8}")

    
    
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
