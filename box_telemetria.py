import paho.mqtt.client as mqtt
import json
import time
import pandas as pd

planilha_VCU = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - VCU.csv',
    header=None, skip_blank_lines=True, comment='/'
)

planilha_BMS = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - BMS.csv',
    header=None, skip_blank_lines=True, comment='/'
)

def on_connect(client, userdata, flags, rc): #conexão com o broker
    print("Conectado ao broker")
    client.subscribe("telemetria")

def on_message(client, userdata, msg): #mensagem do broker
    dados = json.loads(msg.payload) #converte de string para dicionário
    # "Mensagem recebida:{'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}"
    tratamento_mensagem(dados)

def tratamento_mensagem(dados): #dados é um dicionário com as mensagens recebidas
    # filtrar por id
    id = dados['arbitration_id']
    hora = time.ctime(dados['timestamp'])
    data = dados['data']
    id_hexadecimal = f'0x{id:08X}' #volta para hexa para o pandas ler na planilha
    filtro_id_VCU = planilha_VCU[planilha_VCU[1] == id_hexadecimal] # retorna a linha da planilha que tem o id hexadecimal
    filtro_id_BMS = planilha_BMS[planilha_BMS[1] == id_hexadecimal]
    if filtro_id_BMS.empty: #se não encontrar o id na planilha BMS, procura na VCU
        extrai_planilha(id_hexadecimal, data, planilha_VCU)
    else:
        extrai_planilha(id_hexadecimal, data, planilha_BMS)

def associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados):
    # associa o bit da planilha para aquela variável com o bit da mensagem recebida
    if campo_bit.startswith('bit('):
        range_bits = campo_bit.replace('bit(', '').replace(')', '').split('-')
        # 'bit(10-11)' -> bits = ['10', '11']
        bit_ini = int(range_bits[0]) # primeiro bit
        bit_fim = int(range_bits[1]) # último bit
        mensagem = string_bytes_bits_invertidos_concatenados[bit_ini:bit_fim + 1]  # pega os bits da mensagem
    
    elif campo_bit.startswith('byte('):
        bytes_ = campo_bit.replace('byte(', '').replace(')', '').split('-')
        byte_ini = int(bytes_[0]) #primeiro byte
        # 'byte(0-1)' -> bytes = ['0', '1']
        if len(bytes_) == 1:
            # byte(0) -> [0*8 até 0*8 + 8] bits
            mensagem = lista_bytes_bits_invertidos[byte_ini*8:byte_ini*8 + 8]  # pega o byte inteiro, está em bits concatenados
        else:
            byte_fim = int(bytes_[1]) # último byte
            mensagem = string_bytes_bits_invertidos_concatenados[byte_ini*8:byte_fim*8 + 8]

    mensagem_int = int(mensagem, 2)  # converte de binário para inteiro
    print(f"Mensagem '{nome}': bits :{mensagem_int*campo_multiplicador}, descrição: {campo_descrição}")

def extrai_planilha(id_hexadecimal, data, planilha):
    #Achar linha do id na planilha e pegar variável - bit
    idxs = planilha.index[planilha[1] == id_hexadecimal].tolist()
    idx_inicio = idxs[0] + 1 # ir para próxima linha
    idx_fim = idx_inicio # inicializa o índice de fim
    while idx_fim < len(planilha) and pd.notna(planilha.iloc[idx_fim, 1]): #enquanto a linha da coluna 1 não estiver vazia, vá pra próxima
        idx_fim += 1 # incrementa o índice de fim
    
    lista_bytes_bits_invertidos = []
    for byte in data: #para cada byte na mensagem
        bits_str = bin(byte)[2:].zfill(8)  # ex: '00001000'
        bits_str = bits_str[::-1]          # inverte: '00010000'
        lista_bytes_bits_invertidos.append(bits_str)
    string_bytes_bits_invertidos_concatenados = ''.join(lista_bytes_bits_invertidos)  # junta todos os bits (LSB primeiro)

    # Extrair os dados relevantes da planilha
    for i in range(idx_inicio, idx_fim):
        nome = planilha.iloc[i, 1]      # coluna 1: nome da variável analisada
        campo_bit = planilha.iloc[i, 2] # coluna 3: especificação (bit(x-y), byte(x), etc)
        campo_multiplicador = planilha.iloc[i, 5]  # coluna 5: multiplicador
        campo_descrição = planilha.iloc[i, 9]   # coluna 9: descrição
        associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados)

client = mqtt.Client()
client.connect("172.20.10.2", 1883)  #IP do broker, proprio notebook para se escutar
client.subscribe("telemetria")
client.on_message = on_message
client.loop_forever()

