import paho.mqtt.client as mqtt
import json
import time
import pandas as pd
import csv
from datetime import datetime
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

#dicionário global para armazenar os valores por id e variável
dicionario_ids = {}
pasta_dados = r'/home/pedroromero/telemetria_eracing/componentes_csv_linux'
nome_log = f'log{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv' #nome do arquivo de log de acordo com a data e hora
caminho_log = os.path.join(pasta_dados, nome_log) #caminho completo do arquivo de log

planilha_VCU = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - VCU.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_BMS = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - BMS.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_ACD = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - ACD.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_PAINEL = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - PAINEL.csv',
    header=None, skip_blank_lines=True, comment='/'
)   
planilha_PT = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - PT.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_LV_BMS = pd.read_csv(
    r'/home/pedroromero/telemetria_eracing/componentes_csv_linux/CAN Description 2025 - LV_BMS.csv',
    header=None, skip_blank_lines=True, comment='/'
)

def on_connect(client, userdata, flags, rc): #conexão com o broker
    logger = userdata
    if logger:
        logger.info("Conectado ao broker")
    client.subscribe("telemetria")

def on_message(client, userdata, msg): #mensagem do broker
    logger = userdata
    dados = json.loads(msg.payload) #converte de string para dicionário
    # "Mensagem recebida:{'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}"
    tratamento_mensagem(dados,client,userdata,msg,logger)
    
def tratamento_mensagem(dados,client,userdata,msg,logger): #dados é um dicionário com as mensagens recebidas
    # filtrar por id
        id = dados['arbitration_id']
        hora = time.ctime(dados['timestamp'])
        data = dados['data']
        id_hexadecimal = f'0x{id:08X}' #volta para hexa para o pandas ler na planilha
        
        logger.info(f"ID recebido: {id_hexadecimal}")

        #mensagens INS
        if id == 123: # 123 = [0,1,2,3]
            # não faz sentido mas funciona, canking manda invertido
            aceleracao_x = (data[0] << 8) | data[1]  #Byte 1 LSB, desloca byte 0 8 para esquerda
            aceleracao_y = (data[2] << 8) | data[3]  #Byte 3 LSB
            (aceleracao_x,aceleracao_y)
            logger.info(f"Aceleração X: {aceleracao_x}, Aceleração Y: {aceleracao_y}")
           
            #0x7B é 123 em decimal
            if '0x7B' not in dicionario_ids:
                dicionario_ids['0x7B'] = {}

            if 'aceleracao_x' not in dicionario_ids['0x7B']:
                dicionario_ids['0x7B']['aceleracao_x'] = []
            if 'aceleracao_y' not in dicionario_ids['0x7B']:
                dicionario_ids['0x7B']['aceleracao_y'] = []
            dicionario_ids['0x7B']['aceleracao_x'].append(aceleracao_x)
            dicionario_ids['0x7B']['aceleracao_y'].append(aceleracao_y)
            logger.info(str(dicionario_ids))

        filtro_VCU = planilha_VCU[planilha_VCU[1] == id_hexadecimal]
        filtro_BMS = planilha_BMS[planilha_BMS[1] == id_hexadecimal]
        filtro_ACD = planilha_ACD[planilha_ACD[1] == id_hexadecimal]
        filtro_LV_BMS = planilha_LV_BMS[planilha_LV_BMS[1] == id_hexadecimal]
        filtro_PAINEL = planilha_PAINEL[planilha_PAINEL[1] == id_hexadecimal]
        filtro_PT = planilha_PT[planilha_PT[1] == id_hexadecimal]

        if not filtro_VCU.empty:
            logger.info("Pegando VCU")
            extrai_planilha(id_hexadecimal, data, planilha_VCU, 'VCU', logger)
        elif not filtro_BMS.empty:
            logger.info("Pegando BMS")
            extrai_planilha(id_hexadecimal, data, planilha_BMS, 'BMS', logger)
        elif not filtro_ACD.empty:
            logger.info("Pegando ACD")
            extrai_planilha(id_hexadecimal, data, planilha_ACD, 'ACD', logger)
        elif not filtro_PAINEL.empty:
            logger.info("Pegando PAINEL")
            extrai_planilha(id_hexadecimal, data, planilha_PAINEL, 'PAINEL', logger)
        elif not filtro_LV_BMS.empty:
            logger.info("Pegando LV_BMS")
            extrai_planilha(id_hexadecimal, data, planilha_LV_BMS, 'LV_BMS', logger)
        elif not filtro_PT.empty:
            logger.info("Pegando PT")
            extrai_planilha(id_hexadecimal, data, planilha_PT, 'PT', logger)
        else:
            logger.warn(f"ID {id_hexadecimal} não encontrado em nenhuma planilha")

def extrai_planilha(id_hexadecimal, data, planilha, nome_planilha, logger):
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
        campo_multiplicador = planilha.iloc[i, 6]  # coluna 6: multiplicador
        campo_descrição = planilha.iloc[i, 9]   # coluna 9: descrição
        associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados, nome_planilha, id_hexadecimal, logger)

def associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados, nome_planilha, id_hexadecimal,logger):
    # associa o bit da planilha para aquela variável com o bit da mensagem recebida
    print(nome)
    logger.info(f"Variável analisada: {nome}")
    if campo_bit.startswith('bit('):
        range_bits = campo_bit.replace('bit(', '').replace(')', '').split('-')
        if len(range_bits) == 1:
             mensagem = string_bytes_bits_invertidos_concatenados[int(range_bits[0])]  # pega o bit único
        else:
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
            mensagem = string_bytes_bits_invertidos_concatenados[byte_ini*8:byte_ini*8 + 8]  # pega o byte inteiro, está em bits concatenados
    
        else:
            byte_fim = int(bytes_[1]) # último byte
            mensagem = string_bytes_bits_invertidos_concatenados[byte_ini*8:byte_fim*8 + 8]
    
    string_bytes_bits_invertidos_concatenados = ''
    lista_bytes_bits_invertidos = []
    if mensagem == '':
        mensagem_int_binário = 0
    else:
        mensagem_invertida = mensagem[::-1]#desinverte e transforma em inteiro, mensagem de fato que chega
        mensagem_int_binário = int((mensagem_invertida), 2)  # converte de binário para inteiro
    print(f"Mensagem '{nome}': bits :{mensagem_int_binário/float(campo_multiplicador)}, descrição: {campo_descrição}")
    valor_log = mensagem_int_binário / float(campo_multiplicador)  # valor que vai no log salvo
    logger.info(f"{nome} = {valor_log} ({campo_multiplicador})")
    salvar_csv(datetime.now().strftime('%Y%m%d_%H%M%S'), nome, valor_log) 
    salvar_dicionário(nome,valor_log,id_hexadecimal)

def salvar_csv(hora, nome, valor_log):
    variável_arquivo = os.path.isfile(caminho_log) #variável do arquivo aberto
    
    with open(caminho_log, mode='a', newline='', encoding='utf-8') as arquivo_csv:
        escritor = csv.writer(arquivo_csv)
        if not variável_arquivo:
            escritor.writerow(['Tempo', 'Nome', 'Valor'])  # cabeçalho do CSV
        escritor.writerow([hora, nome, valor_log])  # escreve a linha com os dados

def salvar_dicionário(nome, valor_log, id_hexadecimal):
    # lista dos ids nas planilhas de temperaturas, velocidades e tensões (foi na mão)
    lista_IDs_analisados = [
        '0x19B50100','0x19B50101','0x19B50102','0x19B50104',
        '0x19B50105','0x19B50106','0x19B50107','0x19B50108',
        '0x19B50109','0x19B5010A','0x19B5010B','0x19B50800',
        '0x19B50801','0x19B50802','0x19B50803','0x19B50007',
        '0x19B70100','0x19B70800','0x19B70007','0x18FF00EA',
        '0x18FF00F7','0x18FF01EA','0x18FF02EA','0x18FF01F7',
        '0x18FF02F7','0x18FF0EF7','0x18FF0DEA',''
    ]
    global dicionario_ids # mandar um dicionário que dentro dele tem outros filtrados com chave o ID e o valor a lista de valores_log atualizados
    if id_hexadecimal in lista_IDs_analisados:
        if id_hexadecimal not in dicionario_ids:
            dicionario_ids[id_hexadecimal] = {}
        if nome not in dicionario_ids[id_hexadecimal]:
            dicionario_ids[id_hexadecimal][nome] = []
        dicionario_ids[id_hexadecimal][nome].append(valor_log)
    return dicionario_ids

class DicionarioPublisher(Node):
    def __init__(self):
        super().__init__('telemetria_publisher')
        self.publisher_ = self.create_publisher(String, 'telemetria', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publica a cada 1 segundo

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(dicionario_ids)
        self.publisher_.publish(msg)

def start_ros_publisher():
    rclpy.init()
    node = DicionarioPublisher()
    return node

def main():

    rclpy.init()
    node = DicionarioPublisher()

    client = mqtt.Client(userdata=node.get_logger())
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("172.20.10.2", 1883)
    client.subscribe("telemetria")
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    client.loop_forever()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
