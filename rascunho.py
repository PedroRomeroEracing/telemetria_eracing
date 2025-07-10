import paho.mqtt.client as mqtt
import json
import time
import pandas as pd
import csv
from datetime import datetime
import os
import threading
import queue
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import atexit

#fila global no início
data_queue = queue.Queue()

pasta_dados = r'c:\Users\galag\OneDrive\DV\telemetria_eracing\dados_csv' #pasta onde vai salvar os logs
nome_log = f'log{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv' #nome do arquivo de log de acordo com a data e hora
caminho_log = os.path.join(pasta_dados, nome_log) #caminho completo do arquivo de log

#cria o arquivo CSV que vai salvar os logs sem fechar e abrir ele
variavel_arquivo_existe = os.path.isfile(caminho_log) # verifica se o arquivo já existe
arquivo_csv = open(caminho_log, mode='a', newline='', encoding='utf-8') #abre o arquivo uma única vez em modo append
escritor_csv = csv.writer(arquivo_csv)
#escreve a primeira linha caso o arquivo seja novo
if not variavel_arquivo_existe:
    escritor_csv.writerow(['Tempo', 'Nome', 'Valor'])
atexit.register(arquivo_csv.close) #garante que o arquivo será fechado

planilha_VCU = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - VCU.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_BMS = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - BMS.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_ACD = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - ACD.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_PAINEL = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - PAINEL.csv',
    header=None, skip_blank_lines=True, comment='/'
)   
planilha_PT = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - PT.csv',
    header=None, skip_blank_lines=True, comment='/'
)
planilha_LV_BMS = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - LV_BMS.csv',
    header=None, skip_blank_lines=True, comment='/'
)

def on_connect(client, userdata, flags, rc): #conexão com o broker
    print("Conectado ao broker")
    client.subscribe("telemetria")

def on_message(client, userdata, msg): #mensagem do broker
    dados = json.loads(msg.payload) #converte de string para dicionário
    # "Mensagem recebida:{'arbitration_id': 0, 'data': [1, 2, 3, 4, 5, 6, 7, 8], 'timestamp': 1234567890.123456}"
    tratamento_mensagem(dados,client,userdata,msg)

def tratamento_mensagem(dados,client,userdata,msg): #dados é um dicionário com as mensagens recebidas
    # filtrar por id
        id = dados['arbitration_id']
        hora = time.ctime(dados['timestamp'])
        data = dados['data']
        id_hexadecimal = f'0x{id:08X}' #volta para hexa para o pandas ler na planilha
        filtro_id_VCU = planilha_VCU[planilha_VCU[1] == id_hexadecimal] # retorna a linha da planilha que tem o id hexadecimal
        #lógica para ver em qual planilha está o id
        filtro_VCU = planilha_VCU[planilha_VCU[1] == id_hexadecimal]
        filtro_BMS = planilha_BMS[planilha_BMS[1] == id_hexadecimal]
        filtro_ACD = planilha_ACD[planilha_ACD[1] == id_hexadecimal]
        filtro_LV_BMS = planilha_LV_BMS[planilha_LV_BMS[1] == id_hexadecimal]
        filtro_PAINEL = planilha_PAINEL[planilha_PAINEL[1] == id_hexadecimal]
        filtro_PT = planilha_PT[planilha_PT[1] == id_hexadecimal]

        if filtro_VCU.empty == False: #se encontrar o id na planilha VCU
                print('pegando VCU')
                extrai_planilha(id_hexadecimal, data, planilha_VCU)
        elif filtro_BMS.empty == False: #se encontrar o id na planilha BMS
                print('pegando BMS')
                extrai_planilha(id_hexadecimal, data, planilha_BMS)
        elif filtro_ACD.empty == False: #se encontrar o id na planilha ACD
                print('pegando ACD')
                extrai_planilha(id_hexadecimal, data, planilha_ACD)
        elif filtro_PAINEL.empty == False: #se encontrar o id na planilha PAINEL
                print('pegando PAINEL')
                extrai_planilha(id_hexadecimal, data, planilha_PAINEL)
        elif filtro_LV_BMS.empty == False: #se encontrar o id na planilha LV_BMS
                print('pegando LV_BMS')
                extrai_planilha(id_hexadecimal, data, planilha_LV_BMS)
        elif filtro_PT.empty == False: #só sobrou a planilha PT(Pt não pode, 13 é proibido)
                print('pegando PT')
                extrai_planilha(id_hexadecimal, data, planilha_PT)
        else:
                print(f'ID {id_hexadecimal} não encontrado em nenhuma planilha')

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
        campo_multiplicador = planilha.iloc[i, 6]  # coluna 6: multiplicador
        campo_descrição = planilha.iloc[i, 9]   # coluna 9: descrição
        associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados)

def associação_mensagem_planilha(nome, campo_bit, campo_multiplicador, campo_descrição, planilha, data, lista_bytes_bits_invertidos, string_bytes_bits_invertidos_concatenados):
    # associa o bit da planilha para aquela variável com o bit da mensagem recebida
    print(nome)
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
    print(f"Mensagem '{nome}': bits :{mensagem_int_binário*float(campo_multiplicador)}, descrição: {campo_descrição}")
    valor_log = mensagem_int_binário * float(campo_multiplicador)  # valor que vai no log salvo
    hora_atual = datetime.now()
    data_queue.put((hora_atual, nome, valor_log))
    # salvar os dados na planilha CSV
    salvar_csv(datetime.now().strftime('%Y%m%d_%H%M%S'), nome, valor_log) 

def iniciar_tkinter_grafico():
    root = tk.Tk() #função que gerencia a janela do gráfico
    root.title("Telemetria")

    #variáveis para o gráfico
    variaveis_disponiveis = ['act_DCBusPower M0', 'act_DCBusPower M13',
                             'setp_DcLinkVoltage',
    ]

    variavel_selecionada = tk.StringVar()
    variavel_selecionada.set(variaveis_disponiveis[0])  # valor inicial

    #Dropdown para seleção da variável
    dropdown = tk.OptionMenu(root, variavel_selecionada, *variaveis_disponiveis)
    dropdown.pack()

    fig, ax = plt.subplots(figsize=(8, 4)) #cria a figura e os eixos do gráfico em polegadas (8,4)
    #configurações padrão matplotlib + tinker
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack()
    #listas vazias para armazenar os dados do gráfico
    tempos = []
    valores = []
    # função para atualizar internamente o gráfico
    def update_plot():
        while not data_queue.empty():
            hora, nome, valor = data_queue.get()
            if nome == variavel_selecionada.get():  #escolhemos a variável selecionada no dropdown
                tempos.append(hora)
                valores.append(valor)
                if len(tempos) > 100:  #limitar tamanho do gráfico até os últimos 100 pontos
                    tempos.pop(0)
                    valores.pop(0)
        #padrão, limpa o gráfico e plota os novos dados
        ax.clear()
        ax.plot(tempos, valores, marker='o')
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
        ax.xaxis.set_major_locator(mdates.SecondLocator(interval=5)) #traços no eixo x a cada 5 segundos
        ax.set_title(f"{variavel_selecionada.get()}")
        ax.set_xlabel("Tempo")
        ax.set_ylabel("Valor")
        fig.autofmt_xdate()
        canvas.draw()

        root.after(500, update_plot)  # atualiza a cada 500 ms

    update_plot()
    root.mainloop()

def salvar_csv(hora, nome, valor_log):
    escritor_csv.writerow([hora, nome, valor_log])
    arquivo_csv.flush()  #escrita em disco sem fechar o arquivo

client = mqtt.Client()
client.connect("172.20.10.2", 1883)  #IP do broker, proprio notebook para se escutar ou antena da FSAE
client.subscribe("telemetria")
client.on_message = on_message
#MQTT em thread separada para tankar a interface gráfica do Tkinter
mqtt_thread = threading.Thread(target=lambda: client.loop_forever())
mqtt_thread.daemon = True
mqtt_thread.start()
iniciar_tkinter_grafico()
