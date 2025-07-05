import pandas as pd

# Carrega a tabela
df = pd.read_csv(
    r'c:\Users\galag\OneDrive\DV\telemetria_eracing\componentes_csv\CAN Description 2025 - VCU.csv',
    header=None, skip_blank_lines=True, comment='/'
)

def extrai_valor(data, campo):
    campo = str(campo).strip() #pega o campo da planilha e tira espaços
    #se começar com bit(
    if campo.startswith('bit('): #se começar com bit(
        bits = campo.replace('bit(', '').replace(')', '').split('-')
        bit_ini = int(bits[0])
        bit_fim = int(bits[-1])
        # 'bit(10-11)' -> bits = ['10', '11']
        valor_total = 0
        lista_bits = []

        for byte in data: #para cada byte na mensagem
            bits_str = bin(byte)[2:].zfill(8)  # ex: '00001000'
            bits_str = bits_str[::-1]          # inverte: '00010000'
            lista_bits.append(bits_str)

        # junta todos os bits na ordem (LSB primeiro, já invertido)
        bits_concatenados = ''.join(lista_bits[::])

        #binário para inteiro int(x, base=2)
        valor_total = int(bits_concatenados, 2)
        #máscara no formato 0b1111 
        mask = (1 << (bit_fim - bit_ini + 1)) - 1 # 2 elevado ao número de bits - 1
        return (valor_total >> bit_ini) & mask # compara o valor com a máscara, para os bits iguais ele dá 1 e bits diferentes dá 0
    
    #Se o campo for byte()
    elif campo.startswith('byte('):
        bytes_ = campo.replace('byte(', '').replace(')', '').split('-')
        byte_ini = int(bytes_[0])
        if len(bytes_) == 1:
            return data[byte_ini]
        else:
            byte_fim = int(bytes_[1])
            valor = 0
            for i in range(byte_fim, byte_ini - 1, -1):
                valor = (valor << 8) | data[i]
            return valor
    else:
        return None

def decodifica_mensagem_por_id(id_hex, data, df):
    idxs = df.index[df[1] == id_hex].tolist()
    if not idxs:
        print(f"ID {id_hex} não encontrado na tabela.")
        return
    idx_inicio = idxs[0] + 1
    idx_fim = idx_inicio
    # Corrigido: para quando a coluna 1 (nome do campo) está vazia ou NaN
    while idx_fim < len(df) and pd.notna(df.iloc[idx_fim, 1]):
        idx_fim += 1
    sinais = df.iloc[idx_inicio:idx_fim]

    resultado = {}
    for _, row in sinais.iterrows():
        nome = str(row[1]).strip()
        campo = row[2]
        mult = float(row[6]) if not pd.isna(row[6]) else 1
        desc = row[9] if len(row) > 9 else ""
        valor_bruto = extrai_valor(data, campo)
        if valor_bruto is not None:
            valor = valor_bruto * mult
            resultado[nome] = {
                "valor": valor,
                "descricao": desc
            }
    return resultado

if __name__ == "__main__":
    id_hex = "0x18FF00EA"
    data = [21, 5, 1, 0, 0, 1, 1, 0]
    resultado = decodifica_mensagem_por_id(id_hex, data, df)
    if resultado:
        for nome, info in resultado.items():
            print(f"{nome}: {info['valor']} ({info['descricao']})")