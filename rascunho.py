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

