import json
dados = {"valor": 10, "nome": "teste"}
json_str = json.dumps(dados)
print(json_str)  # Saída: {"valor": 10, "nome": "teste"}