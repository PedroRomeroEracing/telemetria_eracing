#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import tkinter as tk
from tkinter import ttk

root = tk.Tk()

# Lista de IDs obrigatórios (mantendo estilo original)
IDS_OBRIGATORIOS = [
    *[f"0x19B5010{i:02X}" for i in range(12)],   # Tensões HV
    "0x19B70100",                               # Tensões LV
    *[f"0x19B5080{i}" for i in range(4)],       # Temp HV
    "0x19B70800",                               # Temp LV
    "0x18FF01EA","0x18FF02EA","0x18FF01F7","0x18FF02F7",  # Temp Motores
    "0x18FF00EA","0x18FF00F7"                              # Temp Inversores
]

class StaticDisplay(ttk.LabelFrame):
    def __init__(self, parent, title="Valor", unit="", **kwargs):
        super().__init__(parent, text=title, padding=10, **kwargs)
        self.var = tk.StringVar(value=f"0 {unit}")
        self.label = ttk.Label(
            self,
            textvariable=self.var,
            font=("Helvetica", 12),
            anchor="center"
        )
        self.label.pack(expand=True, fill="both")

    def update(self, value: float):
        self.var.set(f"{value:.2f}")

class Application:
    def __init__(self):
        self.root = root
        self._build_ui()
        self._build_ros()
        self.root.after(1, self._ros_spin_once)

        # Displays
        self.disp = StaticDisplay(root, title="Velocidade", unit="Km/h")
        self.disp2 = StaticDisplay(root, title="Tensão de alta", unit="V")
        self.disp3 = StaticDisplay(root, title="Tensão de baixa", unit="V")

        self.disp.place( relx=0.03, rely=0.03, relwidth=0.1, relheight=0.1)
        self.disp2.place(relx=0.13, rely=0.03, relwidth=0.1, relheight=0.1)
        self.disp3.place(relx=0.23, rely=0.03, relwidth=0.1, relheight=0.1)

        # Frames e tabelas
        self.frames_da_tela()
        self.lista_frame2()
        self.lista_frame()
        self.lista_frame3()

        # Linhas iniciais nas tabelas
        self.tree_items = [self.listacl1.insert('', 'end', values=("", "")) for _ in range(10)]
        self.tree_items2 = [self.listacl2.insert('', 'end', values=("", "")) for _ in range(10)]
        self.tree_items3 = [self.listacl3.insert('', 'end', values=("", "")) for _ in range(10)]

    def _build_ui(self):
        self.root.configure(background='#ADD8E6')
        self.root.title("Telemetria")
        self.root.geometry("1280x720")
        self.root.minsize(640,360)

    def _build_ros(self):
        self.node = Node('float_subscriber')
        self.node.create_subscription(
            String,
            'telemetria',
            self.listener_callback,
            10
        )

    def _ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(50, self._ros_spin_once)

    def listener_callback(self, msg: String):
        data = json.loads(msg.data)

        # 🔹 Garante que todos os IDs obrigatórios estão presentes
        for id_ in IDS_OBRIGATORIOS:
            if id_ not in data:
                data[id_] = {}

        # Atualiza displays
        if "Velocidade" in data:
            self.disp.update(data["Velocidade"].get("value", 0))
        if "Tensao_pack_alta" in data:
            self.disp2.update(data["Tensao_pack_alta"].get("value", 0))
        if "Tensao_pack_baixa" in data:
            self.disp3.update(data["Tensao_pack_baixa"].get("value", 0))

        # Coleta dados
        vcell_alta = {k: v[-1] for k,v in self._collect_ids(data, prefix="0x19B5010").items() if v}
        vcell_baixa = {k: v[-1] for k,v in self._collect_ids(data, ids=["0x19B70100"]).items() if v}
        temp_alta = {k: v[-1] for k,v in self._collect_ids(data, prefix="0x19B5080").items() if v}
        temp_baixa = {k: v[-1] for k,v in self._collect_ids(data, ids=["0x19B70800"]).items() if v}
        temp_motores = {k: v[-1] for k,v in self._collect_ids(data, ids=["0x18FF01EA","0x18FF02EA","0x18FF01F7","0x18FF02F7"]).items() if v}
        temp_inversores = {k: v[-1] for k,v in self._collect_ids(data, ids=["0x18FF00EA","0x18FF00F7"]).items() if v}

        # Ordenação
        Vcel_a_sorted = sorted(vcell_alta.items(), key=lambda x: x[1])[:10]
        Vcel_b_sorted = sorted(vcell_baixa.items(), key=lambda x: x[0])
        t_pack_h_sorted = sorted(temp_alta.items(), key=lambda x: x[1], reverse=True)[:10]
        t_pack_sorted = sorted(temp_baixa.items(), key=lambda x: x[0])
        t_motores_sorted = sorted(temp_motores.items(), key=lambda x: x[0])
        t_inversores_sorted = sorted(temp_inversores.items(), key=lambda x: x[0])

        # Atualiza tabelas
        self.update_tree1(Vcel_a_sorted, Vcel_b_sorted)
        self.update_tree2(t_pack_h_sorted, t_pack_sorted)
        self.update_tree3(t_motores_sorted, t_inversores_sorted)

    def _collect_ids(self, data, prefix=None, ids=None):
        result = {}
        for k,v in data.items():
            if (prefix and k.startswith(prefix)) or (ids and k in ids):
                for var, valores in v.items():
                    if isinstance(valores, list) and valores:
                        result[f"{k}:{var}"] = valores
        return result

    def frames_da_tela(self):
        self.frame = tk.Frame(self.root)
        self.frame.place(relx=0.4, rely=0.02, relwidth=0.3, relheight=0.35)

        self.frame_2 = tk.Frame(self.root)
        self.frame_2.place(relx=0.7, rely=0.02, relwidth=0.3, relheight=0.35)

        self.frame_3 = tk.Frame(self.root)
        self.frame_3.place(relx=0.7, rely=0.37, relwidth=0.3, relheight=0.35)

    def lista_frame2(self):
        self.listacl2 = ttk.Treeview(self.frame_2, height=10, columns=("col1","col2"), show="headings")
        self.listacl2.heading("col1", text="T° Células alta")
        self.listacl2.heading("col2", text="T° Células baixa")
        self.listacl2.place(relx=0.01, rely=0.03, relwidth=0.95, relheight=0.90)

    def lista_frame(self):
        self.listacl1 = ttk.Treeview(self.frame, height=10, columns=("col1","col2"), show="headings")
        self.listacl1.heading("col1", text="V Cel Alta")
        self.listacl1.heading("col2", text="V Cel baixa")
        self.listacl1.place(relx=0.01, rely=0.03, relwidth=0.95, relheight=0.90)

    def lista_frame3(self):
        self.listacl3 = ttk.Treeview(self.frame_3, height=10, columns=("col1","col2"), show="headings")
        self.listacl3.heading("col1", text="T° Motores")
        self.listacl3.heading("col2", text="T° Inversores")
        self.listacl3.place(relx=0.01, rely=0.03, relwidth=0.95, relheight=0.90)

    def update_tree1(self, cela, celb):
        celb += [("N/A", 0.0)] * (10 - len(celb))
        for idx, item_id in enumerate(self.tree_items):
            id_alta, val_alta = cela[idx] if idx < len(cela) else ("N/A",0.0)
            id_baixa, val_baixa = celb[idx]
            self.listacl1.item(item_id, values=(f"{id_alta}: {val_alta:.2f}", f"{id_baixa}: {val_baixa:.2f}"))

    def update_tree2(self, t_pack_h, t_pack):
        t_pack_h += [("N/A", 0.0)] * (10 - len(t_pack_h))
        t_pack += [("N/A", 0.0)] * (10 - len(t_pack))
        for idx, item_id in enumerate(self.tree_items2):
            id_ph, val_ph = t_pack_h[idx]
            id_p, val_p = t_pack[idx]
            self.listacl2.item(item_id, values=(f"{id_ph}: {val_ph:.2f}", f"{id_p}: {val_p:.2f}"))

    def update_tree3(self, t_motores, t_inversores):
        t_motores += [("N/A", 0.0)] * (10 - len(t_motores))
        t_inversores += [("N/A", 0.0)] * (10 - len(t_inversores))
        for idx, item_id in enumerate(self.tree_items3):
            id_m, val_m = t_motores[idx]
            id_i, val_i = t_inversores[idx]
            self.listacl3.item(item_id, values=(f"{id_m}: {val_m:.2f}", f"{id_i}: {val_i:.2f}"))

def main(args=None):
    rclpy.init(args=args)
    app = Application()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
