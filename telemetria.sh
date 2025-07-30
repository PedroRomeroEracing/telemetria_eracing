#!/bin/bash

echo "Ativando interfaces CAN"

# CAN0
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000 sjw 1
sudo ip link set can0 up
echo "CAN0 configurada."

# CAN1
sudo ip link set can1 down 2>/dev/null
sudo ip link set can1 type can bitrate 500000 sjw 1
sudo ip link set can1 up
echo "CAN1 configurada."

echo "Iniciando códigos..."

#caminho para os scripts

DIR="/home/pedroromero/telemetria_eracing"

python3 $DIR/car_telemetria1.py &
PID1=$!

python3 $DIR/car_telemetria2.py &
PID2=$!

setup ~/.bashrc

ros2 run box_backend box_telemetria 

echo "Todos os processos iniciados."

wait
