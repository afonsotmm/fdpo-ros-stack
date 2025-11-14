#!/bin/bash

# Script para gravar dados do LiDAR com rosbag
# Uso: ./record_lidar.bash [nome_do_bag] [duracao_segundos]

BAG_NAME=${1:-"lidar_data_$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-""}  # Se vazio, grava até Ctrl+C

# Criar diretório para os bags se não existir
BAG_DIR="$HOME/lidar_bags"
mkdir -p "$BAG_DIR"

BAG_PATH="$BAG_DIR/$BAG_NAME"

echo "=========================================="
echo "Gravação de dados do LiDAR"
echo "=========================================="
echo "Nome do bag: $BAG_NAME"
echo "Localização: $BAG_PATH"
echo ""

# Verificar qual driver está ativo
if rostopic list | grep -q "/laser_scan_point_cloud"; then
    echo "✓ YDLidar detectado: gravando /laser_scan_point_cloud"
    TOPICS="/laser_scan_point_cloud"
elif rostopic list | grep -q "/base_scan"; then
    echo "✓ HLS-LFCD2 detectado: gravando /base_scan"
    TOPICS="/base_scan"
else
    echo "⚠ Nenhum tópico LiDAR encontrado! Gravando ambos..."
    TOPICS="/laser_scan_point_cloud /base_scan"
fi

# Adicionar TF para contexto (opcional mas útil)
TOPICS="$TOPICS /tf /tf_static"

echo ""
echo "Tópicos a gravar: $TOPICS"
echo ""
echo "Pressione Ctrl+C para parar a gravação..."
echo ""

# Gravar rosbag
if [ -z "$DURATION" ]; then
    rosbag record -O "$BAG_PATH" $TOPICS
else
    timeout $DURATION rosbag record -O "$BAG_PATH" $TOPICS
    echo ""
    echo "✓ Gravação concluída após $DURATION segundos"
fi

echo ""
echo "Bag salvo em: $BAG_PATH"
echo "Para reproduzir: rosbag play $BAG_PATH"

