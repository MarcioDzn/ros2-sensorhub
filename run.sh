#!/bin/bash

# Script para iniciar o container ROS2 em diferentes ambientes

show_usage() {
    echo "Uso: ./run.sh [pc|raspberry|generico]"
    echo ""
    echo "Exemplos:"
    echo "  ./run.sh pc          # Roda com configurações do PC"
    echo "  ./run.sh raspberry   # Roda com configurações da Raspberry Pi"
    echo ""
    exit 1
}

# Verifica se foi passado argumento
if [ $# -eq 0 ]; then
    show_usage
fi

ENV_TYPE=$1

case $ENV_TYPE in
    pc)
        echo "🖥️  Iniciando container para PC..."
        docker compose --env-file .env.pc run --rm sensorhub
        ;;
    raspberry)
        echo "🍓 Iniciando container para Raspberry Pi..."
        docker compose --env-file .env.raspberry run --rm sensorhub
        ;;
    *)
        echo "❌ Ambiente desconhecido: $ENV_TYPE"
        show_usage
        ;;
esac