#!/bin/bash

# Script simples para configurar CoppeliaSim + ROS2

set -e  # Para em caso de erro

if [ -f ".env.coppelia" ]; then
    source .env.coppelia
    echo "✓ Configurações carregadas de .env.coppelia"
fi

# Configurações
COPPELIA_PATH="${COPPELIASIM_ROOT_DIR:-/home/marcio/Documentos/Dev/UEFS/IC/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04}"
WORKSPACE_PATH="${ROS2_WORKSPACE_PATH:-$HOME/ros2_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "=== Setup CoppeliaSim + ROS2 ==="
echo ""
echo "CoppeliaSim: $COPPELIA_PATH"
echo "Workspace: $WORKSPACE_PATH"
echo "ROS Distro: $ROS_DISTRO"
echo ""

# Instala dependências
echo "[1/4] Instalando dependências..."
sudo apt update
sudo apt install -y xsltproc python3-xmlschema python3-zmq python3-cbor2

# Configura variável de ambiente
echo "[2/4] Configurando COPPELIASIM_ROOT_DIR..."
if ! grep -q "COPPELIASIM_ROOT_DIR" ~/.bashrc; then
    echo "export COPPELIASIM_ROOT_DIR=$COPPELIA_PATH" >> ~/.bashrc
    echo "✓ Adicionado ao ~/.bashrc"
else
    echo "✓ Já existe no ~/.bashrc"
fi

export COPPELIASIM_ROOT_DIR=$COPPELIA_PATH

# Configura ROS2
echo "[3/4] Configurando ROS2..."
cd "$WORKSPACE_PATH"
source /opt/ros/$ROS_DISTRO/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Compila workspace
echo "[4/4] Compilando workspace..."
colcon build --symlink-install --executor sequential --packages-select sim_ros2_interface

echo ""
echo "✅ Setup concluído!"
echo ""
echo "Próximos passos:"
echo "  source ~/.bashrc"
echo "  source $WORKSPACE_PATH/install/setup.bash"
echo "  cd $COPPELIA_PATH && ./coppeliaSim.sh"
echo ""