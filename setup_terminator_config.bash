#!/bin/bash

# Step 1: Prompt user for inputs
# Helper: remove trailing slashes
sanitize_path() {
    echo "$1" | sed 's:/*$::'
}

# Step 1: Prompt user for inputs and sanitize
read -p "Enter CARLA_DIR path: " raw_carla
read -p "Enter BRIDGE_DIR path: " raw_bridge
read -p "Enter CODE_DIR path: " raw_code

CARLA_DIR=$(sanitize_path "$raw_carla")
BRIDGE_DIR=$(sanitize_path "$raw_bridge")
CODE_DIR=$(sanitize_path "$raw_code")
OBJECTS_FILE=$CODE_DIR/src/dev/objects.json	

# Step 2: Check if terminator is installed
if ! command -v terminator &> /dev/null; then
    echo "Terminator not found. Installing..."
    sudo apt update && sudo apt install terminator -y
fi

read -p "Enter CARLA version (9.15 or 9.11): " carla_version

if [ "$carla_version" = "9.11" ]; then
    carla_local="yes"
    echo "CARLA version 9.11 detected. Assuming local installation."
else
    read -p "Is CARLA installed locally? yes->Locally, No->Docker (yes/no) " carla_local
    carla_local=$(echo "$carla_local" | tr '[:upper:]' '[:lower:]')
fi

if [ "$carla_local" = "yes" ]; then
    read -p "Enter CARLA directory path: " raw_carla
    CARLA_DIR=$(sanitize_path "$raw_carla")
fi

read -p "Is ROS installed locally? yes->Locally, No->Docker (yes/no) " ros_local
ros_local=$(echo "$ros_local" | tr '[:upper:]' '[:lower:]')

read -p "Enter path to int_ws workspace: " raw_int_ws
INT_WS_DIR=$(sanitize_path "$raw_int_ws")

# Set PYTHONPATH and CARLA_VERSION based on the CARLA version
if [ "$carla_version" = "9.11" ]; then
    echo "0.9.11" > $INT_WS_DIR/src/sem-apc-ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
elif [ "$carla_version" = "9.15" ]; then
    echo "0.9.15" > $INT_WS_DIR/src/sem-apc-ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
fi

create_command() {
    local alias_name="$1"
    local alias_command="$2"
    local carla_python_path=""

    # Determine the correct CARLA Python path based on the version
    if [ "$carla_version" = "9.11" ]; then
        carla_python_path="$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg"
    elif [ "$carla_version" = "9.15" ]; then
        carla_python_path="$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg"
    fi

    if [ "$ros_local" = "yes" ]; then
        # Export PYTHONPATH globally within the terminal session
        echo "bash --init-file <(echo \". \\\"\$HOME/.bashrc\\\"; export PYTHONPATH=\$PYTHONPATH:$carla_python_path; cd $INT_WS_DIR; source devel/setup.bash; alias $alias_name='$alias_command'; echo '$alias_name = $alias_command'\\n\")"
    else
        # Export PYTHONPATH for Docker-based ROS
        echo "bash -c 'docker start ros_environment && docker exec -ti ros_environment /bin/bash -c \"\
            cd $INT_WS_DIR && \
            echo -e \\\"\\\\n$alias_name = $alias_command\\\\n\\\"; \
            echo -e source devel/setup.bash >> ~/.bashrc && \
            echo -e \\\"\\\\nalias $alias_name=\\\\\\\"$alias_command\\\\\\\"\\\\n\\\" >> ~/.bashrc && \
            exec /bin/bash -i\"'"
    fi
}

TERMINAL1_CMD="echo 'CARLA management terminal' && bash"
if [ "$carla_local" = "yes" ]; then
    TERMINAL1_CMD="bash --init-file <(echo \". \\\"\$HOME/.bashrc\\\"; cd $CARLA_DIR; alias carla='./CarlaUE4.sh -quality-level=Low -windowed -fps -novsync -resx=360 -resy=240'; alias stop_carla='pkill CarlaUE4'; echo -e '\\ncarla = ./CarlaUE4.sh -quality-level=Low -windowed -fps -novsync -resx=360 -resy=240\\n'\")"
else
    TERMINAL1_CMD="bash --init-file <(echo \". \\\"\$HOME/.bashrc\\\"; alias carla='docker start carla_server'; alias stop_carla='docker stop carla_server'; echo -e '\\nAliases:\\ncarla - Start CARLA docker\\nstop_carla - Stop CARLA docker\\n'\")"
fi

TERMINAL2_CMD=$(create_command "bridge" "roslaunch carla_shell_bridge main.launch")
TERMINAL3_CMD=$(create_command "run" "./run.bash")
TERMINAL4_CMD=$(create_command "autonomous_ctr" "rosrun control cmd_topics.py")
TERMINAL5_CMD=$(create_command "manual_ctr" "roslaunch carla_manual_control carla_manual_control.launch")
TERMINAL6_CMD=$(create_command "evaluate" "roslaunch performance_evaluation performance_evaluation.launch")

mkdir -p ~/.config/terminator

cat <<EOF > ~/.config/terminator/config
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    cursor_color = "#aaaaaa"
[layouts]
  [[default]]
    [[[window0]]]
      type = Window
      parent = ""
    [[[child1]]]
      type = Terminal
      parent = window0
      profile = default
  [[carla]]
    [[[child0]]]
      type = Window
      parent = ""
      title = Carla
      fullscreen = False
      size = 1280, 720
    [[[child1]]]
      type = VPaned
      parent = child0
      order = 0
      ratio = 0.5
    [[[child2]]]
      type = HPaned
      parent = child1
      order = 0
      ratio = 0.333333333
    [[[terminal3]]]
      type = Terminal
      parent = child2
      order = 0
      profile = default
      title = carla
      directory = $CARLA_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; alias carla=\"./CarlaUE4.sh -prefernvidia\"; echo -e \"\ncarla: ./CarlaUE4.sh -prefernvidia\n\"")

    [[[child4]]]
      type = HPaned
      parent = child2
      order = 1
      ratio = 0.5
    [[[terminal5]]]
      type = Terminal
      parent = child4
      order = 0
      profile = default
      title = ros bridge
      directory = $BRIDGE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; export CARLA_ROOT=$CARLA_DIR; export PYTHONPATH=\$PYTHONPATH:$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg:$CARLA_DIR/PythonAPI/carla; source devel/setup.bash; alias bridge=\"roslaunch carla_ros_bridge carla_ros_bridge.launch town:=Town05 synchronous_mode:=False\"; echo -e \"\nbridge: roslaunch carla_ros_bridge carla_ros_bridge.launch town:=Town05 synchronous_mode:=False\n\"")

    [[[terminal6]]]
      type = Terminal
      parent = child4
      order = 1
      profile = default
      title = spawn objects
      directory = $BRIDGE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; export CARLA_ROOT=$CARLA_DIR; export PYTHONPATH=\$PYTHONPATH:$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg:$CARLA_DIR/PythonAPI/carla; source devel/setup.bash; alias spawn=\"roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=$OBJECTS_FILE\"; echo -e \"\nspawn: roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=$OBJECTS_FILE\n\"")

    [[[child7]]]
      type = HPaned
      parent = child1
      order = 1
      ratio = 0.333333333
    [[[terminal8]]]
      type = Terminal
      parent = child7
      order = 0
      profile = default
      title = manual control
      directory = $BRIDGE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; export CARLA_ROOT=$CARLA_DIR; export PYTHONPATH=\$PYTHONPATH:$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg:$CARLA_DIR/PythonAPI/carla; source devel/setup.bash; alias control=\"roslaunch carla_manual_control carla_manual_control.launch\"; echo -e \"\ncontrol: roslaunch carla_manual_control carla_manual_control.launch\n\"")

    [[[child9]]]
      type = HPaned
      parent = child7
      order = 1

    [[[child10]]]
      type = VPaned
      parent = child9
      order = 0

    [[[child11]]]
      type = VPaned
      parent = child9
      order = 1

    [[[terminal10]]]
      type = Terminal
      parent = child10
      order = 0
      profile = default
      title = ./run.bash file
      directory = $CODE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; alias run='./run.bash'; echo -e '\nrun: ./run.bash\n'")

    [[[terminal11]]]
      type = Terminal
      parent = child10
      order = 1
      profile = default
      title = Autonomous Control
      directory = $CODE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; source devel/setup.bash; alias auto_ctr='rosrun control cmd_topics.py'; echo -e '\nauto_ctr: rosrun control cmd_topics.py\n'")

    [[[terminal12]]]
      type = Terminal
      parent = child11
      order = 0
      profile = default
      title = rviz
      directory = $CODE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; source devel/setup.bash; alias rviz='rviz -d src/dev/shell.rviz'; echo -e '\nrviz: rviz -d src/dev/shell.rviz\n'")

    [[[terminal13]]]
      type = Terminal
      parent = child11
      order = 1
      profile = default
      title = performance evaluation 
      directory = $CODE_DIR
      command = bash --init-file <(echo ". \"$HOME/.bashrc\"; source devel/setup.bash; alias test_performance='roslaunch performance_evaluation performance_evaluation.launch'; echo -e '\ntest_performance: roslaunch performance_evaluation performance_evaluation.launch\n'")
[plugins]
EOF

echo "✅ Terminator config created successfully in ~/.config/terminator/config"
echo "usage: terminator -l carla"
