#!/bin/bash

sanitize_path() {
    echo "$1" | sed 's:/*$::; s:^~/:'"$HOME"'/:'
}

if ! command -v terminator &> /dev/null; then
    echo "Installing Terminator..."
    sudo apt update && sudo apt install terminator -y
fi

read -p "Is ROS installed locally? yes->Locally, No->Docker (yes/no) " ros_local
ros_local=$(echo "$ros_local" | tr '[:upper:]' '[:lower:]')

read -p "Is CARLA installed locally? yes->Locally, No->Docker (yes/no) " carla_local
carla_local=$(echo "$carla_local" | tr '[:upper:]' '[:lower:]')

if [ "$carla_local" = "yes" ]; then
    read -p "Enter CARLA directory path: " raw_carla
    CARLA_DIR=$(sanitize_path "$raw_carla")
fi

read -p "Enter path to int_ws workspace: " raw_int_ws
INT_WS_DIR=$(sanitize_path "$raw_int_ws")

create_command() {
    local alias_name="$1"
    local alias_command="$2"
    if [ "$ros_local" = "yes" ]; then
        echo "bash --init-file <(echo \". \\\"\$HOME/.bashrc\\\"; export PYTHONPATH=\$PYTHONPATH:$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg; cd $INT_WS_DIR; source devel/setup.bash; alias $alias_name='$alias_command'; echo '$alias_name = $alias_command'\\n\")"
    else
        # Fixed quoting for Docker commands
        echo "bash -c 'docker start ros_environment && docker exec -ti ros_environment /bin/bash -c \"\
            cd $INT_WS_DIR && \
            source devel/setup.bash && \
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
      directory = $HOME
      command = $TERMINAL1_CMD

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
      title = "Launch [Map, Spawn, Rviz]"
      directory = $HOME
      command = $TERMINAL2_CMD

    [[[terminal6]]]
      type = Terminal
      parent = child4
      order = 1
      profile = default
      title = Run
      directory = $HOME
      command = $TERMINAL3_CMD

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
      title = Autonomous Control
      directory = $HOME
      command = $TERMINAL4_CMD

    [[[child9]]]
      type = HPaned
      parent = child7
      order = 1

    [[[terminal10]]]
      type = Terminal
      parent = child9
      order = 0
      profile = default
      title = Manual Control
      directory = $HOME
      command = $TERMINAL5_CMD

    [[[terminal11]]]
      type = Terminal
      parent = child9
      order = 1
      profile = default
      title = Performance Evaluation Package
      directory = $HOME
      command = $TERMINAL6_CMD


[plugins]
EOF

echo -e "\n✅ Configuration complete!\nUsage: terminator -l carla"