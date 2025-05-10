# 🏁 CARLA Terminator Layout Setup

This bash script automates the creation of a Terminator layout configured for your CARLA + ROS development workflow.

---

## 📦 Features

- Prompts for custom paths:
  - `CARLA_DIR`
  - `INT_WS_DIR`
- Checks if **Terminator** is installed (and installs it if not).
- Automatically configures a Terminator layout with multiple terminals for:
  - Launching CARLA
  - ROS Bridge
  - Spawning objects
  - Manual control
  - Launching run script
  - Autonomous control node
  - RViz
  - Performance evaluation
- Dynamically sets the `PYTHONPATH` for CARLA based on the version (`9.11` or `9.15`).
- Saves the layout config under `~/.config/terminator/config`.

---

## 🚀 Usage

### 1. Clone or download the script

```bash
git clone https://github.com/CUERT-2024-2025/terminator_config
cd terminator_config
```

### 2. Run the script

```bash
./setup_terminator_config.bash
```

### 3. Enter the required paths when prompted

Example:

```
Enter CARLA version (9.15 or 9.11): 9.15
Is CARLA installed locally? yes->Locally, No->Docker (yes/no): yes
Enter CARLA directory path: /home/mahmoud/EcoRacingTeam/software/CARLA
Is ROS installed locally? yes->Locally, No->Docker (yes/no): yes
Enter path to int_ws workspace: /home/mahmoud/EcoRacingTeam/int_ws
```

*Trailing slashes will be automatically cleaned.*

---

## 🖥️ Open Terminator with the Layout

After setup, run Terminator like this to load the `carla` layout:

```bash
terminator -l carla
```

---

## 📝 Notes

- Script will attempt to install Terminator using `apt` if not found.
- Existing `~/.config/terminator/config` will be **overwritten**. Back it up if needed.

## Final Look

![image](https://github.com/user-attachments/assets/528565a2-f824-4491-85ec-f2db01b13d39)
