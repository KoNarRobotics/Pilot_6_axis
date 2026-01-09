# 6 axis remote Controller
- ***ROS 2 integration***.
- ***Wi‑Fi connectivity***
- ***Saves up to 6 networks (prioritized 1–6)***
- ***Full configuration via web UI***
- ***Adjustable joystick inertia and signal scaling***
- ***Refresh rates up to 200 Hz***
- ***Auto-zero to prevent unintended motion***
- ***USB‑C charging***
- ***Two 3‑DOF joysticks + 10 buttons (2 on sticks)***
- ***Integrated LCD for status and setup***
- ***Persistent settings across sessions***


![Image](images/pilot_2.png){width=50%}


## ROS 2 Packages

### **remote_6d** (Recommended)
C++ implementation for reliable remote control integration. Receives UDP packets over Wi-Fi, parses 6-axis joystick and 10-button data, applies deadband filtering, and publishes to the `/joy` ROS 2 topic. Handles graceful disconnects by sending a zero frame when the pilot loses connection.

### **pilot_6axis** (Deprecated)
Python-based UDP receiver that converts remote control packets into ROS 2 `sensor_msgs::msg::Joy` messages for joystick control. No longer maintained; use `remote_6d` instead.


## The repo supports demo pc keyboard version of the RC 
 Pc keyboard version for RC without the need for the hardware if you want to test it or forgot to bring the RC with you :).
1. Full version
 ```bash
  python3 -m venv .venv
  python3 -m pip -r requirements.txt
  source .venv/bin/activate
  python3 keyboard_rc.py -i IP_ADDRESS -p PORT -f FREQUENCY -v JOYSTICK_VALUE
 ```
 2. Quick version (because you are most likely testing on localhost)
  ```bash
    python3 keyboard_rc.py -i 127.0.0.1
  ```
3. For help about arguments, and buttons mapping run: 
```bash
  python3 keyboard_rc.py
 ```

 ## How to use
### Setup of the Rc
  1. Enable setup mode by pressing and holding the most centeral button 4 or 5 when turning the Rc On (doesn't matter which one).
  2. Connect to the Rc Wi-Fi network that is displayed on  the LCD.
  3. Open web browser and go to url displayed on the LCD.
  4. Set up the settings of the Rc and press save buttons on the bottom of the page. This will redirect you to the page where you will see if settings were saved. If you made some error it will be shown here.
![Image](images/web.png){width=40%}

  5. To leave setup mode shut down the Rc and turn it on again.
  6. Now the Rc will connect to the network with the highest priority that is available. If it can't connect to any of the saved networks it will keep trying to connect to them. If the Rc is connected to the network it will display the IP address on the LCD and show Connected status.
  7. Now setup is done and you can use the Rc.

### Build ROS packages
- Standard way of building ROS 2 packages.
```bash
  colcon build --symlink-install
```
### Run ROS packages
- Run the remote_6d package
```bash
  ros2 run remote_6d remote_6d_node
```
- If the Rc is connected to the same network as the computer and the port isn't blocked by firewall the package should start receiving data from the Rc and publishing it to the `/joy` topic.




## Settings list:
#### General Settings
  - **Joystick left MMin** -> minimum value of the left joystick x, y, z axis on the min of the range 
  - **Joystick left MMax** -> maximum value of the left joystick x, y, z axis on the max of the range
  - **Joystick left filer** -> inertia of the left joystick (0-1), the higher inertia the response of the joystick value will be faster.
  - **Joystick right MMin** -> minimum value of the right joystick x, y, z axis on the min of the range
  - **Joystick right MMax** -> maximum value of the right joystick x, y, z axis on the max of the range
  - **Joystick right filer** -> inertia of the right joystick (0-1)
  - **WIFI SSID** => SSID of the wifi network created by the RC in setup mode.
  - **WIFI PASSWORD** => password of the wifi network made by the RC in setup mode
  - **Update frequency** => frequency of sending data frame
  - **Target IP** => IP address to which the RC will send data frame
  - **Host Port** => port to which the RC will send data frame

#### WIFI target networks
  - **Wifi [number 1-6] : Seed** => SSID of the wifi network (where number means priority from 1 to 6)
  - **Wifi [number 1-6] : Password** => Password of the wifi network



## Required component list

### Component Images

  | Component | Photo | Component | Photo |
  |-----------|-------|-----------|-------|
  | PCB (order from JLCPCB) | <img src="images/pcb.png" alt="PCB" width="80"/> | Joystick 4-axis JH-D400B-M4 | <img src="images/joystick.png" alt="Joystick" width="80"/> |
  | Toggle switch 6mm MTS-103 | <img src="images/toggle_switch_small.png" alt="Toggle switch" width="80"/> | Monostable switches 12mm PBS-33B | <img src="images/monostable_switches_small.png" alt="Monostable switches" width="80"/> |
  | Li-ion battery 3.7V 4000mAh | <img src="images/li_ion_battery_small.jpg" alt="Li-ion battery" width="80"/> | LCD 1.3" 128x64 OLED SSD1306 | <img src="images/lcd_small.png" alt="LCD" width="80"/> |
  | Antena 2.4GHz 3dBi WiFi RP-SMA | <img src="images/antena_small.png" alt="Antenna" width="80"/> | M3 6mm hex socket flat countersunk | <img src="images/m3_screws_small.png" alt="M3 screws" width="80"/> |
  | M3 heated insert M3xL3xOD4.2 | <img src="images/m3_heated_insert_small.png" alt="M3 heated insert" width="80"/> | M3 5mm hex socket cylindrical | <img src="images/m3_screws_small_cyl.png" alt="M3 screws cylindrical" width="80"/> |


### Component list 

  | Component Name                                   | Parts Count | Description                                              |
  |--------------------------------------------------|-------------|----------------------------------------------------------|
  | Joystick 4-axis JH-D400B-M4                      | 2           | 3-DOF joysticks for axis control                        |
  | Toggle switch 6mm MTS-103                        | 1           | Main toggle switch for power or mode selection           |
  | Monostable switches 12mm PBS-33B                 | 8           | Momentary push buttons for user input                   |
  | Li-ion battery 606090 3.7V 4000mAh               | 1           | Rechargeable battery for power supply                    |
  | LCD 1.3" 128x64 OLED SSD1306                     | 1           | Display for status and configuration                     |
  | Antena 2.4GHz 3dB WiFi 2.4GHz RP-SMA             | 1           | External Wi-Fi antenna for wireless communication        |
  | M3 6mm hex socket flat countersunk screws        | 14          | Screws for assembling the enclosure                      |
  | M3 heated insert M3xL3xOD4.2                     | 19          | Threaded inserts for secure screw mounting               |
  | M3 5mm hex socket cylindrical screws             | 4           | Additional screws for assembly                           |
  | PCB (order from JLCPCB)                          | 1           | Main circuit board for the controller                    |
  | BOM Components (see BOM file)                    | -           | All other electronic components (resistors, ICs, etc.)   |

- BOM file "Electric schematic/Pilot6Axis/jlcpcb/production_files/BOM-Pilot6Axis.csv"
- JLCB gerber files "Electric schematic/Pilot6Axis/jlcpcb/production_files/GERBER-Pilot6Axis.zip"

## How to assembly
  You figure it out.
