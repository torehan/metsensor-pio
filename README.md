# metsensor-pio

## Requirements

1. Download and install MS Visual Studio Code
2. Install **C/C++ for Visual Studio Code** and **PlatformIO**
3. Install PlatformIO extension in Visual Studio Code

## Compiling the project
After installing the requirements, clone the repo to your local machine. 
- If you are using a physical machine, you can use **PIO Home** user interface tool to import and configure the project.
- If you are running on a virtual machine, **PIO Home** won't work but there is also a powerfull CLI tool which has the same functionality with PIO Home.
  - Open the folder that contains the local copy of the repo on Visual Studio Code.
  - Start a new terminal with PlatformIO Core CLI (you can search for the command on Visual Studio with CTRL + SHIFT + P). 
  - Change directory to the project folder.
  - Run project targets over environments declared in **platformio.ini** (Project Configuration File).  
```
pio run
```
  Note: All dependencies of the project declared in the **platform.ini** will be installed by PlatformIO automatically.
Some basic Git commands are:


  
## Downloading the compiled FW to the Teensy
After successfull compilation, you can download the FW to the Teensy.
```
pio run --target upload
```
## Updating the libraries
While the PCAP04-arduino and arduino-MPU-driver is still in development you should always check if you are using the latest version.
You can simply update the driver libraries with `pio lib update`.
