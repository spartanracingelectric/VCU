# Programming for the HY-TTC 50
Note: This file can be viewed with proper formatting by visiting https://github.com/spartanracingelectric/VCU/blob/user/rusty/regen/rusty/readme.md

## About the TTControl HY-TTC 50/60

The HY-TTC 50 and HY-TTC 60 are part of a family of general purpose automotive electronic control units.  They offers a large array of inputs and outputs for sensors and power control, as well as communications via CAN/LIN/serial.  On our team, these ECUs are the central point of control for our entire vehicle, so we call them the VCU, or Vehicle Control Unit.

We used the HY-TTC 50 from 2015 to 2021, and we will be switching to the HY-TTC 60 for 2022.  The HY-TTC 60 is equivalent to the HY-TTC 50, except the HY-TTC 60 has additional analog inputs on some of the pins that were unused on the TTC 50.  Documentation throughout this project will continue to reference the HY-TTC 50.

TTControl is a joint venture company between HYDAC and TTTech, hence HY-TTC ## for their model numbers.  They have generously provided us with an amazing sponsorship package including two HY-TTC 50 controllers, test harnesses, parts for building vehicle harnesses, a development board, and software for their controller as well as tools for development and testing, such as a PCAN dongle.

Programming for the HY-TTC 50 is done in C on our team, though CoDeSys (“Controller Development System”) is also supported by the hardware.  TASKING VX, by Altium, is the recommended toolset (compiler) - our license was sponsored by Altium (separate from the hardware sponsorship from TTControl).

## Setting up your computer for development
### VCU SDK
Our VCU SDK includes the source code and tools necessary for compilation and flashing of the HY-TTC 50.
- Download the VCU SDK from our team drive, under:
  - SRE > SRE Software > Applications > VCU_SDK_2020-04-04.zip
  - Note: The 2020 SDK excludes some tools and documentation which are available on the team drive
- Extract the VCU folder to somewhere easy to access on your computer
  - You will need to remember this location when we get to the cloning step below
  - Note: Cloud folders like google drive or onedrive can cause complications with git (rare)
  - Important: Do not change the folder structure or you will not be able to compile
#### Contents
The zip file contains a single folder, called `VCU`.  It's okay to rename this.
The `VCU` folder contains a single folder, called `Environment`.  Do not rename this folder or move any of its subfolders, or you may not be able to compile later.
The `Environment` folder contains all the code needed for compilation, organized into the following subfolders:
- `bsp`
- `build`
- `dev`
  - Each subfolder in here represents a single "program" that runs on the VCU
  - This folder is empty in the dev kit.  You must clone the SRE-2 folder/repo from github into this folder.  Instructions below
- `examples`
  - These are example programs, provided by TTControl.  They show how to use individual features on the VCU.  You should read through these.
- `inc`
  - These header files are your interface to the VCU's hardware.
- `lib`
- `templates`
  - If you want to make a new program, you can copy the ttc50_template to the dev folder.  (You could also start with a folder from `examples`)

### IDE
We recommend Visual Studio Code ("VSCode" or "Code"), but you can use any IDE/editor you want.
https://code.visualstudio.com/

### git
git is the version control system (VCS) that we use for our source code.  Our git repository is hosted by github.

#### Install git
You will need to download/install git if you don't already have it.  I recommend that you learn to use git from the command line to help you prepare for a real job.  You can download git here:
https://git-scm.com/download

There are GUIs available in case you get lost, or want a visual for what you're working on.
- GitHub Desktop: https://desktop.github.com/
- GitKraken: https://www.gitkraken.com/

#### Clone VCU repo from github
Once you have git installed, clone the VCU repository INTO the **VCU/Environment/dev** folder.

General instructions: https://help.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository

Example:  I extracted the VCU folder from the VCU SDK to
`/mnt/g/fsae/VCU`
therefore I will cd into
`/mnt/g/fsae/VCU/Environment/dev/`
before I run the `git clone` command
```
rusty@Gridania:/mnt/g$ cd fsae/VCU/Environment/dev/
rusty@Gridania:/mnt/g/fsae/VCU/Environment/dev$ git clone git@github.com:spartanracingelectric/VCU.git sre
Cloning into 'sre'...
Enter passphrase for key '/home/rusty/.ssh/id_rsa':
remote: Enumerating objects: 1635, done.
remote: Total 1635 (delta 0), reused 0 (delta 0), pack-reused 1635
Receiving objects: 100% (1635/1635), 5.92 MiB | 9.95 MiB/s, done.
Resolving deltas: 100% (1087/1087), done.
```
Remember: It's important to maintain this folder structure, or else you will have trouble when you want to compile

### To compile/build the software, you need
- Altium TASKING VX – our toolset/compiler 
  - Do not change the default install folder 
- Environment\build\settings.mk must be updated to point to your version of TASKING (4.0 or 3.0) or else compilation will fail with an undescriptive message:  
```
 -compiling: avlTree.c
The system cannot find the path specified.
make: *** [build/avlTree.obj] Error 1
```
- The first time your code compiles successfully, you will receive another error.  It should try to download the .NET framework automatically.  The error should go away when the framework is installed and you compile again. 
```
 ---------------------------------
  Creating APDB... 
 ---------------------------------
make: *** [postbuild] Error -2146232576
```

#### Changing compilation target between TTC 50 / TTC 60
- tbd

### Flashing the VCU
To flash, run and debug software, you will also need:
- TTC Downloader, from the SDK, for flashing the compiled software (.hex file) onto the VCU
- PCAN-View, also from the SDK, or PCAN Explorer (sponsored by PEAK systems – ask your team lead) for monitoring CAN messages
- A PCAN dongle (PCAN-USB) with drivers installed
  - Download “PCAN-USB package” from http://www.peak-system.com/Packages.306.0.html?&L=1
- Optional: RMS GUI for monitoring the motor controller status if writing software that interacts with the motor controller
