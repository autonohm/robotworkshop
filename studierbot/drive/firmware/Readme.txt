1) Update Bootloader of KL25Z: If an old bootloader is installed (V1.09 or earlier), you need a Windows 7 computer to update the board.
2) For Linux and Windows support Segger JLink bootloader was working fine (mbed did not work with Kinetis Studio on Windows).

Next steps are, Installation of
- Kinetis Studio 3.2
- KSDK 1.3 Mainline release
- Install Eclipse Updater from tools/eclipse archive within Kinetis Studio
- Debug configurations/Debugger: Fix initial speed of Debugger to 1000 kHz, Other options: -singlerun -strict -timeout 0 -nogui, Commands: set mem inaccessible-by-default off
