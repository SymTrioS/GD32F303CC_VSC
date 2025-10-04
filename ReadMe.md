<h1 align="center">GD32F303CC Demo Program for board Prime-S73P</h1>

## Install toolchain and Visual Studio Code:

sudo apt install gcc-arm-none-eabi

sudo apt install openocd

sudo apt-get install wget gpg apt-transport-https

wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg

sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg

sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

rm -f packages.microsoft.gpg

sudo apt update

sudo apt install code


## Get this program:

1.  git clone https://github.com/SymTrioS/GD32F303CC_VSC.git
2.  cd GD32F303CC_VSC
3.  Check files in folders openocd programm: 'scripts/target/gd32f30x.cfg' and 'scripts/interface/cmsis-dap.cfg'
    Add configuration files if necessary from folder 'openocd' this projects.
4.  code .

### Install these plugins in the Extensions section:
*C/C++ Extension Pack* – for syntax highlighting and debugging support;  
*Code Spell Checker* – for highlighting errors and typos in code and comments;  
*Makefile Tool* – for convenient syntax highlighting and working with Makefiles;  
*Cortex-Debug* – ARM Cortex-M GDB Debugger for VSCode, can display Peripherals, Core Registers, Function Disassembly, Memory View, and much more (although it can only be used if arm-none-eabi-gdb is installed on the host OS);  
*Memory View* – provides the ability to display memory while working in the debugger;  
*Head-File-Guard* – a tool for working with #ifndef… #define… #endif constructs;  
*Linker Script* – a tool for highlighting and checking the syntax of linker scripts;  
*Command Variable* - is a plugin for conveniently working with variables that can be used in project settings;  
*Tasks* - is a plugin that displays task buttons on the bottom panel of the development window (you'll see what this means a little later);  
*TODO* - Highlight is a highlighter for comments that contain constructs like TODO or FIXME, which can subsequently generate a list of all the improvements specified in the comments.

5.  Push **build**       (user menu on the bottom panel)
6.  Push **write to mk** (user menu on the bottom panel)
7.  To run in debug mode, click **Run and Debug** on the left panel or **Ctrl+Shift+D**
