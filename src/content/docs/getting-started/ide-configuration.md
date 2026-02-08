---
title: "IDE Configuration"
---
# Option 1 - VSCode

Should be automatically installed by Ansible. **Make sure to open the _mrover_ folder in VSCode, NOT a subfolder**. The proper settings and plugins should be contained in the `.vscode` folder there. Make sure to install the recommended plugins that a dialog in the bottom right will suggest on opening the project.

Intellisense should work out of the box with clangd after running `./build.sh`. This command generates `compile_commands.json` which clangd understands. Please contact a lead if this does not work.

[ADVANCED] Microsoft puts a lot of spyware into VSCode so if you want you can use [VSCodium](https://vscodium.com/). It is the same thing except built from source with all the junk removed.

### Debugging in VSCode

The recommended way is to comment out the node you want to debug from your launch script. Then hit Ctrl-Shift-P in VSCode and launch a debugger on the node separately. The CMake plugin in VSCode should enable this.

# Option 2 - CLion + PyCharm

Use Toolbox to install both: https://www.jetbrains.com/toolbox-app/

CLion supports CMake very well out of the box so there is minimal setup. Check: https://www.jetbrains.com/help/clion/ros-setup-tutorial.html

PyCharm requires some more attention: https://www.youtube.com/watch?v=lTew9mbXrAs

# Option 3 - Other: Vim, Sublime, ???

At the end of the day ROS uses CMake for all project configuration. Most IDE's will have plugins that support this. It is up to you to find that out. If you are using Vim, it is recommended that you use clangd with clang-tidy enabled to take advantage of the style hints they give.