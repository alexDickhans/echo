#!/usr/bin/env python3

import os
import sys
import time

# Qual

# os.system("pros v5 rm-all")

autons = ["AWP", "AWP_PUSH", "POS_ELIM", "NEG_ELIM", "SKILLS"]

offset = 1
def getFilePath():
    return os.path.dirname(__file__)

backslash_char = "\\ "

if len(sys.argv) > 1:

    i = int(sys.argv[1])# auton_list.index(list(filter(lambda x: x[0] in sys.argv[1], auton_list))[0])
    f = open("./include/auton.h", "w")
    f.write("#pragma once\n")
    f.write("#include \"autonomous/autons.h\"\n")
    f.write("#define AUTON Auton::" + autons[i] + "\n")
    f.write("auto ALLIANCE=RED;\n")
    f.close()
    command = f"pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\""
    print (command)

    result = os.system(command)

    if result != 0:
        os._exit(2)

else:
    for i in range(len(autons)):
        f = open("./include/auton.h", "w")
        f.write("#pragma once\n")
        f.write("#include \"autonomous/autons.h\"\n")
        f.write("#define AUTON Auton::" + autons[i] + "\n")
        f.write("auto ALLIANCE=RED;\n")
        f.close()

        command = f"pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\""
        print(command)

        result = os.system(command)

        if result != 0:
            os._exit(2)

    os._exit(0)

