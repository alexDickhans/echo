#!/usr/bin/env python3

import os
import sys
import time

# Qual

# os.system("pros v5 rm-all")

autons = {"AWP": True, "AWP_PUSH": True, "SKILLS": False}

auton_list = []

for auton in autons:
    if autons[auton]:
        auton_list.append([auton, "RED"])
        auton_list.append([auton, "BLUE"])
    else:
        auton_list.append([auton, "RED"])

offset = 1
def getFilePath():
    return os.path.dirname(__file__)

backslash_char = "\\ "

if len(sys.argv) > 1:

    i = int(sys.argv[1])# auton_list.index(list(filter(lambda x: x[0] in sys.argv[1], auton_list))[0])
    f = open("./include/auton.h", "w")
    f.write("#pragma once\n")
    f.write("#include \"autonomous/autons.h\"\n")
    f.write("#define AUTON Auton::" + auton_list[i][0] + "\n")
    f.write("auto ALLIANCE=" + auton_list[i][1] + ";\n")
    f.close()
    command = f"pros mu --slot " + str(i+offset) + " --name \"" + auton_list[i][1][0] + auton_list[i][0] + "\""
    print (command)

    result = os.system(command)

    if result != 0:
        os._exit(2)

else:
    for i in range(len(auton_list)):
        f = open("./include/auton.h", "w")
        f.write("#pragma once\n")
        f.write("#include \"autonomous/autons.h\"\n")
        f.write("#define AUTON Auton::" + auton_list[i][0] + "\n")
        f.write("auto ALLIANCE=" + auton_list[i][1] + ";\n")
        f.close()

        command = f"pros mu --slot " + str(i+offset) + " --name \"" + auton_list[i][1][0] + auton_list[i][0] + "\""
        print(command)

        result = os.system(command)

        if result != 0:
            os._exit(2)

    os._exit(0)

