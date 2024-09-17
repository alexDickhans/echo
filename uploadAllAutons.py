#!/usr/bin/env python3

import os
import sys
import time

# Qual

# os.system("pros v5 rm-all")

autons = [
    "FAR 6 MID ELIM",
    "SAFE 6",
    "STATES 6",
    "CLOSE SAFE AWP DELAY",
    "CLOSE RUSH ELIM",
    "CLOSE RUSH AWP",
    "SKILLS"]

offset = 1
def getFilePath():
    return os.path.dirname(__file__)

if len(sys.argv) > 1:
    i = int(sys.argv[1])
    command = f"pros mu --project {getFilePath().replace(' ', '\\ ')} --slot " + str(i+offset) + " --name \"" + autons[i] + "\""
    print (command)

    result = os.system(command)

    if result != 0:
        os._exit(2)

else:
    for i in range(len(autons)):
        f = open(getFilePath() + "/include/auton.h", "w")
        f.write("#define AUTON " + str(i) + "\n")
        f.close()

        command = f"pros mu --project {getFilePath().replace(" ", "\\ ")} --slot " + str(i+offset) + " --name \"" + autons[i] + "\""
        print(command)

        result = os.system(command)

        if result != 0:
            os._exit(2)

    os._exit(0)

