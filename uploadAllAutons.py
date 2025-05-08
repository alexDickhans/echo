#!/usr/bin/env python3

import os
import sys
import argparse

# Autonomous modes
AUTONS = [
    "N_1_6",
    "N_1_6P",
    "N_6",
    "P_4",
    "P_1_3"
]

OFFSET = 1
AUTON_FILE_PATH = "./include/auton.h"

def write_auton_file(auton, alliance="RED"):
    """Writes the auton selection to the header file."""
    if not os.path.exists(AUTON_FILE_PATH) or not (auton in open(AUTON_FILE_PATH).read()):
        with open(AUTON_FILE_PATH, "w") as f:
            f.write("#pragma once\n")
            f.write("#include \"autonomous/autons.h\"\n")
            f.write(f"#define AUTON {auton}\n")
            f.write(f"inline auto ALLIANCE={alliance};\n")
        print(f"Auton file updated with {auton} and default alliance {alliance}.")
    else:
        print(f"No changes made to {AUTON_FILE_PATH}.")

def compile_auton(slot, auton):
    """Compiles the specified auton into the corresponding PROS slot."""
    command = f"pros mu --slot {slot} --name \"{auton}\""
    print(f"Running command: {command}")
    result = os.system(command)
    if result != 0:
        print(f"Error: Failed to compile {auton} into slot {slot}.")
        # sys.exit(2)

def compile_all():
    """Compiles all available autons."""
    for i, auton in enumerate(AUTONS):
        write_auton_file(auton)
        compile_auton(i + OFFSET, auton)

def display_menu():
    """Displays the menu of options."""
    print("\nSelect an autonomous mode:")
    for i, auton in enumerate(AUTONS):
        print(f"{i}: {auton}")
    print("a: Compile all")
    print("q: Quit")

def main():
    parser = argparse.ArgumentParser(description="Auton Selector CLI")
    parser.add_argument("-s", "--slot", type=int, help="Specify the slot for a single auton.")
    parser.add_argument("-a", "--alliance", type=str, default="RED", choices=["RED", "BLUE"],
                        help="Specify the alliance color (RED or BLUE).")
    args = parser.parse_args()

    if args.slot is not None:
        if args.slot < 0 or args.slot >= len(AUTONS):
            print(f"Error: Slot must be between 0 and {len(AUTONS) - 1}.")
            sys.exit(1)
        auton = AUTONS[args.slot]
        write_auton_file(auton, args.alliance)
        compile_auton(args.slot + OFFSET, auton)
    else:
        while True:
            display_menu()
            choice = input("Enter your choice: ").strip().lower()
            if choice == "q":
                print("Exiting...")
                break
            elif choice == "a":
                compile_all()
            elif choice.isdigit() and 0 <= int(choice) < len(AUTONS):
                slot = int(choice)
                auton = AUTONS[slot]
                write_auton_file(auton)
                compile_auton(slot + OFFSET, auton)
            else:
                print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()