#!/usr/bin/env python3

import ntcore
import time

from ntcore import NetworkTableInstance

TEAM = 6377

def pub(instance: NetworkTableInstance):
    table = instance.getTable("datatable")

    xPub = table.getDoubleTopic("x").publish()
    yPub = table.getDoubleTopic("y").publish()
    # xSub = table.getDoubleTopic("x").subscribe(0.12)
    # ySub = table.getDoubleTopic("y").subscribe(0.34)

    counter = 0

    while True:
        time.sleep(1)

        xPub.set(counter)
        yPub.set(counter)
        print(f"Published x/y {counter}/{counter}")

        counter += 1

def persistence(instance: NetworkTableInstance):
    table = instance.getTable("datatable")
    table.putString("who-is-cool", "JD is cool!")
    table.setPersistent("who-is-cool")

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("example writer")
    # inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
    # inst.startDSClient() # recommended if running on DS computer; this gets the robot IP from the DS
    inst.setServer("127.0.0.1")

    persistence(inst)