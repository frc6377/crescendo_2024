#!/usr/bin/env python3

import ntcore
import time

TEAM = 6377

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("example client")
    # inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
    # inst.startDSClient() # recommended if running on DS computer; this gets the robot IP from the DS
    inst.setServer("127.0.0.1")

    table = inst.getTable("datatable")