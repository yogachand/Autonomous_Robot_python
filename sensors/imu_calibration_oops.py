#!/usr/bin/env python3
from sbp.client.drivers.network_drivers import TCPDriver  # ethernet connection tool
from sbp.client import Handler, Framer                    # message reader tools
from sbp.imu import MsgImuRaw                             # IMU message blueprints
import time       
import math         
import json
import numpy as np 

def Imu_connection(self,host,port):
    self.host = host 
    self.port = port 


try:
    with TCPDriver(host)


