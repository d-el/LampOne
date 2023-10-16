#!/usr/bin/env python3
# read and write parameters and calibration
# ./lampone.py /dev/ttyUSB0 --setc=1000 --save

from pymodbus.client.sync import ModbusSerialClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
#from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.compat import iteritems
from collections import OrderedDict
import argparse
import time
import csv
import os
import subprocess

parser = argparse.ArgumentParser(description="LampOne calibration util")
parser.add_argument("serial", help="Serial port name")
parser.add_argument("-a", "--address", type=int, default=1, help="Slave device address, 0 is broadcast")
parser.add_argument("--setaddress", type=int, help="Set device slave address from broadcast")
parser.add_argument("--setc", type=int, help="Set current")
parser.add_argument("--save", help="Save current", action="store_true")
parser.add_argument("--setminv", type=int, help="Set minimum voltage")
parser.add_argument("--calc", type=int, help="Calibrate current")
parser.add_argument("-w", help="Wait", action="store_false")
args = parser.parse_args()

print("connection to {}, address {}".format(args.serial, args.address))
client = ModbusSerialClient(method='rtu', port=args.serial, baudrate=115200, debug=True)

if args.setaddress is not None:
	print("Set slave {}".format(args.setaddress))
	client.write_register(0x003, args.setaddress, unit=0)
	exit(0)

if args.setc is not None:
	print("Set current {c} mA".format(c=args.setc))
	client.write_register(0x0100, args.setc, unit=args.address)

if args.save:
	print("Save current")
	client.write_register(0x0101, 1, unit=args.address)

if args.setminv is not None:
	print("Set minimum voltage {c} mV".format(c=args.setminv))
	client.write_register(0x0102, args.setminv, unit=args.address)

if args.calc is not None:
	print("Set calibration current {c} mA".format(c=args.calc))
	client.write_register(0x0300, args.calc, unit=args.address)

v = client.read_holding_registers(0x0000, 4, unit=args.address)
print("Version {major}.{minor}.{patch}, address {address}".format(major=v.registers[0], minor=v.registers[1], patch=v.registers[2], address=v.registers[3]))
	
while True:
	r = client.read_holding_registers(0x0100, 3, unit=args.address)
	print("Target:\n\t" "setcurrent {setcurrent} mA\n\tmin_input_voltage {minv} mV".format(setcurrent=r.registers[0], minv=r.registers[2]))

	s = client.read_holding_registers(0x200, 5, unit=args.address)
	stringstatus = ""
	status = s.registers[4]
	if status == 0:
		stringstatus = " ok"
	if status & 0x01:
		stringstatus = " overheated"
	if status & 0x02:
		stringstatus += " lowInputVoltage"
	print("State:\n\t" "current {c} mA\n\tinput_voltage {iv} mV\n\ttemperature {t} °C\n\tstatus:{s}".format(
		c=s.registers[0], p=s.registers[1], iv=s.registers[2], t=s.registers[3] / 10.0, s=stringstatus))

	c = client.read_holding_registers(0x0400, 1, unit=args.address)
	print("adcCurrent {c} lsb".format(c=c.registers[0]))
	
	if args.w:
		exit(0)
	time.sleep(0.3)


