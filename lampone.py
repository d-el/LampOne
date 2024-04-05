#!/usr/bin/env python3
# read and write parameters and calibration
# ./lampone.py /dev/ttyUSB0 --setc=1000 --save

import pymodbus.client as modbusClient
from pymodbus import ModbusException
from collections import OrderedDict
from datetime import datetime
import argparse
import time
import csv
import os
import subprocess
from colorama import Fore, Back, Style

parser = argparse.ArgumentParser(description="LampOne calibration util")
parser.add_argument("serial", help="Serial port name")
parser.add_argument("-a", "--address", type=int, default=1, help="Slave device address, 0 is broadcast")
parser.add_argument("--setaddress", type=int, help="Set device slave address from broadcast")
parser.add_argument("--setc", type=int, help="Set current")
parser.add_argument("--save", help="Save current", action="store_true")
parser.add_argument("--ulvo_voltage", type=int, help="Set ulvo voltage")
parser.add_argument("--ulvo_hysteresis", type=int, help="Set ulvo hysteresis")
parser.add_argument("--voltage_threshold", type=int, help="Set voltage_threshold")
parser.add_argument("--limit_max_current", type=int, help="Set limit_max_current")
parser.add_argument("--limit_min_current", type=int, help="Set limit_min_current")
parser.add_argument("--calc", type=int, help="Calibrate current")
parser.add_argument("-g", "--graph", help="Create graph", action="store_true")
parser.add_argument("-w", help="Wait", action="store_true")
args = parser.parse_args()

print("connection to {}, address {}".format(args.serial, args.address))
client = client = modbusClient.ModbusSerialClient(method='rtu', port=args.serial, baudrate=115200, debug=True)

def write_register(reg, val, slave_addr):
	result = client.write_register(reg, val, slave=slave_addr)
	if result.isError():
		print(Fore.RED + "Error write reg {:04X} val {}".format(reg, val) + Style.RESET_ALL)

if args.setaddress is not None:
	print("Set slave {}".format(args.setaddress))
	client.write_register(0x003, args.setaddress, slave=0)
	print("Your need reboot device")
	exit(0)

if args.setc is not None:
	print("Set current {c} mA".format(c=args.setc))
	client.write_register(0x0100, args.setc, slave=args.address)

if args.save:
	print("Save current")
	client.write_register(0x0101, 1, slave=args.address)

if args.ulvo_voltage is not None:
	print("Set ulvo_voltage {c} mV".format(c=args.ulvo_voltage))
	write_register(0x0102, args.ulvo_voltage, args.address)
	
if args.ulvo_hysteresis is not None:
	print("Set ulvo_hysteresis {c} mV".format(c=args.ulvo_hysteresis))
	client.write_register(0x0103, args.ulvo_hysteresis, slave=args.address)

if args.voltage_threshold is not None:
	print("Set voltage_threshold {c} mV".format(c=args.voltage_threshold))
	client.write_register(0x0104, args.voltage_threshold, slave=args.address)

if args.limit_max_current is not None:
	print("Set limit_max_current {c} mV".format(c=args.limit_max_current))
	client.write_register(0x0105, args.limit_max_current, slave=args.address)

if args.limit_min_current is not None:
	print("Set limit_min_current {c} mV".format(c=args.limit_min_current))
	client.write_register(0x0106, args.limit_min_current, slave=args.address)

if args.calc is not None:
	print("Set calibration current {c} mA".format(c=args.calc))
	client.write_register(0x0300, args.calc, slave=args.address)

v = client.read_holding_registers(0x0000, 4, slave=args.address)
print("Version {major}.{minor}.{patch}, address {address}".format(major=v.registers[0], minor=v.registers[1], patch=v.registers[2], address=v.registers[3]))

c = client.read_holding_registers(0x0300, 1, slave=args.address)
print("adcCurrent calibrate {c} mA".format(c=c.registers[0]))
c = client.read_holding_registers(0x0400, 1, slave=args.address)
print("adcCurrent calibrate {c} lsb".format(c=c.registers[0]))

# Create CSV
filename = ""
if args.graph:
	filename = str(datetime.today().strftime('%Y-%m-%d')) + ".csv"
	f = open(filename, 'w')
	writer = csv.writer(f, delimiter = ",")
	writer.writerow(["Time", "Current", "Temperature"]) # Write heater

while True:
	try:
		r = client.read_holding_registers(0x0100, 7, slave=args.address)
		print("Target:\n\t" "setcurrent {} mA".format(r.registers[0]))
		print("\tulvo_voltage {} mV".format(r.registers[2]))
		print("\tulvo_hysteresis {} mV".format(r.registers[3]))
		print("\tvoltage_threshold {} mV".format(r.registers[4]))
		print("\tlimit_max_current {} mA".format(r.registers[5]))
		print("\tlimit_min_current {} mA".format(r.registers[6]))
		
		s = client.read_holding_registers(0x200, 5, slave=args.address)
		stringstatus = ""
		status = s.registers[4]
		if status == 0:
			stringstatus = " ok"
		if status & 0x01:
			stringstatus = " overheated"
		if status & 0x02:
			stringstatus += " lowInputVoltage"
		if status & 0x04:
			stringstatus += " m_adcOverflow"
		
		current = s.registers[0]
		inputVoltage = s.registers[2]
		temperature = s.registers[3] / 10.0
		
		print("State:\n\t" "current {c} mA\n\tinput_voltage {iv} mV\n\ttemperature {t} °C\n\tstatus:{s}".format(
			c=current, iv=inputVoltage, t=temperature, s=stringstatus))
		
		if args.graph:		
			writer.writerow([current, temperature])
		
		if args.w is not True:
			exit(0)

		time.sleep(0.3)
	
	except KeyboardInterrupt:
		break
	except Exception as error:
		print("An error occurred:", error)

if args.graph:
	proc = subprocess.Popen(['gnuplot','--persist'], 
                shell=False,
                stdin=subprocess.PIPE,
                )

	proc.stdin.write(b"""
		set grid
		set y2tics
		set yrange [0:1500]
		set y2range [0:100]
		set ylabel \'Current\'
		set y2label \'Temperature\'
		set datafile separator ','
		""")
	proc.stdin.write(str.encode("""
		plot \
		\"{file}\" using 1 with lines axis x1y1 title \'Current\',\
		\"{file}\" using 2 with lines axis x1y2 title \'Temperature\'
		""".format(file=filename, title="filename")))
	proc.stdin.write(b"pause -1\n")
	proc.stdin.close()

