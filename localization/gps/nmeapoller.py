"""
nmeapoller.py
This example illustrates a simple implementation of a
'pseudo-concurrent' threaded NMEAMessage message
polling utility.
(NB: Since Python implements a Global Interpreter Lock (GIL),
threads are not truly concurrent.)
It connects to the receiver's serial port and sets up a
NMEAReader read thread. With the read thread running
in the background, it polls for a variety of NMEA
messages. The read thread reads and parses any responses
to these polls and outputs them to the terminal.
If a given NMEA message is not supported by your device,
you'll see a '<GNTXT...NMEA unknown msg>' response.
Created on 7 Mar 2021
@author: semuadmin
"""
# pylint: disable=invalid-name

from sys import platform
from io import BufferedReader
from threading import Thread, Lock
from time import sleep
from serial import Serial
import argparse
import datetime
import os
from pynmeagps import (
    NMEAMessage,
    NMEAReader,
    POLL,
    NMEA_MSGIDS,
)

# initialise global variables
reading = False

def parse_args():
	parser = argparse.ArgumentParser()
	parser.add_argument('-v',
						help='Verbose. Set this flag to print readings as the script executes',
						dest='verbose',
						action='store_true')
	return parser.parse_args()

def read_messages(stream, lock, nmeareader, verbose):
    """
    Reads, parses and prints out incoming UBX messages
    """
    # pylint: disable=unused-variable, broad-except

    # Find timestamp (for log)
    dt = datetime.datetime.now()
    dt = str(dt).replace(' ', '_')

    log = 'log/GPS/GPS_log_{}.txt'.format(dt)
    raw_log = 'log/GPS/nmea_{}.txt'.format(dt)
    dump_output = 'out/gps.txt'

    if not os.path.exists('log'):
        os.mkdir('log')
    if not os.path.exists('out'):
        os.mkdir('out')

    if not os.path.exists('log/GPS'):
        os.mkdir('log/GPS')

    # Init readings
    lat = 0.0
    lon = 0.0
    speed_mps = 0.0

    while reading:
        if stream.in_waiting:
            try:
                lock.acquire()
                (raw_data, parsed_data) = nmeareader.read()
                lock.release()
                if parsed_data:
                    if parsed_data._msgID in ['RMC','GLL','GGA']:
                        NS = parsed_data.NS
                        EW = parsed_data.EW
                        lat = abs(parsed_data.lat)
                        lon = abs(parsed_data.lon)
                    elif parsed_data._msgID in ['VTG']: # VGA
                        speed_mps = float(parsed_data.sogk)/3.6

                    simple_str = '{:.7f} {}, {:.7f} {}, {:.7f} mps'.format(lat, NS, lon, EW, speed_mps)
                    if verbose:
                        print(simple_str)
                        print('\t{}'.format(parsed_data))

                    with open(log, 'a') as f:
                        f.write('{}\n'.format(datetime.datetime.now()))
                        f.write(simple_str)
                        f.write('\n\n')
                        f.close()
                    with open(raw_log, 'a') as f:
                        f.write(str(parsed_data))
                        f.write('\n')
                        f.close()    
                    with open(dump_output, 'w') as f:
                        f.write(str(lat))
                        f.write('\n')
                        f.write(str(lon))
                        f.write('\n')
                        f.write(str(speed_mps))
                        f.close()
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                continue


def start_thread(stream, lock, nmeareader, verbose):
    """
    Start read thread
    """

    thr = Thread(target=read_messages, args=(stream, lock, nmeareader, verbose), daemon=True)
    thr.start()
    return thr


def send_message(stream, lock, message):
    """
    Send message to device
    """

    lock.acquire()
    stream.write(message.serialize())
    lock.release()


if __name__ == "__main__":
    args = parse_args()

    # set port, baudrate and timeout to suit your device configuration
    if platform == "win32":  # Windows
        port = "COM13"
    elif platform == "darwin":  # MacOS
        port = "/dev/tty.usbmodem101"
    else:  # Linux
        port = "/dev/ttyACM1"
    baudrate = 38400
    timeout = 0.05

    with Serial(port, baudrate, timeout=timeout) as serial:

        # create NMEAReader instance
        nmr = NMEAReader(BufferedReader(serial))

        print("\nStarting read thread...\n")
        reading = True
        serial_lock = Lock()
        read_thread = start_thread(serial, serial_lock, nmr, args.verbose)

        # DO OTHER STUFF HERE WHILE THREAD RUNS IN BACKGROUND...
        MSGIDS = ['GNRMC', 'GNGLL', 'GNGGA']
        for msgid in NMEA_MSGIDS:
            #print(f"\n\nSending a GNQ message to poll for an {msgid} response...\n\n")
            msg = NMEAMessage("EI", "GNQ", POLL, msgId=msgid)
            #print(msg)
            send_message(serial, serial_lock, msg)
            #if msg.msgID in MSGIDS:
            #print(msg.msgID)
            sleep(1)

        print("\nPolling complete. Pausing for any final responses...\n")
        sleep(1)
        print("\nStopping reader thread...\n")
        reading = False
        read_thread.join()
        print("\nProcessing Complete")