#!/usr/bin/env python

'''
Extract data at one airspeed from logs
'''
from __future__ import print_function

import os
import struct
from tqdm import tqdm
import cProfile
from argparse import ArgumentParser
from pymavlink import mavutil

parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--parameter", required=True, help="Required, parameter to use for parsing")
parser.add_argument("--value", required=True, help="Required, comma seperated parameter value(s)")
parser.add_argument("--link", default=None, type=int, help="only extract specific comms link")
parser.add_argument("--reduce-rate", type=float, default=0, help="reduce messages to maximum rate in Hz")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

os.system('cls' if os.name == 'nt' else 'clear')

last_msg_rate_t = {}

def reduce_rate_msg(m, reduction_rate, mtype):
    '''return True if this msg should be discarded by reduction'''
    global last_msg_rate_t
    if mtype in ['PARM','MSG','FMT','FMTU','MULT','MODE','EVT','UNIT', 'VER']:
        return False
    t = getattr(m,'_timestamp',None)
    if t is None:
        return False
    if not mtype in last_msg_rate_t:
        last_msg_rate_t[mtype] = t
    dt = t - last_msg_rate_t[mtype]

    if dt < 0 or dt >= 1.0/reduction_rate:
        last_msg_rate_t[mtype] = t
        return False
    return True

def older_message(m, lastm):
    '''return true if m is older than lastm by timestamp'''
    atts = {'time_boot_ms' : 1.0e-3,
            'time_unix_usec' : 1.0e-6,
            'time_usec' : 1.0e-6}
    for a in list(atts.keys()):
        if hasattr(m, a):
            mul = atts[a]
            t1 = m.getattr(a) * mul
            t2 = lastm.getattr(a) * mul
            if t2 >= t1 and t2 - t1 < 60:
                return True
    return False

def process(filename):
    '''process one logfile'''
    tqdm.write(f"Loading {filename} - this may take a moment")
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps,
                                robust_parsing=args.robust)

    ext = os.path.splitext(filename)[1]
    isbin = ext in ['.bin', '.BIN']
    islog = ext in ['.log', '.LOG']
    output = None
    dirname = os.path.dirname(filename)

    if isbin or islog:
        extension = "bin"
    else:
        extension = "tlog"

    file_header = bytearray()
    messages = []

    flightmode = None
    param_values = list(map(int, args.value.split(',')))
    tqdm.write(f"Parameter is {args.parameter}")
    tqdm.write(f"Value list is {param_values}")
    current_param_value = -1

    for value in param_values:
        path = os.path.join(dirname, "%s-%s=%4d.%s" % (filename.split('.')[0], args.parameter, value, extension))
        if os.path.exists(path):
            os.remove(path)

    while True:
        m = mlog.recv_match()
        if m is None:
            break
        if args.link is not None and m._link != args.link:
            continue
        
        mtype = m.get_type()
        #print("len", m.fmt.len)
        pbar.update(m.fmt.len)

        if mtype in messages:
            if older_message(m, messages[mtype]):
                continue
        
        if args.reduce_rate > 0 and reduce_rate_msg(m, args.reduce_rate, mtype):
            continue

        msg_buf = m.get_msgbuf()

        # we don't use mlog.flightmode as that can be wrong if we are extracting a single link
        if mtype == 'HEARTBEAT' and m.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_GIMBAL and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            flightmode = mavutil.mode_string_v10(m).upper()
            file_header += msg_buf
        if mtype == 'MODE':
            flightmode = mlog.flightmode
            file_header += msg_buf
        if mtype == 'PARM' and m.Name == args.parameter:
            current_param_value = int(m.Value)

        if flightmode == "TAKEOFF":
            file_header += msg_buf
        if (isbin or islog) and mtype in ["FMT", "PARM", "CMD", "FMTU", "MULT"]:
            file_header += msg_buf
        if (isbin or islog) and mtype == 'MSG' and m.Message.startswith("Ardu"):
            file_header += msg_buf
        if mtype in ['PARAM_VALUE','MISSION_ITEM','MISSION_ITEM_INT']:
            timestamp = getattr(m, '_timestamp', None)
            file_header += struct.pack('>Q', int(timestamp*1.0e6)) + msg_buf

        if not mavutil.evaluate_condition(args.condition, mlog.messages):
            continue

        if current_param_value in param_values:
            path = os.path.join(dirname, "%s-%s=%4d.%s" % (filename.split('.')[0], args.parameter, current_param_value, extension))
            if output is not None and output.name != path:
                output.close()
            if output is None or output.name != path:
                tqdm.write(f"Creating {path}")
                output = open(path, mode='ab')
                output.write(file_header)
            if mtype != 'BAD_DATA':
                if not isbin:
                    timestamp = getattr(m, '_timestamp', None)
                    output.write(struct.pack('>Q', int(timestamp*1.0e6)))
                output.write(msg_buf)

for filename in args.logs:
    with tqdm(total=os.stat(filename).st_size, unit='B', unit_scale=True) as pbar:
        #cProfile.run('process(filename)')
        process(filename)

