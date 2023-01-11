#!/usr/bin/python

"""
    read or unpack Pandar XT-32 LiDAR data
    usage:
        ./XT32_Capture.py <read | unpack> [bin file dir]
"""

import os
import csv
import sys
import socket
import glob
from datetime import datetime, timedelta
import struct
import time
import traceback
import numpy as np
from multiprocessing import Process, Queue, Pool

import logging
import logging.config
import open3d as o3d

from dataclasses import dataclass

# ============= HESAI Pandar XT-32 Specification ============= #

HOST = "192.168.1.201"
PORT = 2368

# LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
LASER_ANGLES = [15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15, -16]
NUM_LASERS = 32

EXPECTED_PACKET_TIME = 0.001327     # Firing Packet Rate (100Mbps?)
EXPECTED_SCAN_DURATION = 0.1        # 10Hz
DISTANCE_RESOLUTION = 0.004         # 0.002
ROTATION_RESOLUTION = 0.01          # 1(degree)
ROTATION_MAX_UNITS = 36000

# ============= Binary Definition ============= #
# TimeStamp_Sensor
WIN_TIMESTAMP_SIZE = 17

# Head
XT_HEAD_SIZE = 12

# Body
XT_BLOCK_NUMBER = 8
XT_BLOCK_HEADER_AZIMUTH = 2
XT_UNIT_NUM = 32
XT_UNIT_SIZE = 4
XT_BLOCK_SIZE = XT_UNIT_SIZE * XT_UNIT_NUM + XT_BLOCK_HEADER_AZIMUTH
XT_BODY_SIZE = XT_BLOCK_SIZE * XT_BLOCK_NUMBER
XT_DUAL_BLOCK_SIZE = 4
XT_SINGLE_BLOCK_SIZE = 8
XT_DUAL_BLOCK_RES = 2
XT_SINGLE_BLOCK_RES = 1
XT_AZIMUTH_SIZE = 2
XT_DISTANCE_SIZE = 2
XT_XYZI_SIZE = 4


# Tail
XT_RESERVED_SIZE = 10
XT_ENGINE_VELOCITY = 2
XT_UTC_SIZE = 6
XT_TIMESTAMP_SIZE = 4
XT_ECHO_SIZE = 1
XT_FACTORY_SIZE = 1
XT_SEQUENCE_SIZE = 4
XT_TAIL_SIZE = 28

# All
XT_DATA_SIZE = XT_HEAD_SIZE + XT_BODY_SIZE + XT_TAIL_SIZE
XT_PACKET_SIZE = XT_DATA_SIZE + WIN_TIMESTAMP_SIZE

SOP = 2
PROTOCOL = 4

RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0

# Reflectivity Mapping(0~255 -> %)
REFLECT_MAP = {
    0:0.0, 1:2.89, 2:4.08, 3:5.0, 4:5.77, 5:6.45, 6:7.07, 7:7.64, 8:8.16, 9:8.66, 10:9.13,
    11:9.57, 12:10.0, 13:10.41, 14:10.8, 15:11.18, 16:11.55, 17:11.9, 18:12.25, 19:12.58, 20:12.91,
    21:13.23, 22:13.54, 23:13.84, 24:14.14, 25:14.43, 26:14.72, 27:15.0, 28:15.28, 29:15.57, 30:15.86,
    31:16.16, 32:16.46, 33:16.77, 34:17.09, 35:17.42, 36:17.75, 37:18.1, 38:18.45, 39:18.82, 40:19.2,
    41:19.59, 42:20.0, 43:20.43, 44:20.87, 45:21.34, 46:21.84, 47:22.36, 48:22.93, 49:23.55, 50:24.23,
    51:25.0, 52:25.92, 53:27.09, 54:28.22, 55:29.35, 56:30.47, 57:31.6, 58:32.73, 59:33.86, 60:34.99,
    61:36.12, 62:37.25, 63:38.37, 64:39.5, 65:40.63, 66:41.76, 67:42.89, 68:44.02, 69:45.15, 70:46.28,
    71:47.4, 72:48.53, 73:49.66, 74:50.79, 75:51.92, 76:53.05, 77:54.18, 78:55.3, 79:56.43, 80:57.56,
    81:58.69, 82:59.82, 83:60.95, 84:62.08, 85:63.21, 86:64.33, 87:65.46, 88:66.59, 89:67.72, 90:68.85,
    91:69.98, 92:71.11, 93:72.23, 94:73.36, 95:74.49, 96:75.62, 97:76.65, 98:77.88, 99:79.01, 100:80.14,
    101:81.26, 102:82.39, 103:83.52, 104:84.65, 105:85.78, 106:86.91, 107:88.04, 108:89.16, 109:90.29, 110:91.42,
    111:92.55, 112:93.68, 113:94.81, 114:95.94, 115:97.07, 116:98.19, 117:99.32, 118:100.45, 119:101.58, 120:102.71,
    121:103.84, 122:104.97, 123:106.09, 124:107.22, 125:108.35, 126:109.48, 127:110.61, 128:111.74, 129:112.87, 130:114.0,
    131:115.12, 132:116.25, 133:117.38, 134:118.51, 135:119.64, 136:120.77, 137:121.9, 138:123.02, 139:124.15, 140:125.28,
    141:126.41, 142:127.54, 143:128.67, 144:129.8, 145:130.93, 146:132.05, 147:133.18, 148:134.31, 149:135.44, 150:136.57,
    151:137.7, 152:138.83, 153:139.95, 154:141.08, 155:142.21, 156:143.34, 157:144.47, 158:145.6, 159:146.73, 160:147.86,
    161:148.9, 162:150.11, 163:151.24, 164:152.37, 165:153.5, 166:154.63, 167:155.76, 168:156.88, 169:158.01, 170:159.14,
    171:160.27, 172:161.4, 173:162.53, 174:163.66, 175:164.79, 176:165.91, 177:167.04, 178:168.17, 179:169.3, 180:170.43,
    181:171.6, 182:172.69, 183:173.81, 184:174.94, 185:176.07, 186:177.2, 187:178.33, 188:179.46, 189:180.59, 190:181.72,
    191:182.84, 192:183.97, 193:185.1, 194:186.23, 195:187.36, 196:188.49, 197:189.62, 198:190.74, 199:191.87, 200:193.0,
    201:194.13, 202:195.26, 203:196.39, 204:197.52, 205:198.65, 206:199.77, 207:200.9, 208:202.03, 209:203.16, 210:204.29,
    211:205.42, 212:206.55, 213:207.67, 214:208.8, 215:209.93, 216:211.06, 217:212.19, 218:213.32, 219:214.45, 220:215.58,
    221:216.7, 222:217.83, 223:218.96, 224:220.09, 225:221.22, 226:222.35, 227:223.48, 228:224.6, 229:225.73, 230:226.86,
    231:227.99, 232:229.12, 233:230.25, 234:231.38, 235:232.51, 236:233.63, 237:234.76, 238:235.89, 239:237.02, 240:238.15,
    241:239.28, 242:240.41, 243:241.53, 244:242.66, 245:243.79, 246:244.92, 247:246.05, 248:247.18, 249:248.31, 250:249.44,
    251:250.56, 252:251.69, 253:252.82, 254:253.95, 255:255.08
}

# ============= Data Logging ============= #
DATA_QUEUE = Queue(-1)


formatter = '[%(asctime)s][%(filename)s:%(lineno)s][%(levelname)s][%(message)s]'

LOGGING_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,  # this fixes the problem

    'formatters': {
        'standard': {
            'format': formatter,
        },
    },
    'handlers': {
        'default': {
            'level': 'DEBUG',
            'class': 'logging.StreamHandler',
            'formatter': 'standard'
        },
        "debug_file_handler": {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": "DEBUG",
            "formatter": "standard",
            "filename": "./lidar.log",
            "when": "D",
            "interval": 1,
            "backupCount": 30,
            "encoding": "utf8"
        },
    },
    'loggers': {
        '': {
            'handlers': ["default", 'debug_file_handler'],
            'level': 'DEBUG',
            'propagate': False
        },
    }
}
    

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("")

def save_csv(path, data):
    with open(path, 'w') as fp:
        wr = csv.writer(fp, delimiter=',')
        wr.writerows(data)

def calc(_azimuth, _dis, _reflectivity, _laser_id):
    R = _dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[_laser_id] * DEG2RAD
    alpha = _azimuth / 100.0 * DEG2RAD
    # print("R:", R, "omega:", omega * RAD2DEG, "alpha", alpha * RAD2DEG)
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    reflect = REFLECT_MAP[_reflectivity]
    return [X, Y, Z, reflect]

def unpack(dirs):
    files = glob.glob(dirs + '/*.bin')
    cnt=0
    for x in files:
        d = open(x, 'rb').read()
        n = len(d)
        
        
        for offset in range(WIN_TIMESTAMP_SIZE, n, XT_PACKET_SIZE):
               

            # Definition block, azimuth, distance, reflection
            Block = [[0 for col in range(XT_BLOCK_SIZE)] for row in range(XT_BLOCK_NUMBER)]
            azimuth = [0 for row in range(XT_SINGLE_BLOCK_SIZE)]
            distance = [[0 for col in range(XT_UNIT_NUM)] for row in range(XT_SINGLE_BLOCK_SIZE)]
            reflection = [[0 for col in range(XT_UNIT_NUM)] for row in range(XT_SINGLE_BLOCK_SIZE)]
            xyzi = np.empty((0,4), float)

            # Data Packet
            data = d[offset : offset + XT_DATA_SIZE]
            
            # Header, Body, Tail Area
            Header = data[0 : XT_HEAD_SIZE]
            Body = data[XT_HEAD_SIZE : XT_HEAD_SIZE + XT_BODY_SIZE]
            Tail = data[XT_HEAD_SIZE + XT_BODY_SIZE : XT_HEAD_SIZE + XT_BODY_SIZE + XT_TAIL_SIZE]
            
            # for byte in Header:
            #     print(format(byte, "x"))
            
            # Body Block 1~8
            
            for i in range(XT_BLOCK_NUMBER):
                Block[i] = Body[i * XT_BLOCK_SIZE : (i+1) * XT_BLOCK_SIZE]
            
            # for i in range(XT_SINGLE_BLOCK_SIZE):
            #     for byte in Block[i*2]:
            #         print(format(byte, "x"), end=" ")
            #     print("\n")

            

            for i in range(XT_SINGLE_BLOCK_SIZE):
                # Azimuth Get
                azimuth[i] = struct.unpack_from("<H", Block[i*XT_SINGLE_BLOCK_RES], offset=0)[0]
                if azimuth[i] < 0:
                    azimuth[i] += 36000
                if azimuth[i] >= 36000:
                    azimuth[i] -= 36000
                
                for j in range(XT_UNIT_NUM):
                    distance[i][j] = struct.unpack_from("<H", Block[i*XT_SINGLE_BLOCK_RES], offset=XT_AZIMUTH_SIZE + j*4)[0]
                    reflection[i][j] = struct.unpack_from("<B", Block[i*XT_SINGLE_BLOCK_RES], offset=XT_AZIMUTH_SIZE + XT_DISTANCE_SIZE + j*4)[0]
            
            # Calculation XYZI
            xyzi = np.empty((0,4), float)
            for i in range(XT_SINGLE_BLOCK_SIZE):
                _xyzi = []
                
                for j in range(XT_UNIT_NUM):
                    if distance[i][j] != 0.0:
                        _xyzi.append(calc(azimuth[i], distance[i][j], reflection[i][j], j))

                xyzi = np.append(xyzi, _xyzi)
            
            xyzi= np.reshape(xyzi, (-1, 4))
            # print(xyzi)
            # print("\n")

            file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
            path = datetime.now().strftime(file_fmt)
            try:
                if os.path.exists(path) is False:
                    os.makedirs(path)
            except Exception as e:
                    print (e)
            # if not list(xyzi):
            #     timestamp_str = '%.6f' % time.time()
            # else:
            save_csv("{}/frame{}.csv".format(path, cnt), xyzi)
            logger.info("{}/frame{}.csv".format(path, cnt))
            
            cnt += 1
        
  
# def unpack(dirs):
#     files = glob.glob(dirs + '/*.bin')
#     points = []
#     scan_index = 0
#     prev_azimuth = None
#     for x in files:
#         d = open(x, 'rb').read()
#         n = len(d)
#         for offset in xrange(0, n, 1223):
#             ts = d[offset : offset + 17]
#             data = d[offset + 17 : offset + 1223]
#             print ts, len(data)
#             timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
#             assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
#             timestamp = float(ts)
#             seq_index = 0
#             for offset in xrange(0, 1200, 100):
#                 flag, azimuth = struct.unpack_from("<HH", data, offset)
#                 assert flag == 0xEEFF, hex(flag)
#                 for step in xrange(2):
#                     seq_index += 1
#                     azimuth += step
#                     azimuth %= ROTATION_MAX_UNITS
#                     if prev_azimuth is not None and azimuth < prev_azimuth:
#                         file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
#                         path = datetime.now().strftime(file_fmt)
#                         try:
#                             if os.path.exists(path) is False:
#                                 os.makedirs(path)
#                         except Exception, e:
#                             print e
#                         if not points:
#                             timestamp_str = '%.6f' % time.time()
#                         else:
#                             timestamp_str = '%.6f' % points[0][3]
#                         csv_index = '%08d' % scan_index
#                         save_csv("{}/i{}_{}.csv".format(path, csv_index, timestamp_str), points)
#                         logger.info("{}/i{}_{}.csv".format(path, csv_index, timestamp_str))
#                         scan_index += 1
#                         points = []
#                     prev_azimuth = azimuth
#                     # H-distance (2mm step), B-reflectivity (0
#                     arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
#                     for i in xrange(NUM_LASERS):
#                         time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
#                         if arr[i * 2] != 0:
#                             points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))


def save_package(dirs, data_queue):
    try:
        if os.path.exists(dirs) is False:
            os.makedirs(dirs)
        cnt = 0
        fp = None
        while True:
            if data_queue.empty():
                pass
            else:
                msg = data_queue.get()
                # print(msg)
                data = msg['data']
                ts = msg['time']
                print('time stamps:', ts, 'data size:', len(data), 'queue size:', data_queue.qsize(), 'count:', cnt)
                if fp == None or cnt == 1000000:
                    if fp != None:
                        fp.close()
                    file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
                    path = str(datetime.now().strftime(file_fmt)) + '.bin'
                    logger.info('save to' + path)
                    print('save to ', path)
                    fp = open(path, 'ab')
                    cnt = 0
                cnt += 1
                fp.write(b'%.6f' % ts)
                fp.write(data)

    except KeyboardInterrupt as e:
        print(e)
    finally:
        if fp != None:
            fp.close()

def capture(port, data_queue):
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', port))
    try:
        while True:
            try:
                data = soc.recv(2000)
                if len(data) > 0:
                    assert len(data) == 1080, len(data)
                    data_queue.put({'data': data, 'time': time.time()})
            except Exception as e:
                print(dir(e), e.message, e.__class__.__name__)
                traceback.print_exc(e)
    except KeyboardInterrupt as e:
        print(e)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    if sys.argv[1] == 'read':
        top_dir = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        processA = Process(target = capture, args = (PORT, DATA_QUEUE))
        processA.start()
        processB = Process(target = save_package, args = (sys.argv[2] + '/' + top_dir, DATA_QUEUE))
        processB.start()
    else:
        unpack(sys.argv[2])
