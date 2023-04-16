'''
Created on 2019. 6. 17.

@author: YJHeo
'''
debugging = False


import socket
import sys
import numpy as np
import math as m
from time import sleep
import json

from ctypes import *
from threading import Lock
import threading
from calibrator.rotation_utils import *

###############################################################################
# Robot Interface                                                             #
###############################################################################
SIZE_HEADER = 52
SIZE_COMMAND = 4
SIZE_HEADER_COMMAND = 56
SIZE_DATA_TCP_MAX  = 200
SIZE_DATA_MAX = 200
SIZE_DATA_ASCII_MAX = 32
SIZE_PACKET = 256

###############################################################################
# Pickit definitions                                                          #
###############################################################################

PICKIT_INTERFACE_VERSION = 11
ROBOT_TYPE = 5  # 5: KUKA(z-y'-x") // 3:STAUBLI(x-y'-z")


RC_PICKIT_NO_COMMAND         = -1
RC_PICKIT_CHECK_MODE         = 0
RC_PICKIT_FIND_CALIB_PLATE   = 10
RC_PICKIT_LOOK_FOR_OBJECTS   = 20
RC_PICKIT_NEXT_OBJECT        = 30
RC_PICKIT_CONFIGURE          = 40
RC_PICKIT_SAVE_SCENE         = 50

PICKIT_UNKNOWN_COMMAND        =   -99
PICKIT_RUNNING_MODE           =     0
PICKIT_IDLE_MODE              =     1
PICKIT_CALIBRATION_MODE       =     2

PICKIT_FIND_CALIB_PLATE_OK     =   10
PICKIT_FIND_CALIB_PLATE_FAILED =   11
PICKIT_OBJECT_FOUND            =   20
PICKIT_NO_OBJECTS              =   21
PICKIT_NO_IMAGE_CAPTURED       =   22
PICKIT_EMPTY_ROI               =   23

PICKIT_CONFIG_OK               =   40
PICKIT_CONFIG_FAILED           =   41
PICKIT_SAVE_SCENE_OK           =   50
PICKIT_SAVE_SCENE_FAILED       =   51

PICKIT_TYPE_SQUARE             =  21
PICKIT_TYPE_RECTANGLE          =  22
PICKIT_TYPE_CIRCLE             =  23
PICKIT_TYPE_ELLIPSE            =  24
PICKIT_TYPE_CYLINDER           =  32
PICKIT_TYPE_SPHERE             =  33
PICKIT_TYPE_POINT_CLOUD        =  35  # See remark below for Teach on 1.9+ versions.
PICKIT_TYPE_BLOB               =  50


###############################################################################
# C-type Data: Pickit                                                         #
###############################################################################
class MetaData(Structure):
    _fields_ = [("robot_type", c_int32),
                ("interface_version", c_int32)]


class RobotToPickitData(Structure):
    _pack_ = 1
    _fields_ = [("actual_pose", c_int32 * 7),
                ("command", c_int32),
                ("setup", c_int32),
                ("product", c_int32),
                ("meta", MetaData)]

class PickitToRobotData(Structure):
    _pack_ = 1
    _fields_ = [("object_pose", c_int32 * 7),
                ("object_age", c_int32),
                ("object_type", c_int32),
                ("object_dimensions", c_int32 * 3),
                ("objects_remaining", c_int32),
                ("status", c_int32),
                ("meta", MetaData)]

###############################################################################
# C-type Data                                                                 #
###############################################################################

class HeaderCommandStruct(Structure):
    _pack_ = 1
    _fields_ = [("robotName", c_ubyte * 20),
                ("robotVersion", c_ubyte * 12),
                ("stepInfo", c_ubyte),
                ("sof", c_ubyte),
                ("invokeId", c_uint32),
                ("dataSize", c_uint32),
                ("status", c_ubyte * 4),
                ("reserved", c_ubyte * 6),
                ("cmdId", c_uint32)]


class HeaderCommand(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("val", HeaderCommandStruct)]


class Data(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("asciiStr", c_ubyte * (SIZE_DATA_ASCII_MAX + 1)),
                ("str", c_ubyte * 200),
                ("charVal", c_ubyte),
                ("boolVal", c_byte),
                ("shortVal", c_uint16),
                ("intVal", c_int32),
                ("floatVal", c_float),
                ("doubleVal", c_double),
                ("byteVal", c_ubyte),
                ("wordVal", c_ubyte * 2),
                ("uwordVal", c_ubyte * 2),
                ("dwordVal", c_ubyte * 4),
                ("lwordVal", c_ubyte * 8),
                ("bool6dArr", c_ubyte * 6),
                ("bool7dArr", c_ubyte * 7),
                ("boolArr", c_ubyte * 200),
                ("char2dArr", c_ubyte * 2),
                ("char3dArr", c_ubyte * 3),
                ("char6dArr", c_ubyte * 6),
                ("char7dArr", c_ubyte * 7),
                ("charArr", c_ubyte * 200),
                ("int2dArr", c_int32 * 2),
                ("int3dArr", c_int32 * 3),
                ("int6dArr", c_int32 * 6),
                ("int7dArr", c_int32 * 7),
                ("intArr", c_int32 * 50),
                ("float3dArr", c_float * 3),
                ("float6dArr", c_float * 6),
                ("float7dArr", c_float * 7),
                ("floatArr", c_float * 50),
                ("double3dArr", c_double * 3),
                ("double6dArr", c_double * 6),
                ("double7dArr", c_double * 7),
                ("doubleArr", c_double * 50),
                ("byteArr", c_ubyte * 200),
                ("wordArr", c_ubyte * 2*100),
                ("uwordArr", c_ubyte * 2*100),
                ("dwordArr", c_ubyte * 4*50),
                ("lwordArr", c_ubyte * 8*25)]

###############################################################################
# Debug                                                                       #
###############################################################################
def dump_buf(msg, buf, length) :
    if debugging:
        print (msg)
        for i in range (0, length):
            print (i, end=' - ')
            print (buf[i])

###############################################################################
# Decorators                                                                  #
###############################################################################
def tcp_command(cmd):
    def decorate(func):
        def decorated(self):
            self._handle_command(cmd)
            return func(self)
        return decorated
    return decorate


def tcp_command_rec(cmd, data_type):
    def decorate(func):
        def decorated(self):
            error_code, _res_data, _res_data_size = self._handle_command(cmd)
            r = func(self)
            if not error_code:
                return np.array(eval('_res_data.' + data_type))
            else:
                return error_code
        return decorated
    return decorate


def tcp_command_req(cmd, data_type, data_size):
    def decorate(func):
        def decorated(*args):
            _req_data = Data()
            _req_data_size = data_size

            if hasattr(args[1], '__len__'):
                for j in range(0, args[1].__len__()):
                    tmp_val = args[1][j]
                    exec('_req_data.' + data_type + '[j] = tmp_val')

            else:
                tmp_val = args[1]
                exec('_req_data.' + data_type + ' = tmp_val')

            args[0]._handle_command(cmd, _req_data, _req_data_size)
            return func(*args)
        return decorated
    return decorate


def tcp_command_req_rec(cmd, data_type_req, data_size, data_type_rec):
    def decorate(func):
        def decorated(*args):
            _req_data = Data()
            _req_data_size = data_size

            if hasattr(args[1], '__len__'):
                for j in range(0, args[1].__len__()):
                    tmp_val = args[1][j]
                    exec('_req_data.' + data_type_req + '[j] = tmp_val')
            else:
                tmp_val = args[1]
                exec('_req_data.' + data_type_req + ' = tmp_val')

            error_code, _res_data, _res_data_size = args[0]._handle_command(cmd, _req_data, _req_data_size)
            r = func(*args)
            if not error_code:
                return np.array(eval('_res_data.' + data_type_rec))
            else:
                return error_code
        return decorated
    return decorate



###############################################################################
# Indy Client Class                                                           #
###############################################################################
class PickitClient:
    def __init__(self, bind_ip, server_ip, server_port):
        self.__server_port  = server_port
        self.__lock = Lock()

        self.__is_connected = False

        self.time_out = 50
        self.v_invokeId = 0

        self.bind_ip = bind_ip
        self.server_ip = server_ip

        self.sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.__lock.acquire()
        self.sock_fd.bind((self.bind_ip, 0))
        try:
            self.sock_fd.connect((self.server_ip, self.__server_port))
        except socket.error as e:
            print("Socket connection error: {}".format(e))
            self.sock_fd.close()
            self.__lock.release()
            return False
        else:
            self.__lock.release()
            print("Connect: Bind IP ({bin_ip}), Server IP ({ser_ip})".format(bin_ip=self.bind_ip, ser_ip=self.server_ip))
            self.__is_connected = True
            return True

    def disconnect(self):
        self.__lock.acquire()
        self.sock_fd.close()
        self.__lock.release()
        self.__is_connected = False
        print("Disconnected")

    def shutdown(self):
        self.sock_fd.shutdown(socket.SHUT_RDWR)
        self.__is_connected = False
        print("Shut down")

    def is_connected(self):
        if not self.__is_connected:
            return False

        self.__lock.acquire()
        try:
            ret_val = self.sock_fd.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)
        except socket.error as e:
            print("{}".format(e))
            self.sock_fd.close()
            self.__lock.release()
            return False
        else:
            if ret_val != 0:
                print("Invalid Socket")
                return False
            self.__lock.release()
            return True

    def set_timeout_sec(self, time_out):
        if time_out < 0:
            print("Invalid time out setting: {}<0".format(time_out))
        self.time_out = time_out

    def _send_message(self, buf, size):
        dump_buf("SendBytes: ", buf, size)
        total_sent = 0
        while total_sent < size:
            self.sock_fd.settimeout(self.time_out)
            sent = self.sock_fd.send(buf[total_sent:size])
            if sent == -1:
                print('Error: sent == -1')
                return -1
            elif sent == 0:
                self.__lock.release()
                print('Error: sent == 0')
                return -1
            total_sent = total_sent + sent
        return 0

    def _recv_message(self, buf, size):
        chunks = []
        bytes_recd = 0
        while bytes_recd < size:
            self.sock_fd.settimeout(self.time_out)
            chunk = self.sock_fd.recv(size - bytes_recd)
            if chunk == b'':
                print('Error: receive error')
                memset (buf, 0, sizeof (buf))
                self.__lock.release()
                self.shutdown()
                return -1
            chunks.append(chunk)
            if (bytes_recd + len(chunk)) > sizeof (buf):
                break
            bytes_recd += len(chunk)
        data = b''.join(chunks)
        memset(buf, 0, sizeof (buf))
        memmove(buf, data, len(data))
        return buf

    def _handle_command(self, cmd, req_data=RobotToPickitData(), req_data_size=0):
        self.__lock.acquire()

        write_buffer = (c_char* 1024)()
        read_buffer = (c_char* 1024)()

        if req_data_size > SIZE_DATA_TCP_MAX or req_data_size < 0:
            self.__lock.release()
            self.disconnect()
            raise Exception("Request size is invalid {}: Disconnected".format(req_data_size))

        # Send packet to socket
        if req_data_size > 0:
            memmove(write_buffer, req_data.byte, sizeof(req_data))
            self._send_message(write_buffer, sizeof(req_data))


        # Recv data from Pickit
        res_data = PickitToRobotData()
        print('size res data', sizeof(res_data))
        read_buffer = self._recv_message(read_buffer, sizeof(res_data))
        memmove(res_data.byte, read_buffer, sizeof(res_data))

        self.__lock.release()

        return res_data

    ############################################################################
    ## Command function (Check all)                                     #
    ############################################################################
    def convert_pos(self, pos):
        MULT = 10000
        pos_tmp = np.array(pos.copy())
        pos_tmp[3:6] = pos_tmp[[5, 4, 3]]
        pos_tmp = pos_tmp.tolist()
        new_pos = []
        for p in pos_tmp:
            _p = socket.htonl(int(p * MULT) & 0xffffffff)
            new_pos.append(_p)
        return new_pos

    def ntohl_signed(self, num):
        recover = socket.ntohl(int(num) & 0xffffffff)
        if recover > 0x50000000:
            recover = recover - 0x100000000
        return recover

    def periodic(self, robot_pos):
        req_data = RobotToPickitData()
        active_pos = self.convert_pos(robot_pos)
        for i in range(0, len(active_pos)):
            req_data.actual_pose[i] = active_pos[i]

        req_data.command = socket.htonl(-1 & 0xffffffff)
        req_data.meta.robot_type = socket.htonl(ROBOT_TYPE & 0xffffffff)
        req_data.meta.interface_version = socket.htonl(PICKIT_INTERFACE_VERSION & 0xffffffff)

        self.__lock.acquire()
        write_buffer = (c_char * 1024)()
        req_data_size = sizeof(req_data)
        if req_data_size > SIZE_DATA_TCP_MAX:
            self.__lock.release()
            self.disconnect()
            raise Exception("Request size is invalid {}: Disconnected".format(req_data_size))

        # Send packet to socket
        if req_data_size > 0:
            memmove(write_buffer, req_data.actual_pose, req_data_size)
            self._send_message(write_buffer, req_data_size)
        self.__lock.release()

    def retrieve(self, robot_pos, cmd, pickit_setup=0, pickit_product=0):
        write_buffer = (c_char * 1024)()
        read_buffer = (c_char * 1024)()

        req_data = RobotToPickitData()
        active_pos = self.convert_pos(robot_pos)
        for i in range(0, len(active_pos)):
            req_data.actual_pose[i] = active_pos[i]

        req_data.command = socket.htonl(cmd & 0xffffffff)
        req_data.meta.robot_type = socket.htonl(ROBOT_TYPE & 0xffffffff)
        req_data.meta.interface_version = socket.htonl(PICKIT_INTERFACE_VERSION & 0xffffffff)

        req_data.setup = socket.htonl(pickit_setup & 0xffffffff)
        req_data.product = socket.htonl(pickit_product & 0xffffffff)

        self.__lock.acquire()
        req_data_size = sizeof(req_data)
        if req_data_size > SIZE_DATA_TCP_MAX:
            self.__lock.release()
            self.disconnect()
            raise Exception("Request size is invalid {}: Disconnected".format(req_data_size))

        # Send packet to socket
        if req_data_size > 0:
            memmove(write_buffer, req_data.actual_pose, req_data_size)
            self._send_message(write_buffer, req_data_size)

        # Recv data from Pickit
        res_data = PickitToRobotData()
        print('size res data', sizeof(res_data))
        read_buffer = self._recv_message(read_buffer, sizeof(res_data))
        memmove(res_data.object_pose, read_buffer, sizeof(res_data))
        self.__lock.release()
        return res_data


    ############################################################################
    ## Robot command function (Check all)                                     #
    ############################################################################
    def get_object_pos(self, indy):
        res = self.retrieve(indy.get_task_pos(), RC_PICKIT_LOOK_FOR_OBJECTS)
        MULT = 10000
        obj_pos = np.zeros((6,))
        obj_pos[0] = self.ntohl_signed(int(res.object_pose[0]) & 0xffffffff) / MULT
        obj_pos[1] = self.ntohl_signed(int(res.object_pose[1]) & 0xffffffff) / MULT
        obj_pos[2] = self.ntohl_signed(int(res.object_pose[2]) & 0xffffffff) / MULT
        obj_pos[3] = self.ntohl_signed(int(res.object_pose[5]) & 0xffffffff) / MULT
        obj_pos[4] = self.ntohl_signed(int(res.object_pose[4]) & 0xffffffff) / MULT
        obj_pos[5] = self.ntohl_signed(int(res.object_pose[3]) & 0xffffffff) / MULT

        objects_remaining = socket.ntohl(int(res.objects_remaining) & 0xffffffff)
        status = socket.ntohl(int(res.status) & 0xffffffff)

        return obj_pos, objects_remaining, status

    def get_next_object_pos(self, indy):
        res = self.retrieve(indy.get_task_pos(), RC_PICKIT_NEXT_OBJECT)
        MULT = 10000
        obj_pos = np.zeros((6,))
        obj_pos[0] = self.ntohl_signed(int(res.object_pose[0]) & 0xffffffff) / MULT
        obj_pos[1] = self.ntohl_signed(int(res.object_pose[1]) & 0xffffffff) / MULT
        obj_pos[2] = self.ntohl_signed(int(res.object_pose[2]) & 0xffffffff) / MULT
        obj_pos[3] = self.ntohl_signed(int(res.object_pose[5]) & 0xffffffff) / MULT
        obj_pos[4] = self.ntohl_signed(int(res.object_pose[4]) & 0xffffffff) / MULT
        obj_pos[5] = self.ntohl_signed(int(res.object_pose[3]) & 0xffffffff) / MULT

        objects_remaining = socket.ntohl(int(res.objects_remaining) & 0xffffffff)
        status = socket.ntohl(int(res.status) & 0xffffffff)

        return obj_pos, objects_remaining, status

    def request_config(self, indy, pickit_setup, pickit_prod):
        res = self.retrieve(indy.get_task_pos(), RC_PICKIT_CONFIGURE, pickit_setup, pickit_prod)
        status = socket.ntohl(int(res.status) & 0xffffffff)
        
        return status

    def check_object_found(self, status):
        return status == PICKIT_OBJECT_FOUND

    def check_mode(self, indy):
        res = self.retrieve(indy.get_task_pos(), RC_PICKIT_CHECK_MODE)
        status = socket.ntohl(int(res.status) & 0xffffffff)

        return status == PICKIT_RUNNING_MODE


    @tcp_command(1)
    def emergency_stop(self):
        pass

    @tcp_command(1)
    def reset_robot(self):
        pass

    @tcp_command_req(1, 'bool6dArr', 6 * 1)
    def set_servo_on_off(self, val_arr):
        pass

    @tcp_command_req(11, 'bool6dArr', 6 * 1)
    def set_brake_on_off(self, val_arr):
        pass

    @tcp_command(2)
    def stop_motion(self):
        pass

    def execute_move_command(self, cmd_name):
        _req_data = cmd_name.encode('ascii')
        _req_data_size = len(cmd_name)
        self._handle_command(CMD_MOVE, _req_data, _req_data_size)


    ############################################################################
    ## Tool teaching                                                           #
    ############################################################################
    def set_tool_offset(self, indy, obj_pos):
        filename = 'teaching/tool_config.json'

        task_pos = indy.get_task_pos()

        Tbe = cmd2T(task_pos)
        Tbo = cmd2T(obj_pos)

        Toe_grip = np.matmul(SE3_inv(Tbo), Tbe)
        tool_pos = T2cmd(Toe_grip)

        tool_config = {}
        tool_config['tool_offset'] = list(tool_pos)

        with open(filename, 'w') as fp:
            json.dump(tool_config, fp)

        with open(filename) as json_file:
            tool_config = json.load(json_file)

        return tool_config


    ############################################################################
    ## Execution                                                               #
    ############################################################################
    def get_pickit_pos(self, indy, height, tool_offset):
#         filename = 'teaching/tool_config.json'
        obj_pos, objects_remaining, status = self.get_object_pos(indy)

#         with open(filename) as json_file:
#             tool_config = json.load(json_file)

#         tool_offset = tool_config['tool_offset']

        Toe_pick = cmd2T(tool_offset)
        Tbo = cmd2T(obj_pos)

        R_above = Rot_zyx(0,0,0)
        P_above = np.array([0,0,height])
        Toabove = SE3(R_above, P_above)

        Tbo_above = np.matmul(np.matmul(Tbo, Toabove), Toe_pick)
        Tbo_pick = np.matmul(Tbo, Toe_pick)

        pick_pos = T2cmd(Tbo_pick)
        pick_above = T2cmd(Tbo_above)

        return pick_pos, pick_above, objects_remaining, status


###############################################################################
# Test                                                                        #
###############################################################################
if __name__ == '__main__':
    pass
