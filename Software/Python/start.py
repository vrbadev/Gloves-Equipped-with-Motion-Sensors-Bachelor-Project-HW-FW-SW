# -*- coding: utf-8 -*-
"""
Vizualization application for IMU gloves.

Created on Wed May  6 19:57:21 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""

from multiprocessing import Process, Pipe, Array
import signal
import traceback
import sys


APP_RUNNING = True        

def signal_handler(sig, frame):
    global APP_RUNNING
    print("Pressed CTRL + C!")
    APP_RUNNING = False

def visualizer_thread(pipe, shared_mem):
    from visualizer import Visualizer
    pipe.send("Imported visualizer package")
    try:
        v = Visualizer(pipe, shared_mem)
        pipe.send("Visualizer object created, running...")
        v.run()
    except Exception:
        try:
            exc_type, exc_value, exc_traceback = sys.exc_info()
        finally:
            pipe.send("ERROR in Visualizer: %s" % ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            del exc_type, exc_value, exc_traceback
            v.stop()
    
def ble_client_thread(pipe, shared_mem):
    from ble_client import BLEClient
    pipe.send("Imported ble_client package")
    try:
        BLEClient(pipe, shared_mem)
    except Exception:
        try:
            exc_type, exc_value, exc_traceback = sys.exc_info()
        finally:
            pipe.send("ERROR in BLEClient: %s" % ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            del exc_type, exc_value, exc_traceback

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    sys.setrecursionlimit(1500)
    
    parent_conn, child_conn = Pipe()
    shared_mem = Array('f', 15*3+4)
    
    p = Process(target=visualizer_thread, args=(child_conn, shared_mem))
    p.start()
    
    p2 = Process(target=ble_client_thread, args=(child_conn, shared_mem))
    p2.start()
    
    print("Threads running")
    
    while APP_RUNNING:
        r = parent_conn.recv()
        print(r)
        if r == "Window closed!" or "ERROR" in r:
            APP_RUNNING = False
    
    print("Ending thread of visualizer...")
    p.terminate()
    p.join()
    
    print("Ending thread of BLE client...")
    p2.terminate()
    p2.join()
    
    print("Program End.")
    input("Press Enter to continue...")
    
    
    
    