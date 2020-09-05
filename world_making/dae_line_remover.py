# -*- coding: utf-8 -*-
"""
Created on Wed July 20 23:01 2020

@author: Mason EungChang Lee
"""

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def remove_lines_from_dae(dae):
    file_ = open(dae, 'r')
    lines = file_.readlines()
    file_.close()

    file_old = open('backup_'+dae, 'w')
    file_old.writelines(lines)

    idx = []
    for i, line in enumerate(lines):
        if 'lines ' in line:
            idx.append(i)
    for j in idx:
        lines[j]=''
        lines[j+1]=''
        lines[j+2]=''
        lines[j+3]=''
    file_new = open(dae, 'w')
    file_new.writelines(lines)
    file_new.close()
            

if __name__ == '__main__':
#    x = raw_input('input dae file name (in same directory) : ')
    remove_lines_from_dae('model.dae')
