from subprocess import Popen, PIPE
import shlex
from time import sleep, strptime
import numpy as np
from numpy import average
import paramiko
from re import findall
from math import ceil, floor
from _thread import start_new_thread
import os
from scp import SCPClient
import asyncio, asyncssh


def local_cmd(command):
    stdout = Popen(command, shell=True, stdout=PIPE).stdout
    return stdout.read()


def ssh_connect(host):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username='ubuntu', password='**r0b0t**')
    return ssh


def ssh_cmd(command, ssh):
    output = ""
    stdout = ssh.exec_command(command)[1]
    for line in stdout:
        output += line
    if (len(output) > 0):
        return output


def log(file, entry):
    print(entry)
    file.write(entry)
    return



async def run_async(host, command):
    async with asyncssh.connect(host, username='ubuntu', password='**r0b0t**', known_hosts=None) as conn:
        return await conn.run(command)


setupConfig = 1  # 0 = test PIs, 1 = robomasters.

# [[ManagementIP, ADHocIP]...]. Management IP is the non-adhoc ip that can be reached by the host running this script
if setupConfig == 1:
    ipAssign = [['10.3.1.2', '10.3.2.2'], ['10.3.1.3', '10.3.2.3'], ['10.3.1.4', '10.3.2.4'], ['10.3.1.5', '10.3.2.5'], ['10.3.1.6', '10.3.2.6']]
else:
    ipAssign = [['192.168.2.101', '192.168.199.50'], ['192.168.2.102', '192.168.199.51'],
                ['192.168.2.103', '192.168.199.52'], ['192.168.2.104', '192.168.199.53'],
                ['192.168.2.105', '192.168.199.54']]


for [hostIp, ahIp] in ipAssign:
    print('Reseting firwall for ' + str(hostIp))
    ssh = ssh_connect(str(hostIp))
    result = ssh_cmd('sudo ufw --force reset', ssh)
    print('Firewall reset for ' + str(hostIp))
