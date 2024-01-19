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
    ssh.connect(host,username='ubuntu',password='**r0b0t**')
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
    async with asyncssh.connect(host,username='ubuntu',password='**r0b0t**',known_hosts=None) as conn:
        return await conn.run(command)

setupConfig = 1 #0 = test PIs, 1 = robomasters.

# [[ManagementIP, ADHocIP]...]. Management IP is the non-adhoc ip that can be reached by the host running this script
if setupConfig == 1:
    ipAssign = [['10.3.1.2', '10.3.2.2'], ['10.3.1.3', '10.3.2.3'], ['10.3.1.4', '10.3.2.4'], ['10.3.1.5', '10.3.2.5'], ['10.3.1.6', '10.3.2.6']]
else:
    ipAssign = [['10.7.1.2', '10.7.2.2'],['10.7.1.3','10.7.2.3'],['10.7.1.4','10.7.2.4'],['10.7.1.5','10.7.2.5'],['10.7.1.6','10.7.2.6']]

for [hostIp, ahIp] in ipAssign:
    print('Configuring firewall; ' + str(hostIp) + ' as host IP, ' + str(ahIp) + ' as ad-hoc interface')
    ssh = ssh_connect(str(hostIp))
    result = ssh_cmd('sudo ufw allow in to any port 22', ssh)
    result = ssh_cmd('sudo ufw allow in proto udp from any to any port 123', ssh)
    result = ssh_cmd('sudo ufw allow out proto udp from any to any port 123', ssh)
    result = ssh_cmd('sudo ufw deny out protop tcp from any to 169.254.0.0/16', ssh)
    result = ssh_cmd('sudo ufw deny out protop udp from any to 169.254.0.0/16', ssh)
    result = ssh_cmd('sudo ufw --force enable', ssh)
    for [hostIp2, ahIp2] in ipAssign:
        if hostIp == hostIp2:
            continue
        result = ssh_cmd('sudo ufw deny in proto tcp from ' + hostIp2 + ' to any', ssh)
        result = ssh_cmd('sudo ufw deny in proto udp from ' + hostIp2 + ' to any', ssh)
        result = ssh_cmd('sudo ufw deny out proto tcp from any to '  + hostIp2, ssh)
        result = ssh_cmd('sudo ufw deny out proto udp from any to '  + hostIp2, ssh)
    result = ssh_cmd('sudo ufw allow in from any to any', ssh)
    print('Configured firwall for ' + str(hostIp))
