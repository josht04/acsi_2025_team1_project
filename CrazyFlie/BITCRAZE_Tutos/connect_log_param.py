import logging
import time

import cflib.crtp       # Used for for scanning for Crazyflies instances.
from cflib.crazyflie import Crazyflie # Used to easily connect/send/receive data from a Crazyflie.
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie # Wrapper around the “normal” Crazyflie class. It handles the asynchronous nature of the Crazyflie API and turns it into blocking function.
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
#LogConfig class is a representation of one log configuration that enables logging from the Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger
#The SyncLogger class provides synchronous access to log data from the Crazyflie.

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Read and set parameters settings
def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    cf.param.add_update_callback(group=groupstr, name=namestr,
                                           cb=param_stab_est_callback) # Read params
    time.sleep(1) # give script a bit more time to wait for CF response & not lose connection immediately
    cf.param.set_value(full_name,2) # Set params
    time.sleep(1)
    cf.param.set_value(full_name,1) # Reset to original
    time.sleep(1)
"""
^ What it can’t do is to set a Read Only (RO) parameter, only Read Write (RW) parameters, which can be checked by the parameter TOC in the CFclient. 
You can check this by changing the parameter name to group 'CPU' and name flash'. Then you will get the following error:

AttributeError: cpu.flash is read-only!
"""

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

# Asynchronous Logging (in a callback independently of the main loop-rate)
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

# Synchronous Logging (directly in the loop)
def simple_log(scf, logconf):
    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            break


def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")



if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Log variables we wanna read out. To check variables name: cfclient > log TOC tab
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    group = "stabilizer"
    name = "estimator"

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        #simple_connect()
        #simple_log(scf, lg_stab)
        #simple_log_async(scf, lg_stab) 
        simple_param_async(scf, group, name)