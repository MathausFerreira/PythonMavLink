###################################################################################################
####################### Serial Conexion via MAVLink Protocol with APM #############################
###################################################################################################
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# --------------------------------------
# Importing MAVLink libray conexion
# --------------------------------------
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

# --------------------------------------
# Initializing MAVLink conection - Master Object
# --------------------------------------
device = '/dev/ttyACM0'
baudrate = 115200  # To use USB Port
# baudrate = 57600  #To use Telemetry, this is the requested baudrate, which is commented.

master = mavutil.mavlink_connection(device, baud=baudrate)
# Waiting for heartbeat message from the APM board
conexao = master.wait_heartbeat()

# Requesting message transmissions
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL,4, 1)
# ----------------------------------------
# Classes and functions for the messages
# ----------------------------------------
# Buffer class to write and read, necessary to start the MAVLink dialect


class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)


# Create a buffer class
f = fifo()

# Create the dialect
mav = mavlink1.MAVLink(f)

# Variables used to save the received variables from the APM board
it_number = 1000

Theta_mot_1 = [0] * it_number
Theta_mot_2 = [0] * it_number
Theta_mot_3 = [0] * it_number
Theta_mot_4 = [0] * it_number

PWM1 = [0] * it_number
PWM3 = [0] * it_number
PWM2 = [0] * it_number
PWM4 = [0] * it_number

FT = [0] * it_number
FX = [0] * it_number
FY = [0] * it_number
N  = [0] * it_number


att_roll     = [0] * it_number
att_pitch    = [0] * it_number
att_yaw      = [0] * it_number
att_rollsdp  = [0] * it_number
att_pitchsdp = [0] * it_number
att_yawsdp   = [0] * it_number

pos_lat = [0] * it_number
pos_lon = [0] * it_number
pos_alt = [0] * it_number
pos_vx = [0] * it_number
pos_vy = [0] * it_number
pos_vz = [0] * it_number

PI      = 3.141592653589793
DEG2RAD = np.pi/180.0
RAD2DEG = 180.0/np.pi

# ------------------------------------------
# Main loop where the messages are exchanged
# ------------------------------------------

# Inicialização da figura

fig = plt.figure()
ax1 = plt.subplot(2,2,2)
ax2 = plt.subplot(2,2,3)
ax3 = plt.subplot(2,2,1)
ax4 = plt.subplot(2,2,4)

ln1, = ax1.plot([], [], 'b+-')
ln2, = ax2.plot([], [], 'b+-')
ln3, = ax3.plot([], [], 'b+-')
ln4, = ax4.plot([], [], 'b+-')


def init():
    ax1.set_xlim(-15, 15)
    ax1.set_ylim(-15, 15)
    ax1.set_title("Motor 1")

    ax2.set_xlim(-15, 15)
    ax2.set_ylim(-15, 15)
    ax2.set_title("Motor 2")

    ax3.set_xlim(-15, 15)
    ax3.set_ylim(-15, 15)
    ax3.set_title("Motor 3")

    ax4.set_xlim(-15, 15)
    ax4.set_ylim(-15, 15)
    ax4.set_title("Motor 4")

    return ln1, ln2, ln3, ln4


def update(frame):
    if conexao:
        for i in range(it_number):
            # Here, between parenthesis, is set up the amount of iterations this structure will run
            # Receiving messages and taking only the necessary
            msg_rcv_out = master.recv_match(type="SYS_STATUS", blocking=True)
            msg_rcv_inputs = master.recv_match(type="NAV_CONTROLLER_OUTPUT", blocking=True)
            attitude = master.recv_match(type="ATTITUDE", blocking=True)
            position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

            # Testing if the variables with received data exist
            if msg_rcv_inputs:
                # Recebendo Valores
                Theta_mot_1[i] = float(msg_rcv_inputs.nav_roll) / 100.0
                Theta_mot_2[i] = float(msg_rcv_inputs.nav_pitch) / 100.0
                Theta_mot_3[i] = float(msg_rcv_inputs.nav_bearing) / 100.0
                Theta_mot_4[i] = float(msg_rcv_inputs.target_bearing) / 100.0

                FT[i] = float(msg_rcv_inputs.wp_dist) / 100
                N[i]  = float(msg_rcv_inputs.alt_error) / 1000
                FX[i] = float(msg_rcv_inputs.aspd_error) / 1000
                FY[i] = float(msg_rcv_inputs.xtrack_error) / 1000
            if msg_rcv_out:
                PWM1[i] = float(msg_rcv_out.errors_count1) / 10
                PWM2[i] = float(msg_rcv_out.errors_count2) / 10
                PWM3[i] = float(msg_rcv_out.errors_count3) / 10
                PWM4[i] = float(msg_rcv_out.errors_count4) / 10

            if attitude:
                att_roll[i]     = float(attitude.roll) * RAD2DEG
                att_pitch[i]    = float(attitude.pitch) * RAD2DEG
                att_yaw[i]      = float(attitude.yaw) * RAD2DEG
                att_rollsdp[i]  = float(attitude.rollspeed) * RAD2DEG
                att_pitchsdp[i] = float(attitude.pitchspeed) * RAD2DEG
                att_yawsdp[i]   = float(attitude.yawspeed) * RAD2DEG

            if position:
                pos_lat[i] = float(position.lat) #/ 10000000  #degE7
                pos_lon[i] = float(position.lon) #/ 10000000
                pos_alt[i] = float(position.relative_alt) * 1000 #* RAD2DEG em mm
                pos_vx[i]  = float(position.vx) * 100.0  # Ground X Speed (Latitude, positive north) cm/s
                pos_vy[i]  = float(position.vy) * 100.0
                pos_vz[i]  = float(position.vz) * 100.0


            print "Theta Motor 1: %.3f \t Theta Motor 2: %.3f \t Theta Motor 3: %.3f \t Theta Motor 4: %.3f  " % (Theta_mot_1[i], Theta_mot_2[i], Theta_mot_3[i], Theta_mot_4[i])
            print "LAT: %.3f \t LON: %.3f \t ALT: %.3f \t VX: %.3f  " % (pos_lat[i], pos_lon[i], pos_alt[i], pos_vx[i])
            print "PWM   Motor 1: %.3f \t PWM   Motor 2: %.3f \t PWM   Motor 3: %.3f \t PWM   Motor 4: %.3f  " % (PWM1[i], PWM2[i], PWM3[i], PWM4[i])

            xdata, ydata   = [], []
            x2data, y2data = [], []
            x3data, y3data = [], []
            x4data, y4data = [], []

            xdata.append(0)
            xdata.append(PWM1[i]*10*np.sin(Theta_mot_1[i]*DEG2RAD))
            ydata.append(0)
            ydata.append(PWM1[i]*10*np.cos(Theta_mot_1[i]*DEG2RAD))

            x2data.append(0)
            x2data.append(PWM2[i]*10 * np.sin(Theta_mot_2[i]*DEG2RAD))
            y2data.append(0)
            y2data.append(PWM2[i]*10 * np.cos(Theta_mot_2[i]*DEG2RAD))

            x3data.append(0)
            x3data.append(PWM3[i]*10 * np.sin(Theta_mot_3[i]*DEG2RAD))
            y3data.append(0)
            y3data.append(PWM3[i]*10 * np.cos(Theta_mot_3[i]*DEG2RAD))

            x4data.append(0)
            x4data.append(PWM4[i]*10 * np.sin(Theta_mot_4[i]*DEG2RAD))
            y4data.append(0)
            y4data.append(PWM4[i]*10 * np.cos(Theta_mot_4[i]*DEG2RAD))

            ln1.set_data(xdata, ydata)
            ln2.set_data(x2data, y2data)
            ln3.set_data(x3data, y3data)
            ln4.set_data(x4data, y4data)

            return ln1, ln2, ln3, ln4


ani = FuncAnimation(fig, update, frames=np.linspace(0, it_number, it_number), init_func=init, blit=True)
plt.show()

# Saving gathered data to .mat file for further processing
# io.savemat('Atitude.mat', {'Roll_angle': roll_angle,'Pitch_angle': pitch_angle,'Yaw_angle': yaw_angle})
# arq.close()
# print "FIM!"

# xdata.append(frame)
# ydata.append(Theta_mot_1[i])
# ax1.set_xlim(0, frame + 50)

# x2data.append(frame)
# y2data.append(Theta_mot_2[i])
# ax2.set_xlim(0, frame + 50)
#
# x3data.append(frame)
# y3data.append(Theta_mot_3[i])
# ax3.set_xlim(0, frame + 50)
#
# x4data.append(frame)
# y4data.append(Theta_mot_4[i])
# ax4.set_xlim(0, frame + 50)
#
# ln1.set_data(xdata, ydata)
# ln2.set_data(x2data, y2data)
# ln3.set_data(x3data, y3data)
# ln4.set_data(x4data, y4data)

# arq.write("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n" % (Theta_mot_1[i],Theta_mot_2[i],Theta_mot_3[i],Theta_mot_4[i],PWM1[i],PWM2[i],PWM3[i],PWM4[i],FT[i],N[i],FX[i],FY[i]))

# print "PWM   Motor 1: %.3f \t PWM   Motor 2: %.3f \t PWM   Motor 3: %.3f \t PWM   Motor 4: %.3f  " % (PWM1[i], PWM2[i], PWM3[i], PWM4[i])
# print "Forca total: %.3f  " % (PWM1[i]+PWM2[i]+PWM3[i]+PWM4[i])
# print "Tork  Guinada: %.3f \t Força Total  : %.3f \t Força       X: %.3f \t Força       Y: %.3f  " % (N[i],FT[i],FX[i],FY[i])

# Saving gathered data to .mat file for further processing
# io.savemat('Atitude.mat', {'Roll_angle': roll_angle,'Pitch_angle': pitch_angle,'Yaw_angle': yaw_angle})
# arq.close()
# print "FIM!"
