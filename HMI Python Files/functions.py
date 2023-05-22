# ========== Copyright (C) CCBY 2023 by Ivo Tredal, Leidulv Tønnesland, Ludvig Vartdal & Bjørn K.T. Solheim ========== #

# ==================================================== Libraries =======================================================
import customtkinter as ctk
import numpy as np
import sys
import os
import signal
import serial
import time
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import multiprocessing as mp
from functools import partial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
matplotlib.use('Agg')

# ================================================ Global Variables ====================================================

# GUI definitions, texts and colors
default_text = "Helvetica"
text_size_header = 19
text_size_default = 18
text_size_button = 17
button_width = 120

imu_data_names = ["Roll", "Pitch", "Gyro Roll", "Gyro Pitch", "Accelerometer Roll", "Accelerometer Pitch"]

motor_data_names = ["Motor 1 Speed", "Motor 2 Speed", "Motor 1 Temperature", "Motor 2 Temperature",
                    "Motor 1 Driver Temperature", "Motor 2 Driver Temperature"]
batteries_data_names = ["Battery 1", "Battery 2", "Battery 3"]

entry_id = ["kp_1", "ti_1", "td_1", "kp_2", "ti_2", "td_2", "nu", "alpha"]
controller_pid_params = ["Kp", "Ti", "Td"]
imu_params = ["µ", "α"]

notification_names = ["", " Connected To Bluetooth Device ", " Could Not Find Bluetooth Device ",
                      " Lost connection to Bluetooth Device ", " All Input Parameters Updated ", " Entry Error ",
                      " Disconnected From Bluetooth Device ", " Starting ", " Stopping ", " Calibrating ",
                      " Calibration Completed ", " Device Not Connected ", " Device is already found ",
                      " Device is already calibrated "]

notification_color = ['#2b2b2b', '#1c8a21', '#94231b', '#94231b', '#1c8a21', '#94231b', '#94231b', '#1c8a21', '#94231b',
                      '#b88312', '#1c8a21', '#94231b', '#138da8', '#540fa8']

gui_headers = ["Inertial Measurement Unit", "Motors", "Batteries", "Controller 1 Parameters", "Controller 2 Parameters",
               "Filter Parameters", "Notifications"]

button_texts = ["Connect", "Update", "Calibrate", "Start", "Stop", "Disconnect"]

# Update rates
temperature_graph_update_rate = 500   # millisecond
nofifiaction_update_rate = 1000       # millisecond
nofifiaction_visable_time = 2000      # millisecond
display_data_update_rate = 100        # millisecond
imu_graph_update_rate = 100           # millisecond

# Graphing lists
graph_2D_roll_list = []
graph_2D_pitch_list = []
graph_temperature_motor1_list = []
graph_temperature_motor1_driver_list = []
graph_temperature_motor2_list = []
graph_temperature_motor2_driver_list = []
graph_temperature_battery_1_list = []
graph_temperature_battery_2_list = []
graph_temperature_battery_3_list = []

# Graphing settings
graph_width = 681
graph_height = 330
graph_3D_cords = [1131, 117]
graph_2D_roll_cords = [430, 117]
graph_2D_pitch_cords = [430, 465]
graph_temperature_cords = [1131, 465]
graph_list_length_max = 30
graph_radian_range = [-0.6, 0.6]
graph_temperature_range = [0, 70]
graph_temperature_legend = ['Motor 1', 'Driver 1', 'Motor 2', 'Driver 2', 'Battery 1', 'Battery 2', 'Battery 3']
graph_grid_color = '#545148'
graph_background_color = '#2b2b2b'
graph_text_color = '#FFFFFF'
graph_titles = [r'${\Phi}_p$', r'${\Theta}_p$', r'Temperatures']
graph_labels = [r'Angle (rad)', r'Degree Celsius']
graph_text_header_size = 14
graph_text_label_size = 10
graph_temperature_font_size = 13

# Misc.
common_data_threshold_length = [70, 300]
rpmElements = [0, 3]  # ordering of received data
temperatureElements = [1, 4, 2, 5]  # ordering of received data

# =============================================== Bluetooth Functions ==================================================


def check_ble_communication(device_found, port_name, lock_object, notif_type):

    # Find available BLE serial ports
    ble_ports = []
    for ble_port in serial.tools.list_ports.comports():
        if "Bluetooth" in ble_port.description:
            ble_ports.append(ble_port.device)

    # Try connecting to each BLE serial port until one responds
    for ble_port in ble_ports:
        try:
            # Check if device has allready been found
            if device_found.value is True:
                notif_type.value = 12
                break

            # Connecting to port
            ble_link = serial.Serial(ble_port, baudrate=115200, timeout=1)
            ble_data = ble_link.readline()

            # Checking for response
            if ble_data.decode('utf') != '':
                with lock_object:
                    ble_link.close()
                    port_name.value = ble_port
                    device_found.value = True
                    notif_type.value = 1
                    break

            # Closing port if no response
            ble_link.close()

        # Iteration error handling
        except serial.SerialException:
            device_found.value = False
            port_name.value = None

    # No device found
    if not device_found.value:
        notif_type.value = 2
        device_found.value = False


def receive_ble_data(device_found, port_name, updated_imu_gui_data, updated_motors_gui_data, updated_batteries_gui_data,
                     awaiting_to_send, awaiting_to_disconnect, device_is_calibrated, data_to_send, notif_type,
                     program_state):
    ble_link = None
    connected = False

    while True:
        # Check for program termination
        if program_state.value:
            os.kill(mp.current_process().pid, signal.SIGTERM)

        # Check if device is found and what the user is selecting
        if device_found.value and not connected and not awaiting_to_send.value and not awaiting_to_disconnect.value:
            ble_link = serial.Serial(port_name.value, baudrate=115200, timeout=1)
            connected = True

        if connected:
            if awaiting_to_send.value or awaiting_to_disconnect.value is True:
                ble_link.close()
                connected = False
            else:
                # Read incomming data
                received_data = ble_link.readline()

                # Check if user selected calibrating
                calibration_status(received_data, data_to_send, device_is_calibrated, notif_type)

                # Check if data lenght indicates more than only BLE presence and is valid
                if len(received_data) in range(common_data_threshold_length[0], common_data_threshold_length[1]):
                    try:
                        # Converts data to array
                        received_data_array = convert_data_to_array(received_data)

                        # Split data to spesific arrays
                        imu_data_array = received_data_array[:6]
                        motors_data_array = received_data_array[6:12]
                        batteries_data_array = received_data_array[12:]

                        # Send values to display function using global lists
                        for i, item in enumerate(imu_data_names):
                            updated_imu_gui_data[i] = imu_data_array[i]

                        for j, item in enumerate(motors_data_array):
                            updated_motors_gui_data[j] = motors_data_array[j]

                        for k, item in enumerate(batteries_data_array):
                            updated_batteries_gui_data[k] = batteries_data_array[k]

                    except ValueError:
                        pass


def send_ble_data(device_found, port_name, awaiting_to_send, data_to_send):
    ble_link = None
    connected = False
    awaiting_to_send.value = True

    time.sleep(1)  # simple delay to compensate for closing time of BLE receving connection

    if device_found.value:
        ble_link = serial.Serial(port_name.value, baudrate=115200, timeout=1)
        connected = True

    if connected:
        # Converts data to string and transmits
        data_to_ble_send_string = ', '.join(str(element) for element in data_to_send)
        print(data_to_ble_send_string)
        ble_link.write(data_to_ble_send_string.encode())
        ble_link.close()
        awaiting_to_send.value = False

# ================================================= General Functions ==================================================


def calibration_status(received_data, data_to_send, device_is_calibrated, notif_type):
    # Check if user wants to calibrate
    if data_to_send[10] == 1:
        # filter out common receiving data
        if received_data != b'' and len(received_data) < common_data_threshold_length[0]:
            calibration_data_int = convert_data_to_int(received_data)
            # Check if calibration is done
            if calibration_data_int == 1:
                notif_type.value = 10
                data_to_send[10] = 0
                device_is_calibrated.value = True


# ================================================= Helper Functions ===================================================

def convert_data_to_array(data):
    data = data.decode("utf-8").strip()  # Decodes and removes \r\n

    # Splits string of comma-separated numerical values into a list of floats with 3 decimals
    float_list = [float("{:.3f}".format(float(x))) for x in data.split(",")]

    array = np.array(float_list)
    return array


def convert_data_to_int(data):
    str_value = data.decode('utf-8').strip()  # Decodes and removes \r\n
    int_value = int(str_value)
    return int_value


def on_value_entry(entry, entry_type, data_to_send, notif_type):
    # Check if entry is int or float
    if entry.get().replace('.', '', 1).isdigit() or entry.get().isnumeric():
        # Update correct index of receive entry value
        index = entry_id.index(entry_type)
        received_entry = entry.get()
        data_to_send[index] = received_entry

    else:
        notif_type.value = 5  # Entry value error


def rotation(roll, pitch):
    theta = pitch
    phi = roll

    return (np.array([[np.cos(phi), 0, np.sin(phi)],
                      [-np.sin(theta) * np.sin(phi), np.cos(theta), np.sin(theta) * np.cos(phi)],
                      [-np.cos(theta) * np.sin(phi), -np.sin(theta), np.cos(theta) * np.cos(phi)]]))


# ================================================ Button Functions ====================================================


def update(device_found, port_name, awaiting_to_send, data_to_send, notif_type):
    if device_found.value:
        notif_type.value = 4
        send_ble_data(device_found, port_name, awaiting_to_send, data_to_send)
    else:
        notif_type.value = 11  # Device not connected


def start(device_found, port_name, awaiting_to_send, data_to_send, notif_type):
    if device_found.value:
        data_to_send[8] = 1
        data_to_send[9] = 0
        data_to_send[10] = 0
        notif_type.value = 7
        send_ble_data(device_found, port_name, awaiting_to_send, data_to_send)
    else:
        notif_type.value = 11  # Device not connected


def stop(device_found, port_name, awaiting_to_send, data_to_send, notif_type):
    if device_found.value:
        data_to_send[8] = 0
        data_to_send[9] = 1
        data_to_send[10] = 0
        notif_type.value = 8
        send_ble_data(device_found, port_name, awaiting_to_send, data_to_send)
    else:
        notif_type.value = 11  # Device not connected


def calibrate(device_found, port_name, awaiting_to_send, data_to_send, device_is_calibrated, notif_type):

    if device_found.value and device_is_calibrated.value:
        notif_type.value = 13  # Device is already calibrated

    if device_found.value and not device_is_calibrated.value:
        data_to_send[8] = 0
        data_to_send[9] = 0
        data_to_send[10] = 1
        send_ble_data(device_found, port_name, awaiting_to_send, data_to_send)
        notif_type.value = 9
        device_is_calibrated.value = True

    if not device_found.value:
        notif_type.value = 11  # Device not connected


def disconnect(notif_type, device_found, awaiting_to_disconnect):
    if device_found.value:
        awaiting_to_disconnect.value = True
        time.sleep(1)  # delay to compensate for closing time of BLE receving connection
        device_found.value = None
        notif_type.value = 6
        awaiting_to_disconnect.value = False
    else:
        notif_type.value = 11  # Device not connected


# =============================================== Graphing functions ===================================================

def plot_3D_graph(roll, pitch, ax):
    # Unit vectors
    e1 = [1, 0, 0]
    e2 = [0, 1, 0]
    e3 = [0, 0, 1]

    # Transformed unit vectors
    x = rotation(roll, pitch) @ e1
    y = rotation(roll, pitch) @ e2
    z = rotation(roll, pitch) @ e3

    # Plot unit vectors
    ax.quiver(0, 0, 0, e1[0], e1[1], e1[2], color='r', arrow_length_ratio=0.1, alpha=0.4)
    ax.quiver(0, 0, 0, e2[0], e2[1], e2[2], color='g', arrow_length_ratio=0.1, alpha=0.4)
    ax.quiver(0, 0, 0, e3[0], e3[1], e3[2], color='b', arrow_length_ratio=0.1, alpha=0.4)

    # Plot rotated vectors
    ax.quiver(z[0], z[1], z[2], x[0], x[1], x[2], color='r', arrow_length_ratio=0.1)
    ax.quiver(z[0], z[1], z[2], y[0], y[1], y[2], color='g', arrow_length_ratio=0.1)
    ax.quiver(z[0], z[1], z[2], z[0], z[1], z[2], color='b', arrow_length_ratio=0.1)
    ax.plot([0, z[0]], [0, z[1]], [0, z[2]], '--', color='black')

    # Plot title, color and orientation
    ax.set_facecolor(graph_background_color)
    ax.view_init(azim=45, elev=30)

    # Axis limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 2])
    ax.tick_params(axis='both', which='both', bottom=False, left=False, labelbottom=False, labelleft=False)


def init_plot_2D_graph_roll(ax):
    ax.set_ylim(graph_radian_range[0], graph_radian_range[1])  # Set viewable range

    # Plot color, grid, title and label
    ax.set_facecolor(graph_background_color)
    ax.grid(color=graph_grid_color)
    ax.set_title(graph_titles[0], color=graph_text_color, fontsize=graph_text_header_size)
    ax.set_ylabel(graph_labels[0], color=graph_text_color, fontsize=graph_text_label_size)

    # Remove ticks
    ax.tick_params(right=False, labelbottom=False, bottom=False, colors=graph_text_color)


def init_plot_2D_graph_pitch(ax):
    ax.set_ylim(graph_radian_range[0], graph_radian_range[1])  # Set viewable range

    # Plot color, grid, title and label
    ax.set_facecolor(graph_background_color)
    ax.grid(color=graph_grid_color)
    ax.set_title(graph_titles[1], color=graph_text_color, fontsize=graph_text_header_size)
    ax.set_ylabel(graph_labels[0], color=graph_text_color, fontsize=graph_text_label_size)

    # Remove ticks
    ax.tick_params(right=False, labelbottom=False, bottom=False, colors=graph_text_color)


def init_plot_temperature(ax):
    ax.set_ylim(graph_temperature_range[0], graph_temperature_range[1])  # Set viewable range

    # Plot color, grid, title and label
    ax.set_facecolor(graph_background_color)
    ax.grid(color=graph_grid_color)
    ax.set_title(graph_titles[2], color=graph_text_color, fontsize=graph_temperature_font_size)
    ax.set_ylabel(graph_labels[1], color=graph_text_color, fontsize=graph_text_label_size)

    # Remove ticks
    ax.tick_params(right=False, labelbottom=False, bottom=False, colors=graph_text_color)


# ============================================= Graphical User Interface ===============================================

def gui(device_found, port_name, lock_object, data_to_send, updated_imu_gui_data, updated_motors_gui_data,
        updated_batteries_gui_data, awaiting_to_send, awaiting_to_disconnect, device_is_calibrated, notif_type,
        program_state):

    def on_window_close():
        program_state.value = True  # Informs other process to shutdown
        os.kill(mp.current_process().pid, signal.SIGTERM)
        sys.exit(0)  # Exit the program

    # ---------------------------------------------- Setup -------------------------------------------------------------
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    root = ctk.CTk()
    root.wm_title("Inverted Pendulum Interface")
    root.geometry("1920x1020")
    root.grid_propagate(False)  # prevents frames from resizing to fit content
    root.protocol("WM_DELETE_WINDOW", on_window_close)  # Register window close event

    main_title = ctk.CTkLabel(master=root, text="Inverted Pendulum")
    main_title.configure(font=(default_text, 28))
    main_title.grid(column=0, row=0, padx=0, pady=30, sticky="nsew")

    main_under_title = ctk.CTkLabel(master=root, text="Control Interface by E2306")
    main_under_title.configure(font=(default_text, 17))
    main_under_title.grid(column=0, row=0, padx=0, pady=0, sticky="s")

    # ---------------------------------------------- Frames ------------------------------------------------------------
    # Configure the columns to have equal widths
    root.grid_columnconfigure(0, weight=0)
    root.grid_columnconfigure(1, weight=0)
    root.grid_columnconfigure(2, weight=0)
    root.grid_columnconfigure(3, weight=0)

    # Set the same minimum width for all frames
    for i in range(0, 4):
        root.grid_columnconfigure(i, minsize=400)

    button_frame = ctk.CTkFrame(master=root)
    button_frame.grid(column=0, row=1, padx=15, pady=22.5, sticky="nsew")
    notification_frame = ctk.CTkFrame(master=root)
    notification_frame.grid(column=0, row=3, padx=15, pady=10, sticky="nsew")
    imu_data_frame = ctk.CTkFrame(master=root)
    imu_data_frame.grid(column=0, row=4, padx=15, pady=10, sticky="nsew")
    motors_data_frame = ctk.CTkFrame(master=root)
    motors_data_frame.grid(column=0, row=5, padx=15, pady=10, sticky="nsew")
    batteries_data_frame = ctk.CTkFrame(master=root)
    batteries_data_frame.grid(column=0, row=6, padx=15, pady=10, sticky="nsew")
    controller_1_frame = ctk.CTkFrame(master=root)
    controller_1_frame .grid(column=1, row=6, padx=10, pady=10, sticky="nsew")
    controller_2_frame = ctk.CTkFrame(master=root)
    controller_2_frame.grid(column=2, row=6, padx=10, pady=10, sticky="nsew")
    imu_settings_frame = ctk.CTkFrame(master=root)
    imu_settings_frame.grid(column=3, row=6, padx=10, pady=10, sticky="nsew")

    # ---------------------------------------------- Buttons -----------------------------------------------------------
    connect_button = ctk.CTkButton(master=button_frame, text=button_texts[0], command=lambda:
                                   check_ble_communication(device_found, port_name, lock_object, notif_type))
    connect_button.configure(font=(default_text, text_size_button), width=button_width)
    connect_button.grid(padx=5, pady=5, column=0, row=1)

    update_button = ctk.CTkButton(master=button_frame, text=button_texts[1], command=lambda: update(
                                  device_found, port_name, awaiting_to_send, data_to_send, notif_type))
    update_button.configure(font=(default_text, text_size_button), width=button_width)
    update_button.grid(padx=5, pady=5, column=1, row=1)

    calibrate_button = ctk.CTkButton(master=button_frame, text=button_texts[2], command=lambda: calibrate(device_found,
                                     port_name, awaiting_to_send, data_to_send, device_is_calibrated, notif_type))
    calibrate_button.configure(font=(default_text, text_size_button), width=button_width)
    calibrate_button.grid(padx=5, pady=5, column=2, row=1)

    start_button = ctk.CTkButton(master=button_frame, text=button_texts[3], command=lambda: start(device_found,
                                 port_name, awaiting_to_send, data_to_send, notif_type))
    start_button.configure(font=(default_text, text_size_button), width=button_width)
    start_button.grid(padx=5, pady=5, column=0, row=2)

    stop_button = ctk.CTkButton(master=button_frame, text=button_texts[4], command=lambda: stop(device_found, port_name,
                                awaiting_to_send, data_to_send, notif_type))
    stop_button.configure(font=(default_text, text_size_button), width=button_width)
    stop_button.grid(padx=5, pady=5, column=1, row=2)

    disconnect_button = ctk.CTkButton(master=button_frame, text=button_texts[5], command=lambda:
                                      disconnect(notif_type, device_found, awaiting_to_disconnect))
    disconnect_button.configure(font=(default_text, text_size_button), width=button_width)
    disconnect_button.grid(padx=5, pady=5, column=2, row=2)

    # ------------------------------------------- Display data class ---------------------------------------------------

    class DisplayData:
        def __init__(self, master, data, row, column, text, element_number):
            self.label = ctk.CTkLabel(master=master)
            self.label.grid(row=row, column=column, padx=20, pady=0, sticky='w')
            self.label.configure(font=(default_text, text_size_default))
            self.curr_imu_value = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
            self.curr_motor_value = [0, 0, 0.0, 0.0, 0.0, 0.0]
            self.curr_bat_value = [0.0, 0.0, 0.0, 0.0]
            self.data = data
            self.master = master
            self.text = text
            self.element_number = element_number
            self.update_values()

        def update_values(self):
            # Unpack global arrays
            for a, item in enumerate(updated_imu_gui_data):
                self.curr_imu_value[a] = updated_imu_gui_data[a]

            for b, item in enumerate(updated_motors_gui_data):
                self.curr_motor_value[b] = updated_motors_gui_data[b]

            for c, item in enumerate(updated_batteries_gui_data):
                self.curr_bat_value[c] = updated_batteries_gui_data[c]

            # Display Motors data
            if self.master == motors_data_frame and self.element_number <= 1:

                self.label.configure(text=self.text + ': {:.0f}'.format
                                     (self.curr_motor_value[rpmElements[self.element_number]]) + " rpm")
                self.label.after(display_data_update_rate, self.update_values)

            if self.master == motors_data_frame and self.element_number > 1:
                self.label.configure(text=self.text + ': {:.2f}'.format
                                     (self.curr_motor_value[temperatureElements[self.element_number-2]]) + " °C")
                self.label.after(display_data_update_rate, self.update_values)

            # Display batteries data
            if self.master == batteries_data_frame:
                self.label.configure(text=self.text +
                                     ': {:.2f}'.format(self.curr_bat_value[self.element_number]) + " °C")
                self.label.after(display_data_update_rate, self.update_values)

            # Display IMU data
            if self.master == imu_data_frame:
                self.label.configure(text=self.text +
                                     ': {:.2f}'.format(self.curr_imu_value[self.element_number]) + " rad")
                self.label.after(display_data_update_rate, self.update_values)

    # ------------------------------------------- Display IMU data -----------------------------------------------------
    imu_title = ctk.CTkLabel(master=imu_data_frame, text=gui_headers[0])
    imu_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    imu_title.grid(column=0, row=0, padx=20, pady=10)

    for i, value in enumerate(imu_data_names):
        DisplayData(master=imu_data_frame, data=updated_imu_gui_data, row=i+1, column=0, text=imu_data_names[i],
                    element_number=i)

    # ------------------------------------------ Display Motors data ---------------------------------------------------
    motors_title = ctk.CTkLabel(master=motors_data_frame, text=gui_headers[1])
    motors_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    motors_title.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    for i, value in enumerate(motor_data_names):
        DisplayData(master=motors_data_frame, data=updated_motors_gui_data, row=i+1, column=0,
                    text=motor_data_names[i], element_number=i)

    # ---------------------------------------- Display Batteries data --------------------------------------------------
    batteries_title = ctk.CTkLabel(master=batteries_data_frame, text=gui_headers[2])
    batteries_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    batteries_title.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    for i, value in enumerate(batteries_data_names):
        DisplayData(master=batteries_data_frame, data=updated_batteries_gui_data, row=i+1, column=0,
                    text=batteries_data_names[i], element_number=i)

    # -------------------------------------------- Display Graphs  -----------------------------------------------------

    graph_3D_fig = plt.figure(facecolor=graph_background_color, dpi=80)
    graph_3D = FigureCanvasTkAgg(graph_3D_fig, master=root)
    graph_3D_widget = graph_3D.get_tk_widget()
    graph_3D_widget.place(x=graph_3D_cords[0], y=graph_3D_cords[1], width=graph_width, height=graph_height)

    graph_2D_roll_fig = plt.figure(facecolor=graph_background_color, dpi=80)
    graph_2D_roll = FigureCanvasTkAgg(graph_2D_roll_fig, master=root)
    graph_2D_roll_widget = graph_2D_roll.get_tk_widget()
    graph_2D_roll_widget.place(x=graph_2D_roll_cords[0], y=graph_2D_roll_cords[1],
                               width=graph_width, height=graph_height)

    graph_2D_pitch_fig = plt.figure(facecolor=graph_background_color, dpi=80)
    graph_2D_pitch = FigureCanvasTkAgg(graph_2D_pitch_fig, master=root)
    graph_2D_pitch_widget = graph_2D_pitch.get_tk_widget()
    graph_2D_pitch_widget.place(x=graph_2D_pitch_cords[0], y=graph_2D_pitch_cords[1], width=graph_width,
                                height=graph_height)

    graph_temperature_fig = plt.figure(facecolor=graph_background_color, dpi=80)
    graph_temperature = FigureCanvasTkAgg(graph_temperature_fig, master=root)
    graph_temperature_widget = graph_temperature.get_tk_widget()
    graph_temperature_widget.place(x=graph_temperature_cords[0], y=graph_temperature_cords[1], width=graph_width,
                                   height=graph_height)

    def update_imu_graphs():

        ax0 = graph_3D.figure.add_subplot(111, projection='3d')
        ax1 = graph_2D_roll.figure.add_subplot(111)
        ax2 = graph_2D_pitch.figure.add_subplot(111)

        init_plot_2D_graph_roll(ax1)
        init_plot_2D_graph_pitch(ax2)

        # Create new sub plots and update data
        plot_3D_graph(updated_imu_gui_data[0], updated_imu_gui_data[1], ax0)
        graph_2D_roll_list.append(updated_imu_gui_data[0])  # Phi
        ax1.plot(graph_2D_roll_list[-graph_list_length_max:])  # Plots and sets max number of visable values
        graph_2D_pitch_list.append(updated_imu_gui_data[1])  # Theta
        ax2.plot(graph_2D_pitch_list[-graph_list_length_max:])  # Plots and sets max number of visable values

        # Draw plots on subplots
        ax0.draw_artist(ax0)
        ax1.draw_artist(ax1)
        ax2.draw_artist(ax2)

        # Update the figures using blitting
        graph_3D_fig.canvas.blit(ax0.bbox)
        graph_2D_roll_fig.canvas.blit(ax1.bbox)
        graph_2D_pitch_fig.canvas.blit(ax2.bbox)

        graph_3D_fig.canvas.flush_events()
        graph_2D_roll_fig.canvas.flush_events()
        graph_2D_pitch_fig.canvas.flush_events()

        # Keep function running with set interval
        root.after(imu_graph_update_rate, update_imu_graphs)

    def update_temperature_graph():
        ax = graph_temperature.figure.add_subplot()
        init_plot_temperature(ax)

        # Updates corresponding lists to data array value
        graph_temperature_motor1_list.append(updated_motors_gui_data[1])
        graph_temperature_motor1_driver_list.append(updated_motors_gui_data[2])
        graph_temperature_motor2_list.append(updated_motors_gui_data[4])
        graph_temperature_motor2_driver_list.append(updated_motors_gui_data[5])
        graph_temperature_battery_1_list.append(updated_batteries_gui_data[0])
        graph_temperature_battery_2_list.append(updated_batteries_gui_data[1])
        graph_temperature_battery_3_list.append(updated_batteries_gui_data[2])

        # Plots and sets max number of visable values
        ax.plot(graph_temperature_motor1_list[-graph_list_length_max:], label=graph_temperature_legend[0])
        ax.plot(graph_temperature_motor1_driver_list[-graph_list_length_max:], label=graph_temperature_legend[1])
        ax.plot(graph_temperature_motor2_list[-graph_list_length_max:], label=graph_temperature_legend[2])
        ax.plot(graph_temperature_motor2_driver_list[-graph_list_length_max:], label=graph_temperature_legend[3])
        ax.plot(graph_temperature_battery_1_list[-graph_list_length_max:], label=graph_temperature_legend[4])
        ax.plot(graph_temperature_battery_2_list[-graph_list_length_max:], label=graph_temperature_legend[5])
        ax.plot(graph_temperature_battery_3_list[-graph_list_length_max:], label=graph_temperature_legend[6])

        # Add legend
        ax.legend(facecolor=graph_background_color, labelcolor='white')

        # Update the figure using blitting
        ax.draw_artist(ax)
        graph_temperature_fig.canvas.blit(ax.bbox)
        graph_temperature_fig.canvas.flush_events()

        # Keep function running with set interval
        root.after(temperature_graph_update_rate, update_temperature_graph)

    update_imu_graphs()
    update_temperature_graph()

    # -------------------------------------- Display Controller 1 entries ----------------------------------------------
    controller_1_title = ctk.CTkLabel(master=controller_1_frame, text=gui_headers[3])
    controller_1_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    controller_1_title.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    # PID parameters for controller 1
    for i, value in enumerate(controller_pid_params):
        controller_1_pid_labels = ctk.CTkLabel(
            master=controller_1_frame, text=controller_pid_params[i] + " :")
        controller_1_pid_labels.configure(font=(default_text, text_size_default))
        controller_1_pid_labels.grid(column=0, row=i+1, padx=20, pady=10, sticky="w")

        controller_1_pid_entry = ctk.CTkEntry(master=controller_1_frame)
        controller_1_pid_entry.configure(font=(default_text, text_size_default))
        controller_1_pid_entry.grid(column=0, row=i+1, padx=60, pady=10, sticky="w")

        # Partial function to pass on the correct entry widget
        on_entry = partial(on_value_entry, controller_1_pid_entry, entry_id[i], data_to_send, notif_type)
        controller_1_pid_entry.bind("<KeyRelease>", lambda event, arg=on_entry: arg())

    # -------------------------------------- Display Controller 2 entries ----------------------------------------------
    controller_2_title = ctk.CTkLabel(master=controller_2_frame, text=gui_headers[4])
    controller_2_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    controller_2_title.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    # PID parameters for controller 2
    for i, value in enumerate(controller_pid_params):
        controller_2_pid_labels = ctk.CTkLabel(
            master=controller_2_frame, text=controller_pid_params[i] + " :")
        controller_2_pid_labels.configure(font=(default_text, text_size_default))
        controller_2_pid_labels.grid(column=0, row=i+1, padx=20, pady=10, sticky="w")

        controller_2_pid_entry = ctk.CTkEntry(master=controller_2_frame)
        controller_2_pid_entry.configure(font=(default_text, text_size_default))
        controller_2_pid_entry.grid(column=0, row=i+1, padx=60, pady=10, sticky="w")

        # Partial function to pass on the correct entry widget
        on_entry = partial(on_value_entry, controller_2_pid_entry, entry_id[i+3], data_to_send, notif_type)
        controller_2_pid_entry.bind("<KeyRelease>", lambda event, arg=on_entry: arg())

    # ------------------------------------------ Display IMU entries ---------------------------------------------------

    imu_settings = ctk.CTkLabel(master=imu_settings_frame, text=gui_headers[5])
    imu_settings.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    imu_settings.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    for i, value in enumerate(imu_params):
        imu_label = ctk.CTkLabel(
            master=imu_settings_frame, text=imu_params[i] + " :")
        imu_label.configure(font=(default_text, text_size_default))
        imu_label.grid(column=0, row=i+1, padx=20, pady=10, sticky="w")

        imu_entry = ctk.CTkEntry(master=imu_settings_frame)
        imu_entry.configure(font=(default_text, text_size_default))
        imu_entry.grid(column=0, row=i+1, padx=50, pady=10, sticky="w")

        # Partial function to pass on the correct entry widget
        on_entry = partial(on_value_entry, imu_entry, entry_id[i+6], data_to_send, notif_type)
        imu_entry.bind("<KeyRelease>", lambda event, arg=on_entry: arg())

    def on_checkbox_select():
        if checkbox_state.get() == "c1_on":
            data_to_send[11] = 1

        if checkbox_state.get() == "c2_on":
            data_to_send[12] = 1

        if checkbox_state.get() == "c1_off":
            data_to_send[11] = 0

        if checkbox_state.get() == "c2_off":
            data_to_send[12] = 0

    # ---------------------------------------- Display Motor Checkboxes ------------------------------------------------
    checkbox_state = ctk.StringVar()
    controller1_checkbox = ctk.CTkCheckBox(master=controller_1_frame, text="Enable", variable=checkbox_state,
                                           command=lambda: on_checkbox_select(), onvalue="c1_on", offvalue="c1_off")
    controller1_checkbox.configure(font=(default_text, text_size_header))
    controller1_checkbox.grid(column=1, row=0, padx=60, pady=10, sticky="w")

    controller2_checkbox = ctk.CTkCheckBox(master=controller_2_frame, text="Enable", variable=checkbox_state,
                                           command=lambda: on_checkbox_select(), onvalue="c2_on", offvalue="c2_off")
    controller2_checkbox.configure(font=(default_text, text_size_header))
    controller2_checkbox.grid(column=1, row=0, padx=60, pady=10, sticky="w")

    # ----------------------------------------- Display notifications --------------------------------------------------
    notification_title = ctk.CTkLabel(master=notification_frame, text=gui_headers[6])
    notification_title.configure(font=(default_text, text_size_header, 'underline', 'bold'))
    notification_title.grid(column=0, row=0, padx=20, pady=10, sticky="w")

    class DisplayNotification:
        def __init__(self):
            self.text = ''
            self.label = ctk.CTkLabel(master=notification_frame, text=self.text)
            self.label.configure(font=(default_text, text_size_default))
            self.label.grid(column=0, row=1, padx=20, pady=10, sticky="w")
            self.update_label()

        def update_label(self):
            # Updates and handels notifications
            if notif_type.value == 0:
                self.text = notification_names[notif_type.value]
                self.label.configure(text=self.text, fg_color=notification_color[notif_type.value])
                self.label.after(nofifiaction_update_rate, self.update_label)

            else:
                # Shows received notification status
                self.text = notification_names[notif_type.value]
                self.label.configure(text=self.text, fg_color=notification_color[notif_type.value])
                self.label.after(nofifiaction_visable_time, self.update_label)
                notif_type.value = 0

    DisplayNotification()

    root.mainloop()
