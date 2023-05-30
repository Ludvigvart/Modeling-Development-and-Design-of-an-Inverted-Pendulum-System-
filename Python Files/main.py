# ========== Copyright (C) CCBY 2023 by Ivo Tredal, Leidulv Tønnesland, Ludvig Vartdal & Bjørn K.T. Solheim ========== #

if __name__ == '__main__':
    # ================================================= Libraries ======================================================
    import multiprocessing as mp
    from multiprocessing import Manager, freeze_support
    from functions import *
    # =========================================== Shared Global Variables ==============================================

    # Allow multiprocessing to take control
    mp.freeze_support()

    # Create manager for multiproccessing shared global variables
    manager = mp.Manager()

    notification = manager.Value('i', 0)  # default state = 0
    ble_device_found = manager.Value('b', False)
    ble_com_port = manager.Value('s', '')
    ble_receive_duration = manager.Value('f', 0.1)  # seconds
    ble_awaiting_to_send = manager.Value('b', False)
    ble_awaiting_to_disconnect = manager.Value('b', False)
    ble_device_is_calibrated = manager.Value('b', False)
    ble_imu_data = manager.list([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
    ble_motors_data = manager.list([0, 0, 0.0, 0.0, 0.0, 0.0])
    ble_data_to_send = manager.list([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ble_updated_imu_data = manager.list([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
    ble_updated_motors_data = manager.list([0, 0, 0.0, 0.0, 0.0, 0.0])
    ble_update_batteries_data = manager.list([0.0, 0.0, 0.0])
    stop_program = manager.Value('b', False)

    # Defining a lock for multiproccessing sync of shared global variables
    lock = mp.Lock()

    # ================================================= Processes ======================================================

    # Start Bluetooth data receiver process
    ble_process = mp.Process(target=receive_ble_data, args=(ble_device_found, ble_com_port,
                             ble_updated_imu_data, ble_updated_motors_data, ble_update_batteries_data,
                             ble_awaiting_to_send, ble_awaiting_to_disconnect, ble_device_is_calibrated,
                             ble_data_to_send, notification, stop_program))
    ble_process.start()

    # Start GUI process
    gui_process = mp.Process(target=gui, args=(ble_device_found, ble_com_port, lock, ble_data_to_send,
                             ble_updated_imu_data, ble_updated_motors_data, ble_update_batteries_data,
                             ble_awaiting_to_send, ble_awaiting_to_disconnect, ble_device_is_calibrated, notification,
                             stop_program))
    gui_process.start()

    # Wait for both processes to finish
    ble_process.join()
    gui_process.join()



