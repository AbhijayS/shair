UBIT_COM_PORT = "COM7"
IMU_COM_PORT = "COM12" # TODO: specify your COM port here
IMU_BAUD_RATE = 115200
OPTICAL_DPI = 1130 # TODO: specify optical sensor DPI here or use the default one
OPTICAL_SIGNS = [-1, 1, 0] # order: x,y,switch

# use calibrate_magnet.py and magneto 1.2 to get these values
MAG_HARD_IRON = [-1251.140526, 121.490963, 3250.619550] # order: xyz
MAG_SOFT_IRON = [
    1.028982, 0.009700, -0.014950,
    0.009700, 1.125870, -0.000530,
    -0.014950, -0.000530, 1.013084]

# stepper
STEPPER_COM_PORT = "COM6"
