del mouse.csv
type NUL > mouse.csv
env\Scripts\activate.bat ^
    && start mouse.ahk ^
    && start py main.py ^
    && start py main_vis.py

@REM && start py 2d_vis.py