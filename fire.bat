del mouse.csv
type NUL > mouse.csv
env\Scripts\activate.bat ^
    && start mouse.ahk ^
    && start py main.py ^
    && start py main_vis.py