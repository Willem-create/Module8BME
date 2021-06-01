import eel


class Gui:
    def __init__(self):
        eel.init('app')
        eel.start('dashboard.html')

    def update_angle(self, angle):
        eel.set_angle(angle)

    def register_imus(self, imus):
        eel.register_imus(imus)
        
    def set_imu_status(self,name,status):
        eel.update_imu_status(name,status)