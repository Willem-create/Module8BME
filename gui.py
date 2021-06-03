import eel
global remeasure


class Gui:
    mode = False
    remeasure = False
    def __init__(self):

        eel.init('../app')
        eel.start('dashboard.html', block=False)
        print("Starting webserver")
        eel.sleep(1)

    def get_eel(self):
        return eel

    def update_angle(self, angle):
        eel.set_angle(angle)

    def register_imus(self, imus):
        eel.register_imus(imus)
        
    def set_imu_status(self,name,status):
        eel.update_imu_status(name,status)

    def set_error_graph(self, errorlabel, errorvalue):
        eel.set_error_graph(errorlabel ,errorvalue)

    def set_average_graph(self, label, value):
        eel.set_average_graph(label, value)

    def set_baseline_graph(self, label, value):
        eel.set_baseline_graph(label, value)

    def feedback(self, feedback, mode):
        eel.feedback(feedback, mode)

    def sleep(self,seconds):
        eel.sleep(seconds)

    def notification(self, notification):
        eel.notification(notification)