import eel


class Gui:
    def __init__(self):
        eel.init('app')
        eel.start('dashboard.html')

    def update_angle(self, angle):
        eel.set_angle(angle)
