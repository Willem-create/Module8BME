import eel


class Gui:
    def __init__(self):
        eel.init('../app')
        eel.start('dashboard.html', block=False)
        print("Starting webserver")
        eel.sleep(1)

    def update_angle(self, angle):
        eel.set_angle(angle)

    def send_status(self, status_message):
        eel.status_message(status_message)

    def sleep(self, s):
        eel.sleep(s)

    @eel.expose
    def echo(x):
        print(x)