import sys
from PyQt6 import QtWidgets, QtCore;

def main():

    # creates the Qt app
    app = QtWidgets.QApplication(sys.argv)

    # makes a bare window widget
    win = QtWidgets.QWidget()
    win.setWindowTitle("Minimal PyQt code")
    win.resize(400, 400)

    # putting a label in the window
    label = QtWidgets.QLabel("Hellow Qt!!!", parent=win)
    label.move(20, 20)

    # create a timer that fires 60 times a second, actual wor kwill be attached to this later
    timer = QtCore.QTimer()
    timer.setInterval(16)   # sets the timer interval to 16 mS which is approx 60 times per sec
    
    timer.timeout.connect(lambda: None)     # currently doing no work
    timer.start()

    win.show()      # show the window
    sys.exit(app.exec())        # start the event loop, this is blocking until closed

if __name__ == "__main__":
    main()