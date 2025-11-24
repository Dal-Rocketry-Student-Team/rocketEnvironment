import sys
from PyQt6 import QtWidgets, QtCore

def m
    timer.timeout.connect(lambda: None)     # currently doing no work
    timer.start()

    win.show()      # show the window
    sys.exit(app.exec())        # start the event loop, this is blocking until closed

if __name__ == "__main__":
    main()