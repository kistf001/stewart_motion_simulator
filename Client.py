
import sys,os

from protocol import Serial as sl

sys.path.append("./Plugin/ACC")
import ACC_sMEM

#ACC_sMEM.do_test()
#ACC_sMEM.demo()

## Ex 3-1. 창 띄우기.

import sys
from PyQt5.QtWidgets import QApplication, QWidget


class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('My First Application')
        self.move(300, 300)
        self.resize(400, 200)
        self.show()


if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = MyApp()
   sys.exit(app.exec_())