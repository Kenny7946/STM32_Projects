import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl

class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        self.browser = QWebEngineView()
        self.setCentralWidget(self.browser)

        path = os.path.abspath("viewer.html")
        self.browser.load(QUrl.fromLocalFile(path))

app = QApplication(sys.argv)
window = Window()
window.show()
sys.exit(app.exec_())