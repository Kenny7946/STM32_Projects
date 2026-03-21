import sys
import threading
import http.server
import socketserver
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage
from PyQt6.QtCore import QUrl
import mimetypes
import os

PORT = 8000

class MyWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        print(f"JS Console ({level}): {message} (line {lineNumber})")

# Sicherstellen, dass .js als JavaScript ausgeliefert wird
mimetypes.add_type('application/javascript', '.js')

# HTTP-Handler
class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Debug-Ausgabe optional
        print(format % args)

# Server starten
httpd = socketserver.TCPServer(("", PORT), Handler)

def serve():
    print(f"Serving at port {PORT}")
    httpd.serve_forever()

threading.Thread(target=serve, daemon=True).start()

# PyQt App
app = QApplication(sys.argv)
window = QMainWindow()
browser = QWebEngineView()
page = MyWebEnginePage(browser)
browser.setPage(page)
window.setCentralWidget(browser)

# Lade HTML über HTTP, nicht file://
#browser.load(QUrl(f"http://localhost:{PORT}/three/examples/viewer.html"))
browser.load(QUrl(f"http://localhost:{PORT}/viewer.html"))

window.show()
sys.exit(app.exec())