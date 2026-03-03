from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
from collections import defaultdict

class SensorPlotWidget(QtWidgets.QWidget):
    """
    Flexibles Plot-Widget für Sensoren.
    - beliebige Anzahl Graphen
    - vertikaler Cursor für aktuellen Frame
    - Klick auf Graph liefert Frame und Wert
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QtWidgets.QVBoxLayout(self)
        self.setLayout(layout)

        # PlotWidget
        self.graph = pg.PlotWidget()
        self.graph.setBackground('k')
        self.graph.showGrid(x=True, y=True)
        self.graph.addLegend()
        layout.addWidget(self.graph)

        # Datenstrukturen
        self.curves = {}            # name -> PlotDataItem
        self.data_x = defaultdict(list)
        self.data_y = defaultdict(list)

        # Vertikale Linie für Cursor
        self.cursor_line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w'))
        self.graph.addItem(self.cursor_line)

        # Klick Callback
        self.graph.scene().sigMouseClicked.connect(self._mouse_clicked)
        self.click_func = None

    # ----------------------------
    # Sensoren verwalten
    # ----------------------------
    def add_sensor_curve(self, name: str, color='r'):
        """Fügt einen neuen Sensor hinzu"""
        if name in self.curves:
            return
        pen = pg.mkPen(color, width=1)
        curve = self.graph.plot(pen=pen, name=name)
        self.curves[name] = curve

    def remove_sensor_curve(self, name: str):
        if name in self.curves:
            self.graph.removeItem(self.curves[name])
            del self.curves[name]

    def update_data(self, name: str, x, y):
        """Setzt die Daten eines Sensors"""
        self.data_x[name] = x
        self.data_y[name] = y
        if name in self.curves:
            self.curves[name].setData(x, y)

    def set_cursor(self, frame_index):
        self.cursor_line.setValue(frame_index)

    def click_callback(self, func):
        """Callback auf Klick: func(frame_index, dict{name: value})"""
        self.click_func = func

    # ----------------------------
    # Interne Callback
    # ----------------------------
    def _mouse_clicked(self, event):
        pos = event.scenePos()
        vb = self.graph.getViewBox()
        if vb.sceneBoundingRect().contains(pos):
            mouse_point = vb.mapSceneToView(pos)
            x_click = int(mouse_point.x())
            values = {}
            for name in self.curves:
                y_data = self.data_y[name]
                if 0 <= x_click < len(y_data):
                    values[name] = y_data[x_click]
            if self.click_func:
                self.click_func(x_click, values)