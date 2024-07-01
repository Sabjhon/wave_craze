import sys
from PyQt6.QtWidgets import QMainWindow, QApplication, QTabWidget, QVBoxLayout, QWidget, QPushButton, QLabel, QSpinBox, QLineEdit, QGridLayout, QTableWidget, QHBoxLayout, QTableWidgetItem
from PyQt6.QtCore import QRect
from PyQt6 import QtCore, QtWidgets
from PyQt6.QtGui import QFont



class SettingTab(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi()

    def setupUi(self):
        self.setObjectName("SettingTab")
        self.resize(1133, 707)

        self.verticalLayoutWidget = QWidget(self)
        # self.setStyleSheet("background-color: rgb(255, 229, 167);")
        self.verticalLayoutWidget.setGeometry(QRect(10, 0, 1111, 661))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.layoutWidget = QWidget(self)
        self.layoutWidget.setGeometry(QRect(110, 110, 191, 301))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")

        self.lineEdit = QLineEdit(self.layoutWidget)
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout.addWidget(self.lineEdit, 0, 0, 1, 1)
        self.lineEdit_5 = QLineEdit(self.layoutWidget)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.gridLayout.addWidget(self.lineEdit_5, 0, 1, 1, 1)
        self.lineEdit_2 = QLineEdit(self.layoutWidget)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.gridLayout.addWidget(self.lineEdit_2, 1, 0, 1, 1)
        self.lineEdit_6 = QLineEdit(self.layoutWidget)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.gridLayout.addWidget(self.lineEdit_6, 1, 1, 1, 1)
        self.lineEdit_3 = QLineEdit(self.layoutWidget)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.gridLayout.addWidget(self.lineEdit_3, 2, 0, 1, 1)
        self.lineEdit_7 = QLineEdit(self.layoutWidget)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.gridLayout.addWidget(self.lineEdit_7, 2, 1, 1, 1)
        self.lineEdit_4 = QLineEdit(self.layoutWidget)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.gridLayout.addWidget(self.lineEdit_4, 3, 0, 1, 1)
        self.lineEdit_8 = QLineEdit(self.layoutWidget)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.gridLayout.addWidget(self.lineEdit_8, 3, 1, 1, 1)

        self.label = QLabel(self)
        self.label.setGeometry(QRect(40, 40, 91, 41))
        self.label.setObjectName("label")
        
        self.com_sf = QLineEdit(self)
        self.com_sf.setObjectName("com_sf")
        self.com_sf.setFixedHeight(20)
        self.com_sf.setFixedWidth(100)
        self.com_sf.setGeometry(QRect(400, 60, 51, 21))
        
        self.com_port_label = QtWidgets.QLabel(self)
        self.com_port_label.setObjectName("com_port_label")
        self.com_port_label.setText("SF :")
        # self.horizontalLayout.addWidget(self.com_port_label)
        self.com_port_label.setGeometry(QRect(380, 50, 71, 41))

        self.add_button = QPushButton(self)
        self.add_button.setGeometry(QRect(580, 520, 93, 28))
        self.add_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.add_button.setObjectName("pushButton_2")

        self.layoutWidget_2 = QWidget(self)
        self.layoutWidget_2.setGeometry(QRect(40, 120, 53, 271))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.gridLayout_2 = QGridLayout(self.layoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")

        self.label_2 = QLabel(self.layoutWidget_2)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)
        self.label_3 = QLabel(self.layoutWidget_2)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 1, 0, 1, 1)
        self.label_4 = QLabel(self.layoutWidget_2)
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 2, 0, 1, 1)
        self.label_5 = QLabel(self.layoutWidget_2)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 3, 0, 1, 1)

        self.tableWidget = QTableWidget(self)
        self.tableWidget.setGeometry(QRect(500, 50, 275, 451))
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(2)
        self.tableWidget.setColumnWidth(0, 150)
        self.tableWidget.setColumnWidth(1, 100)
        self.tableWidget.setRowCount(14)

        item1 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(0, item1)
        item2 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(1, item2)

        self.save_button = QPushButton(self)
        self.save_button.setGeometry(QRect(680, 520, 93, 28))
        self.save_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.save_button.setObjectName("pushButton")

        self.setLayout(self.verticalLayout)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Y- AXIS LIMITS"))
        self.add_button.setText(_translate("MainWindow", "Add "))
        self.label_2.setText(_translate("MainWindow", "Y1        :"))
        # self.label_2.font.setBold(True)
        self.label_3.setText(_translate("MainWindow", "Y2       :"))
        self.label_4.setText(_translate("MainWindow", "Y3        :"))
        self.label_5.setText(_translate("MainWindow", "Y4        :"))
        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "List of Sensors"))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Data Types"))
        self.save_button.setText(_translate("MainWindow", "Save"))

class MainTab(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi()

    def setupUi(self):
        self.setObjectName("MainTab")
        self.resize(1096, 850)

        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")

        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.label = QtWidgets.QLabel(self)
        self.label.setObjectName("label")
        self.label.setText("Windows")
        self.horizontalLayout.addWidget(self.label)

        self.spinBox = QtWidgets.QSpinBox(self)
        self.spinBox.setObjectName("spinBox")
        self.spinBox.setMaximum(4)
        self.spinBox.setMinimum(1)
        self.spinBox.setFixedHeight(30)
        self.spinBox.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.spinBox)

        self.com_port_label = QtWidgets.QLabel(self)
        self.com_port_label.setObjectName("com_port_label")
        self.com_port_label.setText("COM Port:")
        self.horizontalLayout.addWidget(self.com_port_label)

        self.com_port_combo = QtWidgets.QComboBox(self)
        self.com_port_combo.setObjectName("com_port_combo")
        self.com_port_combo.setFixedHeight(30)
        
        self.com_port_combo.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.com_port_combo)
        
        self.baud_rate_label = QtWidgets.QLabel(self)
        self.baud_rate_label.setObjectName("baud_rate_label")
        self.baud_rate_label.setText("Baud Rate:")
        self.horizontalLayout.addWidget(self.baud_rate_label)
        
        self.baud_rate_combo = QtWidgets.QComboBox(self)
        self.baud_rate_combo.setObjectName("baud_rate_combo")
        self.baud_rate_combo.setFixedHeight(30)
        self.baud_rate_combo.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.baud_rate_combo)

        # Add the seconds selection combo box to the horizontal layout
        self.seconds_label = QtWidgets.QLabel(self)
        self.seconds_label.setObjectName("seconds_label")
        self.seconds_label.setText("Seconds:")
        self.horizontalLayout.addWidget(self.seconds_label)

        self.seconds_combo = QtWidgets.QComboBox(self)
        self.seconds_combo.setObjectName("seconds_combo")
        self.seconds_combo.setFixedHeight(30)
        self.seconds_combo.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.seconds_combo)

        # Add the buttons to the horizontal layout
        self.start_button = QtWidgets.QPushButton(self)
        self.start_button.setStyleSheet("background-color: rgb(155, 170, 200);")
        self.start_button.setObjectName("start_button")
        self.start_button.setText("Start")
        self.start_button.setFixedHeight(30)
        self.start_button.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.start_button)

        self.stop_button = QtWidgets.QPushButton(self)
        self.stop_button.setStyleSheet("background-color: rgb(155, 170, 200);")
        self.stop_button.setObjectName("stop_button")
        self.stop_button.setText("Stop")
        self.stop_button.setFixedHeight(30)
        self.stop_button.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.stop_button)

        self.record_button = QtWidgets.QPushButton(self)
        self.record_button.setStyleSheet("background-color: rgb(155, 170, 200);")
        self.record_button.setObjectName("record_button")
        self.record_button.setText("Record")
        self.record_button.setFixedHeight(30)
        self.record_button.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.record_button)

        # Create an empty vertical layout for the bottom section
        self.plot_layout = QtWidgets.QVBoxLayout()
        self.plot_layout.setObjectName("plot_layout")
        self.verticalLayout.addLayout(self.plot_layout)

        # Set the layout for the MainTab
        self.setLayout(self.verticalLayout)
        
class functionTab(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi()

    def setupUi(self):
        self.setObjectName("functionTab")
        self.resize(1096, 850)

        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")

        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.dropdown1 = QtWidgets.QComboBox(self)
        self.dropdown1.setObjectName("dropdown1")
        self.dropdown1.setFixedHeight(30)
        self.dropdown1.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.dropdown1)
        
        self.add_sensor_button = QPushButton(self)
        self.add_sensor_button.setFixedHeight(30)
        self.add_sensor_button.setFixedWidth(100)
        self.add_sensor_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.add_sensor_button.setObjectName("Add")
        self.add_sensor_button.setText("Add")
        self.horizontalLayout.addWidget(self.add_sensor_button)

        self.dropdown2 = QtWidgets.QComboBox(self)
        self.dropdown2.setObjectName("dropdown2")
        self.dropdown2.setFixedHeight(30)
        self.dropdown2.setFixedWidth(100)
        self.horizontalLayout.addWidget(self.dropdown2)
        
        self.add_operator_button = QPushButton(self)
        self.add_operator_button.setFixedHeight(30)
        self.add_operator_button.setFixedWidth(100)
        self.add_operator_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.add_operator_button.setObjectName("Add")
        self.add_operator_button.setText("Add")
        self.horizontalLayout.addWidget(self.add_operator_button)

        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout.addLayout(self.verticalLayout_2)

        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.text_box = QLineEdit(self)
        self.text_box.setObjectName("lineEdit")
        self.text_box.setFixedWidth(100)
        self.text_box.setFixedHeight(30)
        self.horizontalLayout_2.addWidget(self.text_box)

        self.implement_button = QPushButton(self)
        self.implement_button.setFixedHeight(30)
        self.implement_button.setFixedWidth(100)
        self.implement_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.implement_button.setObjectName("Implement")
        self.implement_button.setText("Implement")
        self.horizontalLayout_2.addWidget(self.implement_button)

        self.clear_button = QPushButton(self)
        self.clear_button.setFixedHeight(30)
        self.clear_button.setFixedWidth(100)
        self.clear_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.clear_button.setObjectName("Clear")
        self.clear_button.setText("Clear")
        self.horizontalLayout_2.addWidget(self.clear_button)

        self.plot_layout = QtWidgets.QVBoxLayout()
        self.plot_layout.setObjectName("plot_layout")
        self.verticalLayout.addLayout(self.plot_layout)

        self.plot_layout_2 = QtWidgets.QVBoxLayout()
        self.plot_layout_2.setObjectName("plot_layout_2")
        self.verticalLayout.addLayout(self.plot_layout_2)


class MainWindow_MultiTab(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Data Readings")

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.viewTab = MainTab()
        self.tabs.addTab(self.viewTab, "Realtime view")
        
        self.functionTab = functionTab()
        self.tabs.addTab(self.functionTab, "MATH")

        self.settingstab = SettingTab()
        self.tabs.addTab(self.settingstab, "Settings")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow_MultiTab()
    window.show()
    sys.exit(app.exec())

