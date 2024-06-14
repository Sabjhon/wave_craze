
import os
from PyQt6.QtCore import QObject, QRunnable, pyqtSignal, pyqtSlot, QThreadPool
from PyQt6.QtWidgets import QMainWindow, QTabWidget
import pyqtgraph as pg
import sys
import toml
import serial
import struct
import numpy as np
import csv
from datetime import datetime
from main_gui import *
import serial.tools.list_ports
from PyQt6.QtWidgets import QTableWidgetItem
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QMessageBox,QComboBox
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFileDialog
from PyQt6.QtCore import QTimer
import operator

def roll(array):
    return np.roll(array, -1)

class WorkerSignals(QObject):
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(list)
    progress = pyqtSignal(list)

class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @pyqtSlot()
    def run(self):
        '''
        Initialize the runner function with passed args, kwargs.
        '''
        try:
            result = self.fn(*self.args, **self.kwargs)
        except Exception as e:
            self.signals.error.emit((e,))
        else:
            if result is not None:  # Only emit result if it's not None
                self.signals.result.emit(result)
        finally:
            self.signals.finished.emit()

class MainWindow(QMainWindow):
    data_saved = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.viewTab = MainTab()
        self.mathTab = functionTab()
        self.settingsTab = SettingTab()

        self.tabs.addTab(self.viewTab, "Realtime View")
        self.tabs.addTab(self.mathTab, "MATH")
        self.tabs.addTab(self.settingsTab, "Settings")
        self.threadpool = QThreadPool()
        self.x_data = np.linspace(0, 100, 100)
        self.viewTab.spinBox.setMinimum(1)
        self.spin_value = 1
        self.recording_counter = 1
        self.parameters = self.read_config()['parameters']
        # self.add_val = self.read_val()['added_values']
        self.sampling_frequency = self.read_config()['sampling_frequency']
        self.dynamic_widgets()
        self.val = 0
        self.f_data = [np.zeros(100) for _ in range(4)]
        self.curve_dict = {}
        self.trigger = 0
        self.worker_thread = None
        self.stop_thread = False
        self.trig = False
        self.recording = False 
        self.viewTab.record_button.clicked.connect(self.toggle_record)
        self.viewTab.record_button.setStyleSheet("background-color: green")
        self.settingsTab.save_button.clicked.connect(self.handle_stop)
        self.viewTab.spinBox.valueChanged.connect(self.manage_widgets)
        self.viewTab.seconds_combo.activated.connect(self.select_time)
        self.settingsTab.add_button.clicked.connect(self.add_rows)
        self.settingsTab.save_button.clicked.connect(self.save_data)
        self.settingsTab.tableWidget.setHorizontalHeaderLabels(["Sensor ", "Data Type"])
        self.data_saved.connect(self.update_main_tab)
        for i, combo_box in enumerate(self.combo_boxes):
            combo_box.activated.connect(self.crv_set)
        self.combo_boxes[1].setVisible(False)
        self.combo_boxes[2].setVisible(False)
        self.combo_boxes[3].setVisible(False)
        self.plot_widgets[1].setVisible(False)
        self.plot_widgets[2].setVisible(False)
        self.plot_widgets[3].setVisible(False)
        self.select_port()
        self.update_time()
        self.load_data()
        self.is_running = False  
        self.viewTab.start_button.clicked.connect(self.handle_start)
        self.viewTab.stop_button.clicked.connect(self.handle_stop)
        self.mathTab.dropdown1.addItems(self.parameters.keys())
        self.mathTab.dropdown2.addItems(self.parameters.keys())
        self.browse_button = QtWidgets.QPushButton(self.settingsTab.verticalLayoutWidget)
        self.browse_button.setFixedHeight(30)
        self.browse_button.setFixedWidth(100)
        self.browse_button.setGeometry(QRect(465, 520, 93, 28))
        self.browse_button.setStyleSheet("background-color: rgb(7, 117, 127);")
        self.browse_button.setText("Browse")
        self.browse_button.clicked.connect(self.open_file_dialog)
        self.settingsTab.tableWidget.keyPressEvent = self.handle_key_press_event
        self.mathTab.add_button.clicked.connect(self.add_button_clicked)
        self.config = self.load_config()
        self.add_val = self.config.get('added_values', {})
        self.new_sensor_name = None
        self.flag = None
        self.added_data = []
        self.operations = []
        self.added_values = {}
        self.add_values_enabled = True
        # Initialize combo box for baud rates
        self.baud_rate_combo = QComboBox()  
        self.b_rates = ["300", "600", "750", "1200", "2400", "4800", "9600", "115200", "19200", "38400", "57600"]
        self.viewTab.baud_rate_combo.addItems(self.b_rates)
        
        # Connect the signal to the handler method
        self.viewTab.baud_rate_combo.currentIndexChanged.connect(self.handle_baud_rate_change)
        self.mark = 0
        self.serial_port = None  
        self.baud_rate = 115200  

    def select_port(self):
        ports = serial.tools.list_ports.comports()
        self.viewTab.com_port_combo.clear()  # Clear existing items
        self.viewTab.com_port_combo.addItems([port.device for port in ports])
        self.viewTab.com_port_combo.activated.connect(self.update_serial_port)

    def handle_baud_rate_change(self):
        self.baud_rate = int(self.viewTab.baud_rate_combo.currentText())
        print("Selected Baud Rate:", self.baud_rate)
        self.update_serial_port(self.viewTab.com_port_combo.currentIndex())  # Update the serial port with the new baud rate

    def update_serial_port(self, index):
        selected_port = self.viewTab.com_port_combo.itemText(index)
        if selected_port:
            try:
                self.setup_serial_connection(selected_port, self.baud_rate)
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Error opening serial port: {e}")
                QApplication.processEvents()  # Force processing of events to show the message box
        else:
            QMessageBox.warning(self, "Warning", "No COM port selected")
            QApplication.processEvents()  # Force processing of events to show the message box

    def setup_serial_connection(self, port, baud_rate):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=1  # Adjust timeout as needed
            )
            print(f"Serial connection established with port: {port} and baud rate: {baud_rate}")
        except serial.SerialException as e:
            print(f"Failed to establish serial connection: {e}")
            
    def load_config(self):
        config_path = 'config.toml'
        if os.path.exists(config_path):
            return toml.load(config_path)
        else:
            return {}
        
    def open_file_dialog(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open File", "", "TOML Files (*.toml)")

        if file_name:
            print(file_name)
            try:
                with open(file_name, 'r') as file:
                    content = file.read()
                    # Parse the TOML content into a dictionary
                    config = toml.loads(content)

                    # Update the settings tab's widgets with the loaded data
                    for sensor, datatype in config.get('parameters', {}).items():
                        row_position = self.settingsTab.tableWidget.rowCount()
                        self.settingsTab.tableWidget.insertRow(row_position)
                        self.settingsTab.tableWidget.setItem(row_position, 0, QTableWidgetItem(sensor))
                        self.settingsTab.tableWidget.setItem(row_position, 1, QTableWidgetItem(datatype))

                    self.settingsTab.lineEdit.setText(str(config.get('y_limit', {}).get('y1', [])[0]))
                    self.settingsTab.lineEdit_2.setText(str(config.get('y_limit', {}).get('y2', [])[0]))
                    self.settingsTab.lineEdit_3.setText(str(config.get('y_limit', {}).get('y3', [])[0]))
                    self.settingsTab.lineEdit_4.setText(str(config.get('y_limit', {}).get('y4', [])[0]))
                    self.settingsTab.lineEdit_5.setText(str(config.get('y_limit', {}).get('y1', [])[1]))
                    self.settingsTab.lineEdit_6.setText(str(config.get('y_limit', {}).get('y2', [])[1]))
                    self.settingsTab.lineEdit_7.setText(str(config.get('y_limit', {}).get('y3', [])[1]))
                    self.settingsTab.lineEdit_8.setText(str(config.get('y_limit', {}).get('y4', [])[1]))
                    self.settingsTab.com_sf.setText(str(config.get('sampling_frequency', 1)))

            except Exception as e:
                QMessageBox.warning(self, "Error", f"Error loading TOML file: {e}")

    def handle_key_press_event(self, event):
        if event.key() == Qt.Key.Key_Delete:
            # Delete the selected rows
            for item in self.settingsTab.tableWidget.selectedItems():
                self.settingsTab.tableWidget.removeRow(item.row())

            # Delete the selected columns
            for item in self.settingsTab.tableWidget.selectedItems():
                self.settingsTab.tableWidget.removeColumn(item.column())

    def handle_mouse_click(self, plot_widget, event):
        if event.button() == Qt.MouseButton.LeftButton and event.double():
            state = plot_widget.getViewBox().getState()
            auto_range = state['autoRange'][1]
            plot_widget.enableAutoRange(axis='y', enable=not auto_range)
            if not auto_range:
                self.trig = True
        elif event.button() == Qt.MouseButton.LeftButton and not event.double() and self.trig:
            y_limits = self.read_y_limits()  # Assuming this method reads the y-axis limits from your settings
            y_key = f'y{self.plot_widgets.index(plot_widget) + 1}'
            if y_key in y_limits:
                min_value, max_value = y_limits[y_key]
                plot_widget.setYRange(min_value, max_value)  # Set the y-axis range based on the y_limits
                self.trig = False

    def handle_start(self):
        if not self.is_running:
            self.start_program()
            self.is_running = True

    def handle_stop(self):
        if self.is_running:
            self.stop_program()
            self.is_running = False
            
    def read_config(self):
        config_dir = 'C:/wave_craze'
        os.makedirs(config_dir, exist_ok=True)  # Create 'wave_craze' folder if it doesn't exist
        config_path = os.path.join(config_dir, 'config.toml')
        if not os.path.exists(config_path):
            # Create a default config file if it doesn't exist
            default_config = {
                'parameters': {
                    'parameter1': 'float',
                    'parameter2': 'int',
                },
                'sampling_frequency': 1,
                'y_limit': {
                    'y1': [0, 10],
                    'y2': [0, 10],
                    'y3': [0, 10],
                    'y4': [0, 10]
                }
            }
            with open(config_path, 'w') as f:
                toml.dump(default_config, f)

        config = toml.load(config_path)
        return config

    def read_sf(self):
        config = toml.load('config.toml')
        return config.get('sampling_frequency'[0], {})

    def read_y_limits(self):
        # Get the directory of the executable
        exe_dir = os.path.dirname(sys.executable)
        # Construct the full path to the 'config.toml' file
        config_path = os.path.join(exe_dir, 'config.toml')

        try:
            config = toml.load(config_path)
        except FileNotFoundError:
            # If the 'config.toml' file is not found, create a default config
            config = {
                'y_limit': {
                    'y1': [0, 10],
                    'y2': [0, 10],
                    'y3': [0, 10],
                    'y4': [0, 10]
                }
            }
            # Optionally, you can write the default config to the 'config.toml' file here
        return config.get('y_limit', {})

    # Logic for the com port selection

    # logic for the data in seconds 
    def select_time(self):
        selected_time = int(self.viewTab.seconds_combo.currentText())
        num_data_points = int(selected_time * self.sampling_frequency)  # Calculate the number of data points based on sampling frequency
        self.x_data = np.linspace(0, selected_time, num_data_points)  # Update the X array based on selected time and sampling frequency

        # Update the f_data arrays for each sensor to match the new length
        for i, combo_box in enumerate(self.combo_boxes):
            if combo_box.isVisible():
                index = combo_box.currentIndex()
                if index < len(self.parameters):
                    self.f_data[index] = np.roll(self.f_data[index], -num_data_points)
                    self.f_data[index][:num_data_points] = 0  # Reset old data
        self.x_range = selected_time
    def update_time(self):
        self.time = [1, 5, 10]
        self.viewTab.seconds_combo.addItems([str(t) for t in self.time])

    def toggle_record(self):
        if not self.recording:
            self.start_record()
        else:
            self.stop_record()
            
    def start_record(self):
        self.recording = True
        self.viewTab.record_button.setStyleSheet("background-color: red")
        folder_path = "C:/wave_craze"
        os.makedirs(folder_path, exist_ok=True)  # Create the folder if it doesn't exist
        self.recording_counter += 1  # Increment the recording counter
        file_path = os.path.join(folder_path, f'sensor_data{self.recording_counter}.csv')
        self.csv_file = open(file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time"] + list(self.parameters.keys()))

    def stop_record(self):
        self.recording = False
        self.viewTab.record_button.setStyleSheet("background-color: green")
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()  # Close the CSV file

    def change_color(self):
        self.toggle_record()

    # Creation of dynamic plot-widgets
    def dynamic_widgets(self):
        self.combo_boxes = []
        self.plot_widgets = []
        y_limits = self.read_y_limits()

        for i in range(4):
            combo_box = QtWidgets.QComboBox()
            combo_box.setObjectName(f"combo_box_{i+1}")
            combo_box.addItems(self.parameters.keys())
            self.viewTab.verticalLayout.addWidget(combo_box)
            self.combo_boxes.append(combo_box)
            combo_box.setCurrentIndex(-1)  # Set no current index

            plot_widget = pg.PlotWidget()
            self.viewTab.verticalLayout.addWidget(plot_widget, stretch=1)
            plot_widget.setBackground('w')
            self.plot_widgets.append(plot_widget)

            if f'y{i+1}' in y_limits:
                plot_widget.setYRange(y_limits[f'y{i+1}'][0], y_limits[f'y{i+1}'][1])
            else:
                print(f"Y limits not found for index {i}")
        
        for plot_widget in self.plot_widgets:
            plot_widget.scene().sigMouseClicked.connect(lambda event, plot_widget=plot_widget: self.handle_mouse_click(plot_widget, event))

    def manage_widgets(self):
        self.spin_value = self.viewTab.spinBox.value()
        for i in range(4):
            self.combo_boxes[i].setVisible(i < self.spin_value)
            self.plot_widgets[i].setVisible(i < self.spin_value)

    def create_curve(self, plot_widget, index):
        plot_widget.clear()
        colors = ['r', 'g', 'b', 'y']
        color = colors[index % len(colors)]
        curve = plot_widget.plot(pen=pg.mkPen(color=color, width=2.5), name=f"Parameter {index+1}")

        # Update f_data for the selected sensor or added sensor
        if index < len(self.parameters):
            if index >= len(self.f_data):
                num_data_points = len(self.x_data)
                self.f_data.extend([np.zeros(num_data_points) for _ in range(index - len(self.f_data) + 1)])
            curve.setData(self.x_data, self.f_data[index], autoDownsample=True)
        else:
            num_data_points = len(self.x_data)
            if index >= len(self.f_data):
                self.f_data.extend([np.zeros(num_data_points) for _ in range(index - len(self.f_data) + 1)])
            curve.setData(self.x_data, self.f_data[index], autoDownsample=True)
            
        plot_widget.setXRange(self.x_data[0], self.x_data[-1])

        if index in self.curve_dict:
            self.curve_dict[index].append(curve)
        else:
            self.curve_dict[index] = [curve]

    def crv_set(self):
        for i in range(self.spin_value):
            if self.plot_widgets[i].isVisible():
                selected_index = self.combo_boxes[i].currentIndex()
                if selected_index < len(self.parameters):
                    self.create_curve(self.plot_widgets[i], selected_index)
                else:
                    added_sensor_index = selected_index - len(self.parameters)
                    self.create_curve(self.plot_widgets[i], added_sensor_index)

    def serial_read(self):
        if (self.serial_port.read() == b'\xff') and (self.serial_port.read() == b'\xff'):
            self.connected = True
            chksum = 255 + 255
            self.plSz = self.serial_port.read()[0]
            chksum += self.plSz
            self.payload = self.serial_port.read(self.plSz - 1)
            chksum += sum(self.payload)
            chksum = bytes([chksum % 256])
            _chksum = self.serial_port.read()
            return _chksum == chksum
        return False

    def return_str(self):
        format_str = ""
        for key in self.parameters.keys():
            if self.parameters[key] == 'float':
                format_str += 'f'
            elif self.parameters[key] == 'int':
                format_str += 'i'
            elif self.parameters[key] == 'long':
                format_str += 'l'
            elif self.parameters[key] == 'char':
                format_str += 'c'
            elif self.parameters[key] == 'bool':
                format_str += '?'

        self.val = format_str

    @pyqtSlot(list)
    def update_plot(self, data):
        if self.recording:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Microseconds to milliseconds
            self.csv_writer.writerow([current_time] + data)

        all_data = data.copy()

        # Add the computed values to the data array
        if self.add_values_enabled:
            for sensor_name, added_value in self.added_values.items():
                all_data.append(added_value)

        for i, value in enumerate(all_data):
            for j, combo_box in enumerate(self.combo_boxes):
                if combo_box.isVisible() and combo_box.currentIndex() == i:
                    parameter_name = combo_box.currentText()
                    if parameter_name in self.parameters:
                        index = list(self.parameters.keys()).index(parameter_name)
                        if index < len(self.f_data):
                            if len(self.f_data[index]) != len(self.x_data):
                                self.f_data[index] = np.zeros_like(self.x_data)
                            else:
                                self.f_data[index] = np.roll(self.f_data[index], -1)
                            if isinstance(value, bool):
                                value = int(value)
                            self.f_data[index][-1] = value
                            if len(self.f_data[index]) % 10 == 0:
                                for curve in self.curve_dict.get(index, []):
                                    curve.setData(self.x_data[:len(self.f_data[index])], self.f_data[index])

    def add_button_clicked(self):
        sensor1 = self.mathTab.dropdown1.currentText()
        sensor2 = self.mathTab.dropdown2.currentText()

        if sensor1 in self.parameters and sensor2 in self.parameters:
            operator = self.mathTab.text_box.text().strip()
            if operator not in ['+', '-', '/', '*']:
                print("Invalid operator. Please enter +, -, /, or * in the text box.")
                return

            new_sensor_name = f"{sensor1}_{operator}_{sensor2}"
            self.parameters[new_sensor_name] = []
            self.operations.append((sensor1, sensor2, operator, new_sensor_name))

            self.config['parameters'] = self.parameters

            for combo_box in self.combo_boxes:
                combo_box.addItem(new_sensor_name)
            self.mathTab.dropdown1.addItem(new_sensor_name)
            self.mathTab.dropdown2.addItem(new_sensor_name)

            with open('config.toml', 'w') as config_file:
                toml.dump(self.config, config_file)

            print(f"New sensor added: {new_sensor_name}")

            self.add_values_enabled = True

    def add_and_plot_values(self):
        for sensor1, sensor2, operator_str, new_sensor_name in self.operations:
            if sensor1 in self.parameters and sensor2 in self.parameters:
                index1 = list(self.parameters.keys()).index(sensor1)
                index2 = list(self.parameters.keys()).index(sensor2)

                if index1 < len(self.unpacked_data) and index2 < len(self.unpacked_data):
                    value1 = self.unpacked_data[index1]
                    value2 = self.unpacked_data[index2]

                    operator_funcs = {
                        '+': operator.add,
                        '-': operator.sub,
                        '*': operator.mul,
                        '/': operator.truediv
                    }

                    if operator_str in operator_funcs:
                        try:
                            added_value = operator_funcs[operator_str](value1, value2)
                        except ZeroDivisionError:
                            print("Division by zero error.")
                            continue
                    else:
                        print(f"Invalid operator: {operator_str}")
                        continue

                    self.added_values[new_sensor_name] = added_value

    def unpack_values(self, progress_callback):
        while self.serial_port.is_open:
            if self.serial_read():
                try:
                    self.return_str()
                    payload_format = self.val
                    payload_size = struct.calcsize(payload_format)
                    self.unpacked_data = struct.unpack(payload_format, self.payload[:payload_size])
                    progress_callback.emit(self.unpacked_data)

                    # Add and plot values after receiving new data if addition is enabled
                    if self.add_values_enabled:
                        self.add_and_plot_values()

                except struct.error as e:
                    print("Error unpacking data:", e)
            if self.trigger == 1:
                self.trigger = 2
                break

    def stop_program(self):
        self.recording = False
        self.trigger = 1
        self.flag = False
        self.viewTab.stop_button.setEnabled(False)
        self.viewTab.start_button.setEnabled(True)

    # Start the program
    def start_program(self):
        self.flag = True
        self.worker_thread = Worker(self.unpack_values)
        self.worker_thread.signals.progress.connect(self.update_plot)
        self.worker_thread.signals.finished.connect(self.thread_complete)
        self.worker_thread.signals.result.connect(self.thread_complete)
        self.threadpool.start(self.worker_thread)
        self.viewTab.start_button.setEnabled(False)
        self.viewTab.stop_button.setEnabled(True)

    def add_rows(self):
        self.settingsTab.tableWidget.insertRow(self.settingsTab.tableWidget.rowCount())

    def load_data(self):
        config_dir = 'C:/wave_craze'
        config_path = os.path.join(config_dir, 'config.toml')

        try:
            config = toml.load(config_path)
        except FileNotFoundError:
            config = {
                'parameters': {},
                'y_limit': {
                    'y1': [0, 10],
                    'y2': [0, 10],
                    'y3': [0, 10],
                    'y4': [0, 10]
                },
                'sampling_frequency': 1
            }

        self.settingsTab.tableWidget.setRowCount(len(config.get('parameters', {})))
        self.settingsTab.tableWidget.setColumnCount(2)
        self.settingsTab.tableWidget.setHorizontalHeaderLabels(["Sensor ", "Data Type"])

        # Load sensor parameters
        for i, (sensor, datatype) in enumerate(config.get('parameters', {}).items()):
            sensor_item = QTableWidgetItem(sensor)
            datatype_item = QTableWidgetItem(datatype)
            self.settingsTab.tableWidget.setItem(i, 0, sensor_item)
            self.settingsTab.tableWidget.setItem(i, 1, datatype_item)

        # Load y limits
        y_limits = config.get('y_limit', {})
        self.settingsTab.lineEdit.setText(str(y_limits.get('y1', [])[0]))
        self.settingsTab.lineEdit_2.setText(str(y_limits.get('y2', [])[0]))
        self.settingsTab.lineEdit_3.setText(str(y_limits.get('y3', [])[0]))
        self.settingsTab.lineEdit_4.setText(str(y_limits.get('y4', [])[0]))
        self.settingsTab.lineEdit_5.setText(str(y_limits.get('y1', [])[1]))
        self.settingsTab.lineEdit_6.setText(str(y_limits.get('y2', [])[1]))
        self.settingsTab.lineEdit_7.setText(str(y_limits.get('y3', [])[1]))
        self.settingsTab.lineEdit_8.setText(str(y_limits.get('y4', [])[1]))
        self.settingsTab.com_sf.setText(str(config.get('sampling_frequency', 1)))

    def save_data(self):
        if self.settingsTab.tableWidget.rowCount() == 0 or self.settingsTab.tableWidget.columnCount() == 0:
            QMessageBox.warning(self, "Warning", "No data to save. Please add sensor parameters.")
            return

        # Create 'C:/wave_craze' directory if it doesn't exist
        config_dir = 'C:/wave_craze'
        os.makedirs(config_dir, exist_ok=True)

        config_path = os.path.join(config_dir, 'config.toml')

        try:
            config = toml.load(config_path)
        except FileNotFoundError:
            config = {
                'parameters': {},
                'y_limit': {
                    'y1': [0, 10],
                    'y2': [0, 10],
                    'y3': [0, 10],
                    'y4': [0, 10]
                },
                'sampling_frequency': 1
            }
        parameters = {}
        for row in range(self.settingsTab.tableWidget.rowCount()):
            sensor_item = self.settingsTab.tableWidget.item(row, 0)
            datatype_item = self.settingsTab.tableWidget.item(row, 1)
            if sensor_item is not None and datatype_item is not None:
                sensor = sensor_item.text().lower()
                datatype = datatype_item.text().lower()
                parameters[sensor] = datatype

        for sensor, datatype in parameters.items():
            if datatype not in ['float', 'int', 'long', 'bool', 'char']:
                QMessageBox.warning(self, "Warning", f"Unwanted datatype '{datatype}' found for sensor '{sensor}'")
                return
        # Proceed with saving data if everything is okay
        config['parameters'] = parameters

        y_limits = {
            'y1': [float(self.settingsTab.lineEdit.text()), float(self.settingsTab.lineEdit_5.text())],
            'y2': [float(self.settingsTab.lineEdit_2.text()), float(self.settingsTab.lineEdit_6.text())],
            'y3': [float(self.settingsTab.lineEdit_3.text()), float(self.settingsTab.lineEdit_7.text())],
            'y4': [float(self.settingsTab.lineEdit_4.text()), float(self.settingsTab.lineEdit_8.text())]
        }
        config['y_limit'] = y_limits
        config['sampling_frequency'] = int(self.settingsTab.com_sf.text())
        with open(config_path, 'w') as f:
            toml.dump(config, f)

        print("Data saved successfully!")
        print("Updated parameters:", parameters)
        
        self.update_main_tab()
        self.update_plot_from_settings()
        self.load_data()

    def update_main_tab(self):
        config = self.read_config()
        y_limits = config.get('y_limit', {})
        parameters = config.get('parameters', {})
        self.curve_dict = {}

        for i, (plot_widget, combo_box) in enumerate(zip(self.plot_widgets[:4], self.combo_boxes[:4])):
            y_limit_key = f'y{i+1}'
            if y_limit_key in y_limits:
                plot_widget.setYRange(y_limits[y_limit_key][0], y_limits[y_limit_key][1])
            else:
                print(f"Y limit key {y_limit_key} not found in y_limits")
            combo_box.clear()
            combo_box.addItems(parameters.keys())
            self.mathTab.dropdown1.clear()
            self.mathTab.dropdown1.addItems(parameters.keys())
            self.mathTab.dropdown2.clear() 
            self.mathTab.dropdown2.addItems(parameters.keys())
        
    def update_plot_from_settings(self):
        y_limits = {
            'y1': [float(self.settingsTab.lineEdit.text()), float(self.settingsTab.lineEdit_5.text())],
            'y2': [float(self.settingsTab.lineEdit_2.text()), float(self.settingsTab.lineEdit_6.text())],
            'y3': [float(self.settingsTab.lineEdit_3.text()), float(self.settingsTab.lineEdit_7.text())],
            'y4': [float(self.settingsTab.lineEdit_4.text()), float(self.settingsTab.lineEdit_8.text())]
        }
        for i, plot_widget in enumerate(self.plot_widgets):
            y_limit_key = f'y{i+1}'
            if y_limit_key in y_limits:
                y_min, y_max = y_limits[y_limit_key]
                plot_widget.setYRange(y_min, y_max)
            else:
                print(f"Y limit key {y_limit_key} not found in y_limits")

        new_sf = int(self.settingsTab.com_sf.text())
        if new_sf != self.sampling_frequency:
            self.sampling_frequency = new_sf
            self.select_time()

        parameters = {}
        for row in range(self.settingsTab.tableWidget.rowCount()):
            sensor_item = self.settingsTab.tableWidget.item(row, 0)
            datatype_item = self.settingsTab.tableWidget.item(row, 1)
            if sensor_item is not None and datatype_item is not None:
                sensor = sensor_item.text().lower()
                datatype = datatype_item.text().lower()
                parameters[sensor] = datatype

        if parameters != self.parameters:
            self.parameters = parameters
            self.load_data()
            self.crv_set() 
            
    def thread_complete(self):
        print("THREAD COMPLETED!")
        
    def handle_error(self, error_tuple):       
        print("ERROR:", error_tuple[0])
        
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())