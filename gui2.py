# --- VESC Control GUI (Split Settings Tabs) ---
import sys
import os
import serial
import time
import struct
import pprint
import copy
import threading
import queue
import tkinter.messagebox  # For write confirmation

# GUI Library
try:
    import customtkinter
except ImportError:
    print(
        "Error: customtkinter library not found. Please install it: pip install customtkinter"
    )
    sys.exit(1)

# Serial Library
try:
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial library not found. Please install it: pip install pyserial")
    sys.exit(1)

# --- Custom pyvesc imports (Ensure pyvesc is accessible) ---
try:
    # Protocol level
    from pyvesc.protocol.packet.codec import *
    from pyvesc.protocol.interface import *

    # VESC Messages & Parsers/Packers
    from pyvesc.VESC.messages.getters import GetMcConfRequest, GetAppConfRequest
    from pyvesc.VESC.messages.setters import SetMcConf, SetAppConf, DetectApplyAllFOC
    from pyvesc.VESC.messages.parser import (
        parse_mc_conf_serialized,
        parse_app_conf_serialized,
    )
    from pyvesc.VESC.messages.vesc_protocol_utils import (
        encode_set_mcconf,
        encode_set_appconf,
        encode_detect_apply_all_foc,
    )

except ImportError as e:
    print(f"Error: Failed to import required components from pyvesc ({e}).")
    print(
        "       Ensure your custom pyvesc library is correctly installed or accessible."
    )
    sys.exit(1)
except AttributeError as e:
    print(
        f"Error: Attribute error during pyvesc import ({e}). Check class/function definitions."
    )
    sys.exit(1)

# --- Constants ---
DEFAULT_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0
DETECTION_WIZARD_TIMEOUT_GUI = 120.0
READ_BUFFER_SIZE = 4096

# --- Parameter Mappings (String <-> VESC Enum/Value) ---
# !! These values MUST match your firmware version !!
MOTOR_TYPES_MAP = {0: "BLDC", 1: "DC", 2: "FOC"}
SENSOR_PORT_MODES_MAP = {0: "HALL", 1: "ABI", 2: "AS5047 SPI"}
FOC_SENSOR_MODES_MAP = {0: "Sensorless", 1: "Encoder", 2: "Hall"}
BOOL_MAP = {False: "False", True: "True"}
APP_MODES_MAP = {
    0: "None",
    1: "PPM",
    2: "ADC",
    3: "UART",
    4: "PPM+UART",
    5: "ADC+UART",
    7: "CUSTOM",
}
CAN_BAUD_RATES_MAP = {0: "125K", 1: "250K", 2: "500K", 3: "1M"}
CAN_MODES_MAP = {
    0: "VESC",
    1: "UAVCAN",
    2: "COMM BRIDGE",
    3: "UNUSED",
}

UAVCAN_RAW_MODES_MAP = {
    0: "Current",
    1: "Current NoRev Brake",
    2: "Duty Cycle",
    3: "RPM",
}

PPM_CONTROL_TYPES_MAP = {
    0: "Disabled",
    1: "Current",
    2: "Current No Reverse",
    3: "Current No Reverse with Brake",
    4: "Duty Cycle",
    5: "Duty Cycle No Reverse",
    6: "PID Pos",
    7: "PID Pos No Rev",
    10: "PID POS 180 deg",
    11: "PID POS 360 deg",
}


# --- Reverse Mappings (String -> VESC Enum/Value) ---
def create_reverse_map(mapping):
    return {v: k for k, v in mapping.items()}


MOTOR_TYPES_MAP_REV = create_reverse_map(MOTOR_TYPES_MAP)
SENSOR_PORT_MODES_MAP_REV = create_reverse_map(SENSOR_PORT_MODES_MAP)
FOC_SENSOR_MODES_MAP_REV = create_reverse_map(FOC_SENSOR_MODES_MAP)
BOOL_MAP_REV = create_reverse_map(BOOL_MAP)
APP_MODES_MAP_REV = create_reverse_map(APP_MODES_MAP)
CAN_BAUD_RATES_MAP_REV = create_reverse_map(CAN_BAUD_RATES_MAP)
CAN_MODES_MAP_REV = create_reverse_map(CAN_MODES_MAP)
UAVCAN_RAW_MODES_MAP_REV = create_reverse_map(UAVCAN_RAW_MODES_MAP)
PPM_CONTROL_TYPES_MAP_REV = create_reverse_map(PPM_CONTROL_TYPES_MAP)


# --- Main Application Class ---
class VescApp(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.title("ROTOM FLUXIUM CONFIGURATOR")
        self.geometry("900x700")

        # --- State Variables ---
        self.vesc_serial: serial.Serial | None = None
        self.connected = False
        self.port_list = []
        self.current_mc_config = None
        self.current_app_config = None
        self.mcconf_widgets = {}
        self.appconf_widgets = {}

        # --- Appearance ---
        customtkinter.set_appearance_mode("System")
        customtkinter.set_default_color_theme("blue")
        # Themes: "blue" (default), "green", "dark-blue"

        # --- Configure grid layout (2x1) ---
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- Create Sidebar Frame ---
        self._create_sidebar()

        # --- Create Main Tabview ---
        self._create_main_tabs()

        # --- Initial Status ---
        self._log("Ready. Select port and connect.")
        self._update_port_list()
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    # --- UI Creation Methods ---
    def _create_sidebar(self):
        # (Same as before)
        self.sidebar_frame = customtkinter.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=1, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(5, weight=1)  # Push status label down

        self.logo_label = customtkinter.CTkLabel(
            self.sidebar_frame,
            text="ROTOM" + "\n" + "FLUXIUM",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.port_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Serial Port:", anchor="w"
        )
        self.port_label.grid(row=1, column=0, padx=20, pady=(10, 0), sticky="ew")
        self.port_combobox = customtkinter.CTkComboBox(
            self.sidebar_frame, values=[""], command=self._port_selected
        )
        self.port_combobox.grid(row=2, column=0, padx=20, pady=(0, 10), sticky="ew")

        self.refresh_button = customtkinter.CTkButton(
            self.sidebar_frame, text="Refresh Ports", command=self._update_port_list
        )
        self.refresh_button.grid(row=3, column=0, padx=20, pady=(0, 10))

        self.connect_button = customtkinter.CTkButton(
            self.sidebar_frame, text="Connect", command=self._toggle_connection
        )
        self.connect_button.grid(row=4, column=0, padx=20, pady=10)

        self.status_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Status: Disconnected", anchor="w", wraplength=180
        )
        self.status_label.grid(row=6, column=0, padx=20, pady=(10, 20), sticky="sew")

    def _create_main_tabs(self):
        self.tabview = customtkinter.CTkTabview(self, width=650)
        self.tabview.grid(row=0, column=1, padx=(20, 20), pady=(10, 20), sticky="nsew")

        # Create the specific tabs
        self.tabview.add("Motor Config")
        self.tabview.add("App Config")
        self.tabview.add("Detection")

        # Populate the tabs
        self._create_mcconf_tab(self.tabview.tab("Motor Config"))
        self._create_appconf_tab(self.tabview.tab("App Config"))
        self._create_detection_tab(self.tabview.tab("Detection"))

    def _create_mcconf_tab(self, tab):
        """Creates the content for the Motor Configuration tab."""
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)  # Scroll Frame expands

        # --- Actions for this tab ---
        action_frame = customtkinter.CTkFrame(tab)
        action_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")

        read_button = customtkinter.CTkButton(
            action_frame, text="Read Motor Config", command=self._read_mcconf_gui
        )
        read_button.pack(side="left", padx=10, pady=5)
        self.read_mcconf_button = (
            read_button  # Store reference if needed for state changes
        )

        write_button = customtkinter.CTkButton(
            action_frame,
            text="Write Motor Config",
            command=self._write_mcconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        write_button.pack(side="right", padx=10, pady=5)
        self.write_mcconf_button = write_button  # Store reference

        # --- Motor Configuration Scrollable Frame ---
        scroll_frame = customtkinter.CTkScrollableFrame(
            tab, label_text="Motor Parameters"
        )
        scroll_frame.grid(row=1, column=0, padx=10, pady=(5, 10), sticky="nsew")
        scroll_frame.grid_columnconfigure(1, weight=1)

        self.mcconf_widgets = {}  # Reset/Initialize widget dict for this tab
        row_mc = 0

        def add_mc_setting(key, label_text, widget_type, options=None):
            nonlocal row_mc
            label = customtkinter.CTkLabel(scroll_frame, text=label_text, anchor="w")
            label.grid(row=row_mc, column=0, padx=(5, 10), pady=5, sticky="w")

            if widget_type == "combobox":
                widget = customtkinter.CTkComboBox(
                    scroll_frame, values=options, state="readonly"
                )
                widget.set(options[0] if options else "")
            elif widget_type == "entry":
                widget = customtkinter.CTkEntry(scroll_frame)
            else:
                widget = customtkinter.CTkLabel(
                    scroll_frame, text="ERR", text_color="red"
                )

            widget.grid(row=row_mc, column=1, padx=5, pady=5, sticky="ew")
            self.mcconf_widgets[key] = widget
            row_mc += 1

        # Add MCConf Widgets (Key MUST match pyvesc dict key)
        add_mc_setting(
            "motor_type", "Motor Type:", "combobox", list(MOTOR_TYPES_MAP.values())
        )
        add_mc_setting(
            "m_invert_direction",
            "Invert Direction:",
            "combobox",
            list(BOOL_MAP.values()),
        )
        add_mc_setting(
            "m_sensor_port_mode",
            "Sensor Port Mode:",
            "combobox",
            list(SENSOR_PORT_MODES_MAP.values()),
        )
        add_mc_setting("l_current_max", "Motor Current Max (A):", "entry")
        add_mc_setting("l_abs_current_max", "Absolute Max Current (A):", "entry")
        add_mc_setting("l_in_current_max", "Batt Current Max (A):", "entry")
        # add_mc_setting("l_in_current_min", "Batt Current Max Regen (A):", "entry")
        add_mc_setting("l_battery_cut_start", "Batt Volt Cutoff Start (V):", "entry")
        add_mc_setting("l_battery_cut_end", "Batt Volt Cutoff End (V):", "entry")
        add_mc_setting(
            "foc_sensor_mode",
            "FOC Sensor Mode:",
            "combobox",
            list(FOC_SENSOR_MODES_MAP.values()),
        )
        add_mc_setting(
            "s_pid_allow_braking", "Allow Braking:", "combobox", list(BOOL_MAP.values())
        )
        add_mc_setting("p_pid_kp", "Position PID Kp:", "entry")
        add_mc_setting("p_pid_ki", "Position PID Ki:", "entry")
        add_mc_setting("p_pid_kd", "Position PID Kd:", "entry")

        # Disable widgets initially
        self._set_mcconf_widgets_state("disabled")

    def _create_appconf_tab(self, tab):
        """Creates the content for the Application Configuration tab."""
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)  # Scroll Frame expands

        # --- Actions for this tab ---
        action_frame = customtkinter.CTkFrame(tab)
        action_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")

        read_button = customtkinter.CTkButton(
            action_frame, text="Read APP Config", command=self._read_appconf_gui
        )
        read_button.pack(side="left", padx=10, pady=5)
        self.read_appconf_button = read_button

        write_button = customtkinter.CTkButton(
            action_frame,
            text="Write APP Config",
            command=self._write_appconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        write_button.pack(side="right", padx=10, pady=5)
        self.write_appconf_button = write_button

        # --- Application Configuration Scrollable Frame ---
        scroll_frame = customtkinter.CTkScrollableFrame(
            tab, label_text="Application Parameters"
        )
        scroll_frame.grid(row=1, column=0, padx=10, pady=(5, 10), sticky="nsew")
        scroll_frame.grid_columnconfigure(1, weight=1)

        self.appconf_widgets = {}  # Reset/Initialize widget dict for this tab
        row_app = 0

        def add_app_setting(key, label_text, widget_type, options=None):
            nonlocal row_app
            label = customtkinter.CTkLabel(scroll_frame, text=label_text, anchor="w")
            label.grid(row=row_app, column=0, padx=(5, 10), pady=5, sticky="w")

            if widget_type == "combobox":
                widget = customtkinter.CTkComboBox(
                    scroll_frame, values=options, state="readonly"
                )
                widget.set(options[0] if options else "")
            elif widget_type == "entry":
                widget = customtkinter.CTkEntry(scroll_frame)
            else:
                widget = customtkinter.CTkLabel(
                    scroll_frame, text="ERR", text_color="red"
                )

            widget.grid(row=row_app, column=1, padx=5, pady=5, sticky="ew")
            self.appconf_widgets[key] = widget
            row_app += 1

        # Add AppConf Widgets (Key MUST match pyvesc dict key)
        add_app_setting(
            "app_to_use", "App to Use:", "combobox", list(APP_MODES_MAP.values())
        )
        add_app_setting("controller_id", "VESC ID:", "entry")
        add_app_setting(
            "can_baud_rate",
            "CAN Baud Rate:",
            "combobox",
            list(CAN_BAUD_RATES_MAP.values()),
        )
        add_app_setting(
            "can_mode",
            "CAN Mode:",
            "combobox",
            list(CAN_MODES_MAP.values()),
        )
        add_app_setting("uavcan_esc_index", "UAVCAN ESC Index:", "entry")
        add_app_setting(
            "uavcan_raw_mode",
            "UAVCAN Raw Mode:",
            "combobox",
            list(UAVCAN_RAW_MODES_MAP.values()),
        )  # 새 맵 사용

        add_app_setting(
            "app_ppm_conf.ctrl_type",
            "PPM Control Type:",
            "combobox",
            list(PPM_CONTROL_TYPES_MAP.values()),
        )

        # Disable widgets initially
        self._set_appconf_widgets_state("disabled")

    def _create_detection_tab(self, tab):
        # (Same as before)
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(3, weight=1)

        self.detection_label = customtkinter.CTkLabel(
            tab,
            text="FOC Motor Detection Wizard",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.detection_label.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="w")

        self.detection_warning = customtkinter.CTkLabel(
            tab,
            text="!!! WARNING !!!\nMotor WILL spin/make noise.\nEnsure safety precautions!",
            text_color="red",
            font=customtkinter.CTkFont(weight="bold"),
            justify="left",
        )
        self.detection_warning.grid(row=1, column=0, padx=10, pady=5, sticky="w")

        self.param_frame = customtkinter.CTkFrame(tab)
        self.param_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        self.max_loss_label = customtkinter.CTkLabel(
            self.param_frame, text="Max Power Loss (W):"
        )
        self.max_loss_label.pack(side="left", padx=(10, 5), pady=10)
        self.max_loss_entry = customtkinter.CTkEntry(self.param_frame, width=80)
        self.max_loss_entry.pack(side="left", padx=(0, 20), pady=10)
        self.max_loss_entry.insert(0, "100.0")
        self.run_detection_button = customtkinter.CTkButton(
            self.param_frame,
            text="Run FOC Detection",
            command=self._run_detection_gui,
            fg_color="red",
            hover_color="dark red",
        )
        self.run_detection_button.pack(side="right", padx=10, pady=10)

        self.detection_output_textbox = customtkinter.CTkTextbox(tab, wrap="word")
        self.detection_output_textbox.grid(
            row=3, column=0, padx=10, pady=(0, 10), sticky="nsew"
        )
        self.detection_output_textbox.configure(state="disabled")

    # --- UI Update & Logging ---
    def _log(self, message):
        print(f"LOG: {message}")
        self.after(0, self._update_status_label, message)

    def _update_status_label(self, message):
        self.status_label.configure(text=f"Status: {message}")

    def _update_textbox(self, textbox, content, read_only=True):
        # (Same as before)
        textbox.configure(state="normal")
        textbox.delete("1.0", "end")
        textbox.insert("1.0", content)
        if read_only:
            textbox.configure(state="disabled")

    def _set_ui_state(self, state):
        """Enable/disable UI elements based on connection state and operation."""
        # Connection controls
        is_currently_connected = self.connected
        allow_connect_ops = state == "normal"

        connect_button_state = "disabled" if not allow_connect_ops else "normal"
        port_state = (
            "disabled"
            if (not allow_connect_ops or is_currently_connected)
            else "normal"
        )
        refresh_state = "disabled" if not allow_connect_ops else "normal"

        self.port_combobox.configure(state=port_state)
        self.refresh_button.configure(state=refresh_state)
        self.connect_button.configure(state=connect_button_state)

        # Configuration and Detection Tabs (enabled only if connected and UI state is normal)
        conf_allowed = state == "normal" and is_currently_connected
        conf_state = "normal" if conf_allowed else "disabled"

        # MCConf Tab
        if hasattr(self, "read_mcconf_button"):
            self.read_mcconf_button.configure(state=conf_state)
        if hasattr(self, "write_mcconf_button"):
            self.write_mcconf_button.configure(state=conf_state)
        self._set_mcconf_widgets_state(conf_state)

        # AppConf Tab
        if hasattr(self, "read_appconf_button"):
            self.read_appconf_button.configure(state=conf_state)
        if hasattr(self, "write_appconf_button"):
            self.write_appconf_button.configure(state=conf_state)
        self._set_appconf_widgets_state(conf_state)

        # Detection Tab
        if hasattr(self, "run_detection_button"):
            self.run_detection_button.configure(state=conf_state)
        if hasattr(self, "max_loss_entry"):
            self.max_loss_entry.configure(state=conf_state)

    def _set_mcconf_widgets_state(self, state):
        """Enable/disable all widgets in the MCConf tab."""
        for widget in self.mcconf_widgets.values():
            if widget:
                widget.configure(state=state)

    def _set_appconf_widgets_state(self, state):
        """Enable/disable all widgets in the AppConf tab."""
        for widget in self.appconf_widgets.values():
            if widget:
                widget.configure(state=state)

    # --- Port Handling ---
    def _update_port_list(self):
        # (Same as before)
        self.port_list = [p.device for p in serial.tools.list_ports.comports()]
        if not self.port_list:
            self.port_list = ["No ports found"]
            self.port_combobox.set("")
        else:
            current = self.port_combobox.get()
            if current not in self.port_list:
                self.port_combobox.set(
                    self.port_list[0] if self.port_list[0] != "No ports found" else ""
                )
        self.port_combobox.configure(values=self.port_list)

    def _port_selected(self, choice):
        pass

    # --- Connection Logic ---
    def _toggle_connection(self):
        # (Same as before, calls _set_ui_state)
        if not self.connected:
            port = self.port_combobox.get()
            baud = DEFAULT_BAUD_RATE
            if not port or port == "No ports found":
                self._log("Error: Select valid port.")
                return
            self._log(f"Connecting to {port}...")
            self._set_ui_state("disabled")
            self.connect_button.configure(text="Connecting...")
            thread = threading.Thread(
                target=self._connect_vesc_thread, args=(port, int(baud)), daemon=True
            )
            thread.start()
        else:
            self._log("Disconnecting...")
            self._set_ui_state("disabled")
            self.connect_button.configure(text="Disconnecting...")
            thread = threading.Thread(target=self._disconnect_vesc_thread, daemon=True)
            thread.start()

    def _connect_vesc_thread(self, port, baudrate):
        # (Same as before)
        try:
            self.vesc_serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=SERIAL_TIMEOUT
            )
            time.sleep(1.5)
            self.vesc_serial.reset_input_buffer()
            time.sleep(0.1)
            if self.vesc_serial.in_waiting > 0:
                self.vesc_serial.read(self.vesc_serial.in_waiting)
            if self.vesc_serial.is_open:
                self.connected = True
                self._log(f"Connected to {port}")
                self.after(0, self._update_connection_ui, True)
            else:
                raise serial.SerialException("Port did not open.")
        except serial.SerialException as e:
            self.vesc_serial = None
            self.connected = False
            self._log(f"Error: {e}")
            self.after(0, self._update_connection_ui, False)
        except Exception as e:
            self.vesc_serial = None
            self.connected = False
            self._log(f"Connection failed: {e}")
            self.after(0, self._update_connection_ui, False)

    def _disconnect_vesc_thread(self):
        # (Same as before, includes clearing config)
        if self.vesc_serial and self.vesc_serial.is_open:
            try:
                self.vesc_serial.close()
                self._log("Disconnected.")
            except Exception as e:
                self._log(f"Error during disconnect: {e}")
        self.vesc_serial = None
        self.connected = False
        self.current_mc_config = None
        self.current_app_config = None
        self.after(0, self._clear_config_displays)
        self.after(0, self._update_connection_ui, False)

    def _update_connection_ui(self, is_connected):
        # (Updated to call _set_ui_state)
        self.connected = is_connected
        if self.connected:
            self.connect_button.configure(text="Disconnect")
            self._set_ui_state("normal")  # Enable UI elements now connected

            # --- 자동 읽기 시작 ---
            self._log("Connection successful. Auto-reading configurations...")
            # 약간의 딜레이 후 MCConf 읽기 시작 (UI 업데이트 반영 시간)
            self.after(100, self._read_mcconf_gui)
            # --- 자동 읽기 끝 ---

        else:
            self.connect_button.configure(text="Connect")
            # 연결 해제 시 UI 상태 재설정 (연결 관련 버튼만 활성화)
            self._set_ui_state(
                "normal"
            )  # 이렇게 하면 연결 관련 UI만 활성화됨 (내부 로직 확인)

    # --- VESC Communication Wrappers (Thread Handling) ---
    def _run_in_thread(self, target_func, args=(), callback=None):
        # (Same as before)
        if not self.connected or not self.vesc_serial:
            self._log("Error: Not connected.")
            return
        self._set_ui_state("disabled")
        self._log(f"Starting: {target_func.__name__}...")

        def wrap():
            res, err = None, None
            try:
                res = target_func(*args)
            except serial.SerialTimeoutException:
                err = "Timeout"
            except serial.SerialException as e:
                err = f"Serial Error: {e}"
                self.after(0, self._handle_serial_error)
            except ValueError as e:
                err = f"Data Error: {e}"
            except Exception as e:
                err = f"Error: {e}"
                import traceback

                traceback.print_exc()
            self.after(0, self._operation_complete, res, err, callback)

        thread = threading.Thread(target=wrap, daemon=True)
        thread.start()

    def _operation_complete(self, result, error, callback):
        # (Same as before - note: UI state re-enabled AFTER callback)
        self._set_ui_state("normal")  # Re-enable UI based on connection status first

        if error:
            self._log(f"Error: {error}")
            # 에러 발생 시에도 콜백을 호출하여 에러 처리를 할 수 있게 함
            if callback:
                callback(None, error)  # 결과는 None, 에러 전달
        else:
            self._log(f"Operation completed.")
            if callback:
                callback(result, None)  # 결과 전달, 에러는 None
        # self._set_ui_state("normal")  # Re-enable UI based on connection status

    def _handle_serial_error(self):
        # (Same as before)
        self._log("Serial error detected. Disconnecting.")
        if self.connected:
            self._toggle_connection()

    # --- Backend VESC Functions (Called by _run_in_thread) ---
    # _clear_input_buffer, _read_vesc_response, _backend_read_mcconf,
    # _backend_read_appconf, _backend_write_mcconf, _backend_write_appconf,
    # _backend_run_detection
    # --- (These functions remain the same as in the previous version) ---
    def _clear_input_buffer(self):
        if self.vesc_serial and self.vesc_serial.is_open:
            self.vesc_serial.reset_input_buffer()
            time.sleep(0.05)
            if self.vesc_serial.in_waiting > 0:
                try:
                    self.vesc_serial.read(self.vesc_serial.in_waiting)
                except:
                    pass

    def _read_vesc_response(self, expected_id, timeout_override=None):
        if not self.vesc_serial:
            raise serial.SerialException("Serial port not available.")
        start_time = time.time()
        read_timeout = timeout_override if timeout_override else SERIAL_TIMEOUT
        buffer = b""
        while time.time() - start_time < read_timeout:
            if self.vesc_serial.in_waiting > 0:
                buffer += self.vesc_serial.read(self.vesc_serial.in_waiting)
                try:
                    payload, consumed = unframe(buffer)
                    if payload is not None:
                        if payload[0] == expected_id:
                            return payload[1:], buffer
                        else:
                            buffer = buffer[consumed:]
                            continue
                    elif consumed > 0:
                        buffer = buffer[consumed:]
                except Exception:
                    pass
            time.sleep(0.02)
        raise serial.SerialTimeoutException(
            f"Timeout for ID {expected_id}. Buf: {buffer[:100]}..."
        )

    def _backend_read_mcconf(self):
        self._clear_input_buffer()
        request_packet = encode_request(GetMcConfRequest)
        self.vesc_serial.write(request_packet)
        payload, _ = self._read_vesc_response(GetMcConfRequest.id)
        config = parse_mc_conf_serialized(payload)
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError("Bad MCConf")
        return config

    def _backend_read_appconf(self):
        self._clear_input_buffer()
        request_packet = encode_request(GetAppConfRequest)
        self.vesc_serial.write(request_packet)
        payload, _ = self._read_vesc_response(GetAppConfRequest.id)
        config = parse_app_conf_serialized(payload)
        if not config or "APPCONF_SIGNATURE" not in config:
            raise ValueError("Bad AppConf")
        return config

    def _backend_write_mcconf(self, config):
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError("Invalid MCConf")
        set_message = SetMcConf()
        set_message.mc_configuration = config
        set_packet = encode_set_mcconf(set_message)
        self._clear_input_buffer()
        self.vesc_serial.write(set_packet)
        time.sleep(0.5)
        return True

    def _backend_write_appconf(self, config):
        if not config or "APPCONF_SIGNATURE" not in config:
            raise ValueError("Invalid AppConf")
        set_message = SetAppConf()
        set_message.app_configuration = config
        set_packet = encode_set_appconf(set_message)
        self._clear_input_buffer()
        self.vesc_serial.write(set_packet)
        time.sleep(0.5)
        return True

    def _backend_run_detection(self, max_loss):
        wizard_message = DetectApplyAllFOC()
        wizard_message.detect_can = False
        wizard_message.max_power_loss = float(max_loss)
        wizard_message.min_current_in = 0.0
        wizard_message.max_current_in = 0.0
        wizard_message.openloop_rpm = 0.0
        wizard_message.sl_erpm = 0.0
        packet_wizard = encode_detect_apply_all_foc(wizard_message)
        self._clear_input_buffer()
        self.vesc_serial.write(packet_wizard)
        payload, raw = self._read_vesc_response(
            GetMcConfRequest.id, timeout_override=DETECTION_WIZARD_TIMEOUT_GUI
        )
        detected_config = parse_mc_conf_serialized(payload)
        if not detected_config or "MCCONF_SIGNATURE" not in detected_config:
            raise ValueError(f"Bad detection response. Raw: {raw[:100]}...")
        return detected_config

    # --- GUI Action Handlers (Refactored for separate tabs) ---

    def _read_mcconf_gui(self):
        """Reads MCConf and updates the Motor Config tab."""
        self._log("Reading MCConf...")
        self._run_in_thread(
            self._backend_read_mcconf, callback=self._handle_mcconf_read_result
        )

    def _handle_mcconf_read_result(self, result, error):
        """Callback after MCConf read attempt. Populates MCConf widgets."""
        if error:
            self._log(f"MCConf Read Error: {error}")
            self.current_mc_config = None
            # --- MCConf 실패 시에도 AppConf 읽기 시도 (선택적) ---
            self._log("Attempting to read AppConf despite MCConf error...")
            self.after(100, self._read_appconf_gui)  # 다음 읽기 예약
            # --- AppConf 읽기 시도 끝 ---
            return  # 실패 시 위젯 업데이트 중단

        # MCConf Read Success
        self.current_mc_config = result
        self._log("MCConf read successfully. Populating fields...")
        self._populate_widgets(self.mcconf_widgets, result)  # Use helper

        # --- 성공 시 AppConf 자동 읽기 시작 ---
        self._log("Auto-reading AppConf...")
        self.after(100, self._read_appconf_gui)  # 다음 읽기 예약
        # --- AppConf 자동 읽기 끝 ---

    def _read_appconf_gui(self):
        """Reads AppConf and updates the App Config tab."""
        self._log("Reading AppConf...")
        self._run_in_thread(
            self._backend_read_appconf, callback=self._handle_appconf_read_result
        )

    def _handle_appconf_read_result(self, result, error):
        """Callback after AppConf read attempt. Populates AppConf widgets."""
        if error:
            self._log(f"AppConf Read Error: {error}")
            self.current_app_config = None
            # AppConf 읽기 실패 시 추가 작업 없음 (자동 읽기 완료)
            return  # Stop here

        # AppConf Read Success
        self.current_app_config = result
        self._log("AppConf read successfully. Populating fields...")
        self._populate_widgets(self.appconf_widgets, result)  # Use helper
        self._log("Auto-reading finished.")  # 모든 자동 읽기 완료 로그

    def _populate_widgets(self, widget_dict, config_data):
        """Helper to populate a set of widgets from config data."""
        print(f"--- Populating Widgets ---")
        for key, widget in widget_dict.items():
            if not widget:
                continue
            print(f"Processing key: '{key}', Widget: {type(widget)}")
            if key in config_data:
                value = config_data[key]
                print(f"  Value found: {value} (Type: {type(value)})")
                try:
                    if isinstance(widget, customtkinter.CTkComboBox):
                        # 1. Correct map and default value
                        map_to_use = None
                        is_bool = False
                        default_str = "Unknown"  # 기본값 설정

                        if key in [
                            "m_invert_direction",
                            "m_allow_braking",
                            # "uavcan_raw_mode",
                        ]:
                            map_to_use = BOOL_MAP
                            is_bool = True
                        elif key == "motor_type":
                            map_to_use = MOTOR_TYPES_MAP
                        elif key == "m_sensor_port_mode":
                            map_to_use = SENSOR_PORT_MODES_MAP
                        elif key == "foc_sensor_mode":
                            map_to_use = FOC_SENSOR_MODES_MAP
                        elif key == "app_to_use":
                            map_to_use = APP_MODES_MAP
                        elif key == "can_baud_rate":
                            map_to_use = CAN_BAUD_RATES_MAP
                        elif key == "can_mode":
                            map_to_use = CAN_MODES_MAP
                        elif key == "uavcan_raw_mode":
                            map_to_use = UAVCAN_RAW_MODES_MAP
                        elif key == "app_ppm_conf.ctrl_type":
                            map_to_use = PPM_CONTROL_TYPES_MAP

                        # 2. Get the string value from the map
                        value_to_set = default_str  # 기본값으로 시작
                        if map_to_use:
                            # 불리언 키는 bool()로 변환 후 매핑 조회
                            lookup_key = bool(value) if is_bool else value
                            value_to_set = map_to_use.get(lookup_key, default_str)
                        else:
                            # 매핑 없는 콤보박스면 그냥 문자열 변환 (이 경우는 없어야 함)
                            value_to_set = str(value)

                        # 3. Set the string value using widget.set()
                        widget.set(value_to_set)
                        print(f"  Set ComboBox to: '{value_to_set}'")

                    elif isinstance(widget, customtkinter.CTkEntry):
                        # 1. Format the value as a string
                        #    Floats: 소수점 4자리까지 표시 (필요에 따라 조정)
                        #    Others: 그냥 문자열 변환
                        value_str = (
                            f"{value:.4f}" if isinstance(value, float) else str(value)
                        )

                        # 2. Delete existing content first
                        widget.delete(0, "end")

                        # 3. Insert the new string value at the beginning (index 0)
                        widget.insert(0, value_str)
                        print(f"  Set Entry to: '{value_str}'")

                except Exception as e:
                    print(f"  ERROR populating widget '{key}': {e}")
                    # Optionally indicate error in the widget itself
                    if isinstance(widget, customtkinter.CTkEntry):
                        widget.delete(0, "end")
                        widget.insert(0, "ERROR")
                    elif isinstance(widget, customtkinter.CTkComboBox):
                        widget.set("ERROR")
            else:
                print(f"  WARNING: Key '{key}' not found in config data.")
                # Clear the widget if the key is missing
                if isinstance(widget, customtkinter.CTkEntry):
                    widget.delete(0, "end")
                    widget.insert(0, "N/A")
                elif isinstance(widget, customtkinter.CTkComboBox):
                    widget.set(
                        "N/A"
                    )  # Consider adding "N/A" to combo options if needed
        print(f"--- Finished Populating ---")

    def _write_mcconf_gui(self):
        """Gathers MCConf data from widgets, validates, and writes MCConf."""
        if not self.current_mc_config:
            self._log("Error: Read MCConf first before writing.")
            tkinter.messagebox.showerror("Error", "Read MCConf first.")
            return

        confirm = tkinter.messagebox.askyesno(
            "Confirm Write MCConf",
            "Overwrite Motor Config on VESC?\nIncorrect values can cause damage.\n\nProceed?",
        )
        if not confirm:
            self._log("MCConf write cancelled.")
            return

        config_to_write, errors = self._gather_and_validate_config(
            self.mcconf_widgets, self.current_mc_config
        )

        if errors:
            msg = "MCConf Validation Errors:\n\n" + "\n".join(errors)
            self._log("MCConf validation failed.")
            tkinter.messagebox.showerror("Validation Error", msg)
            return

        config_to_write.pop("crc", None)  # Ensure CRC not included
        self._log("MCConf validation passed. Writing MCConf...")
        self._run_in_thread(
            self._backend_write_mcconf,
            args=(config_to_write,),
            callback=self._handle_mcconf_write_result_final,
        )

    def _write_appconf_gui(self):
        """Gathers AppConf data from widgets, validates, and writes AppConf."""
        if not self.current_app_config:
            self._log("Error: Read AppConf first.")
            tkinter.messagebox.showerror("Error", "Read AppConf first.")
            return

        confirm = tkinter.messagebox.askyesno(
            "Confirm Write AppConf", "Overwrite Application Config on VESC?\n\nProceed?"
        )
        if not confirm:
            self._log("AppConf write cancelled.")
            return

        config_to_write, errors = self._gather_and_validate_config(
            self.appconf_widgets, self.current_app_config
        )

        if errors:
            msg = "AppConf Validation Errors:\n\n" + "\n".join(errors)
            self._log("AppConf validation failed.")
            tkinter.messagebox.showerror("Validation Error", msg)
            return

        config_to_write.pop("crc", None)
        self._log("AppConf validation passed. Writing AppConf...")
        self._run_in_thread(
            self._backend_write_appconf,
            args=(config_to_write,),
            callback=self._handle_appconf_write_result_final,
        )

    def _gather_and_validate_config(self, widget_dict, base_config):
        """Helper to gather data from widgets and validate against base config type."""
        config_to_write = copy.deepcopy(base_config)
        errors = []
        for key, widget in widget_dict.items():
            if not widget:
                continue
            try:
                gui_value_str = widget.get()
                original_value = base_config.get(key)
                original_type = (
                    type(original_value) if original_value is not None else str
                )

                # --- "N/A" 값 처리 추가 ---
                if gui_value_str in ["N/A", "Unknown", "ERROR"]:
                    # 'N/A'는 보통 읽기 실패 시 표시되므로, 쓰기 시에는 오류로 간주
                    errors.append(
                        f"Invalid value '{gui_value_str}' for {key}. Read settings again."
                    )
                    continue  # 다음 필드로 넘어감
                # --- "N/A" 값 처리 끝 ---

                vesc_value = None
                map_rev_to_use = None
                is_bool = False

                # 1. 어떤 역방향 맵을 사용할지 결정 (MCConf 및 AppConf 키 모두 포함)
                # MCConf Keys
                if key == "m_invert_direction":
                    map_rev_to_use = BOOL_MAP_REV
                    is_bool = True
                elif key == "s_pid_allow_braking":
                    map_rev_to_use = BOOL_MAP_REV
                    is_bool = True
                elif key == "motor_type":
                    map_rev_to_use = MOTOR_TYPES_MAP_REV
                elif key == "m_sensor_port_mode":
                    map_rev_to_use = (
                        SENSOR_PORT_MODES_MAP_REV  # <-- 정확한 키 이름 확인!
                    )
                elif key == "foc_sensor_mode":
                    map_rev_to_use = FOC_SENSOR_MODES_MAP_REV
                # AppConf Keys - !!!! 여기가 중요 !!!!
                elif key == "app_to_use":
                    map_rev_to_use = APP_MODES_MAP_REV
                elif key == "can_baud_rate":
                    map_rev_to_use = CAN_BAUD_RATES_MAP_REV
                elif key == "can_mode":
                    map_rev_to_use = (
                        CAN_MODES_MAP_REV  # <-- 이것도 콤보박스일 수 있음! 확인 필요
                    )
                elif (
                    key == "uavcan_raw_mode"
                ):  # uavcan_raw_mode 처리 블록 추가 또는 수정
                    map_rev_to_use = UAVCAN_RAW_MODES_MAP_REV
                elif key == "app_ppm_conf.ctrl_type":
                    map_rev_to_use = PPM_CONTROL_TYPES_MAP_REV  # <-- 이것도 콤보박스일 수 있음! 확인 필요

                # 2. 값 변환 시도
                if isinstance(widget, customtkinter.CTkComboBox) and map_rev_to_use:
                    if gui_value_str in map_rev_to_use:
                        vesc_value = map_rev_to_use[gui_value_str]
                    else:
                        errors.append(f"Invalid selection '{gui_value_str}' for {key}.")
                        continue
                elif isinstance(widget, customtkinter.CTkEntry):
                    try:
                        # AppConf는 주로 int 타입이 많으므로 int 변환 우선 고려
                        if original_type == int:
                            vesc_value = int(gui_value_str)
                        elif original_type == float:
                            vesc_value = float(gui_value_str)
                        else:
                            vesc_value = gui_value_str  # Fallback
                    except ValueError:
                        errors.append(
                            f"Invalid number format '{gui_value_str}' for {key}."
                        )
                        continue
                elif isinstance(
                    widget, customtkinter.CTkComboBox
                ):  # 콤보인데 map_rev_to_use가 None
                    # 이전에 정의되지 않은 콤보박스 키 처리
                    # !!!! can_status_mode 와 ppm_ctrl_type 가 이 경우에 해당할 수 있음 !!!!
                    # 이 키들이 콤보박스라면 위쪽 if/elif 블록에 추가해야 함.
                    errors.append(
                        f"Configuration error for ComboBox '{key}'. Check map definition."
                    )
                    continue
                else:
                    errors.append(
                        f"Unhandled widget type for key '{key}': {type(widget)}"
                    )
                    continue

                # 3. 변환된 vesc_value 저장
                if vesc_value is not None:
                    config_to_write[key] = vesc_value
                else:
                    errors.append(f"Value processing failed for {key}.")

            except KeyError as e:
                errors.append(
                    f"Invalid selection '{gui_value_str}' for {key} (Not in map: {e})."
                )
            except Exception as e:
                errors.append(f"Unexpected error processing '{key}': {e}")
                import traceback

                traceback.print_exc()

        return config_to_write, errors

    def _handle_mcconf_write_result_final(self, result, error):
        """Callback after MCConf write attempt."""
        if error:
            self._log(f"!!! MCConf Write FAILED: {error} !!!")
            tkinter.messagebox.showerror(
                "Write Error", f"Failed to write Motor Config:\n{error}"
            )
        elif result:
            self._log("MCConf write command sent successfully.")
            tkinter.messagebox.showinfo(
                "Write Success",
                "Motor Config written.\nRecommend re-reading to verify.",
            )
        else:
            self._log("MCConf write failed (unknown).")
            tkinter.messagebox.showerror(
                "Write Error", "Failed to write Motor Config (Unknown Reason)."
            )

    def _handle_appconf_write_result_final(self, result, error):
        """Callback after AppConf write attempt."""
        if error:
            self._log(f"!!! AppConf Write FAILED: {error} !!!")
            tkinter.messagebox.showerror(
                "Write Error", f"Failed to write App Config:\n{error}"
            )
        elif result:
            self._log("AppConf write command sent successfully.")
            tkinter.messagebox.showinfo(
                "Write Success", "App Config written.\nRecommend re-reading to verify."
            )
        else:
            self._log("AppConf write failed (unknown).")
            tkinter.messagebox.showerror(
                "Write Error", "Failed to write App Config (Unknown Reason)."
            )

    # --- Detection Tab ---
    def _run_detection_gui(self):
        # (Same as before, includes confirmation)
        max_loss_str = self.max_loss_entry.get()
        try:
            max_loss = float(max_loss_str)
        except ValueError:
            self._log("Err: Invalid Max Power Loss.")
            self._update_textbox(
                self.detection_output_textbox, "Err: Invalid Max Power Loss."
            )
            return

        confirm = tkinter.messagebox.askyesno(
            "Run Detection?", "Motor WILL spin/make noise!\nEnsure safety.\n\nProceed?"
        )
        if not confirm:
            self._log("Detection cancelled.")
            self._update_textbox(self.detection_output_textbox, "Detection cancelled.")
            return

        self._update_textbox(
            self.detection_output_textbox,
            f"Starting FOC Detection (Max Loss: {max_loss}W)...\nVESC measuring... DO NOT INTERRUPT.\nTimeout: {DETECTION_WIZARD_TIMEOUT_GUI}s",
        )
        self._run_in_thread(
            self._backend_run_detection,
            args=(max_loss,),
            callback=self._handle_detection_result,
        )

    def _handle_detection_result(self, result, error):
        # (Same as before, but no full config text box update)
        current_output = self.detection_output_textbox.get("1.0", "end-1c")
        if error:
            output = current_output + f"\n\nDETECTION FAILED:\n{error}"
            self._update_textbox(self.detection_output_textbox, output)
            self._log(f"Detection failed: {error}")
        elif result:
            # Update the internally stored MCConf immediately
            self.current_mc_config = result
            output = current_output + "\n\nDETECTION SUCCEEDED!\n"
            output += "New Motor Config received & applied by VESC.\n"
            output += "Motor Config tab fields updated.\n"
            output += "----------------------------------------------------\n"
            r = result.get("foc_motor_r")
            l = result.get("foc_motor_l")
            fl = result.get("foc_motor_flux_linkage")
            kp = result.get("foc_current_kp")
            ki = result.get("foc_current_ki")
            og = result.get("foc_observer_gain")
            output += f"R:{r*1e3:.3f}mΩ L:{l*1e6:.3f}uH λ:{fl*1e3:.3f}mWb\n"
            output += f"Kp:{kp:.6f} Ki:{ki:.6f}" + (f" Obs:{og:.2f}\n" if og else "\n")
            output += "----------------------------------------------------\n"
            self._update_textbox(self.detection_output_textbox, output)

            # Update the interactive widgets on Motor Config tab
            self._log("Populating Motor Config tab with detected values...")
            self._populate_widgets(self.mcconf_widgets, result)  # Use helper
            self._log("Detection successful. MCConf updated.")
        else:
            output = current_output + "\n\nDETECTION FAILED (Unknown)."
            self._update_textbox(self.detection_output_textbox, output)
            self._log("Detection failed (No data).")

    def _clear_config_displays(self):
        # (Clear specific widgets only)
        for widget in self.mcconf_widgets.values():
            if isinstance(widget, customtkinter.CTkEntry):
                widget.delete(0, "end")
            elif isinstance(widget, customtkinter.CTkComboBox):
                widget.set("")
        for widget in self.appconf_widgets.values():
            if isinstance(widget, customtkinter.CTkEntry):
                widget.delete(0, "end")
            elif isinstance(widget, customtkinter.CTkComboBox):
                widget.set("")
        # Clear detection output
        self._update_textbox(self.detection_output_textbox, "Disconnected.")

    # --- Window Closing ---
    def _on_closing(self):
        # (Same as before)
        print("Window closing...")
        if self.connected:
            self._log("Disconnecting before closing...")
            if self.vesc_serial and self.vesc_serial.is_open:
                try:
                    self.vesc_serial.close()
                    print("Serial port closed.")
                except Exception as e:
                    print(f"Error closing serial port on exit: {e}")
        self.destroy()


# --- Run the Application ---
if __name__ == "__main__":
    # DPI Scaling (Windows)
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError):
        pass  # Ignore if not windows or fails

    app = VescApp()
    app.mainloop()
