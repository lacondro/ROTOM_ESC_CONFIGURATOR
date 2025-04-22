# --- START OF FILE with_monitor.py ---

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
import math  # For position dial/slider

# crcmod 임포트 시도, 없으면 오류 메시지 표시 준비
try:
    import crcmod
except ImportError:
    print(
        "Error: 'crcmod' library not found. CRC calculation for terminal commands will fail."
    )
    print("Please install it: pip install crcmod")
    # crcmod를 사용할 수 없음을 나타내는 플래그 또는 None 처리
    crcmod = None

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
    from pyvesc.protocol.packet.codec import unframe
    from pyvesc.protocol.interface import encode_request, encode
    import pyvesc
    from pyvesc.VESC.messages import *

    # Getters
    from pyvesc.VESC.messages.getters import (
        GetMcConfRequest,
        GetAppConfRequest,
        GetValues,
        GetVersion,
        GetRotorPosition,
    )

    # Setters
    from pyvesc.VESC.messages.setters import (
        SetMcConf,
        SetAppConf,
        DetectApplyAllFOC,
        SetDutyCycle,
        SetCurrent,
        SetRPM,
        SetPosition,
    )

    # Parsers
    from pyvesc.VESC.messages.parser import (
        parse_mc_conf_serialized,
        parse_app_conf_serialized,
    )

    # Encoders / Utils
    from pyvesc.VESC.messages.vesc_protocol_utils import (
        encode_set_mcconf,
        encode_set_appconf,
        encode_detect_apply_all_foc,
    )

except ImportError as e:
    # crcmod 외 다른 임포트 오류 처리
    if "crcmod" not in str(e):
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
MONITOR_INTERVAL = 0.1
DETECTION_WIZARD_TIMEOUT_GUI = 120.0
READ_BUFFER_SIZE = 4096
CONNECTION_VERIFY_TIMEOUT = 2.0

# --- Parameter Mappings ---
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
CAN_MODES_MAP = {0: "VESC", 1: "UAVCAN", 2: "COMM BRIDGE", 3: "UNUSED"}
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
MC_FAULT_CODES_MAP = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
    7: "GATE_DRIVER_OVER_VOLTAGE",
    8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE",
    10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI",
    12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FLASH_CORRUPTION",
    15: "HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "UNBALANCED_CURRENTS",
    19: "BRK",
    20: "RESOLVER_LOT",
    21: "RESOLVER_DOS",
    22: "RESOLVER_LOS",
    23: "FLASH_CORRUPTION_APP_CFG",
    24: "FLASH_CORRUPTION_MC_CFG",
    25: "ENCODER_NO_MAGNET",
    26: "ENCODER_MAGNET_TOO_STRONG",
    27: "PHASE_FILTER",
    28: "ENCODER_FAULT",
    29: "LV_OUTPUT_FAULT",
}


# --- Reverse Mappings ---
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

CONTROL_MODES = ["Inactive", "Current", "Duty Cycle", "Position"]


# --- Main Application Class ---
class VescApp(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        self.title("ROTOM FLUXIUM CONFIGURATOR")
        self.geometry("900x750")
        self.vesc_serial: serial.Serial | None = None
        self.connected = False
        self.port_list = []
        self.current_mc_config = None
        self.current_app_config = None
        self.mcconf_widgets = {}
        self.appconf_widgets = {}
        self.monitor_widgets = {}
        self.monitor_control_widgets = {}
        self._monitor_running = False
        self._monitor_thread = None
        self._monitor_lock = threading.RLock()  # Use RLock
        self._control_mode = customtkinter.StringVar(value=CONTROL_MODES[0])
        self._current_limit_amps = 10.0
        self._command_queue = queue.Queue()
        # Temp storage for offset calculation steps
        self.temp_mc_config_for_offset = None
        # self.temp_rotor_pos_raw = None # No longer needed with terminal command

        customtkinter.set_appearance_mode("System")
        customtkinter.set_default_color_theme("blue")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self._create_sidebar()
        self._create_main_tabs()
        self._log("Ready. Select port and connect.")
        self._update_port_list()
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    # --- CRC Calculation and Packet Framing ---
    def _compute_crc16(self, data: bytes) -> bytes:
        """Computes CRC16 for VESC communication using crcmod."""
        if crcmod is None:
            raise ImportError(
                "crcmod library is required for CRC calculation but not found."
            )
        try:
            crc16 = crcmod.predefined.Crc("crc-ccitt-false")
            crc16.update(data)
            return crc16.crcValue.to_bytes(2, byteorder="big")
        except crcmod.crcmod.NoCRCModuleError:
            self._log(
                "Error: crcmod C extension not found. Using slower pure Python fallback."
            )
            poly = 0x1021
            crc = 0x0000
            for byte in data:
                crc ^= byte << 8
                for _ in range(8):
                    crc = (crc << 1) ^ poly if (crc & 0x8000) else crc << 1
            return (crc & 0xFFFF).to_bytes(2, byteorder="big")
        except Exception as e:
            raise RuntimeError(f"CRC16 calculation failed: {e}") from e

    def _frame_packet_manual(self, payload: bytes) -> bytes:
        """Manually frames a VESC packet (length, payload, CRC, start/end bytes)."""
        payload_len = len(payload)
        if payload_len <= 255:
            len_bytes = bytes([payload_len])
            start_byte = b"\x02"
        else:
            len_bytes = payload_len.to_bytes(2, byteorder="big")
            start_byte = b"\x03"
        crc_data = len_bytes + payload
        crc_bytes = self._compute_crc16(crc_data)
        packet = start_byte + crc_data + crc_bytes + b"\x03"
        return packet

    # --- UI Creation Methods ---
    def _create_sidebar(self):
        self.sidebar_frame = customtkinter.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=1, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(5, weight=1)
        self.logo_label = customtkinter.CTkLabel(
            self.sidebar_frame,
            text="ROTOM\nFLUXIUM",
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
        self.tabview = customtkinter.CTkTabview(
            self, width=650, command=self._on_tab_change
        )
        self.tabview.grid(row=0, column=1, padx=(20, 20), pady=(10, 20), sticky="nsew")
        self.tabview.add("Motor Config")
        self.tabview.add("App Config")
        self.tabview.add("Detection")
        self.tabview.add("Monitor")
        self._create_mcconf_tab(self.tabview.tab("Motor Config"))
        self._create_appconf_tab(self.tabview.tab("App Config"))
        self._create_detection_tab(self.tabview.tab("Detection"))
        self._create_monitor_tab(self.tabview.tab("Monitor"))

    def _create_mcconf_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        action_frame = customtkinter.CTkFrame(tab)
        action_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")
        self.read_mcconf_button = customtkinter.CTkButton(
            action_frame, text="Read Motor Config", command=self._read_mcconf_gui
        )
        self.read_mcconf_button.pack(side="left", padx=10, pady=5)
        self.write_mcconf_button = customtkinter.CTkButton(
            action_frame,
            text="Write Motor Config",
            command=self._write_mcconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        self.write_mcconf_button.pack(side="right", padx=10, pady=5)
        scroll_frame = customtkinter.CTkScrollableFrame(
            tab, label_text="Motor Parameters"
        )
        scroll_frame.grid(row=1, column=0, padx=10, pady=(5, 10), sticky="nsew")
        scroll_frame.grid_columnconfigure(1, weight=1)
        self.mcconf_widgets = {}
        row_mc = 0

        def add_mc_setting_local(key, label_text, widget_type, options=None):
            nonlocal row_mc
            label = customtkinter.CTkLabel(scroll_frame, text=label_text, anchor="w")
            label.grid(row=row_mc, column=0, padx=(5, 10), pady=5, sticky="w")
            if widget_type == "combobox":
                widget = customtkinter.CTkComboBox(
                    scroll_frame, values=options if options else [], state="readonly"
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

        add_mc_setting_local(
            "motor_type", "Motor Type:", "combobox", list(MOTOR_TYPES_MAP.values())
        )
        add_mc_setting_local(
            "m_invert_direction",
            "Invert Direction:",
            "combobox",
            list(BOOL_MAP.values()),
        )
        add_mc_setting_local(
            "m_sensor_port_mode",
            "Sensor Port Mode:",
            "combobox",
            list(SENSOR_PORT_MODES_MAP.values()),
        )
        add_mc_setting_local("l_current_max", "Motor Current Max (A):", "entry")
        add_mc_setting_local("l_abs_current_max", "Absolute Max Current (A):", "entry")
        add_mc_setting_local("l_in_current_max", "Batt Current Max (A):", "entry")
        add_mc_setting_local(
            "l_battery_cut_start", "Batt Volt Cutoff Start (V):", "entry"
        )
        add_mc_setting_local("l_battery_cut_end", "Batt Volt Cutoff End (V):", "entry")
        add_mc_setting_local(
            "foc_sensor_mode",
            "FOC Sensor Mode:",
            "combobox",
            list(FOC_SENSOR_MODES_MAP.values()),
        )
        add_mc_setting_local(
            "s_pid_allow_braking", "Allow Braking:", "combobox", list(BOOL_MAP.values())
        )
        add_mc_setting_local("p_pid_kp", "Position PID Kp:", "entry")
        add_mc_setting_local("p_pid_ki", "Position PID Ki:", "entry")
        add_mc_setting_local("p_pid_kd", "Position PID Kd:", "entry")
        sep_frame = customtkinter.CTkFrame(scroll_frame, height=2, fg_color="gray")
        sep_frame.grid(
            row=row_mc, column=0, columnspan=2, padx=5, pady=(10, 5), sticky="ew"
        )
        row_mc += 1
        set_zero_offset_button = customtkinter.CTkButton(
            scroll_frame,
            text="Set Current Position to 0 Degrees",
            command=self._apply_pos_offset_gui,
        )
        set_zero_offset_button.grid(
            row=row_mc, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )
        self.mcconf_widgets["set_zero_offset_button"] = set_zero_offset_button
        row_mc += 1
        self._set_mcconf_widgets_state("disabled")

    def _create_appconf_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        action_frame = customtkinter.CTkFrame(tab)
        action_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")
        self.read_appconf_button = customtkinter.CTkButton(
            action_frame, text="Read APP Config", command=self._read_appconf_gui
        )
        self.read_appconf_button.pack(side="left", padx=10, pady=5)
        self.write_appconf_button = customtkinter.CTkButton(
            action_frame,
            text="Write APP Config",
            command=self._write_appconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        self.write_appconf_button.pack(side="right", padx=10, pady=5)
        scroll_frame = customtkinter.CTkScrollableFrame(
            tab, label_text="Application Parameters"
        )
        scroll_frame.grid(row=1, column=0, padx=10, pady=(5, 10), sticky="nsew")
        scroll_frame.grid_columnconfigure(1, weight=1)
        self.appconf_widgets = {}
        row_app = 0

        def add_app_setting(key, label_text, widget_type, options=None):
            nonlocal row_app
            label = customtkinter.CTkLabel(scroll_frame, text=label_text, anchor="w")
            label.grid(row=row_app, column=0, padx=(5, 10), pady=5, sticky="w")
            if widget_type == "combobox":
                widget = customtkinter.CTkComboBox(
                    scroll_frame, values=options if options else [], state="readonly"
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
            "can_mode", "CAN Mode:", "combobox", list(CAN_MODES_MAP.values())
        )
        add_app_setting("uavcan_esc_index", "UAVCAN ESC Index:", "entry")
        add_app_setting(
            "uavcan_raw_mode",
            "UAVCAN Raw Mode:",
            "combobox",
            list(UAVCAN_RAW_MODES_MAP.values()),
        )
        add_app_setting(
            "app_ppm_conf.ctrl_type",
            "PPM Control Type:",
            "combobox",
            list(PPM_CONTROL_TYPES_MAP.values()),
        )
        self._set_appconf_widgets_state("disabled")

    def _create_detection_tab(self, tab):
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
        self.motor_size_label = customtkinter.CTkLabel(
            self.param_frame, text="Select Motor Size:"
        )
        self.motor_size_label.pack(side="left", padx=(10, 5), pady=10)
        self.motor_size_options = [
            "Mini (~75g)",
            "Small (~200g)",
            "Medium (~750g)",
            "Large (~2000g)",
        ]
        self.motor_size_combobox = customtkinter.CTkComboBox(
            self.param_frame, values=self.motor_size_options, width=150
        )
        self.motor_size_combobox.pack(side="left", padx=(0, 20), pady=10)
        self.motor_size_combobox.set(self.motor_size_options[2])
        self.run_detection_button = customtkinter.CTkButton(
            self.param_frame,
            text="Run FOC Detection",
            command=self._run_detection_gui,
            fg_color="red",
            hover_color="dark red",
        )
        self.run_detection_button.pack(side="right", padx=10, pady=10)
        self.monitor_control_widgets["motor_size_label"] = self.motor_size_label
        self.monitor_control_widgets["motor_size_combobox"] = self.motor_size_combobox
        self.monitor_control_widgets["run_detection_button"] = self.run_detection_button
        self.detection_output_textbox = customtkinter.CTkTextbox(tab, wrap="word")
        self.detection_output_textbox.grid(
            row=3, column=0, padx=10, pady=(0, 10), sticky="nsew"
        )
        self.detection_output_textbox.configure(state="disabled")
        self._set_detection_widgets_state("disabled")

    def _set_detection_widgets_state(self, state):
        if hasattr(self, "motor_size_label") and self.motor_size_label.winfo_exists():
            pass
        if (
            hasattr(self, "motor_size_combobox")
            and self.motor_size_combobox.winfo_exists()
        ):
            try:
                self.motor_size_combobox.configure(
                    state=state if state != "normal" else "readonly"
                )
            except Exception as e:
                print(f"Warn: Error setting state for motor_size_combobox: {e}")
        if (
            hasattr(self, "run_detection_button")
            and self.run_detection_button.winfo_exists()
        ):
            try:
                self.run_detection_button.configure(state=state)
            except Exception as e:
                print(f"Warn: Error setting state for run_detection_button: {e}")

    def _create_monitor_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_rowconfigure(2, weight=3)
        control_frame = customtkinter.CTkFrame(tab)
        control_frame.grid(row=1, column=0, padx=10, pady=(10, 5), sticky="nsew")
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=0)
        slider_frame = customtkinter.CTkFrame(control_frame, fg_color="transparent")
        slider_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        slider_frame.grid_columnconfigure(1, weight=1)
        label_curr = customtkinter.CTkLabel(
            slider_frame, text="Current (A): 0.0", anchor="e"
        )
        label_curr.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        slider_curr = customtkinter.CTkSlider(
            slider_frame,
            from_=-self._current_limit_amps,
            to=self._current_limit_amps,
            number_of_steps=int(self._current_limit_amps * 2 * 10),
            command=self._slider_current_changed,
        )
        slider_curr.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        slider_curr.set(0)
        self.monitor_control_widgets["current_slider"] = slider_curr
        self.monitor_control_widgets["current_label"] = label_curr
        label_duty = customtkinter.CTkLabel(
            slider_frame, text="Duty Cycle: 0.00", anchor="e"
        )
        label_duty.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        slider_duty = customtkinter.CTkSlider(
            slider_frame,
            from_=-1.0,
            to=1.0,
            number_of_steps=200,
            command=self._slider_duty_changed,
        )
        slider_duty.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        slider_duty.set(0)
        self.monitor_control_widgets["duty_slider"] = slider_duty
        self.monitor_control_widgets["duty_label"] = label_duty
        label_pos = customtkinter.CTkLabel(
            slider_frame, text="Position (°): 0", anchor="e"
        )
        label_pos.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        slider_pos = customtkinter.CTkSlider(
            slider_frame,
            from_=0,
            to=360,
            number_of_steps=360,
            command=self._slider_position_changed,
        )
        slider_pos.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        slider_pos.set(0)
        self.monitor_control_widgets["position_slider"] = slider_pos
        self.monitor_control_widgets["position_label"] = label_pos
        mode_frame = customtkinter.CTkFrame(control_frame, fg_color="transparent")
        mode_frame.grid(row=0, column=1, padx=10, pady=5, sticky="ns")
        mode_label = customtkinter.CTkLabel(mode_frame, text="Control Mode:")
        mode_label.pack(pady=(0, 5))
        for mode in CONTROL_MODES:
            rb = customtkinter.CTkRadioButton(
                mode_frame,
                text=mode,
                variable=self._control_mode,
                value=mode,
                command=self._control_mode_changed,
            )
            rb.pack(anchor="w", pady=2)
            self.monitor_control_widgets[f"mode_{mode.lower().replace(' ', '_')}"] = rb
        stop_button = customtkinter.CTkButton(
            mode_frame,
            text="STOP MOTOR",
            command=self._send_stop_command,
            fg_color="red",
            hover_color="dark red",
        )
        stop_button.pack(pady=(15, 5), fill="x")
        self.monitor_control_widgets["stop_button"] = stop_button
        data_frame = customtkinter.CTkScrollableFrame(tab, label_text="Realtime Data")
        data_frame.grid(row=2, column=0, padx=10, pady=(5, 10), sticky="nsew")
        data_frame.grid_columnconfigure(1, weight=1)
        row_data = 0

        def add_monitor_value(key, label_text):
            nonlocal row_data
            desc_label = customtkinter.CTkLabel(data_frame, text=label_text, anchor="w")
            desc_label.grid(row=row_data, column=0, padx=5, pady=2, sticky="w")
            value_label = customtkinter.CTkLabel(data_frame, text="---", anchor="w")
            value_label.grid(row=row_data, column=1, padx=5, pady=2, sticky="ew")
            self.monitor_widgets[key] = value_label
            row_data += 1

        add_monitor_value("v_in", "Input Voltage (V):")
        add_monitor_value("avg_motor_current", "Motor Current (A):")
        add_monitor_value("avg_input_current", "Input Current (A):")
        add_monitor_value("duty_cycle_now", "Duty Cycle:")
        add_monitor_value("rpm", "RPM:")
        add_monitor_value("temp_fet", "FET Temp (°C):")
        add_monitor_value("temp_motor", "Motor Temp (°C):")
        add_monitor_value("pid_pos_now", "PID Position:")
        add_monitor_value("mc_fault_code", "Fault Code:")
        self._set_monitor_widgets_state("disabled")

    # --- UI Update & Logging ---
    def _log(self, message):
        print(f"LOG: {message}")
        (
            hasattr(self, "status_label")
            and self.status_label.winfo_exists()
            and self.after(0, self._update_status_label, message)
        )

    def _update_status_label(self, message):
        (
            hasattr(self, "status_label")
            and self.status_label.winfo_exists()
            and self.status_label.configure(text=f"Status: {message}")
        )

    def _update_textbox(self, textbox, content, read_only=True):
        try:
            if hasattr(self, "detection_output_textbox") and textbox.winfo_exists():
                textbox.configure(state="normal")
                textbox.delete("1.0", "end")
                textbox.insert("1.0", content)
                if read_only:
                    textbox.configure(state="disabled")
        except Exception as e:
            print(f"Error updating textbox: {e}")

    def _set_ui_state(self, state):
        is_currently_connected = self.connected
        allow_connect_ops = state == "normal"
        connect_button_state = "normal" if allow_connect_ops else "disabled"
        port_state = (
            "readonly"
            if (allow_connect_ops and not is_currently_connected)
            else "disabled"
        )
        refresh_state = "normal" if allow_connect_ops else "disabled"
        if hasattr(self, "port_combobox") and self.port_combobox.winfo_exists():
            self.port_combobox.configure(state=port_state)
        if hasattr(self, "refresh_button") and self.refresh_button.winfo_exists():
            self.refresh_button.configure(state=refresh_state)
        if hasattr(self, "connect_button") and self.connect_button.winfo_exists():
            self.connect_button.configure(state=connect_button_state)
        conf_allowed = state == "normal" and is_currently_connected
        conf_state = "normal" if conf_allowed else "disabled"
        if (
            hasattr(self, "read_mcconf_button")
            and self.read_mcconf_button.winfo_exists()
        ):
            self.read_mcconf_button.configure(state=conf_state)
        if (
            hasattr(self, "write_mcconf_button")
            and self.write_mcconf_button.winfo_exists()
        ):
            self.write_mcconf_button.configure(state=conf_state)
        self._set_mcconf_widgets_state(conf_state)
        if (
            hasattr(self, "read_appconf_button")
            and self.read_appconf_button.winfo_exists()
        ):
            self.read_appconf_button.configure(state=conf_state)
        if (
            hasattr(self, "write_appconf_button")
            and self.write_appconf_button.winfo_exists()
        ):
            self.write_appconf_button.configure(state=conf_state)
        self._set_appconf_widgets_state(conf_state)
        self._set_detection_widgets_state(conf_state)
        self._set_monitor_widgets_state(conf_state)

    def _set_mcconf_widgets_state(self, state):
        for key, widget in self.mcconf_widgets.items():
            if widget and widget.winfo_exists():
                try:
                    if isinstance(
                        widget, (customtkinter.CTkButton, customtkinter.CTkEntry)
                    ):
                        widget.configure(state=state)
                    elif isinstance(widget, customtkinter.CTkComboBox):
                        widget.configure(
                            state=state if state == "disabled" else "readonly"
                        )
                except Exception as e:
                    print(f"Warn: Failed to configure MCConf widget '{key}' state: {e}")

    def _set_appconf_widgets_state(self, state):
        for widget in self.appconf_widgets.values():
            if widget and widget.winfo_exists():
                try:
                    widget.configure(state=state)
                except Exception as e:
                    print(f"Warn: Failed to configure AppConf widget state: {e}")

    def _set_monitor_widgets_state(self, state):
        current_mode = self._control_mode.get()
        is_connected = self.connected
        general_state = "normal" if (state == "normal" and is_connected) else "disabled"
        for key, widget in self.monitor_control_widgets.items():
            if widget and widget.winfo_exists():
                try:
                    widget.configure(state=general_state)
                except Exception as e:
                    print(
                        f"Warn: Failed to configure Monitor widget '{key}' base state: {e}"
                    )
        if general_state == "normal":
            try:
                if (
                    "current_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["current_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["current_slider"].configure(
                        state="normal" if current_mode == "Current" else "disabled"
                    )
                if (
                    "duty_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["duty_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["duty_slider"].configure(
                        state="normal" if current_mode == "Duty Cycle" else "disabled"
                    )
                if (
                    "position_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["position_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["position_slider"].configure(
                        state="normal" if current_mode == "Position" else "disabled"
                    )
            except Exception as e:
                print(f"Warn: Failed to configure Monitor slider state: {e}")
        else:
            try:
                if (
                    "current_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["current_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["current_slider"].configure(
                        state="disabled"
                    )
                if (
                    "duty_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["duty_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["duty_slider"].configure(
                        state="disabled"
                    )
                if (
                    "position_slider" in self.monitor_control_widgets
                    and self.monitor_control_widgets["position_slider"].winfo_exists()
                ):
                    self.monitor_control_widgets["position_slider"].configure(
                        state="disabled"
                    )
            except Exception as e:
                print(f"Warn: Failed to disable Monitor sliders: {e}")

    # --- Port Handling ---
    def _update_port_list(self):
        try:
            self.port_list = [p.device for p in serial.tools.list_ports.comports()]
        except Exception as e:
            self._log(f"Error listing ports: {e}")
            self.port_list = ["Error listing ports"]
        if not self.port_list or self.port_list == ["Error listing ports"]:
            self.port_list = ["No ports found"]
        if hasattr(self, "port_combobox") and self.port_combobox.winfo_exists():
            current = self.port_combobox.get()
            if current not in self.port_list:
                self.port_combobox.set(
                    self.port_list[0]
                    if self.port_list[0]
                    not in ["No ports found", "Error listing ports"]
                    else ""
                )
            self.port_combobox.configure(values=self.port_list)

    def _port_selected(self, choice):
        pass

    # --- Connection Logic ---
    def _toggle_connection(self):
        if not self.connected:
            if (
                not hasattr(self, "port_combobox")
                or not self.port_combobox.winfo_exists()
            ):
                return
            port = self.port_combobox.get()
            baud = DEFAULT_BAUD_RATE
            if not port or port in ["No ports found", "Error listing ports"]:
                self._log("Error: Select valid port.")
                tkinter.messagebox.showerror(
                    "Connection Error", "Please select a valid serial port."
                )
                return
            self._log(f"Connecting to {port}...")
            self._set_ui_state("disabled")
            if hasattr(self, "connect_button") and self.connect_button.winfo_exists():
                self.connect_button.configure(text="Connecting...")
            thread = threading.Thread(
                target=self._connect_vesc_thread, args=(port, int(baud)), daemon=True
            )
            thread.start()
        else:
            self._log("Disconnecting...")
            self._set_ui_state("disabled")
            if hasattr(self, "connect_button") and self.connect_button.winfo_exists():
                self.connect_button.configure(text="Disconnecting...")
            thread = threading.Thread(target=self._disconnect_vesc_thread, daemon=True)
            thread.start()

    def _connect_vesc_thread(self, port, baudrate):
        try:
            if self.vesc_serial and self.vesc_serial.is_open:
                try:
                    self.vesc_serial.close()
                except:
                    pass
                    self.vesc_serial = None
            self.vesc_serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=SERIAL_TIMEOUT
            )
            time.sleep(0.5)
            self.vesc_serial.reset_input_buffer()
            time.sleep(0.1)
            if self.vesc_serial.in_waiting > 0:
                self.vesc_serial.read(self.vesc_serial.in_waiting)
            time.sleep(0.1)
            self.vesc_serial.reset_output_buffer()
            if not self.vesc_serial.is_open:
                raise serial.SerialException("Port did not open after Serial() call.")
            self._log("Verifying connection (waiting for firmware version)...")
            request = encode_request(GetVersion)
            self.vesc_serial.write(request)
            start_verify_time = time.monotonic()
            read_buffer = b""
            verified = False
            fw_version_response = None
            while time.monotonic() - start_verify_time < CONNECTION_VERIFY_TIMEOUT:
                try:
                    if self.vesc_serial.in_waiting > 0:
                        read_buffer += self.vesc_serial.read(
                            self.vesc_serial.in_waiting
                        )
                    while True:
                        if not read_buffer:
                            break
                            response, consumed = None, 0
                        try:
                            response, consumed = pyvesc.decode(read_buffer)
                            if consumed > 0:
                                if isinstance(response, GetVersion):
                                    fw_version_response = response
                                    verified = True
                                    read_buffer = read_buffer[consumed:]
                                    break
                                else:
                                    read_buffer = read_buffer[consumed:]
                            else:
                                break
                        except (
                            ValueError,
                            IndexError,
                            struct.error,
                            AttributeError,
                            NotImplementedError,
                        ):
                            next_sof = -1
                            sof2_idx = read_buffer.find(b"\x02", 1)
                            sof3_idx = read_buffer.find(
                                b"\x03", 1
                            )  # Corrected find start index
                            if sof2_idx != -1 and sof3_idx != -1:
                                next_sof = min(sof2_idx, sof3_idx)
                            elif sof2_idx != -1:
                                next_sof = sof2_idx
                            elif sof3_idx != -1:
                                next_sof = sof3_idx  # Corrected elif
                            if next_sof > 0:
                                read_buffer = read_buffer[next_sof:]
                            elif (
                                next_sof == -1
                                and len(read_buffer) > READ_BUFFER_SIZE / 4
                            ):
                                read_buffer = read_buffer[int(len(read_buffer) / 4) :]
                                break
                            else:
                                break
                    if verified:
                        break
                except serial.SerialException as serial_verify_err:
                    raise serial.SerialException(
                        f"Serial error during verification: {serial_verify_err}"
                    )
                except Exception as verify_loop_err:
                    raise Exception(
                        f"Unexpected error during verification loop: {verify_loop_err}"
                    )
                if not verified:
                    time.sleep(0.05)
            if verified:
                self.connected = True
                self._log(f"Connected to {port}. VESC FW: {fw_version_response}")
                self.after(0, self._update_connection_ui, True)
            else:
                raise serial.SerialTimeoutException(
                    f"Verification failed (Did not receive GetVersion within {CONNECTION_VERIFY_TIMEOUT}s). Check VESC UART settings."
                )
        except serial.SerialException as e:
            if self.vesc_serial and self.vesc_serial.is_open:
                try:
                    self.vesc_serial.close()
                except:
                    pass
            self.vesc_serial = None
            self.connected = False
            self._log(f"Connection Error: {e}")
            self.after(
                0,
                tkinter.messagebox.showerror,
                "Connection Error",
                f"Could not connect to {port}:\n{e}",
            )
            self.after(0, self._update_connection_ui, False)
        except Exception as e:
            if self.vesc_serial and self.vesc_serial.is_open:
                try:
                    self.vesc_serial.close()
                except:
                    pass
            self.vesc_serial = None
            self.connected = False
            self._log(f"Unexpected Connection failed: {e}")
            import traceback

            traceback.print_exc()
            self.after(
                0,
                tkinter.messagebox.showerror,
                "Connection Error",
                f"An unexpected error occurred:\n{e}",
            )
            self.after(0, self._update_connection_ui, False)

    def _disconnect_vesc_thread(self):
        """Handles disconnection: stops monitor, sends stop command, closes port, resets state."""
        self._stop_monitor_loop()  # 모니터 루프 먼저 중지
        time.sleep(MONITOR_INTERVAL * 1.1)  # 잠시 대기

        # 시리얼 포트 닫기 (STOP 명령 포함)
        if self.vesc_serial and self.vesc_serial.is_open:
            try:
                stop_cmd = encode(SetCurrent(0))
                with self._monitor_lock:
                    if self.vesc_serial.is_open:
                        self.vesc_serial.write(stop_cmd)
                        time.sleep(0.1)  # 명령 전송 후 짧은 대기
                        self.vesc_serial.close()
                        self._log("Disconnected.")
                    else:
                        self._log("Disconnected (port already closed).")
            except Exception as e:
                self._log(f"Error during disconnect (stop/close): {e}")
                if self.vesc_serial.is_open:
                    try:
                        self.vesc_serial.close()
                        self._log("Serial port closed after error.")
                    except Exception as close_e:
                        self._log(
                            f"Error closing serial port after disconnect error: {close_e}"
                        )

        # 내부 상태 변수 리셋
        self.vesc_serial = None
        self.connected = False
        self.current_mc_config = None
        self.current_app_config = None

        # --- GUI 업데이트 예약 제거 ---
        # 한국어 주석: 이 스레드에서는 GUI 업데이트를 예약하지 않습니다.
        # self.after(0, self._clear_config_displays)
        # self.after(0, self._reset_monitor_display)
        # self.after(0, self._update_connection_ui, False)
        # --- 제거 끝 ---

        # 한국어 주석: UI 업데이트는 이 스레드를 시작한 _toggle_connection 또는 _on_closing 에서 처리해야 함
        #             또는 프로그램 종료 시에는 굳이 UI를 업데이트할 필요 없음
        #             하지만 상태 변경은 로그로 남김
        self._log("Internal state reset after disconnection.")
        # 연결 해제 완료 후 메인 스레드에서 UI 상태 업데이트를 트리거할 수 있도록 플래그 설정 또는 이벤트 사용 가능 (선택적 고급 구현)
        # 예: self.after(0, self._finalize_disconnection_ui)
        # 여기서는 단순히 스레드 종료. _on_closing 이나 _toggle_connection에서 UI 마무리.

    def _update_connection_ui(self, is_connected):
        self.connected = is_connected
        if self.connected:
            if hasattr(self, "connect_button") and self.connect_button.winfo_exists():
                self.connect_button.configure(text="Disconnect")
            self._set_ui_state("normal")
            self._log("Connection successful. Auto-reading configurations...")
            self.after(200, self._read_mcconf_gui)
            current_tab = ""
            if hasattr(self, "tabview") and self.tabview.winfo_exists():
                try:
                    current_tab = self.tabview.get()
                except:
                    pass
            if current_tab == "Monitor":
                self.after(500, self._start_monitor_loop)
        else:
            if hasattr(self, "connect_button") and self.connect_button.winfo_exists():
                self.connect_button.configure(text="Connect")
            self._stop_monitor_loop()
            self._set_ui_state("normal")
            self._current_limit_amps = 10.0
            self._update_current_slider_range()
            self._clear_config_displays()
            self._reset_monitor_display()

    # --- VESC Communication Wrappers ---
    def _run_in_thread(self, target_func, args=(), callback=None):
        if not self.connected or not self.vesc_serial:
            self._log("Error: Not connected.")
            (callback and self.after(0, callback, None, "Not Connected"))
            return
        self._set_ui_state("disabled")
        self._log(f"Starting: {target_func.__name__}...")

        def wrap():
            res, err = None, None
            original_timeout = SERIAL_TIMEOUT
            try:
                if hasattr(self.vesc_serial, "timeout"):
                    original_timeout = self.vesc_serial.timeout
                    new_timeout = SERIAL_TIMEOUT
                    if target_func == self._backend_run_detection:
                        new_timeout = DETECTION_WIZARD_TIMEOUT_GUI + 5.0
                    self.vesc_serial.timeout = new_timeout
                res = target_func(*args)
            except serial.SerialTimeoutException as e:
                err = f"Timeout during {target_func.__name__}: {e}"
            except serial.SerialException as e:
                err = f"Serial Error during {target_func.__name__}: {e}"
                self.after(0, self._handle_serial_error)
            except (
                ValueError,
                ConnectionError,
                NotImplementedError,
                RuntimeError,
                ImportError,
            ) as e:  # Catch more specific errors
                err = f"{type(e).__name__} during {target_func.__name__}: {e}"
            except AttributeError as e:
                err = f"Attribute Error (likely disconnected) during {target_func.__name__}: {e}"
            except Exception as e:
                err = f"Unexpected Error during {target_func.__name__}: {e}"
                import traceback

                traceback.print_exc()
            finally:
                if (
                    hasattr(self, "vesc_serial")
                    and self.vesc_serial
                    and hasattr(self.vesc_serial, "timeout")
                ):
                    try:
                        self.vesc_serial.timeout = original_timeout
                    except Exception as restore_e:
                        print(f"Warn: Could not restore serial timeout: {restore_e}")
                callback_name = callback.__name__ if callback else target_func.__name__
                self.after(
                    0, self._operation_complete, res, err, callback, callback_name
                )

        thread = threading.Thread(target=wrap, daemon=True)
        thread.start()

    def _operation_complete(self, result, error, callback, op_name="Unknown"):
        self._set_ui_state("normal")
        if error:
            self._log(f"Error during '{op_name}': {error}")
            (callback and callback(None, error))
        else:
            self._log(f"Operation '{op_name}' completed successfully.")
            (callback and callback(result, None))

    def _handle_serial_error(self):
        if self.connected:
            self._log("Serial error detected. Disconnecting.")
            self.after(0, self._initiate_disconnect_from_error)

    def _initiate_disconnect_from_error(self):
        if self.connected:
            self._log("Initiating disconnect due to serial error.")
            self._toggle_connection()

    # --- Backend VESC Functions ---
    def _clear_input_buffer(self):
        if self.vesc_serial and self.vesc_serial.is_open:
            try:
                self.vesc_serial.reset_input_buffer()
                time.sleep(0.02)
                (
                    self.vesc_serial.in_waiting > 0
                    and self.vesc_serial.read(self.vesc_serial.in_waiting)
                )
            except Exception:
                pass

    def _read_vesc_response(self, expected_id, timeout_override=None):
        if not self.vesc_serial or not self.vesc_serial.is_open:
            raise serial.SerialException("Serial port not available.")
        start_time = time.time()
        read_timeout = (
            timeout_override
            if timeout_override is not None
            else self.vesc_serial.timeout
        )
        buffer = b""
        packet_found = False
        response_payload = None
        while time.time() - start_time < read_timeout:
            try:
                if self.vesc_serial.in_waiting > 0:
                    buffer += self.vesc_serial.read(self.vesc_serial.in_waiting)
                while len(buffer) > 3:
                    try:
                        raw_payload, consumed = unframe(buffer)
                        if consumed > 0 and raw_payload is not None:
                            if raw_payload[0] == expected_id:
                                response_payload = raw_payload[1:]
                                packet_found = True
                                buffer = buffer[consumed:]
                                break
                            else:
                                buffer = buffer[consumed:]
                        elif consumed > 0 and raw_payload is None:
                            buffer = buffer[consumed:]
                        else:
                            break
                    except (ValueError, IndexError, struct.error, AttributeError):
                        next_sof = -1
                        sof2 = buffer.find(b"\x02", 1)
                        sof3 = buffer.find(b"\x03", 1)
                        if sof2 != -1 and sof3 != -1:
                            next_sof = min(sof2, sof3)
                        elif sof2 != -1:
                            next_sof = sof2
                        elif sof3 != -1:
                            next_sof = sof3
                        if next_sof > 0:
                            buffer = buffer[next_sof:]
                        else:
                            break
                if packet_found:
                    return response_payload, buffer
            except serial.SerialException as serial_err:
                raise serial.SerialException(f"Serial error during read: {serial_err}")
            except Exception as e:
                raise Exception(f"Unexpected error in read loop: {e}")
            time.sleep(0.01)
        raise serial.SerialTimeoutException(
            f"Timeout waiting for response ID {expected_id}. Buffer (first 100): {buffer[:100]}..."
        )

    def _backend_read_mcconf(self):
        with self._monitor_lock:
            self._clear_input_buffer()
            request_packet = encode_request(GetMcConfRequest)
            self.vesc_serial.write(request_packet)
            payload, _ = self._read_vesc_response(
                GetMcConfRequest.id, timeout_override=SERIAL_TIMEOUT
            )
        config = parse_mc_conf_serialized(payload)
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError(
                f"Bad MCConf received (parsing failed). Payload: {payload[:50]}..."
            )
        return config

    def _backend_read_appconf(self):
        with self._monitor_lock:
            self._clear_input_buffer()
            request_packet = encode_request(GetAppConfRequest)
            self.vesc_serial.write(request_packet)
            payload, _ = self._read_vesc_response(
                GetAppConfRequest.id, timeout_override=SERIAL_TIMEOUT
            )
        config = parse_app_conf_serialized(payload)
        if not config or (
            "APPCONF_SIGNATURE" not in config and "APPCONF_SİGNATURE" not in config
        ):
            raise ValueError(
                f"Bad AppConf received (parsing failed or missing signature). Payload: {payload[:50]}..."
            )
        return config

    def _backend_write_mcconf(self, config):
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError("Invalid MCConf for writing (Missing Signature)")
        set_message = SetMcConf()
        set_message.mc_configuration = config
        try:
            set_packet = encode_set_mcconf(set_message)
        except Exception as encode_e:
            raise ValueError(f"Failed to encode MCConf: {encode_e}")
        with self._monitor_lock:
            self._clear_input_buffer()
            self.vesc_serial.write(set_packet)
            time.sleep(0.5)
        return True

    def _backend_write_appconf(self, config):
        if "APPCONF_SIGNATURE" not in config and "APPCONF_SİGNATURE" not in config:
            raise ValueError("Invalid AppConf for writing (Missing Signature)")
        set_message = SetAppConf()
        set_message.app_configuration = config
        try:
            set_packet = encode_set_appconf(set_message)
        except Exception as encode_e:
            raise ValueError(f"Failed to encode AppConf: {encode_e}")
        with self._monitor_lock:
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
        try:
            packet_wizard = encode_detect_apply_all_foc(wizard_message)
        except Exception as encode_e:
            raise ValueError(f"Failed to encode Detection command: {encode_e}")
        detected_config = None
        with self._monitor_lock:
            self._clear_input_buffer()
            self.vesc_serial.write(packet_wizard)
            try:
                payload, raw_buffer = self._read_vesc_response(
                    GetMcConfRequest.id, timeout_override=DETECTION_WIZARD_TIMEOUT_GUI
                )
                detected_config = parse_mc_conf_serialized(payload)
                if not detected_config or "MCCONF_SIGNATURE" not in detected_config:
                    error_msg = f"Bad/incomplete detection response (parsing failed). Payload: {payload[:100]}"
                    text_response = ""
                    try:
                        text_response = raw_buffer.decode("utf-8", errors="ignore")
                    except:
                        pass

                    if text_response and (
                        "Detection results:" in text_response
                        or "Error:" in text_response
                        or "failed" in text_response
                    ):
                        error_msg += f" VESC Output: {text_response[:200]}"
                    raise ValueError(error_msg)
            except serial.SerialTimeoutException:
                error_msg = "Timeout waiting for detection results."
                extra_data = b""
                try:
                    if (
                        self.vesc_serial
                        and self.vesc_serial.is_open
                        and self.vesc_serial.in_waiting > 0
                    ):
                        extra_data = self.vesc_serial.read(self.vesc_serial.in_waiting)
                    if extra_data:
                        text_response = extra_data.decode("utf-8", errors="ignore")
                        error_msg += f" VESC Output: {text_response[:200]}"
                except Exception as read_e:
                    error_msg += f" (Error reading extra data: {read_e})"
                raise serial.SerialTimeoutException(error_msg)
        return detected_config

    def _backend_send_terminal_command(self, command_str):
        """Encodes and sends a terminal command string, then reads the response."""
        if not self.connected or not self.vesc_serial or not self.vesc_serial.is_open:
            raise ConnectionError(
                "Not connected while trying to send terminal command."
            )

        response_str = None  # 응답 문자열 저장 변수

        try:
            command_id_byte = bytes([VedderCmd.COMM_TERMINAL_CMD])
            command_payload_bytes = command_str.encode("utf-8")
            payload = command_id_byte + command_payload_bytes
            packet_to_send = self._frame_packet_manual(payload)

            # RLock 사용하여 시리얼 쓰기 및 읽기
            with self._monitor_lock:
                if (
                    not self.connected
                    or not self.vesc_serial
                    or not self.vesc_serial.is_open
                ):
                    raise ConnectionError(
                        "Disconnected during terminal command send/read."
                    )

                # 보내기 전 버퍼 비우기
                self._clear_input_buffer()
                self.vesc_serial.write(packet_to_send)
                # print(f"Debug: Sent terminal command: {command_str}")

                # --- 응답 읽기 로직 추가 ---
                response_buffer = b""
                start_read_time = time.monotonic()
                # 응답을 기다리는 시간 (예: 0.5초). VESC가 응답하는 데 시간이 걸릴 수 있음.
                read_timeout = 0.5
                # 짧은 non-blocking 읽기를 반복하여 응답 수신 시도
                original_timeout = self.vesc_serial.timeout
                self.vesc_serial.timeout = 0.01  # 거의 non-blocking
                while time.monotonic() - start_read_time < read_timeout:
                    try:
                        if self.vesc_serial.in_waiting > 0:
                            response_buffer += self.vesc_serial.read(
                                self.vesc_serial.in_waiting
                            )
                            # 간단하게는 개행 문자(\n 또는 \r)를 포함하면 응답 끝으로 간주
                            if b"\n" in response_buffer or b"\r" in response_buffer:
                                break
                        else:
                            time.sleep(0.02)  # 데이터 없으면 잠시 대기
                    except serial.SerialException as e:
                        # 읽기 중 오류 발생 시 루프 중단
                        self._log(
                            f"Warn: Serial exception while reading terminal response: {e}"
                        )
                        break
                self.vesc_serial.timeout = original_timeout  # 원래 타임아웃 복원

                # 수신된 바이트를 문자열로 디코딩 (오류 무시)
                if response_buffer:
                    response_str = response_buffer.decode(
                        "utf-8", errors="ignore"
                    ).strip()
                    # print(f"Debug: Received terminal response: '{response_str}'")
                else:
                    self._log("Warn: No response received from terminal command.")
                # --- 응답 읽기 로직 끝 ---

            return response_str  # 성공 시 읽은 응답 문자열 반환 (없으면 None)

        except AttributeError as e:
            raise NotImplementedError(
                f"Failed to send terminal command: Missing pyvesc component? ({e})"
            )
        except ImportError as e:
            if "crcmod" in str(e):
                raise ImportError("CRC calculation requires 'crcmod' library.")
            else:
                raise e
        except Exception as e:
            raise RuntimeError(
                f"Failed to send/read terminal command '{command_str}': {e}"
            )

    # --- GUI Action Handlers ---
    def _read_mcconf_gui(self):
        self._run_in_thread(
            self._backend_read_mcconf, callback=self._handle_mcconf_read_result
        )

    def _handle_mcconf_read_result(self, result, error):
        should_read_appconf = True
        if error:
            self._log(f"MCConf Read Error: {error}")
            self.current_mc_config = None
            self._clear_config_displays(mc_only=True)
            self._current_limit_amps = 10.0
            self._update_current_slider_range()
        else:
            self.current_mc_config = result
            self._log("MCConf read successfully. Populating fields.")
            self._populate_widgets(self.mcconf_widgets, result)
            try:
                limit_val = result.get("l_current_max", self._current_limit_amps)
                limit = float(limit_val)
                self._current_limit_amps = abs(limit) if limit is not None else 10.0
                self._log(
                    f"Set monitor current slider limit to: +/- {self._current_limit_amps:.1f} A"
                )
            except (ValueError, TypeError) as e:
                self._log(
                    f"Warning: Could not parse l_current_max '{limit_val}' ({e}), using default {self._current_limit_amps}A"
                )
                self._current_limit_amps = 10.0
            self._update_current_slider_range()
        if should_read_appconf:
            self._log("Proceeding to read AppConf...")
            self.after(50, self._read_appconf_gui)

    def _read_appconf_gui(self):
        self._run_in_thread(
            self._backend_read_appconf, callback=self._handle_appconf_read_result
        )

    def _handle_appconf_read_result(self, result, error):
        if error:
            self._log(f"AppConf Read Error: {error}")
            self.current_app_config = None
            self._clear_config_displays(app_only=True)
        else:
            self.current_app_config = result
            self._log("AppConf read successfully. Populating fields.")
            self._populate_widgets(self.appconf_widgets, result)
        self._log("Configuration auto-reading finished.")

    def _populate_widgets(self, widget_dict, config_data):
        tab_name = "MCConf" if widget_dict is self.mcconf_widgets else "AppConf"
        if not isinstance(config_data, dict):
            print(f"  ERROR ({tab_name}): Config data is not a dictionary.")
            return
        for key, widget in widget_dict.items():
            if not widget or not widget.winfo_exists():
                continue
            value = None
            if "." in key:
                try:
                    keys = key.split(".")
                    temp_val = config_data
                    for k in keys:
                        temp_val = (
                            temp_val[k]
                            if isinstance(temp_val, dict) and k in temp_val
                            else getattr(temp_val, k, None)
                        )
                        value = temp_val
                except Exception as e:
                    print(
                        f"  WARN ({tab_name}): Error accessing nested key '{key}': {e}"
                    )
            elif key in config_data:
                value = config_data[key]
            try:
                if value is not None:
                    if isinstance(widget, customtkinter.CTkComboBox):
                        map_to_use, is_bool = None, False
                        if key in ["m_invert_direction", "s_pid_allow_braking"]:
                            map_to_use, is_bool = BOOL_MAP, True
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
                        value_to_set = "Unknown"
                        if map_to_use:
                            lookup_key = bool(value) if is_bool else value
                            value_to_set = map_to_use.get(
                                lookup_key, f"Raw:{lookup_key}"
                            )
                        else:
                            value_to_set = str(value)
                        widget.set(value_to_set)
                    elif isinstance(widget, customtkinter.CTkEntry):
                        value_str = (
                            f"{value:.4f}" if isinstance(value, float) else str(value)
                        )
                        widget.delete(0, "end")
                        widget.insert(0, value_str)
                else:
                    if isinstance(widget, customtkinter.CTkEntry):
                        widget.delete(0, "end")
                        widget.insert(0, "N/A")
                    elif isinstance(widget, customtkinter.CTkComboBox):
                        options = widget.cget("values")
                        if "N/A" in options:
                            widget.set("N/A")
                        elif "Unknown" in options:
                            widget.set("Unknown")
                        elif options:
                            widget.set(options[0])
                        else:
                            widget.set("")
            except Exception as e:
                print(f"  ERROR ({tab_name}) populating widget '{key}': {e}")
                if isinstance(widget, customtkinter.CTkEntry):
                    widget.delete(0, "end")
                    widget.insert(0, "ERROR")
                elif isinstance(widget, customtkinter.CTkComboBox):
                    widget.set("ERROR")

    def _write_mcconf_gui(self):
        if not self.current_mc_config:
            self._log("Error: Read MCConf first before writing.")
            tkinter.messagebox.showerror("Error", "Read MCConf first before writing.")
            return
        if not tkinter.messagebox.askyesno(
            "Confirm Write MCConf",
            "Overwrite Motor Config on VESC?\nIncorrect values can cause damage.\n\nProceed?",
            icon="warning",
        ):
            self._log("MCConf write cancelled.")
            return
        config_to_write, errors = self._gather_and_validate_config(
            self.mcconf_widgets, self.current_mc_config
        )
        if errors:
            msg = "MCConf Validation Errors:\n\n" + "\n".join(errors)
            self._log(f"MCConf validation failed: {errors}")
            tkinter.messagebox.showerror("Validation Error", msg)
            return
        if "MCCONF_SIGNATURE" not in config_to_write:
            errors.append("Internal Error: MCCONF_SIGNATURE missing...")
            tkinter.messagebox.showerror(
                "Internal Error", "MCCONF_SIGNATURE missing. Cannot write."
            )
            return
        config_to_write.pop("crc", None)
        self._log("MCConf validation passed. Writing MCConf...")
        self._run_in_thread(
            self._backend_write_mcconf,
            args=(config_to_write,),
            callback=self._handle_mcconf_write_result_final,
        )

    def _write_appconf_gui(self):
        if not self.current_app_config:
            self._log("Error: Read AppConf first before writing.")
            tkinter.messagebox.showerror("Error", "Read AppConf first before writing.")
            return
        if not tkinter.messagebox.askyesno(
            "Confirm Write AppConf",
            "Overwrite Application Config on VESC?\n\nProceed?",
            icon="warning",
        ):
            self._log("AppConf write cancelled.")
            return
        config_to_write, errors = self._gather_and_validate_config(
            self.appconf_widgets, self.current_app_config
        )
        if errors:
            msg = "AppConf Validation Errors:\n\n" + "\n".join(errors)
            self._log(f"AppConf validation failed: {errors}")
            tkinter.messagebox.showerror("Validation Error", msg)
            return
        if (
            "APPCONF_SIGNATURE" not in config_to_write
            and "APPCONF_SİGNATURE" not in config_to_write
        ):
            errors.append("Internal Error: APPCONF_SIGNATURE missing...")
            tkinter.messagebox.showerror(
                "Internal Error", "APPCONF_SIGNATURE missing. Cannot write."
            )
            return
        config_to_write.pop("crc", None)
        self._log("AppConf validation passed. Writing AppConf...")
        self._run_in_thread(
            self._backend_write_appconf,
            args=(config_to_write,),
            callback=self._handle_appconf_write_result_final,
        )

    def _gather_and_validate_config(self, widget_dict, base_config):
        config_to_write = copy.deepcopy(base_config)
        errors = []
        tab_name = "MCConf" if widget_dict is self.mcconf_widgets else "AppConf"
        # print(f"\n--- Start Gathering & Validating {tab_name} ---")
        if not isinstance(base_config, dict):
            errors.append(
                f"Internal Error ({tab_name}): Base configuration is not a dictionary."
            )
            print(f"  ERROR: Base config is not a dict!")
            return config_to_write, errors
        for key, widget in widget_dict.items():
            if not widget or not widget.winfo_exists():
                continue
            try:
                gui_value_str = ""
                widget_type = type(widget).__name__
                if isinstance(
                    widget, (customtkinter.CTkEntry, customtkinter.CTkComboBox)
                ):
                    gui_value_str = widget.get()
                    # print(f"\nProcessing Key: '{key}', Widget: {widget_type}, GUI Value: '{gui_value_str}'")
                else:
                    continue
                if gui_value_str in ["N/A", "Unknown", "ERROR", "---"]:
                    continue  # print(f"  Skipping key '{key}' due to placeholder value.")
                vesc_value = None
                map_rev_to_use = None
                target_type = None
                original_value = None
                if "." in key:
                    try:
                        keys = key.split(".")
                        temp_val = base_config
                        for k in keys:
                            temp_val = (
                                temp_val[k]
                                if isinstance(temp_val, dict) and k in temp_val
                                else getattr(temp_val, k, None)
                            )
                            original_value = temp_val
                    except Exception:
                        pass

                elif key in base_config:
                    original_value = base_config[key]
                if original_value is not None:
                    target_type = type(original_value)
                    # print(f"  Original Value: {original_value} (Type: {target_type.__name__})")
                else:  # print(f"  Original value for key '{key}' not found in base_config.")
                    if isinstance(widget, customtkinter.CTkEntry):
                        target_type = str
                    elif isinstance(widget, customtkinter.CTkComboBox):
                        target_type = int
                if key in ["m_invert_direction", "s_pid_allow_braking"]:
                    map_rev_to_use, target_type = BOOL_MAP_REV, bool
                elif key == "motor_type":
                    map_rev_to_use, target_type = MOTOR_TYPES_MAP_REV, int
                elif key == "m_sensor_port_mode":
                    map_rev_to_use, target_type = SENSOR_PORT_MODES_MAP_REV, int
                elif key == "foc_sensor_mode":
                    map_rev_to_use, target_type = FOC_SENSOR_MODES_MAP_REV, int
                elif key == "app_to_use":
                    map_rev_to_use, target_type = APP_MODES_MAP_REV, int
                elif key == "can_baud_rate":
                    map_rev_to_use, target_type = CAN_BAUD_RATES_MAP_REV, int
                elif key == "can_mode":
                    map_rev_to_use, target_type = CAN_MODES_MAP_REV, int
                elif key == "uavcan_raw_mode":
                    map_rev_to_use, target_type = UAVCAN_RAW_MODES_MAP_REV, int
                elif key == "app_ppm_conf.ctrl_type":
                    map_rev_to_use, target_type = PPM_CONTROL_TYPES_MAP_REV, int
                if isinstance(widget, customtkinter.CTkComboBox):
                    # print(f"  Processing ComboBox. Using Rev Map: {'Yes' if map_rev_to_use else 'No'}")
                    if map_rev_to_use:
                        if gui_value_str in map_rev_to_use:
                            vesc_value = map_rev_to_use[gui_value_str]
                            # print(f"    Map Found: '{gui_value_str}' -> {vesc_value} (Type: {type(vesc_value).__name__})")
                        else:
                            errors.append(
                                f"({tab_name}) Invalid selection '{gui_value_str}' for {key}. Not in map."
                            )
                            print(
                                f"    ERROR: Selection '{gui_value_str}' not found in reverse map!"
                            )
                            print(
                                f"           Available map keys: {list(map_rev_to_use.keys())}"
                            )
                            continue
                    else:
                        errors.append(
                            f"({tab_name}) Internal Error: No reverse map defined for ComboBox '{key}'."
                        )
                        print(
                            f"    ERROR: No reverse map specified for ComboBox '{key}'"
                        )
                        continue
                elif isinstance(widget, customtkinter.CTkEntry):
                    # print(f"  Processing Entry. Target Type: {target_type.__name__ if target_type else 'Unknown'}")
                    if target_type == float:
                        try:
                            vesc_value = float(gui_value_str)
                            # print(f"    Converted to float: {vesc_value}")
                        except ValueError:
                            errors.append(
                                f"({tab_name}) Invalid float '{gui_value_str}' for {key}."
                            )
                            print("    ERROR: Float conversion failed.")
                            continue
                    elif target_type == int:
                        try:
                            vesc_value = int(gui_value_str)
                            # print(f"    Converted to int: {vesc_value}")
                        except ValueError:
                            errors.append(
                                f"({tab_name}) Invalid integer '{gui_value_str}' for {key}."
                            )
                            print("    ERROR: Int conversion failed.")
                            continue
                    elif target_type == bool:
                        if gui_value_str.lower() in ["true", "1", "yes"]:
                            vesc_value = True
                            # print("    Converted to bool: True")
                        elif gui_value_str.lower() in ["false", "0", "no"]:
                            vesc_value = False
                            # print("    Converted to bool: False")
                        else:
                            errors.append(
                                f"({tab_name}) Invalid boolean '{gui_value_str}' for {key}."
                            )
                            print("    ERROR: Bool conversion failed.")
                            continue
                    elif target_type == str:
                        vesc_value = gui_value_str
                        # print("    Kept as string.")
                    else:
                        try:
                            vesc_value = float(gui_value_str)
                            # print(f"    Fallback converted to float: {vesc_value}")
                        except ValueError:
                            try:
                                vesc_value = int(gui_value_str)
                                # print(f"    Fallback converted to int: {vesc_value}")
                            except ValueError:
                                vesc_value = gui_value_str
                                # print("    Fallback kept as string.")
                else:
                    continue
                if vesc_value is not None:
                    # print(f"  Attempting to save value: {vesc_value} for key: '{key}'")
                    if "." in key:
                        keys = key.split(".")
                        target_dict = config_to_write
                        valid_path = True
                        for i, k in enumerate(keys[:-1]):
                            # print(f"    Navigating nested key: '{k}'")
                            if k not in target_dict:
                                target_dict[k] = {}
                                # print(f"      Created missing dict for '{k}'");
                            elif not isinstance(target_dict[k], dict):
                                errors.append(
                                    f"({tab_name}) Path conflict: '{k}' in '{key}' is not a dictionary."
                                )
                                print(
                                    f"      ERROR: Path conflict, '{k}' is not a dict."
                                )
                                valid_path = False
                                break
                            target_dict = target_dict[k]
                            if valid_path:
                                last_key = keys[
                                    -1
                                ]  # print(f"    Setting nested key '{last_key}' to {vesc_value}");
                            target_dict[last_key] = vesc_value
                        # else: print("    Skipping save due to invalid path.")
                    else:  # print(f"    Setting key '{key}' to {vesc_value}");
                        config_to_write[key] = vesc_value
                # else: print(f"  Value for '{key}' is None after conversion, not saving.")
            except Exception as e:
                errors.append(f"({tab_name}) Unexpected error processing '{key}': {e}")
                print(f"  ERROR: Unexpected exception for key '{key}': {e}")
                import traceback

                traceback.print_exc()
        # print(f"--- End Gathering & Validating {tab_name} ---\n")
        return config_to_write, errors

    def _handle_mcconf_write_result_final(self, result, error):
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
            self._log("MCConf write failed (unknown backend signal).")
            tkinter.messagebox.showerror(
                "Write Error", "Failed to write Motor Config (Unknown Reason)."
            )

    def _handle_appconf_write_result_final(self, result, error):
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
            self._log("AppConf write failed (unknown backend signal).")
            tkinter.messagebox.showerror(
                "Write Error", "Failed to write App Config (Unknown Reason)."
            )

    # --- Detection Tab Actions ---
    def _run_detection_gui(self):
        if (
            not hasattr(self, "motor_size_combobox")
            or not self.motor_size_combobox.winfo_exists()
        ):
            self._log("Error: Motor size combobox not found.")
            return
        motor_size_to_power_loss = {
            "Mini (~75g)": 20.0,
            "Small (~200g)": 50.0,
            "Medium (~750g)": 120.0,
            "Large (~2000g)": 400.0,
        }
        selected_size_str = self.motor_size_combobox.get()
        max_loss = motor_size_to_power_loss.get(selected_size_str)
        if max_loss is None:
            self._log(f"Error: Invalid motor size selection '{selected_size_str}'.")
            tkinter.messagebox.showerror(
                "Error", f"Invalid motor size selected: {selected_size_str}"
            )
            (
                hasattr(self, "detection_output_textbox")
                and self._update_textbox(
                    self.detection_output_textbox, "Error: Invalid motor size."
                )
            )
            return
        confirm_msg = f"Motor WILL spin/make noise!\nSelected Size: {selected_size_str} (Max Loss: {max_loss}W)\nEnsure safety.\nEnsure motor can spin freely.\n\nProceed?"
        if not tkinter.messagebox.askyesno(
            "Run Detection?", confirm_msg, icon="warning"
        ):
            self._log("Detection cancelled.")
            (
                hasattr(self, "detection_output_textbox")
                and self._update_textbox(
                    self.detection_output_textbox, "Detection cancelled."
                )
            )
            return
        if hasattr(self, "detection_output_textbox"):
            self._update_textbox(
                self.detection_output_textbox,
                f"Starting FOC Detection (Size: {selected_size_str}, Max Loss: {max_loss}W)...\nVESC measuring... DO NOT INTERRUPT.\nThis may take up to {int(DETECTION_WIZARD_TIMEOUT_GUI)} seconds.",
                read_only=False,
            )
        self._run_in_thread(
            self._backend_run_detection,
            args=(max_loss,),
            callback=self._handle_detection_result,
        )

    def _handle_detection_result(self, result, error):
        if (
            not hasattr(self, "detection_output_textbox")
            or not self.detection_output_textbox.winfo_exists()
        ):
            return
        current_output = ""
        try:
            current_output = self.detection_output_textbox.get("1.0", "end-1c")
        except:
            pass
        output_to_add = ""
        final_state = "disabled"
        if error:
            output_to_add = f"\n\n--- DETECTION FAILED ---\n{error}"
            self._log(f"Detection failed: {error}")
        elif result:
            self.current_mc_config = result
            output_to_add = "\n\n--- DETECTION SUCCEEDED! --- \nNew Motor Config received & applied by VESC.\nMotor Config tab fields automatically updated.\n-> Remember to WRITE the config to make it permanent! <-\n----------------------------------------------------\n"
            params = {
                "R (mΩ)": result.get("foc_motor_r", 0) * 1e3,
                "L (µH)": result.get("foc_motor_l", 0) * 1e6,
                "λ (mWb)": result.get("foc_motor_flux_linkage", 0) * 1e3,
                "Temp Comp": result.get("foc_temp_comp", False),
                "Observer Gain": result.get("foc_observer_gain", 0),
            }
            for name, val in params.items():
                output_to_add += (
                    f"{name}: {val:.3f}\n"
                    if isinstance(val, float)
                    else f"{name}: {val}\n"
                )
            output_to_add += "----------------------------------------------------\n"
            self._log("Detection successful. MCConf updated.")
            self.after(50, self._populate_widgets, self.mcconf_widgets, result)
        else:
            output_to_add = "\n\n--- DETECTION FAILED (Unknown Reason/No Result) ---"
            self._log("Detection failed (No data/result).")
        self._update_textbox(
            self.detection_output_textbox,
            current_output + output_to_add,
            read_only=(final_state == "disabled"),
        )

    # --- Offset Setting Actions (Using Terminal Command) ---
    def _apply_pos_offset_gui(self):
        """Handles the 'Set Current Position to 0 Degrees' button click using GetValues."""
        if not self.connected:
            self._log("Error: Not connected.")
            tkinter.messagebox.showerror("Error", "Not connected to VESC.")
            return

        # 1. 현재 MC 설정 읽기 시작 (current_offset 값 필요)
        self._log("Reading current Motor Config to apply offset...")
        self._run_in_thread(
            self._backend_read_mcconf,
            callback=self._handle_read_mc_for_offset_using_gv,  # GetValues 읽는 단계로 진행
        )

    def _handle_read_mc_for_offset_using_gv(self, mc_config_result, error):
        """Callback after reading MC Conf, proceeds to read GetValues."""
        if error:
            self._log(f"Error reading MC Config for offset: {error}")
            tkinter.messagebox.showerror(
                "Read MC Config Error",
                f"Failed to read Motor Config before applying offset:\n{error}",
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return
        if not mc_config_result:
            self._log("Error: MC Config read returned empty result.")
            tkinter.messagebox.showerror(
                "Read MC Config Error", "Failed to get valid Motor Config data."
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return

        self.temp_mc_config_for_offset = mc_config_result  # 임시 저장
        self._log(
            "MC Config read successfully. Now reading current values (GetValues)..."
        )

        # 2. GetValues 읽기 시작
        self._run_in_thread(
            self._backend_get_values,  # GetValues 읽는 백엔드 함수 호출
            callback=self._handle_get_values_and_write_offset,  # 다음 단계 콜백
        )

    def _backend_get_values(self):
        """Reads one GetValues message from the VESC."""
        # ... (이전 답변의 전체 코드 내용) ...
        if not self.connected or not self.vesc_serial or not self.vesc_serial.is_open:
            raise ConnectionError("...")
        request_packet = encode_request(GetValues)
        values_obj = None
        with self._monitor_lock:
            if (
                not self.connected
                or not self.vesc_serial
                or not self.vesc_serial.is_open
            ):
                raise ConnectionError("...")
            self._clear_input_buffer()
            self.vesc_serial.write(request_packet)
            response_found = False
            read_buffer = b""
            start_time = time.monotonic()
            timeout_duration = 2.0
            while time.monotonic() - start_time < timeout_duration:
                try:
                    if self.vesc_serial.in_waiting > 0:
                        read_buffer += self.vesc_serial.read(
                            self.vesc_serial.in_waiting
                        )
                    response, consumed = pyvesc.decode(read_buffer)
                    if consumed > 0:
                        if isinstance(response, GetValues):
                            values_obj = response
                            response_found = True
                            break
                        else:
                            read_buffer = read_buffer[consumed:]
                    elif len(read_buffer) > READ_BUFFER_SIZE:
                        read_buffer = read_buffer[-int(READ_BUFFER_SIZE / 2) :]
                        time.sleep(0.01)
                    else:
                        time.sleep(0.02)
                except (
                    ValueError,
                    IndexError,
                    struct.error,
                    AttributeError,
                    NotImplementedError,
                ):
                    time.sleep(0.02)  # Allow buffer to fill on decode error
                except serial.SerialException as read_err:
                    raise ConnectionError(
                        f"Serial error during GetValues read: {read_err}"
                    ) from read_err
                except Exception as general_e:
                    raise RuntimeError(
                        f"Unexpected error during GetValues decode: {general_e}"
                    ) from general_e
            if not response_found:
                raise TimeoutError(
                    f"Did not receive GetValues response within {timeout_duration}s."
                )
        return values_obj

    def _handle_get_values_and_write_offset(self, get_values_result, error):
        """Calculates offset mimicking VESC firmware logic (current_offset + pid_pos_now), modifies MC Config, and initiates writing."""
        # --- 오류 처리 ---
        if error:
            self._log(f"Error reading GetValues for offset: {error}")
            tkinter.messagebox.showerror(
                "Read Values Error", f"Failed to read current values:\n{error}"
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return
        if not get_values_result or not isinstance(get_values_result, GetValues):
            self._log("Error: GetValues read returned invalid result.")
            tkinter.messagebox.showerror(
                "Read Values Error", "Failed to get valid GetValues data."
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return

        # --- 값 추출 ---
        pid_pos_now = getattr(get_values_result, "pid_pos_now", None)
        if pid_pos_now is None or not isinstance(pid_pos_now, (float, int)):
            self._log(
                f"Error: Could not get valid 'pid_pos_now' from GetValues: {pid_pos_now}"
            )
            tkinter.messagebox.showerror(
                "Read Values Error",
                f"'pid_pos_now' not found or invalid in GetValues response: {pid_pos_now}",
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return

        config_to_modify = getattr(self, "temp_mc_config_for_offset", None)
        if not config_to_modify or not isinstance(config_to_modify, dict):
            self._log("Error: Stored MC Config for modification is invalid or missing.")
            tkinter.messagebox.showerror(
                "Internal Error", "Failed to retrieve MC Config for modification."
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return

        offset_key = "p_pid_offset"
        current_offset = 0.0
        try:
            current_offset = float(config_to_modify.get(offset_key, 0.0))
        except (ValueError, TypeError):
            self._log(
                f"Warning: Current '{offset_key}' value is not a valid number. Assuming 0.0."
            )
            current_offset = 0.0

        # --- 오프셋 계산 (VESC 펌웨어 로직 기반) ---
        try:
            current_pid_pos_float = float(pid_pos_now)
            new_offset_calculated = (
                current_offset + current_pid_pos_float
            )  # VESC 로직: current + (pid_pos - angle_now) where angle_now=0
        except ValueError:
            self._log(
                f"Error: Could not convert pid_pos_now '{pid_pos_now}' to float for calculation."
            )
            tkinter.messagebox.showerror(
                "Calculation Error",
                f"Could not calculate offset from pid_pos_now: {pid_pos_now}",
            )
            if hasattr(self, "temp_mc_config_for_offset"):
                self.temp_mc_config_for_offset = None
            return

        # --- 정규화 (-180 ~ +180) ---
        new_offset_normalized = (new_offset_calculated + 180.0) % 360.0 - 180.0
        value_to_save = float(new_offset_normalized)

        # --- 로그 및 MC 설정 쓰기 시작 ---
        self._log(
            f"Current pid_pos_now: {current_pid_pos_float:.2f} deg. Current offset: {current_offset:.2f} deg."
        )
        self._log(
            f"Calculated new offset: {value_to_save:.2f} deg for key '{offset_key}'. (Using firmware logic, normalized -180~180)"
        )

        config_to_modify[offset_key] = value_to_save
        config_to_modify.pop("crc", None)

        self._log(
            f"Writing modified Motor Config with new '{offset_key}' = {value_to_save:.2f}..."
        )
        self._run_in_thread(
            self._backend_write_mcconf,
            args=(config_to_modify,),
            callback=self._handle_write_offset_mc_result,  # 쓰기 완료 콜백
        )

        # --- 임시 데이터 정리 ---
        self.temp_mc_config_for_offset = None

    def _handle_write_offset_mc_result(self, result, error):
        """Callback after attempting to write the modified MC config with new offset."""
        if error:
            self._log(f"Error writing modified MC Config for offset: {error}")
            tkinter.messagebox.showerror(
                "Write MC Config Error",
                f"Failed to write modified Motor Config:\n{error}",
            )
        elif result:
            self._log("Modified Motor Config with new offset written successfully.")
            # --- 안정화 시간 확보를 위한 딜레이 추가 (선택 사항, 테스트용) ---
            # wait_time = 0.5 # 초 단위
            # self._log(f"Waiting {wait_time}s for VESC to apply new offset...")
            # time.sleep(wait_time) # GUI 스레드 블록 주의!
            # --- 딜레이 끝 ---
            tkinter.messagebox.showinfo(
                "Offset Applied",
                "Current position set to 0 degrees (offset updated).\nRe-reading Motor Config to verify...",
            )
            # 성공 시 Motor Config 다시 읽기
            self.after(200, self._read_mcconf_gui)
        else:
            self._log("Write modified MC Config failed (unknown backend signal).")
            tkinter.messagebox.showerror(
                "Write MC Config Error",
                "Failed to write modified Motor Config (Unknown Reason).",
            )

    def _handle_apply_offset_result(self, response_str, error):
        """Callback after attempting to apply position offset via terminal command, checks response."""
        offset_applied_successfully = False  # 성공 여부 플래그

        if error:
            self._log(f"Error applying position offset: {error}")
            if isinstance(error, ImportError) and "crcmod" in str(error):
                tkinter.messagebox.showerror("Dependency Error", f"{error}")
            elif isinstance(error, NotImplementedError):
                tkinter.messagebox.showerror(
                    "Compatibility Error",
                    f"Could not send command:\n{error}\n\nPlease check your pyvesc library version.",
                )
            else:
                tkinter.messagebox.showerror(
                    "Apply Offset Error", f"Failed to apply offset:\n{error}"
                )
        else:
            # 오류 없이 완료되었고, 응답 문자열 확인
            self._log(f"Terminal command sent. Response from VESC: '{response_str}'")
            # VESC Tool에서는 'OK' 문자열을 확인하는 경우가 많음
            if (
                response_str and "OK" in response_str.upper()
            ):  # 대소문자 무시하고 "OK" 포함 확인
                offset_applied_successfully = True
                self._log(
                    "Position offset command seems successful (Got 'OK' response)."
                )
                tkinter.messagebox.showinfo(
                    "Apply Offset",
                    "Position offset command sent successfully.\nRe-reading Motor Config...",
                )
            elif response_str:  # OK는 아니지만 응답이 있는 경우
                self._log(
                    "Position offset command sent, but response was not 'OK'. Check VESC status."
                )
                tkinter.messagebox.showwarning(
                    "Apply Offset",
                    f"Command sent, but VESC response was:\n'{response_str}'\n\nOffset might not be applied correctly. Re-reading config anyway...",
                )
                # 성공으로 간주하고 재로딩은 시도
                offset_applied_successfully = True  # 재로딩을 위해 임시로 True 설정
            else:  # 응답이 없는 경우
                self._log(
                    "Position offset command sent, but no response received from VESC."
                )
                tkinter.messagebox.showwarning(
                    "Apply Offset",
                    "Command sent, but no response received.\nOffset might not be applied. Re-reading config...",
                )
                # 성공으로 간주하고 재로딩은 시도
                offset_applied_successfully = True  # 재로딩을 위해 임시로 True 설정

        # 성공했거나 (또는 응답 없어도) MC 설정을 다시 읽어 상태 확인
        if offset_applied_successfully or error is None:  # 오류가 없을 때도 재로딩 시도
            self.after(200, self._read_mcconf_gui)

    # --- Monitor Tab Actions ---
    def _on_tab_change(self):
        selected_tab = ""
        if hasattr(self, "tabview") and self.tabview.winfo_exists():
            try:
                selected_tab = self.tabview.get()
            except Exception as e:
                print(f"Warn: Could not get current tab: {e}")
                return
        if selected_tab == "Monitor" and self.connected:
            self._start_monitor_loop()
        else:
            was_monitoring = self._monitor_running
            self._stop_monitor_loop()
            if self.connected and was_monitoring:
                self._log("Switched tab away from Monitor, sending STOP command.")
                self._send_stop_command()
                self._control_mode.set("Inactive")

    def _start_monitor_loop(self):
        if not self.connected:
            self._log("Cannot start monitor: Not connected.")
            return
        if self._monitor_running:
            return
        self._monitor_running = True
        self._reset_monitor_display()
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self._log("Realtime monitor started.")

    def _stop_monitor_loop(self):
        if self._monitor_running:
            self._monitor_running = False
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                self._monitor_thread.join(timeout=MONITOR_INTERVAL * 2)
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                print("Warn: Monitor thread did not exit cleanly.")
            self._monitor_thread = None
            self._log("Realtime monitor stopped.")
            self.after(50, self._reset_monitor_display)

    def _monitor_loop(self):
        last_request_time = 0
        last_keepalive_time = 0
        get_values_request = encode_request(GetValues)
        read_buffer = b""
        while self._monitor_running:
            if (
                not self.connected
                or not self.vesc_serial
                or not self.vesc_serial.is_open
            ):
                self._monitor_running = False
                self.after(0, self._log, "Monitor stopped: Disconnected.")
                break
            current_time = time.monotonic()
            response_object = None
            command_to_send = None
            try:
                command_to_send = self._command_queue.get_nowait()
            except queue.Empty:
                pass
            except Exception as q_err:
                print(f"Error getting command from queue: {q_err}")
            send_keepalive_or_stop = False
            request_get_values = False
            if current_time - last_request_time >= MONITOR_INTERVAL:
                request_get_values = True
                last_request_time = current_time
                if current_time - last_keepalive_time >= MONITOR_INTERVAL * 2:
                    send_keepalive_or_stop = True
                    last_keepalive_time = current_time
            if command_to_send or request_get_values or send_keepalive_or_stop:
                with self._monitor_lock:
                    if (
                        not self._monitor_running
                        or not self.connected
                        or not self.vesc_serial
                        or not self.vesc_serial.is_open
                    ):
                        continue
                    try:
                        if command_to_send:
                            try:
                                packet = encode(command_to_send)
                                self.vesc_serial.write(packet)
                                if (
                                    isinstance(command_to_send, SetCurrent)
                                    and command_to_send.current == 0
                                ):
                                    last_keepalive_time = current_time
                                    command_to_send = None
                            except Exception as e:
                                self._log(
                                    f"Error sending queued command ({type(command_to_send).__name__}): {e}"
                                )
                                command_to_send = None
                        if request_get_values:
                            try:
                                self.vesc_serial.write(get_values_request)
                            except Exception as e:
                                self._log(f"Error sending GetValues request: {e}")
                        if send_keepalive_or_stop and command_to_send is None:
                            current_mode = self._control_mode.get()
                            periodic_cmd = None
                            try:
                                if current_mode == "Inactive":
                                    periodic_cmd = SetCurrent(0)
                                elif current_mode == "Current":
                                    periodic_cmd = (
                                        SetCurrent(
                                            self.monitor_control_widgets[
                                                "current_slider"
                                            ].get()
                                        )
                                        if "current_slider"
                                        in self.monitor_control_widgets
                                        and self.monitor_control_widgets[
                                            "current_slider"
                                        ].winfo_exists()
                                        else None
                                    )
                                elif current_mode == "Duty Cycle":
                                    periodic_cmd = (
                                        SetDutyCycle(
                                            self.monitor_control_widgets[
                                                "duty_slider"
                                            ].get()
                                        )
                                        if "duty_slider" in self.monitor_control_widgets
                                        and self.monitor_control_widgets[
                                            "duty_slider"
                                        ].winfo_exists()
                                        else None
                                    )
                                elif current_mode == "Position":
                                    periodic_cmd = (
                                        SetPosition(
                                            self.monitor_control_widgets[
                                                "position_slider"
                                            ].get()
                                        )
                                        if "position_slider"
                                        in self.monitor_control_widgets
                                        and self.monitor_control_widgets[
                                            "position_slider"
                                        ].winfo_exists()
                                        else None
                                    )
                                if periodic_cmd:
                                    packet = encode(periodic_cmd)
                                    self.vesc_serial.write(packet)
                            except Exception as e:
                                self._log(
                                    f"Error sending periodic command ({current_mode}): {e}"
                                )
                        read_start_time = time.monotonic()
                        read_timeout = MONITOR_INTERVAL * 0.1
                        temp_buffer = b""
                        original_timeout = SERIAL_TIMEOUT
                        try:
                            original_timeout = self.vesc_serial.timeout
                            self.vesc_serial.timeout = 0.001
                            while (time.monotonic() - read_start_time) < read_timeout:
                                chunk = self.vesc_serial.read(512)
                                if chunk:
                                    temp_buffer += chunk
                                else:
                                    break
                            self.vesc_serial.timeout = original_timeout
                        except serial.SerialTimeoutException:
                            pass
                        except serial.SerialException as read_err:
                            raise read_err
                        except Exception as read_gen_err:
                            raise read_gen_err
                        finally:
                            if (
                                hasattr(self.vesc_serial, "timeout")
                                and self.vesc_serial.is_open
                            ):
                                try:
                                    self.vesc_serial.timeout = original_timeout
                                except:
                                    pass
                        read_buffer += temp_buffer
                        if len(read_buffer) > READ_BUFFER_SIZE * 2:
                            read_buffer = read_buffer[-READ_BUFFER_SIZE:]
                    except serial.SerialException as e:
                        self.after(
                            0,
                            self._log,
                            f"Error: Monitor serial error inside lock: {e}",
                        )
                        self._monitor_running = False
                        self.after(0, self._handle_serial_error)
                        continue
                    except Exception as e:
                        self.after(
                            0,
                            self._log,
                            f"Error: Unexpected monitor error inside lock: {e}",
                        )
                        import traceback

                        traceback.print_exc()
                processed_buffer = read_buffer
                while True:
                    found_get_values = False
                    try:
                        response, consumed = pyvesc.decode(processed_buffer)
                        if consumed > 0:
                            if isinstance(response, GetValues):
                                response_object = response
                                found_get_values = True
                            processed_buffer = processed_buffer[consumed:]
                            if found_get_values:
                                break
                        else:
                            break
                    except (
                        ValueError,
                        IndexError,
                        struct.error,
                        AttributeError,
                        NotImplementedError,
                    ):
                        next_sof = -1
                        sof2 = processed_buffer.find(b"\x02", 1)
                        sof3 = processed_buffer.find(b"\x03", 1)
                        if sof2 != -1 and sof3 != -1:
                            next_sof = min(sof2, sof3)
                        elif sof2 != -1:
                            next_sof = sof2
                        elif sof3 != -1:
                            next_sof = sof3
                        if next_sof > 0:
                            processed_buffer = processed_buffer[next_sof:]
                        else:
                            break
                read_buffer = processed_buffer
                if response_object:
                    self.after(0, self._update_monitor_display, response_object)
            adaptive_sleep = max(0.01, MONITOR_INTERVAL / 10)
            time.sleep(adaptive_sleep)

    def _update_monitor_display(self, data: GetValues):
        if not isinstance(data, GetValues):
            return
        for key, widget in self.monitor_widgets.items():
            if widget and widget.winfo_exists():
                value = getattr(data, key, None)
                text_to_set = "N/A"
                if value is not None:
                    try:
                        if isinstance(value, float):
                            if key in ["v_in", "temp_fet", "temp_motor"]:
                                fmt = "{:.1f}"
                            elif key in ["avg_motor_current", "avg_input_current"]:
                                fmt = "{:.2f}"
                            elif key == "duty_cycle_now":
                                fmt = "{:.3f}"
                            elif key in [
                                "amp_hours",
                                "watt_hours",
                                "amp_hours_charged",
                                "watt_hours_charged",
                            ]:
                                fmt = "{:.3f}"
                            else:
                                fmt = "{:.2f}"
                            text_to_set = fmt.format(value)
                        elif key == "mc_fault_code":
                            int_code = -1
                            if isinstance(value, bytes) and len(value) == 1:
                                int_code = int.from_bytes(value, byteorder="little")
                            elif isinstance(value, int):
                                int_code = value
                            elif "FaultCode" in globals() and isinstance(
                                value, FaultCode
                            ):
                                int_code = value.value
                            else:
                                text_to_set = f"TypeErr: {value!r}"
                            if int_code != -1:
                                text_to_set = MC_FAULT_CODES_MAP.get(
                                    int_code, f"Unknown ({int_code})"
                                )
                        else:
                            text_to_set = str(value)
                    except Exception as format_e:
                        print(
                            f"Warn: Error formatting monitor value for {key}: {format_e}"
                        )
                        text_to_set = "FmtErr"
                try:
                    widget.configure(text=text_to_set)
                except Exception as widget_e:
                    print(f"Warn: Error updating monitor widget for {key}: {widget_e}")

    def _reset_monitor_display(self):
        for widget in self.monitor_widgets.values():
            if widget and widget.winfo_exists():
                try:
                    widget.configure(text="---")
                except:
                    pass

    def _control_mode_changed(self):
        try:
            new_mode = self._control_mode.get()
            self._log(f"Control mode changed via radio button to: {new_mode}")
            try:
                print(
                    f"Debug: Updating widget states from _control_mode_changed (mode: {new_mode})"
                )
                self._set_monitor_widgets_state(
                    "normal" if self.connected else "disabled"
                )
            except Exception as e:
                self._log(f"Error updating widget states after mode change: {e}")
                import traceback

                traceback.print_exc()
        except Exception as e:
            self._log(f"Critical Error in _control_mode_changed: {e}")
            import traceback

            traceback.print_exc()
            tkinter.messagebox.showerror(
                "Error", f"Failed to process mode change:\n{e}"
            )

    def _update_slider_label(self, slider_type, value):
        widget_key = f"{slider_type}_label"
        unit = ""
        fmt = "{:.1f}"
        if slider_type == "current":
            unit = " A"
        elif slider_type == "duty":
            fmt = "{:.2f}"
            unit = ""
        elif slider_type == "position":
            fmt = "{:.0f}"
            unit = " °"
        if widget_key in self.monitor_control_widgets:
            label_widget = self.monitor_control_widgets[widget_key]
            if label_widget and label_widget.winfo_exists():
                try:
                    base_text = label_widget.cget("text").split(":")[0]
                    label_widget.configure(
                        text=f"{base_text}: {fmt.format(value)}{unit}"
                    )
                except Exception as e:
                    print(f"Warn: Failed to update slider label {widget_key}: {e}")

    def _slider_current_changed(self, value):
        self._update_slider_label("current", value)
        (
            self._control_mode.get() == "Current"
            and self._queue_command(SetCurrent(value))
        )

    def _slider_duty_changed(self, value):
        self._update_slider_label("duty", value)
        (
            self._control_mode.get() == "Duty Cycle"
            and self._queue_command(SetDutyCycle(value))
        )

    def _slider_position_changed(self, value):
        self._update_slider_label("position", value)
        (
            self._control_mode.get() == "Position"
            and self._queue_command(SetPosition(value))
        )

    def _queue_command(self, message):
        if not self.connected:
            return False
        try:
            self._command_queue.put(message)
            return True
        except Exception as e:
            self._log(f"Error queueing command ({type(message).__name__}): {e}")
            return False

    def _send_stop_command(self):
        self._log(
            "STOP button pressed. Setting mode to Inactive and resetting sliders."
        )
        current_mode = self._control_mode.get()
        if current_mode != "Inactive":
            print("Debug: Setting mode to Inactive from STOP button.")
            self._control_mode.set("Inactive")
        print("Debug: Calling _reset_sliders_and_update_state from STOP button.")
        self._reset_sliders_and_update_state()

    def _reset_sliders_and_update_state(self):
        print("Debug: _reset_sliders_and_update_state called")
        widgets_to_reset = ["current_slider", "duty_slider", "position_slider"]
        labels_to_update = {
            "current_slider": "current",
            "duty_slider": "duty",
            "position_slider": "position",
        }
        for widget_key in widgets_to_reset:
            if (
                widget_key in self.monitor_control_widgets
                and self.monitor_control_widgets[widget_key].winfo_exists()
            ):
                try:
                    slider = self.monitor_control_widgets[widget_key]
                    original_command = slider.cget("command")
                    slider.configure(command=None)
                    slider.set(0)
                    slider.configure(command=original_command)
                    self._update_slider_label(labels_to_update[widget_key], 0)
                    print(f"  - {labels_to_update[widget_key]} slider reset")
                except Exception as e:
                    print(f"Warn: Error resetting {widget_key}: {e}")
        try:
            print("  - Updating widget states for Inactive mode")
            self._set_monitor_widgets_state("normal" if self.connected else "disabled")
        except Exception as e:
            self._log(f"Error updating widget states in helper: {e}")

    def _update_current_slider_range(self):
        if (
            "current_slider" in self.monitor_control_widgets
            and self.monitor_control_widgets["current_slider"].winfo_exists()
        ):
            slider = self.monitor_control_widgets["current_slider"]
            limit = max(1.0, self._current_limit_amps)
            steps = min(2000, int(limit * 2 * 10))
            try:
                slider.configure(from_=-limit, to=limit, number_of_steps=steps)
            except Exception as e:
                print(f"Warn: Failed to update current slider range: {e}")

    # --- Window Closing ---
    def _on_closing(self):
        print("Window closing...")
        self._stop_monitor_loop()
        if self.connected:
            self._log("Disconnecting before closing...")
            if self.vesc_serial and self.vesc_serial.is_open:
                try:
                    stop_cmd = encode(SetCurrent(0))
                    with self._monitor_lock:
                        if self.vesc_serial.is_open:
                            self.vesc_serial.write(stop_cmd)
                            time.sleep(0.05)
                            self.vesc_serial.close()
                            print("Serial port closed.")
                except Exception as e:
                    print(f"Error during serial close on exit: {e}")
                finally:  # Ensure close is attempted even if write fails
                    if self.vesc_serial and self.vesc_serial.is_open:
                        try:
                            self.vesc_serial.close()
                        except:
                            pass
            self.vesc_serial = None
            self.connected = False
        try:
            self.destroy()
        except Exception as e:
            print(f"Error during window destroy: {e}")


# --- Run the Application ---
if __name__ == "__main__":
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError):
        pass
    app = VescApp()
    app.mainloop()

# --- END OF FILE with_monitor.py ---
