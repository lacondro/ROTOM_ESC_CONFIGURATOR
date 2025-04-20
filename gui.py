# --- VESC Control GUI ---
import sys
import os
import serial
import time
import struct
import pprint
import copy
import threading
import queue  # For thread communication (optional, can use `after`)

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
    from pyvesc.protocol.interface import *  # Assuming encode_response might be needed by helpers

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

    # Helper might have connect/close, but we'll manage connection in the GUI class
    # from pyvesc.VESC.messages.helper import * # Avoid wildcard imports if possible

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
SERIAL_TIMEOUT = 1.0  # Shorter timeout for basic reads
DETECTION_WIZARD_TIMEOUT_GUI = 120.0  # Timeout for the detection wizard process
READ_BUFFER_SIZE = 4096


# --- Main Application Class ---
class VescApp(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.title("ROTOM FLUXIUM CONFIGURATOR")
        self.geometry("800x600")

        # --- State Variables ---
        self.vesc_serial: serial.Serial | None = None
        self.connected = False
        self.port_list = []
        self.current_mc_config = None
        self.current_app_config = None

        # --- Appearance ---
        customtkinter.set_appearance_mode(
            "System"
        )  # Modes: "System" (default), "Dark", "Light"
        customtkinter.set_default_color_theme(
            "blue"
        )  # Themes: "blue" (default), "green", "dark-blue"

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
        self.protocol("WM_DELETE_WINDOW", self._on_closing)  # Handle window close

    # --- UI Creation Methods ---
    def _create_sidebar(self):
        self.sidebar_frame = customtkinter.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=1, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(5, weight=1)  # Push status label down

        self.logo_label = customtkinter.CTkLabel(
            self.sidebar_frame,
            text="ROTOM" + "\n" + "FLUXIUM",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Port Selection
        self.port_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Serial Port:", anchor="w"
        )
        self.port_label.grid(row=1, column=0, padx=20, pady=(10, 0), sticky="ew")
        self.port_combobox = customtkinter.CTkComboBox(
            self.sidebar_frame, values=[""], command=self._port_selected
        )
        self.port_combobox.grid(row=2, column=0, padx=20, pady=(0, 10), sticky="ew")

        # Refresh Ports Button
        self.refresh_button = customtkinter.CTkButton(
            self.sidebar_frame, text="Refresh Ports", command=self._update_port_list
        )
        self.refresh_button.grid(row=3, column=0, padx=20, pady=(0, 10))

        # Baud Rate (Fixed for now, could be ComboBox later)
        # self.baud_label = customtkinter.CTkLabel(self.sidebar_frame, text="Baud Rate:", anchor="w")
        # self.baud_label.grid(row=3, column=0, padx=20, pady=(10, 0), sticky="ew")
        # self.baud_entry = customtkinter.CTkEntry(self.sidebar_frame, placeholder_text="e.g., 115200")
        # self.baud_entry.grid(row=4, column=0, padx=20, pady=(0, 10), sticky="ew")
        # self.baud_entry.insert(0, str(DEFAULT_BAUD_RATE))

        # Connect Button
        self.connect_button = customtkinter.CTkButton(
            self.sidebar_frame, text="Connect", command=self._toggle_connection
        )
        self.connect_button.grid(row=4, column=0, padx=20, pady=10)

        # Status Label
        self.status_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Status: Disconnected", anchor="w", wraplength=180
        )
        self.status_label.grid(row=6, column=0, padx=20, pady=(10, 20), sticky="ew")

    def _create_main_tabs(self):
        self.tabview = customtkinter.CTkTabview(self, width=600)
        self.tabview.grid(row=0, column=1, padx=(20, 20), pady=(20, 20), sticky="nsew")

        self.tabview.add("Settings")
        self.tabview.add("Detection")

        self._create_settings_tab(self.tabview.tab("Settings"))
        self._create_detection_tab(self.tabview.tab("Detection"))

    def _create_settings_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)  # MCConf Text
        tab.grid_rowconfigure(4, weight=1)  # AppConf Text

        # --- Motor Configuration (MCConf) ---
        self.mcconf_frame = customtkinter.CTkFrame(tab)
        self.mcconf_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")
        self.mcconf_label = customtkinter.CTkLabel(
            self.mcconf_frame,
            text="Motor Configuration (MCConf)",
            font=customtkinter.CTkFont(weight="bold"),
        )
        self.mcconf_label.pack(side="left", padx=10)
        self.mcconf_read_button = customtkinter.CTkButton(
            self.mcconf_frame, text="Read MCConf", command=self._read_mcconf_gui
        )
        self.mcconf_read_button.pack(side="right", padx=10, pady=5)
        self.mcconf_write_button = customtkinter.CTkButton(
            self.mcconf_frame,
            text="Write MCConf*",
            command=self._write_mcconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        self.mcconf_write_button.pack(side="right", padx=10, pady=5)

        self.mcconf_textbox = customtkinter.CTkTextbox(
            tab, wrap="none", height=180
        )  # Read Only
        self.mcconf_textbox.grid(row=1, column=0, padx=10, pady=(0, 10), sticky="nsew")
        self.mcconf_textbox.configure(state="disabled")  # Make read-only initially

        self.mcconf_write_warning = customtkinter.CTkLabel(
            tab,
            text="*Write MCConf modifies 'l_current_max' to 40.0 based on the last read config. Use with caution!",
            text_color="orange",
            wraplength=550,
        )
        self.mcconf_write_warning.grid(
            row=2, column=0, padx=10, pady=(0, 10), sticky="w"
        )

        # --- Application Configuration (AppConf) ---
        self.appconf_frame = customtkinter.CTkFrame(tab)
        self.appconf_frame.grid(row=3, column=0, padx=10, pady=(10, 5), sticky="ew")
        self.appconf_label = customtkinter.CTkLabel(
            self.appconf_frame,
            text="Application Configuration (AppConf)",
            font=customtkinter.CTkFont(weight="bold"),
        )
        self.appconf_label.pack(side="left", padx=10)
        self.appconf_read_button = customtkinter.CTkButton(
            self.appconf_frame, text="Read AppConf", command=self._read_appconf_gui
        )
        self.appconf_read_button.pack(side="right", padx=10, pady=5)
        self.appconf_write_button = customtkinter.CTkButton(
            self.appconf_frame,
            text="Write AppConf*",
            command=self._write_appconf_gui,
            fg_color="orange",
            hover_color="dark orange",
        )
        self.appconf_write_button.pack(side="right", padx=10, pady=5)

        self.appconf_textbox = customtkinter.CTkTextbox(
            tab, wrap="none", height=180
        )  # Read Only
        self.appconf_textbox.grid(row=4, column=0, padx=10, pady=(0, 10), sticky="nsew")
        self.appconf_textbox.configure(state="disabled")  # Make read-only initially

        self.appconf_write_warning = customtkinter.CTkLabel(
            tab,
            text="*Write AppConf toggles 'app_uart_baudrate' (9600/115200) based on the last read config. Use with caution!",
            text_color="orange",
            wraplength=550,
        )
        self.appconf_write_warning.grid(
            row=5, column=0, padx=10, pady=(0, 10), sticky="w"
        )

    def _create_detection_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(3, weight=1)  # Output Textbox

        self.detection_label = customtkinter.CTkLabel(
            tab,
            text="FOC Motor Detection Wizard",
            font=customtkinter.CTkFont(size=16, weight="bold"),
        )
        self.detection_label.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="w")

        self.detection_warning = customtkinter.CTkLabel(
            tab,
            text="!!! WARNING !!!\nThis will run motor detection. The motor WILL spin and make noise.\nEnsure safety precautions are taken!",
            text_color="red",
            font=customtkinter.CTkFont(weight="bold"),
            justify="left",
        )
        self.detection_warning.grid(row=1, column=0, padx=10, pady=5, sticky="w")

        # Parameters Frame
        self.param_frame = customtkinter.CTkFrame(tab)
        self.param_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

        self.max_loss_label = customtkinter.CTkLabel(
            self.param_frame, text="Max Power Loss (W):"
        )
        self.max_loss_label.pack(side="left", padx=(10, 5), pady=10)
        self.max_loss_entry = customtkinter.CTkEntry(self.param_frame, width=80)
        self.max_loss_entry.pack(side="left", padx=(0, 20), pady=10)
        self.max_loss_entry.insert(0, "100.0")

        # Add other parameters (CAN, Currents, RPMs) here if needed using similar CTkEntry/CTkCheckBox widgets

        self.run_detection_button = customtkinter.CTkButton(
            self.param_frame,
            text="Run FOC Detection",
            command=self._run_detection_gui,
            fg_color="red",
            hover_color="dark red",
        )
        self.run_detection_button.pack(side="right", padx=10, pady=10)

        # Output/Log Textbox
        self.detection_output_textbox = customtkinter.CTkTextbox(tab, wrap="word")
        self.detection_output_textbox.grid(
            row=3, column=0, padx=10, pady=(0, 10), sticky="nsew"
        )
        self.detection_output_textbox.configure(state="disabled")

    # --- UI Update & Logging ---
    def _log(self, message):
        print(f"LOG: {message}")  # Log to console as well
        # Schedule the GUI update to run in the main thread
        self.after(0, self._update_status_label, message)

    def _update_status_label(self, message):
        self.status_label.configure(text=f"Status: {message}")

    def _update_textbox(self, textbox, content, read_only=True):
        textbox.configure(state="normal")  # Enable writing
        textbox.delete("1.0", "end")  # Clear existing content
        textbox.insert("1.0", content)  # Insert new content
        if read_only:
            textbox.configure(state="disabled")  # Disable writing

    def _set_ui_state(self, state):  # state = "normal" or "disabled"
        # Connection controls
        self.port_combobox.configure(state=state if state == "normal" else "disabled")
        self.refresh_button.configure(state=state if state == "normal" else "disabled")
        # self.baud_entry.configure(state=state if state == "normal" else "disabled")

        # Enable connect button only if state is normal, disable otherwise
        if state == "normal":
            self.connect_button.configure(state="normal")
            # Keep connect button enabled during connection attempt? No, disable it.
        # else:
        #     self.connect_button.configure(state="disabled")

        # Settings buttons (only enable if connected and state is normal)
        settings_state = (
            "normal" if self.connected and state == "normal" else "disabled"
        )
        self.mcconf_read_button.configure(state=settings_state)
        self.mcconf_write_button.configure(state=settings_state)
        self.appconf_read_button.configure(state=settings_state)
        self.appconf_write_button.configure(state=settings_state)

        # Detection button (only enable if connected and state is normal)
        detection_state = (
            "normal" if self.connected and state == "normal" else "disabled"
        )
        self.run_detection_button.configure(state=detection_state)
        self.max_loss_entry.configure(state=detection_state)  # Also disable param entry

        # Special handling for connect button text/state during connection attempt
        if state == "disabled":
            self.connect_button.configure(state="disabled")

    # --- Port Handling ---
    def _update_port_list(self):
        self.port_list = [p.device for p in serial.tools.list_ports.comports()]
        if not self.port_list:
            self.port_list = ["No ports found"]
            self.port_combobox.set("")  # Clear selection
        else:
            # Try to keep current selection if it still exists
            current_selection = self.port_combobox.get()
            if current_selection not in self.port_list:
                self.port_combobox.set(
                    self.port_list[0] if self.port_list[0] != "No ports found" else ""
                )  # Default to first valid port
            # Else: keep the existing valid selection

        self.port_combobox.configure(values=self.port_list)
        self._log("Port list refreshed.")

    def _port_selected(self, choice):
        # Optional: Handle port selection change if needed
        pass

    # --- Connection Logic ---
    def _toggle_connection(self):
        if not self.connected:
            port = self.port_combobox.get()
            # baud = self.baud_entry.get()
            baud = DEFAULT_BAUD_RATE  # Fixed for now

            if not port or port == "No ports found":
                self._log("Error: Please select a valid port.")
                return
            # if not baud.isdigit():
            #     self._log("Error: Invalid Baud Rate.")
            #     return

            self._log(f"Connecting to {port}...")
            self._set_ui_state("disabled")  # Disable UI during connection attempt
            self.connect_button.configure(text="Connecting...")

            # Run connection in a separate thread
            thread = threading.Thread(
                target=self._connect_vesc_thread, args=(port, int(baud)), daemon=True
            )
            thread.start()
        else:
            self._log("Disconnecting...")
            self._set_ui_state("disabled")  # Disable UI during disconnection
            self.connect_button.configure(text="Disconnecting...")
            # Run disconnection in a separate thread
            thread = threading.Thread(target=self._disconnect_vesc_thread, daemon=True)
            thread.start()

    def _connect_vesc_thread(self, port, baudrate):
        try:
            self.vesc_serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=SERIAL_TIMEOUT
            )
            time.sleep(1.5)  # Allow time for device to settle
            self.vesc_serial.reset_input_buffer()
            time.sleep(0.1)
            if self.vesc_serial.in_waiting > 0:
                self.vesc_serial.read(
                    self.vesc_serial.in_waiting
                )  # Clear any welcome message

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
        if self.vesc_serial and self.vesc_serial.is_open:
            try:
                self.vesc_serial.close()
                self._log("Disconnected.")
            except Exception as e:
                self._log(f"Error during disconnect: {e}")
        self.vesc_serial = None
        self.connected = False
        # Clear stored configs on disconnect
        self.current_mc_config = None
        self.current_app_config = None
        self.after(0, self._clear_config_displays)
        self.after(0, self._update_connection_ui, False)

    def _update_connection_ui(self, is_connected):
        self.connected = is_connected
        if self.connected:
            self.connect_button.configure(text="Disconnect", state="normal")
            self._set_ui_state("normal")  # Enable other buttons
        else:
            self.connect_button.configure(text="Connect", state="normal")
            self._set_ui_state("normal")  # Re-enable connection UI
            self._set_ui_state("disabled")  # But disable VESC operations

    # --- VESC Communication Wrappers (Run in Threads) ---

    def _run_in_thread(self, target_func, args=(), callback=None):
        """Starts a function in a new thread and handles UI state."""
        if not self.connected or not self.vesc_serial:
            self._log("Error: Not connected to VESC.")
            return

        self._set_ui_state("disabled")  # Disable UI during operation
        self._log(f"Starting operation: {target_func.__name__}...")

        def thread_wrapper():
            result = None
            error = None
            try:
                result = target_func(*args)
            except serial.SerialTimeoutException:
                error = "VESC communication timed out."
            except serial.SerialException as e:
                error = f"Serial Error: {e}. Disconnecting."
                # Trigger disconnect from the thread's context might be tricky,
                # better to handle it in the callback or main thread check.
                # Consider scheduling a disconnect check.
                self.after(0, self._handle_serial_error)  # Schedule disconnect
            except ValueError as e:
                error = f"Data Error: {e}"
            except Exception as e:
                error = f"Error during {target_func.__name__}: {e}"
                import traceback

                traceback.print_exc()  # Print full traceback to console

            # Schedule the callback to run in the main thread
            self.after(0, self._operation_complete, result, error, callback)

        thread = threading.Thread(target=thread_wrapper, daemon=True)
        thread.start()

    def _operation_complete(self, result, error, callback):
        """Called in the main thread after a background operation finishes."""
        if error:
            self._log(f"Error: {error}")
        else:
            self._log(f"Operation completed successfully.")

        if callback:
            callback(result, error)  # Pass result and error to the specific handler

        # Re-enable UI only if still connected
        if self.connected:
            self._set_ui_state("normal")
        else:
            # If an error caused disconnection, update UI accordingly
            self._update_connection_ui(False)

    def _handle_serial_error(self):
        """Handles serial errors that might require disconnection."""
        self._log("Serial error detected. Attempting to disconnect.")
        if self.connected:
            self._toggle_connection()  # Gracefully disconnect

    # --- Backend VESC Functions (Called by _run_in_thread) ---

    def _clear_input_buffer(self):
        if self.vesc_serial and self.vesc_serial.is_open:
            self.vesc_serial.reset_input_buffer()
            time.sleep(0.05)
            if self.vesc_serial.in_waiting > 0:
                try:
                    self.vesc_serial.read(self.vesc_serial.in_waiting)
                except Exception as e:
                    print(f"  Buffer clear read error: {e}")  # Non-critical usually

    def _read_vesc_response(self, expected_id, timeout_override=None):
        """Reads and unframes data, checking the response ID."""
        if not self.vesc_serial:
            raise serial.SerialException("Serial port not available.")

        start_time = time.time()
        read_timeout = timeout_override if timeout_override else SERIAL_TIMEOUT
        buffer = b""

        while time.time() - start_time < read_timeout:
            if self.vesc_serial.in_waiting > 0:
                # Read incrementally to allow faster detection of packet start
                buffer += self.vesc_serial.read(self.vesc_serial.in_waiting)
                # Try to unframe what we have so far
                try:
                    payload, consumed = unframe(buffer)
                    if payload is not None:
                        if payload[0] == expected_id:
                            return (
                                payload[1:],
                                buffer,
                            )  # Return data payload and raw buffer
                        else:
                            # Keep reading if it's not the expected ID yet
                            # print(f"Warning: Received unexpected ID {payload[0]}, expected {expected_id}. Continuing read.")
                            buffer = buffer[consumed:]  # Remove processed frame
                            continue  # Keep reading for the expected packet
                    elif (
                        consumed > 0
                    ):  # Got partial/invalid frame, remove consumed bytes
                        buffer = buffer[consumed:]

                except (IndexError, struct.error) as e:
                    # Often happens with incomplete data, keep reading
                    # print(f"Unframe error (likely incomplete): {e}")
                    pass  # Keep reading
                except ValueError as e:  # crc error from unframe
                    print(f"CRC error in received packet: {e}")
                    buffer = buffer[consumed:]  # discard bad packet
                    continue  # keep reading
            time.sleep(0.02)  # Small delay to prevent busy-waiting

        # Timeout occurred
        raise serial.SerialTimeoutException(
            f"Timeout waiting for response ID {expected_id}. Received: {buffer[:100]}..."
        )

    def _backend_read_mcconf(self):
        """Reads MCConf from VESC."""
        self._clear_input_buffer()
        request_packet = encode_request(GetMcConfRequest)
        self.vesc_serial.write(request_packet)
        payload, _ = self._read_vesc_response(GetMcConfRequest.id)
        # print(f"MCConf Payload received ({len(payload)} bytes)") # Debug
        config = parse_mc_conf_serialized(payload)
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError("Failed to parse MCConf or missing signature.")
        return config

    def _backend_read_appconf(self):
        """Reads AppConf from VESC."""
        self._clear_input_buffer()
        request_packet = encode_request(GetAppConfRequest)
        self.vesc_serial.write(request_packet)
        payload, _ = self._read_vesc_response(GetAppConfRequest.id)
        # print(f"AppConf Payload received ({len(payload)} bytes)") # Debug
        config = parse_app_conf_serialized(payload)
        if not config or "APPCONF_SIGNATURE" not in config:
            raise ValueError("Failed to parse AppConf or missing signature.")
        return config

    def _backend_write_mcconf(self, config):
        """Writes MCConf to VESC."""
        if not config or "MCCONF_SIGNATURE" not in config:
            raise ValueError("Invalid MCConf data for writing.")

        set_message = SetMcConf()
        set_message.mc_configuration = config
        set_packet = encode_set_mcconf(set_message)  # Use the helper

        self._clear_input_buffer()
        bytes_written = self.vesc_serial.write(set_packet)
        # print(f"Wrote {bytes_written} bytes for SetMcConf") # Debug
        time.sleep(0.5)  # Give VESC time to process write commands
        # VESC usually doesn't send a confirmation for SET commands
        return True  # Assume success if no immediate error

    def _backend_write_appconf(self, config):
        """Writes AppConf to VESC."""
        if not config or "APPCONF_SIGNATURE" not in config:
            raise ValueError("Invalid AppConf data for writing.")

        set_message = SetAppConf()
        set_message.app_configuration = config
        set_packet = encode_set_appconf(set_message)  # Use the helper

        self._clear_input_buffer()
        bytes_written = self.vesc_serial.write(set_packet)
        # print(f"Wrote {bytes_written} bytes for SetAppConf") # Debug
        time.sleep(0.5)  # Give VESC time to process
        return True  # Assume success

    def _backend_run_detection(self, max_loss):
        """Runs the FOC detection wizard."""
        wizard_message = DetectApplyAllFOC()
        # Set parameters from GUI or defaults
        wizard_message.detect_can = False  # Defaulting, add GUI element if needed
        wizard_message.max_power_loss = float(max_loss)
        wizard_message.min_current_in = 0.0  # Defaulting
        wizard_message.max_current_in = 0.0  # Defaulting
        wizard_message.openloop_rpm = 0.0  # Defaulting
        wizard_message.sl_erpm = 0.0  # Defaulting

        packet_wizard = encode_detect_apply_all_foc(wizard_message)  # Use helper

        self._clear_input_buffer()
        bytes_written = self.vesc_serial.write(packet_wizard)
        # print(f"Wrote {bytes_written} bytes for DetectApplyAllFOC") # Debug

        # Wait for the response (which is typically the updated MCConf)
        # Use a longer timeout for detection
        payload, raw_response = self._read_vesc_response(
            GetMcConfRequest.id, timeout_override=DETECTION_WIZARD_TIMEOUT_GUI
        )

        detected_config = parse_mc_conf_serialized(payload)
        if not detected_config or "MCCONF_SIGNATURE" not in detected_config:
            # Sometimes detection might fail and send a different response code?
            # Or just timeout. Check raw response?
            # For now, assume failure if parsing fails.
            raise ValueError(
                f"Detection finished, but failed to parse the returned MCConf. Raw response ({len(raw_response)} bytes): {raw_response[:100]}..."
            )

        # Return the newly detected config
        return detected_config

    # --- GUI Action Handlers ---

    def _read_mcconf_gui(self):
        self._run_in_thread(
            self._backend_read_mcconf, callback=self._handle_mcconf_read_result
        )

    def _handle_mcconf_read_result(self, result, error):
        if error:
            self._update_textbox(self.mcconf_textbox, f"Error reading MCConf:\n{error}")
            self.current_mc_config = None
        elif result:
            self.current_mc_config = result  # Store the latest config
            config_str = pprint.pformat(result, indent=2, width=100)
            self._update_textbox(self.mcconf_textbox, config_str)
            self._log("MCConf read successfully.")
        else:
            self._update_textbox(
                self.mcconf_textbox, "Failed to read MCConf (No data)."
            )
            self.current_mc_config = None

    def _read_appconf_gui(self):
        self._run_in_thread(
            self._backend_read_appconf, callback=self._handle_appconf_read_result
        )

    def _handle_appconf_read_result(self, result, error):
        if error:
            self._update_textbox(
                self.appconf_textbox, f"Error reading AppConf:\n{error}"
            )
            self.current_app_config = None
        elif result:
            self.current_app_config = result  # Store the latest config
            config_str = pprint.pformat(result, indent=2, width=100)
            self._update_textbox(self.appconf_textbox, config_str)
            self._log("AppConf read successfully.")
        else:
            self._update_textbox(
                self.appconf_textbox, "Failed to read AppConf (No data)."
            )
            self.current_app_config = None

    def _write_mcconf_gui(self):
        if not self.current_mc_config:
            self._log("Error: Read MCConf first before writing.")
            return

        # --- !!! SAFETY WARNING !!! ---
        # Modify the config based on the hardcoded rule from set_mcconf.py
        config_to_write = copy.deepcopy(self.current_mc_config)
        field_to_change = "l_current_max"
        new_value = 40.0
        original_value = config_to_write.get(field_to_change, "N/A")
        config_to_write[field_to_change] = new_value
        self._log(
            f"Preparing to write MCConf: Changing '{field_to_change}' from {original_value} to {new_value}"
        )
        # --- End Safety Warning ---

        # Remove CRC if present before sending (packer should handle it)
        config_to_write.pop("crc", None)

        self._run_in_thread(
            self._backend_write_mcconf,
            args=(config_to_write,),
            callback=self._handle_mcconf_write_result,
        )

    def _handle_mcconf_write_result(self, result, error):
        if error:
            self._log(f"Error writing MCConf: {error}")
        elif result:
            self._log("MCConf write command sent. Recommend re-reading to verify.")
            # Optionally trigger an automatic re-read here
            # self._read_mcconf_gui()
        else:
            self._log("MCConf write failed (unknown reason).")

    def _write_appconf_gui(self):
        if not self.current_app_config:
            self._log("Error: Read AppConf first before writing.")
            return

        # --- !!! SAFETY WARNING !!! ---
        # Modify the config based on the hardcoded rule from appconf.py
        config_to_write = copy.deepcopy(self.current_app_config)
        field_to_change = "app_uart_baudrate"
        current_baud = config_to_write.get(
            field_to_change, 115200
        )  # Default if missing
        new_baud = 9600 if current_baud != 9600 else 115200  # Toggle
        config_to_write[field_to_change] = new_baud
        self._log(
            f"Preparing to write AppConf: Changing '{field_to_change}' from {current_baud} to {new_baud}"
        )
        # --- End Safety Warning ---

        # Remove CRC if present
        config_to_write.pop("crc", None)

        self._run_in_thread(
            self._backend_write_appconf,
            args=(config_to_write,),
            callback=self._handle_appconf_write_result,
        )

    def _handle_appconf_write_result(self, result, error):
        if error:
            self._log(f"Error writing AppConf: {error}")
        elif result:
            self._log("AppConf write command sent. Recommend re-reading to verify.")
            # self._read_appconf_gui() # Optionally re-read
        else:
            self._log("AppConf write failed (unknown reason).")

    def _run_detection_gui(self):
        max_loss_str = self.max_loss_entry.get()
        try:
            max_loss = float(max_loss_str)
        except ValueError:
            self._log("Error: Invalid Max Power Loss value.")
            self._update_textbox(
                self.detection_output_textbox,
                "Error: Invalid Max Power Loss value.",
                read_only=True,
            )
            return

        # Clear previous output
        self._update_textbox(
            self.detection_output_textbox,
            f"Starting FOC Detection (Max Loss: {max_loss}W)...\nVESC will now measure the motor. DO NOT INTERRUPT.\nTimeout: {DETECTION_WIZARD_TIMEOUT_GUI}s",
            read_only=True,
        )

        # Add confirmation dialog?
        # confirm = messagebox.askyesno("Run Detection?", "Motor will spin! Are you sure?")
        # if not confirm:
        #     self._log("Detection cancelled by user.")
        #     self._update_textbox(self.detection_output_textbox, "Detection cancelled by user.", read_only=True)
        #     return

        self._run_in_thread(
            self._backend_run_detection,
            args=(max_loss,),
            callback=self._handle_detection_result,
        )

    def _handle_detection_result(self, result, error):
        current_output = self.detection_output_textbox.get(
            "1.0", "end-1c"
        )  # Get existing text
        if error:
            output = current_output + f"\n\nDETECTION FAILED:\n{error}"
            self._update_textbox(self.detection_output_textbox, output, read_only=True)
            self._log(f"Detection failed: {error}")
        elif result:
            self.current_mc_config = (
                result  # Update the stored MC Conf with detected values
            )
            output = current_output + "\n\nDETECTION SUCCEEDED!\n"
            output += "New Motor Configuration received and applied by VESC:\n"
            output += "----------------------------------------------------\n"
            output += pprint.pformat(result, indent=2, width=100) + "\n"
            output += "----------------------------------------------------\n"
            output += "MCConf has been updated with these values.\n"

            # Highlight key detected params
            r = result.get("foc_motor_r")
            l = result.get("foc_motor_l")
            lambda_ = result.get("foc_motor_flux_linkage")
            kp = result.get("foc_current_kp")
            ki = result.get("foc_current_ki")
            observer_gain = result.get("foc_observer_gain")

            output += "\n--- Key Detected Parameters ---\n"
            output += (
                f"Resistance (R) : {r * 1e3:.3f} mOhm\n"
                if r is not None
                else "R: N/A\n"
            )
            output += (
                f"Inductance (L) : {l * 1e6:.3f} uH\n" if l is not None else "L: N/A\n"
            )
            output += (
                f"Flux Linkage   : {lambda_ * 1e3:.3f} mWb\n"
                if lambda_ is not None
                else "Flux: N/A\n"
            )
            output += f"Current Kp     : {kp:.6f}\n" if kp is not None else "Kp: N/A\n"
            output += f"Current Ki     : {ki:.6f}\n" if ki is not None else "Ki: N/A\n"
            output += (
                f"Observer Gain  : {observer_gain:.2f}\n"
                if observer_gain is not None
                else "Observer Gain: N/A\n"
            )

            self._update_textbox(self.detection_output_textbox, output, read_only=True)
            # Update the Settings tab display as well
            self.after(
                0,
                self._update_textbox,
                self.mcconf_textbox,
                pprint.pformat(result, indent=2, width=100),
            )
            self._log("Detection successful. MCConf updated.")
        else:
            output = (
                current_output
                + "\n\nDETECTION FAILED (Unknown reason, no data returned)."
            )
            self._update_textbox(self.detection_output_textbox, output, read_only=True)
            self._log("Detection failed (No data).")

    def _clear_config_displays(self):
        """Clears the text boxes when disconnecting."""
        self._update_textbox(self.mcconf_textbox, "")
        self._update_textbox(self.appconf_textbox, "")
        self._update_textbox(self.detection_output_textbox, "")

    # --- Window Closing ---
    def _on_closing(self):
        print("Window closing...")
        if self.connected:
            self._log("Disconnecting before closing...")
            if self.vesc_serial and self.vesc_serial.is_open:
                # Try immediate close, background thread might not finish
                try:
                    self.vesc_serial.close()
                    print("Serial port closed.")
                except Exception as e:
                    print(f"Error closing serial port on exit: {e}")
        self.destroy()


# --- Run the Application ---
if __name__ == "__main__":
    app = VescApp()
    app.mainloop()
