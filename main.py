#!/usr/bin/env python3
"""
NeoArm main.py (updated)
 - Smooth open/close sweeps for fingers
 - pigpio button callback with software debounce
 - Serial FSR/temp reader, ADS1115 wrist/elbow, pigpio servo PWM
 - NEW: Background vision server to get "soft"/"hard" category
"""
import time
import threading
import sys
import signal
import socket  # <-- NEW
import queue   # <-- NEW

import pigpio
import Adafruit_ADS1x15
import serial

# ---------------- Config ----------------
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SERIAL_TIMEOUT = 1.0  # seconds

FINGER_PINS = [17, 18, 19, 20, 21]      # thumb -> pinky
WRIST_ELBOW_PINS = [22, 23]          # wrist, elbow
ALL_SERVOS = FINGER_PINS + WRIST_ELBOW_PINS

BUTTON_PIN = 27                      # BCM, wired to GND, use internal pull-up
BUTTON_DEBOUNCE_S = 0.35             # ignore presses within 350 ms

OPEN_ANGLE = 0
CLOSE_ANGLE = 180
STEP_DEG = 3
STEP_DELAY = 0.04

# --- MODIFIED: THRESHOLDS are now dynamic ---
# This variable will be UPDATED by the vision server.
THRESHOLDS = [500, 500, 500, 500, 500] 

# Safety temperature
TEMP_ABORT = 40.0

# ADC (ADS1115) config
adc = Adafruit_ADS1x15.ADS1115(address=0x48, busnum=1)
ADC_CHANNELS = [0, 1]  # channel 0 -> wrist pot, channel 1 -> elbow pot
ADC_RAW_MIN = 0
ADC_RAW_MAX = 32767

# servo pulse range (microseconds)
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# --- NEW: Socket Server and Queue Config ---
VISION_SERVER_HOST = "0.0.0.0"  # Listen on all interfaces
VISION_SERVER_PORT = 9999
category_queue = queue.Queue() # Thread-safe queue for communication

# --- NEW: LED Configuration (Using BCM pins) ---
LED_RED_PIN = 25   # (BCM 25 for Red) <-- NEW
LED_BLUE_PIN = 26  # (BCM 26 for Blue) <-- NEW
DELAY_SECONDS = 40.0 # <-- NEW

# ---------------- End Config ----------------

# ---------------- Utilities ----------------
def angle_to_pulse(a):
    # (This function is UNCHANGED)
    a = max(0.0, min(180.0, a))
    return int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (a / 180.0))

# ---------------- Servo controller (smooth open/close) ----------------
class SimpleServoController:
    # (This class is UNCHANGED)
    def __init__(self, pi: pigpio.pi, pins):
        self.pi = pi
        self.pins = list(pins)
        self.current = {p: float(OPEN_ANGLE) for p in self.pins}
        for p in self.pins:
            self.pi.set_servo_pulsewidth(p, angle_to_pulse(OPEN_ANGLE))
        time.sleep(0.12)

    def _set_angle_immediate(self, pin, angle):
        self.current[pin] = float(angle)
        self.pi.set_servo_pulsewidth(pin, angle_to_pulse(angle))

    def move_to_angle_by_pin(self, pin, angle):
        if pin not in self.pins:
            raise ValueError(f"Pin {pin} not registered in controller")
        self._set_angle_immediate(pin, angle)

    def stop_all(self):
        for p in self.pins:
            try:
                self.pi.set_servo_pulsewidth(p, 0)
            except Exception:
                pass

    def _smooth_move(self, targets: dict, step_deg=STEP_DEG, step_delay=STEP_DELAY):
        cur = {p: float(self.current.get(p, OPEN_ANGLE)) for p in self.pins}
        tgs = {p: float(targets.get(p, cur[p])) for p in self.pins}
        if all(abs(cur[p] - tgs[p]) < 1e-3 for p in self.pins):
            return
        max_delta = max(abs(cur[p] - tgs[p]) for p in self.pins)
        steps = max(1, int(max_delta / max(1e-6, step_deg)))
        for _ in range(steps):
            if stop_flag.is_set(): return # <-- NEW: Added check to stop move early
            done = True
            for p in self.pins:
                if abs(cur[p] - tgs[p]) < 1e-6:
                    continue
                done = False
                if cur[p] < tgs[p]:
                    cur[p] = min(tgs[p], cur[p] + step_deg)
                else:
                    cur[p] = max(tgs[p], cur[p] - step_deg)
                try:
                    self.pi.set_servo_pulsewidth(p, angle_to_pulse(cur[p]))
                    self.current[p] = float(cur[p])
                except Exception:
                    pass
            if done:
                break
            time.sleep(step_delay)
        # Set final exact positions
        if not stop_flag.is_set(): # <-- NEW: Only set final if not stopping
            for p in self.pins:
                try:
                    self.pi.set_servo_pulsewidth(p, angle_to_pulse(tgs[p]))
                    self.current[p] = float(tgs[p])
                except Exception:
                    pass

    def smooth_open(self, step_deg=STEP_DEG, step_delay=STEP_DELAY):
        targets = {p: float(OPEN_ANGLE) for p in self.pins}
        self._smooth_move(targets, step_deg=step_deg, step_delay=step_delay)

    def smooth_close(self, step_deg=STEP_DEG, step_delay=STEP_DELAY):
        targets = {p: float(CLOSE_ANGLE) for p in self.pins}
        self._smooth_move(targets, step_deg=step_deg, step_delay=step_delay)

    def cleanup(self):
        try:
            self.smooth_open()
            time.sleep(0.12)
            self.stop_all()
        except Exception:
            pass

# ---------------- Serial reader thread ----------------
latest_lock = threading.Lock()
latest = {'fsr': [0, 0, 0, 0, 0], 'temp': 0.0, 'last_ts': None}
stop_flag = threading.Event()

def serial_thread_fn(port, baud):
    # (This function is UNCHANGED)
    global latest
    while not stop_flag.is_set():
        try:
            with serial.Serial(port, baud, timeout=SERIAL_TIMEOUT) as ser:
                time.sleep(1.0); ser.reset_input_buffer()
                while not stop_flag.is_set():
                    try:
                        raw = ser.readline()
                        if not raw: continue
                        try: line = raw.decode('ascii', errors='ignore').strip()
                        except Exception: line = raw.decode('latin1', errors='ignore').strip()
                        if not line: continue
                        parts = [p.strip() for p in line.split(',') if p.strip() != '']
                        if len(parts) >= 6:
                            try:
                                fsrs = [int(float(parts[i])) for i in range(5)]
                                temp = float(parts[5])
                                with latest_lock:
                                    latest['fsr'] = fsrs; latest['temp'] = temp; latest['last_ts'] = time.time()
                            except Exception: pass
                    except Exception: continue
        except Exception as e:
            if not stop_flag.is_set():
                print("[SERIAL] Serial open/read error:", e); time.sleep(2.0)

# --- NEW: Vision Server Thread ---
def _vision_server_thread_fn():
    """
    Listens for a category ("soft"/"hard") from the laptop
    and puts it in the queue for the main thread to handle.
    """
    global category_queue
    server_socket = None # Define server_socket here
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((VISION_SERVER_HOST, VISION_SERVER_PORT))
        server_socket.listen(1)
        print(f"\n[Vision Server] SUCCESS. Listening on {VISION_SERVER_HOST}:{VISION_SERVER_PORT}")
        
        while not stop_flag.is_set():
            try:
                print("\n[Vision Server] Waiting for a detection from laptop...")
                # This call will block, but it's on a background thread
                conn, addr = server_socket.accept() 
                
                with conn:
                    print(f"[Vision Server] Connection established with: {addr[0]}")
                    data = conn.recv(1024)
                    
                    if data:
                        category = data.decode('utf-8')
                        if category in ["soft", "hard"]:
                            print(f"[Vision Server] Received category: '{category}'")
                            category_queue.put(category) # Put the valid category into the queue
                        else:
                            print(f"[Vision Server] Received unknown data: '{category}'")
            except socket.error as e:
                if not stop_flag.is_set():
                    print(f"[Vision Server] Connection error: {e}"); time.sleep(1)
            
    except socket.error as e:
        print(f"\n[Vision Server] FATAL ERROR: Could not start server. Error: {e}")
    except Exception as e:
        if not stop_flag.is_set():
            print(f"\n[Vision Server] Thread error: {e}")
    finally:
        if server_socket: # Check if it exists before closing
            server_socket.close()
        print("[Vision Server] Shutting down.")


# ---------------- Actuation logic ----------------
state_lock = threading.Lock()
state = 'idle'
_last_button_ts = 0.0

def _register_button_press():
    # (This function is UNCHANGED)
    global _last_button_ts
    now = time.time()
    if now - _last_button_ts < BUTTON_DEBOUNCE_S:
        return False
    _last_button_ts = now
    return True

def button_callback(pin):
    # (This function is UNCHANGED)
    global state
    if not _register_button_press():
        return
    with state_lock:
        if state == 'idle':
            state = 'closing'
            print("[BUTTON] Press -> start closing")
        elif state == 'closing':
            state = 'idle'
            print("[BUTTON] Press -> abort/stop (open)")
        elif state == 'gripped':
            state = 'idle'
            print("[BUTTON] Press -> opening")

def finger_actuation_loop(servo_ctrl: SimpleServoController, finger_pins):
    # (This function is UNCHANGED - it will automatically use the new THRESHOLDS)
    global state, THRESHOLDS 
    cur_angle = {p: float(OPEN_ANGLE) for p in finger_pins}
    try:
        while not stop_flag.is_set():
            with state_lock: s = state
            if s == 'idle':
                not_open = any(abs(servo_ctrl.current.get(p, OPEN_ANGLE) - OPEN_ANGLE) > 1.0 for p in finger_pins)
                if not_open:
                    servo_ctrl.smooth_open()
                time.sleep(0.12); continue
            if s == 'closing':
                reached = {p: False for p in finger_pins}
                for p in finger_pins:
                    cur_angle[p] = float(OPEN_ANGLE)
                    servo_ctrl.move_to_angle_by_pin(p, OPEN_ANGLE)
                time.sleep(0.08)
                while not all(reached.values()) and state == 'closing' and not stop_flag.is_set():
                    with latest_lock:
                        fsr_vals = list(latest['fsr']); temp = latest['temp']
                    if temp >= TEMP_ABORT:
                        print(f"[SAFETY] Temperature {temp:.1f}°C >= {TEMP_ABORT}°C: aborting and opening.")
                        with state_lock: state = 'idle'
                        servo_ctrl.smooth_open(); break
                    for idx, p in enumerate(finger_pins):
                        if reached[p]: continue
                        val = fsr_vals[idx] if idx < len(fsr_vals) else 0
                        if val >= THRESHOLDS[idx]: 
                            reached[p] = True
                            print(f"[GRIP] finger {idx} reached threshold ({val} >= {THRESHOLDS[idx]})")
                            continue
                        if cur_angle[p] < CLOSE_ANGLE:
                            cur_angle[p] = min(CLOSE_ANGLE, cur_angle[p] + STEP_DEG)
                            servo_ctrl.move_to_angle_by_pin(p, cur_angle[p])
                    time.sleep(STEP_DELAY)
                with state_lock:
                    if state == 'closing':
                        state = 'gripped'; print("[STATE] Gripping done -> state = gripped")
                    else:
                        print("[STATE] Closing interrupted; new state:", state)
            elif s == 'gripped':
                time.sleep(0.12)
            else:
                time.sleep(0.12)
    except Exception as e:
        print("[ERROR] finger_actuation_loop:", e)

def wrist_elbow_loop(servo_ctrl: SimpleServoController, we_pins, adc_channels):
    # (This function is UNCHANGED)
    def raw_to_angle(raw):
        if raw is None: return 90
        if raw < ADC_RAW_MIN: raw = ADC_RAW_MIN
        if raw > ADC_RAW_MAX: raw = ADC_RAW_MAX
        return int((raw - ADC_RAW_MIN) / max(1, (ADC_RAW_MAX - ADC_RAW_MIN)) * 180)
    try:
        while not stop_flag.is_set():
            for i, p in enumerate(we_pins):
                try:
                    ch = adc_channels[i]; raw = adc.read_adc(ch, gain=1)
                    angle = raw_to_angle(raw)
                    servo_ctrl.move_to_angle_by_pin(p, angle)
                except Exception: continue
            time.sleep(0.04)
    except Exception as e:
        print("[ERROR] wrist_elbow_loop:", e)

# ---------------- Main ----------------
def main():
    global state, button_cb, pi, THRESHOLDS # <-- MODIFIED (added THRESHOLDS)
    button_cb = None
    pi = None

    print("NeoArm main starting... (make sure pigpiod is running)")

    # Start serial thread (UNCHANGED)
    th_serial = threading.Thread(target=serial_thread_fn, args=(SERIAL_PORT, BAUDRATE), daemon=True)
    th_serial.start()
    print("[MAIN] Serial thread started.")

    # Start pigpio (UNCHANGSSED)
    pi = pigpio.pi()
    if not pi.connected:
        print("[MAIN] pigpio not connected. Start pigpiod and retry (sudo pigpiod).")
        stop_flag.set(); return

    # Setup servo controller (UNCHANGED)
    servo_ctrl = SimpleServoController(pi, ALL_SERVOS)
    print("[MAIN] Servo controller initialized. All servos set to OPEN_ANGLE.")

    # --- NEW: Setup LED pins using pigpio ---
    try:
        pi.set_mode(LED_RED_PIN, pigpio.OUTPUT)
        pi.set_mode(LED_BLUE_PIN, pigpio.OUTPUT)
        pi.write(LED_RED_PIN, 0) # Off
        pi.write(LED_BLUE_PIN, 0) # Off
        print(f"[MAIN] LEDs set on BCM {LED_RED_PIN} (Red) and {LED_BLUE_PIN} (Blue).")
    except Exception as e:
        print(f"[MAIN] Failed to setup LED pins: {e}")
    # ---

    # Setup button via pigpio (UNCHANGED)
    try:
        pi.set_mode(BUTTON_PIN, pigpio.INPUT)
        pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_UP)
        def _pigpio_button_cb(gpio, level, tick):
            if level == 0:
                try: button_callback(gpio)
                except Exception as e: print("[MAIN] button callback error:", e)
        button_cb = pi.callback(BUTTON_PIN, pigpio.FALLING_EDGE, _pigpio_button_cb)
        print(f"[MAIN] Button set on GPIO{BUTTON_PIN} via pigpio (FALLING -> callback).")
    except Exception as e:
        print("[MAIN] Failed to setup button via pigpio:", e);
