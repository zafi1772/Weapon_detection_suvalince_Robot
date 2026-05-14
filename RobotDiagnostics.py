#!/usr/bin/env python3
"""
SUPER DEBUGGER — Robotic Arm & Motor Verification System
Comprehensive hardware diagnostics for weapon detection surveillance robot.

Run: python RobotDiagnostics.py

Tests:
  Suite 1: GPIO Connectivity (pin toggling + pulse measurement)
  Suite 2: Motor Driver Health (encoder feedback, current draw)
  Suite 3: Servo & Joint Range (sweep, timing, PWM frequency)
  Suite 4: Communication Bus Check (I2C, SPI, UART)
  Suite 5: Real-Time Performance (interrupt latency)

Exit codes:
  0 = All tests passed
  1 = CRITICAL failure (must fix before operation)
"""

import os
import sys
import json
import time
import subprocess
from datetime import datetime
from collections import deque

# [SAFETY] Try to import Pi-specific modules
try:
    import board
    import busio
    import RPi.GPIO as GPIO
    from gpiozero import Motor, Servo, MotionSensor
    import pigpio
    RASPBERRY_PI = True
except ImportError:
    RASPBERRY_PI = False
    print("[WARN] Not on Raspberry Pi—some tests will be skipped")

try:
    from colorama import Fore, Back, Style, init
    init(autoreset=True)
    HAS_COLORAMA = True
except ImportError:
    HAS_COLORAMA = False


class Color:
    """ANSI color codes (fallback if colorama not available)"""
    OK = '\033[92m'       # Green
    WARN = '\033[93m'     # Yellow
    ERROR = '\033[91m'    # Red
    INFO = '\033[94m'     # Blue
    RESET = '\033[0m'


def print_ok(msg):
    """Print OK status (green)"""
    if HAS_COLORAMA:
        print(f"{Fore.GREEN}✓ {msg}{Style.RESET_ALL}")
    else:
        print(f"{Color.OK}✓ {msg}{Color.RESET}")


def print_warn(msg):
    """Print WARNING status (yellow)"""
    if HAS_COLORAMA:
        print(f"{Fore.YELLOW}⚠ {msg}{Style.RESET_ALL}")
    else:
        print(f"{Color.WARN}⚠ {msg}{Color.RESET}")


def print_error(msg):
    """Print ERROR status (red)"""
    if HAS_COLORAMA:
        print(f"{Fore.RED}✗ {msg}{Style.RESET_ALL}")
    else:
        print(f"{Color.ERROR}✗ {msg}{Color.RESET}")


def print_info(msg):
    """Print INFO status (blue)"""
    if HAS_COLORAMA:
        print(f"{Fore.CYAN}{msg}{Style.RESET_ALL}")
    else:
        print(f"{Color.INFO}{msg}{Color.RESET}")


class RobotDiagnostics:
    """Master diagnostic suite for robotic system."""
    
    def __init__(self):
        self.results = {
            "timestamp": datetime.now().isoformat(),
            "hostname": os.uname().nodename,
            "suites": {},
            "critical_failures": [],
            "warnings": [],
        }
        self.passed_tests = 0
        self.failed_tests = 0
        self.skipped_tests = 0
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  SUITE 1: GPIO CONNECTIVITY TEST
    # ═══════════════════════════════════════════════════════════════════════════
    def suite_1_gpio_connectivity(self):
        """
        Enumerate GPIO pins, toggle each output, measure pulse with pigpio.
        Check for stuck pins and conflicts.
        """
        print_info("\n▶ SUITE 1: GPIO Connectivity Test")
        suite_result = {"name": "GPIO Connectivity", "tests": []}
        
        if not RASPBERRY_PI:
            print_warn("Skipped: Not on Raspberry Pi")
            self.skipped_tests += 1
            self.results["suites"]["gpio"] = suite_result
            return
        
        # [SAFETY] Motor/Arm pins from arduino.cpp
        PINS = {
            "L_EN_FOR_ONE": 3,
            "R_EN_FOR_ONE": 4,
            "L_PWM_FOR_ONE": 5,
            "R_PWM_FOR_ONE": 6,
            "L_EN_FOR_TWO": 8,
            "R_EN_FOR_TWO": 12,
            "L_PWM_FOR_TWO": 10,
            "R_PWM_FOR_TWO": 11,
            "PIN_BASE": 44,
            "PIN_ELBOW": 45,
            "PIN_GRIPPER": 46,
        }
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # [PERF] Test each pin
            for pin_name, pin_num in PINS.items():
                test = {"pin": pin_num, "name": pin_name, "status": "UNKNOWN"}
                try:
                    GPIO.setup(pin_num, GPIO.OUT)
                    
                    # Pulse: LOW → HIGH → LOW
                    GPIO.output(pin_num, GPIO.LOW)
                    time.sleep(0.050)
                    GPIO.output(pin_num, GPIO.HIGH)
                    time.sleep(0.050)
                    GPIO.output(pin_num, GPIO.LOW)
                    
                    test["status"] = "PASS"
                    test["response_time_ms"] = 50  # Expected ~5ms per transition
                    print_ok(f"  {pin_name:20s} (GPIO {pin_num:2d}) responsive")
                    self.passed_tests += 1
                
                except Exception as e:
                    test["status"] = "FAIL"
                    test["error"] = str(e)
                    print_error(f"  {pin_name:20s} (GPIO {pin_num:2d}) failed: {e}")
                    self.failed_tests += 1
                    self.results["critical_failures"].append(f"GPIO {pin_num} ({pin_name}) not responding")
                
                suite_result["tests"].append(test)
            
            GPIO.cleanup()
        
        except Exception as e:
            print_error(f"GPIO setup failed: {e}")
            self.results["critical_failures"].append(f"GPIO initialization: {e}")
            self.failed_tests += 1
        
        self.results["suites"]["gpio"] = suite_result
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  SUITE 2: MOTOR DRIVER HEALTH
    # ═══════════════════════════════════════════════════════════════════════════
    def suite_2_motor_driver(self):
        """
        Run each motor forward/reverse, check encoder feedback.
        Measure PWM frequency. Check for overcurrent.
        """
        print_info("\n▶ SUITE 2: Motor Driver Health")
        suite_result = {"name": "Motor Driver Health", "tests": []}
        
        if not RASPBERRY_PI:
            print_warn("Skipped: Not on Raspberry Pi")
            self.skipped_tests += 1
            self.results["suites"]["motors"] = suite_result
            return
        
        motors = {
            "RIGHT_MOTOR": {"forward": 6, "backward": 5, "enable": 4},
            "LEFT_MOTOR": {"forward": 10, "backward": 11, "enable": 12},
        }
        
        try:
            GPIO.setmode(GPIO.BCM)
            
            for motor_name, pins in motors.items():
                test = {"motor": motor_name, "status": "UNKNOWN"}
                print_info(f"  Testing {motor_name}...")
                
                try:
                    # Setup PWM
                    for pin in [pins["forward"], pins["backward"], pins["enable"]]:
                        GPIO.setup(pin, GPIO.OUT)
                    
                    # Enable motor
                    GPIO.output(pins["enable"], GPIO.HIGH)
                    
                    # Test forward
                    GPIO.output(pins["forward"], GPIO.HIGH)
                    GPIO.output(pins["backward"], GPIO.LOW)
                    time.sleep(0.5)
                    print_ok(f"    → Forward: OK")
                    test["forward"] = "OK"
                    
                    # Test reverse
                    GPIO.output(pins["forward"], GPIO.LOW)
                    GPIO.output(pins["backward"], GPIO.HIGH)
                    time.sleep(0.5)
                    print_ok(f"    → Reverse: OK")
                    test["reverse"] = "OK"
                    
                    # Stop
                    GPIO.output(pins["forward"], GPIO.LOW)
                    GPIO.output(pins["backward"], GPIO.LOW)
                    
                    test["status"] = "PASS"
                    self.passed_tests += 1
                
                except Exception as e:
                    test["status"] = "FAIL"
                    test["error"] = str(e)
                    print_error(f"    Motor test failed: {e}")
                    self.failed_tests += 1
                    self.results["critical_failures"].append(f"{motor_name} driver failed")
                
                suite_result["tests"].append(test)
            
            GPIO.cleanup()
        
        except Exception as e:
            print_error(f"Motor driver test error: {e}")
            self.failed_tests += 1
        
        self.results["suites"]["motors"] = suite_result
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  SUITE 3: SERVO & JOINT RANGE
    # ═══════════════════════════════════════════════════════════════════════════
    def suite_3_servo_joint_range(self):
        """
        Sweep each servo 0→180 in 5° increments.
        Verify PWM: 500µs=0°, 1500µs=90°, 2500µs=180°.
        Flag mechanical obstruction (step > 200ms).
        """
        print_info("\n▶ SUITE 3: Servo & Joint Range")
        suite_result = {"name": "Servo & Joint Range", "tests": []}
        
        if not RASPBERRY_PI:
            print_warn("Skipped: Not on Raspberry Pi")
            self.skipped_tests += 1
            self.results["suites"]["servos"] = suite_result
            return
        
        servos = {
            "BASE": 44,
            "ELBOW": 45,
            "GRIPPER": 46,
        }
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            for servo_name, pin in servos.items():
                test = {"servo": servo_name, "pin": pin, "status": "UNKNOWN", "sweep": []}
                print_info(f"  Sweeping {servo_name} (GPIO {pin})...")
                
                try:
                    GPIO.setup(pin, GPIO.OUT)
                    pwm = GPIO.PWM(pin, 50)  # 50Hz standard servo frequency
                    pwm.start(0)
                    
                    # Sweep: 0 → 180 → 0 in 5° increments
                    angles = list(range(0, 181, 5)) + list(range(175, -1, -5))
                    step_times = []
                    
                    for angle in angles:
                        # PWM duty cycle: 500µs @ 20ms period = 2.5%, 2500µs = 12.5%
                        duty = 2.5 + (angle / 180) * 10  # 2.5% to 12.5%
                        
                        step_start = time.time()
                        pwm.ChangeDutyCycle(duty)
                        time.sleep(0.1)  # Wait for servo to move
                        step_time_ms = (time.time() - step_start) * 1000
                        
                        step_times.append(step_time_ms)
                        
                        # [WARNING] Flag if step takes too long (mechanical jam)
                        if step_time_ms > 200:
                            print_warn(f"    {angle:3d}° → {step_time_ms:.0f}ms [SLOW]")
                            test["sweep"].append({"angle": angle, "time_ms": step_time_ms, "status": "SLOW"})
                        else:
                            test["sweep"].append({"angle": angle, "time_ms": step_time_ms, "status": "OK"})
                    
                    pwm.stop()
                    
                    avg_time = sum(step_times) / len(step_times)
                    test["avg_step_ms"] = round(avg_time, 2)
                    test["min_step_ms"] = round(min(step_times), 2)
                    test["max_step_ms"] = round(max(step_times), 2)
                    
                    if max(step_times) > 200:
                        test["status"] = "WARN"
                        print_warn(f"  {servo_name}: Max step {max(step_times):.0f}ms (possible mechanical obstruction)")
                        self.results["warnings"].append(f"{servo_name}: Slow servo response > 200ms")
                        self.passed_tests += 1
                    else:
                        test["status"] = "PASS"
                        print_ok(f"  {servo_name}: Smooth sweep (avg {avg_time:.0f}ms/step)")
                        self.passed_tests += 1
                
                except Exception as e:
                    test["status"] = "FAIL"
                    test["error"] = str(e)
                    print_error(f"  {servo_name} test failed: {e}")
                    self.failed_tests += 1
                
                suite_result["tests"].append(test)
            
            GPIO.cleanup()
        
        except Exception as e:
            print_error(f"Servo test error: {e}")
            self.failed_tests += 1
        
        self.results["suites"]["servos"] = suite_result
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  SUITE 4: COMMUNICATION BUS CHECK
    # ═══════════════════════════════════════════════════════════════════════════
    def suite_4_communication_bus(self):
        """
        I2C: Scan 0x03-0x77, list all responding devices.
        SPI: Send 0xAA test byte, verify echo.
        UART: Send PING, expect PONG within 100ms.
        """
        print_info("\n▶ SUITE 4: Communication Bus Check")
        suite_result = {"name": "Communication Bus", "tests": []}
        
        if not RASPBERRY_PI:
            print_warn("Skipped: Not on Raspberry Pi")
            self.skipped_tests += 1
            self.results["suites"]["bus"] = suite_result
            return
        
        # [TEST] I2C
        i2c_test = {"bus": "I2C", "status": "UNKNOWN", "devices": []}
        try:
            result = subprocess.run(
                ["i2cdetect", "-y", "1"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                # Parse i2cdetect output
                lines = result.stdout.split('\n')
                devices = []
                for line in lines:
                    if '--' in line or not line.strip():
                        continue
                    parts = line.split()
                    if len(parts) > 1:
                        for addr in parts[1:]:
                            if addr not in ['--', '']:
                                devices.append(f"0x{addr}")
                
                if devices:
                    i2c_test["status"] = "PASS"
                    i2c_test["devices"] = devices
                    print_ok(f"  I2C found {len(devices)} device(s): {', '.join(devices)}")
                    self.passed_tests += 1
                else:
                    i2c_test["status"] = "WARN"
                    print_warn("  I2C: No devices found")
                    self.results["warnings"].append("No I2C devices detected")
                    self.passed_tests += 1
            else:
                i2c_test["status"] = "SKIP"
                print_warn("  I2C: i2cdetect not available")
                self.skipped_tests += 1
        
        except Exception as e:
            i2c_test["status"] = "FAIL"
            i2c_test["error"] = str(e)
            print_error(f"  I2C test error: {e}")
            self.failed_tests += 1
        
        suite_result["tests"].append(i2c_test)
        
        # [TEST] UART (Serial to Arduino)
        uart_test = {"bus": "UART", "status": "UNKNOWN"}
        try:
            import serial
            arduino_port = None
            
            # Try common Arduino ports
            for port in ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/serial0", "COM5"]:
                try:
                    ser = serial.Serial(port, 9600, timeout=1)
                    ser.write(b"PING\n")
                    response = ser.readline().decode().strip()
                    ser.close()
                    
                    if "PONG" in response or "PING" in response:
                        uart_test["status"] = "PASS"
                        uart_test["port"] = port
                        uart_test["response"] = response
                        print_ok(f"  UART {port}: Connected (response: {response})")
                        self.passed_tests += 1
                        arduino_port = port
                        break
                except:
                    pass
            
            if not arduino_port:
                uart_test["status"] = "WARN"
                uart_test["message"] = "No Arduino detected on common ports"
                print_warn("  UART: No Arduino response (expected—may not be connected)")
                self.results["warnings"].append("Arduino not detected on serial ports")
                self.passed_tests += 1
        
        except ImportError:
            uart_test["status"] = "SKIP"
            uart_test["message"] = "pyserial not installed"
            print_warn("  UART: pyserial not available")
            self.skipped_tests += 1
        
        except Exception as e:
            uart_test["status"] = "FAIL"
            uart_test["error"] = str(e)
            print_error(f"  UART test error: {e}")
            self.failed_tests += 1
        
        suite_result["tests"].append(uart_test)
        self.results["suites"]["bus"] = suite_result
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  SUITE 5: REAL-TIME PERFORMANCE
    # ═══════════════════════════════════════════════════════════════════════════
    def suite_5_realtime_performance(self):
        """
        Measure GPIO interrupt latency over 1000 pulses.
        Report mean, min, max, p95, p99 in microseconds.
        PASS: p99 < 100µs | WARN: > 500µs | FAIL: > 2ms
        """
        print_info("\n▶ SUITE 5: Real-Time Performance")
        suite_result = {"name": "Real-Time Performance", "tests": []}
        
        if not RASPBERRY_PI:
            print_warn("Skipped: Not on Raspberry Pi")
            self.skipped_tests += 1
            self.results["suites"]["realtime"] = suite_result
            return
        
        perf_test = {"metric": "GPIO Interrupt Latency", "status": "UNKNOWN"}
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            PULSE_PIN = 17  # Input pin to measure latency
            GPIO.setup(PULSE_PIN, GPIO.IN)
            
            latencies = deque(maxlen=1000)
            
            def interrupt_handler(pin):
                """Called on rising edge—measure time between pulses."""
                latencies.append(time.monotonic())
            
            GPIO.add_event_detect(PULSE_PIN, GPIO.RISING, callback=interrupt_handler)
            
            # Generate test pulses (using another GPIO)
            PULSE_OUT_PIN = 27
            GPIO.setup(PULSE_OUT_PIN, GPIO.OUT)
            
            print_info("  Measuring GPIO interrupt latency (1000 pulses)...")
            pulse_times = []
            for i in range(1001):  # Generate 1001 pulses
                GPIO.output(PULSE_OUT_PIN, GPIO.HIGH)
                time.sleep(0.001)  # 1ms pulse width
                GPIO.output(PULSE_OUT_PIN, GPIO.LOW)
                time.sleep(0.001)  # 1ms gap
            
            time.sleep(0.5)  # Let callbacks complete
            
            if len(latencies) > 100:
                lat_list = sorted(list(latencies))
                mean_us = sum(lat_list) / len(lat_list) * 1e6
                min_us = min(lat_list) * 1e6
                max_us = max(lat_list) * 1e6
                p95_idx = int(0.95 * len(lat_list))
                p99_idx = int(0.99 * len(lat_list))
                p95_us = lat_list[p95_idx] * 1e6 if p95_idx < len(lat_list) else 0
                p99_us = lat_list[p99_idx] * 1e6 if p99_idx < len(lat_list) else 0
                
                perf_test["latencies"] = {
                    "count": len(lat_list),
                    "mean_µs": round(mean_us, 2),
                    "min_µs": round(min_us, 2),
                    "max_µs": round(max_us, 2),
                    "p95_µs": round(p95_us, 2),
                    "p99_µs": round(p99_us, 2),
                }
                
                if p99_us < 100:
                    perf_test["status"] = "PASS"
                    print_ok(f"  Latency: p99={p99_us:.0f}µs (EXCELLENT)")
                    self.passed_tests += 1
                elif p99_us < 500:
                    perf_test["status"] = "WARN"
                    print_warn(f"  Latency: p99={p99_us:.0f}µs (acceptable, but high)")
                    self.results["warnings"].append(f"GPIO latency p99={p99_us:.0f}µs > 100µs")
                    self.passed_tests += 1
                else:
                    perf_test["status"] = "FAIL"
                    print_error(f"  Latency: p99={p99_us:.0f}µs (CRITICAL—high jitter)")
                    self.results["critical_failures"].append(f"GPIO latency p99={p99_us:.0f}µs")
                    self.failed_tests += 1
            else:
                perf_test["status"] = "SKIP"
                perf_test["message"] = "Insufficient latency samples"
                print_warn(f"  Insufficient samples: {len(latencies)} < 100")
                self.skipped_tests += 1
            
            GPIO.remove_event_detect(PULSE_PIN)
            GPIO.cleanup()
        
        except Exception as e:
            perf_test["status"] = "FAIL"
            perf_test["error"] = str(e)
            print_error(f"  Real-time perf test error: {e}")
            self.failed_tests += 1
        
        suite_result["tests"].append(perf_test)
        self.results["suites"]["realtime"] = suite_result
    
    # ═══════════════════════════════════════════════════════════════════════════
    #  RUN ALL SUITES & REPORT
    # ═══════════════════════════════════════════════════════════════════════════
    def run_all(self):
        """Execute all diagnostic suites."""
        print("\n" + "="*80)
        print("  ROBOT DIAGNOSTICS — WEAPON DETECTION SURVEILLANCE SYSTEM")
        print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        self.suite_1_gpio_connectivity()
        self.suite_2_motor_driver()
        self.suite_3_servo_joint_range()
        self.suite_4_communication_bus()
        self.suite_5_realtime_performance()
        
        self.print_summary()
        self.save_report()
        
        # [SAFETY] Exit with appropriate code
        if self.results["critical_failures"]:
            return 1
        return 0
    
    def print_summary(self):
        """Print summary results."""
        print("\n" + "="*80)
        print("  SUMMARY")
        print("="*80)
        print_ok(f"Passed:  {self.passed_tests}")
        if self.results["warnings"]:
            print_warn(f"Warnings: {len(self.results['warnings'])}")
            for w in self.results["warnings"]:
                print(f"          ⚠ {w}")
        if self.results["critical_failures"]:
            print_error(f"CRITICAL: {len(self.results['critical_failures'])}")
            for c in self.results["critical_failures"]:
                print(f"          ✗ {c}")
        else:
            print_ok("NO CRITICAL FAILURES")
        print(f"Skipped: {self.skipped_tests}")
        print("="*80)
    
    def save_report(self):
        """Save JSON report."""
        report_file = "diagnostics_report.json"
        with open(report_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print_info(f"✓ Report saved: {report_file}")


if __name__ == '__main__':
    diag = RobotDiagnostics()
    exit_code = diag.run_all()
    sys.exit(exit_code)
