import os
import threading
import time
from datetime import datetime

class LoggingHandler:
    def __init__(self, serial_handler):
        self.serial_handler = serial_handler
        self.running = False
        self.log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        self.ensure_log_directory()
        self.thread = None
        self.log_file = None
        self.log_lock = threading.Lock()

    def ensure_log_directory(self):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            print(f"Log directory ensured: {self.log_dir}")
        except Exception as e:
            print(f"Error creating log directory: {e}")
            raise

    def logging_thread(self):
        print("Logging thread started")
        while self.running:
            try:
                current_time = datetime.now()
                log_filename = os.path.join(self.log_dir, f'logs_{current_time.strftime("%Y-%m-%d")}.txt')
                
                with self.log_lock:
                    if self.log_file is None or self.log_file.name != log_filename:
                        if self.log_file:
                            self.log_file.close()
                        self.log_file = open(log_filename, 'a')
                        print(f"Opened new log file: {log_filename}")

                    if self.serial_handler:
                        try:
                            status = self.serial_handler.get_status()
                            log_entry = (
                                f"{current_time.strftime('%Y-%m-%d %H:%M:%S')}, "
                                f"ACTUAL VELOCITY = {status['speed']:.2f}, "
                                f"DESIRED VELOCITY = {status['desired_velocity']:.2f}, "
                                f"ACTUAL BRAKE = {status['actual_brake']:.2f}, "
                                f"DESIRED BRAKE = {status['desired_brake']:.2f}, "
                                f"OBSTACLE DISTANCE = {status['obstacle_distance']:.2f}, "
                                f"BRAKE STATE = {status['brake_state']}\n"
                            )
                        except Exception as e:
                            print(f"Error getting status from serial handler: {e}")
                            log_entry = (
                                f"{current_time.strftime('%Y-%m-%d %H:%M:%S')}, "
                                f"ACTUAL VELOCITY = NaN, "
                                f"DESIRED VELOCITY = NaN, "
                                f"ACTUAL BRAKE = NaN, "
                                f"DESIRED BRAKE = NaN, "
                                f"OBSTACLE DISTANCE = NaN, "
                                f"BRAKE STATE = N/A\n"
                            )
                    else:
                        log_entry = (
                            f"{current_time.strftime('%Y-%m-%d %H:%M:%S')}, "
                            f"ACTUAL VELOCITY = NaN, "
                            f"DESIRED VELOCITY = NaN, "
                            f"ACTUAL BRAKE = NaN, "
                            f"DESIRED BRAKE = NaN, "
                            f"OBSTACLE DISTANCE = NaN, "
                            f"BRAKE STATE = N/A\n"
                        )
                    
                    self.log_file.write(log_entry)
                    self.log_file.flush()  # Force write to disk
            except Exception as e:
                print(f"Error in logging thread: {e}")
            
            time.sleep(0.1)  # 100ms logging interval
        print("Logging thread stopped")

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.logging_thread)
            self.thread.start()
            print("Logging thread started")

    def stop(self):
        if self.running:
            print("Stopping logging thread...")
            self.running = False
            if self.thread:
                self.thread.join()
            with self.log_lock:
                if self.log_file:
                    self.log_file.close()
                    self.log_file = None
            print("Logging thread stopped")

    def log_immediate(self, message):
        with self.log_lock:
            if self.log_file:
                try:
                    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    self.log_file.write(f"{current_time} - {message}\n")
                    self.log_file.flush()
                except Exception as e:
                    print(f"Error writing immediate log: {e}")