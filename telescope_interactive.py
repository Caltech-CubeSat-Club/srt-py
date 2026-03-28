#!/usr/bin/env python3
"""
Interactive Telescope Control Interface
Arrow key control for azimuth/elevation with real-time display
"""

import curses
import threading
import time
import logging
from telescope_control import Caltech6m
import sys

class TelescopeInteractiveController:
    def __init__(self, port='COM1', baudrate=115200):
        # Disable logging to reduce overhead
        logging.getLogger().setLevel(logging.CRITICAL)
        
        self.telescope = None
        self.port = port
        self.baudrate = baudrate
        
        # Control variables
        self.commanded_az = 180.0
        self.commanded_el = 45.0
        self.step_size = 1.0  # degrees per arrow key press
        self.auto_update = False
        self.running = True
        
        # Display refresh rate
        self.refresh_rate = 0.03  # 30+ Hz for ultra-smooth commanded position updates
        
        # Initialize telescope status
        self.current_az = 0.0
        self.current_el = 0.0
        self.az_error = 0.0
        self.el_error = 0.0
        self.telescope_mode = "Unknown"
        self.calibration_status = "Unknown"
        
        # Game feature variables - predefined coordinate pairs with messages
        self.predefined_targets = [
            (90.0, 45.0, "[Error 101] Satellite did not return a response. Unknown error. FATAL. Core dump."),
            (352.0, 22.0, "[Error 101] Site-19: EMERGENCY AUTONOMOUS BROADCAST. REBROADCAST 999999"),
            (109.0, 60.0, "[Error 101] Satellite did not return a response. Unknown error. FATAL. Core dump.")
        ]
        self.position_tolerance = 0.5  # degrees - how close we need to be to the target
        self.current_message = ""
        self.message_display_time = 0
        self.message_duration = 60.0  # seconds to display message
        self.triggered_targets = set()  # Track which targets we've already triggered
        
    def connect_telescope(self):
        """Connect to telescope"""
        try:
            self.telescope = Caltech6m(port=self.port, baudrate=self.baudrate, verbose=False)
            # Get initial telescope position and set as commanded position
            self.telescope.get_info()
            self.commanded_az = getattr(self.telescope, 'az', 180.0)
            self.commanded_el = getattr(self.telescope, 'el', 45.0)
            return True
        except Exception as e:
            print(f"Failed to connect to telescope: {e}")
            return False
    
    def update_telescope_status(self):
        """Update telescope status in background thread"""
        while self.running:
            if self.telescope:
                try:
                    self.telescope.get_info()
                    self.current_az = getattr(self.telescope, 'az', 0.0)
                    self.current_el = getattr(self.telescope, 'el', 0.0)
                    self.az_error = getattr(self.telescope, 'azerr', 0.0)
                    self.el_error = getattr(self.telescope, 'elerr', 0.0)
                    self.telescope_mode = getattr(self.telescope, 'mode', 'Unknown')
                    self.calibration_status = getattr(self.telescope, 'CalSts', 'Unknown')
                    
                    # Update game status
                    self.update_game_status()
                    
                except Exception as e:
                    pass  # Ignore errors to prevent crashes
            time.sleep(1.0)  # Update every 3 seconds to reduce load further
    
    def point_telescope(self):
        """Command telescope to point to commanded coordinates"""
        if self.telescope and self.telescope.calibrated:
            try:
                self.telescope.point(self.commanded_az, self.commanded_el)
            except Exception as e:
                pass  # Ignore errors to prevent crashes
        else:
            pass  # Silently ignore if not calibrated
    
    def handle_key(self, key):
        """Handle keyboard input"""
        if key == curses.KEY_UP:
            self.commanded_el = min(self.commanded_el + self.step_size, 81.0)
            if self.auto_update:
                self.point_telescope()
        elif key == curses.KEY_DOWN:
            self.commanded_el = max(self.commanded_el - self.step_size, 15.0)
            if self.auto_update:
                self.point_telescope()
        elif key == curses.KEY_LEFT:
            self.commanded_az = max(self.commanded_az - self.step_size, -89.0)
            if self.auto_update:
                self.point_telescope()
        elif key == curses.KEY_RIGHT:
            self.commanded_az = min(self.commanded_az + self.step_size, 449.0)
            if self.auto_update:
                self.point_telescope()
        elif key == ord('g') or key == ord('G'):
            # Go - command telescope to point
            self.point_telescope()
        elif key == ord('s') or key == ord('S'):
            # Stop telescope
            if self.telescope:
                self.telescope.stop()
        elif key == ord('a') or key == ord('A'):
            # Toggle auto-update mode
            self.auto_update = not self.auto_update
        elif key == ord('1'):
            self.step_size = 0.1
        elif key == ord('2'):
            self.step_size = 1.0
        elif key == ord('3'):
            self.step_size = 5.0
        elif key == ord('4'):
            self.step_size = 10.0
        elif key == ord('b') or key == ord('B'):
            # Toggle brakes
            if self.telescope:
                if getattr(self.telescope, 'AzBrkOn', False) or getattr(self.telescope, 'ElBrkOn', False):
                    self.telescope.brakes_off()
                else:
                    self.telescope.brakes_on()
        elif key == ord('c') or key == ord('C'):
            # Calibrate telescope
            if self.telescope:
                threading.Thread(target=self.telescope.calibrate, daemon=True).start()
        elif key == ord('q') or key == ord('Q') or key == 27:  # ESC
            self.running = False
    
    def draw_interface(self, stdscr):
        """Draw the curses interface"""
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(1)   # Non-blocking input
        stdscr.timeout(100) # 100ms timeout for getch()
        
        height, width = stdscr.getmaxyx()
        
        # Ensure minimum terminal size
        if height < 20 or width < 60:
            stdscr.clear()
            stdscr.addstr(0, 0, "Terminal too small! Minimum: 60x20")
            stdscr.addstr(1, 0, f"Current: {width}x{height}")
            stdscr.addstr(2, 0, "Press Q to quit")
            stdscr.refresh()
            return
        
        # Create windows - full width for main, no logging display
        main_height = height - 2
        main_width = width
        
        main_win = curses.newwin(main_height, main_width, 0, 0)
        status_win = curses.newwin(1, width, height - 2, 0)
        
        # Start background thread for telescope updates
        status_thread = threading.Thread(target=self.update_telescope_status, daemon=True)
        status_thread.start()
        
        while self.running:
            # Clear windows
            main_win.clear()
            status_win.clear()
            
            # Flush input buffer to prevent lag from held keys
            curses.flushinp()
            
            try:
                # Draw main control panel
                main_win.box()
                
                # Safe addstr function that checks bounds
                def safe_addstr(win, y, x, text, attr=curses.A_NORMAL):
                    try:
                        win_height, win_width = win.getmaxyx()
                        if y < win_height - 1 and x < win_width - 1:
                            # Truncate text if too long
                            max_text_len = win_width - x - 1
                            if len(text) > max_text_len:
                                text = text[:max_text_len-3] + "..."
                            win.addstr(y, x, text, attr)
                    except curses.error:
                        pass  # Ignore if we can't write
                
                safe_addstr(main_win, 1, 2, "TELESCOPE INTERACTIVE CONTROL", curses.A_BOLD)
                safe_addstr(main_win, 2, 2, "=" * min(main_width - 4, 30))
                
                # Connection status
                if self.telescope:
                    conn_status = "CONNECTED"
                    safe_addstr(main_win, 4, 2, f"Status: {conn_status}", curses.A_NORMAL)
                    safe_addstr(main_win, 5, 2, f"Calibrated: {self.calibration_status}")
                    safe_addstr(main_win, 6, 2, f"Mode: {self.telescope_mode}")
                else:
                    safe_addstr(main_win, 4, 2, "Status: DISCONNECTED", curses.A_REVERSE)
                
                # Commanded position
                safe_addstr(main_win, 8, 2, "COMMANDED POSITION:", curses.A_BOLD)
                safe_addstr(main_win, 9, 2, f"Azimuth:   {self.commanded_az:7.1f}°")
                safe_addstr(main_win, 10, 2, f"Elevation: {self.commanded_el:7.1f}°")
                
                # Current position
                safe_addstr(main_win, 12, 2, "CURRENT POSITION:", curses.A_BOLD)
                safe_addstr(main_win, 13, 2, f"Azimuth:   {self.current_az:7.1f}°")
                safe_addstr(main_win, 14, 2, f"Elevation: {self.current_el:7.1f}°")
                
                # Pointing errors
                safe_addstr(main_win, 16, 2, "POINTING ERRORS:", curses.A_BOLD)
                safe_addstr(main_win, 17, 2, f"Az Error:  {self.az_error/1000.0:8.4f}°")
                safe_addstr(main_win, 18, 2, f"El Error:  {self.el_error/1000.0:8.4f}°")
                
                # Control instructions
                controls_start = max(main_height - 12, 20)
                safe_addstr(main_win, controls_start, 2, "CONTROLS:", curses.A_BOLD)
                safe_addstr(main_win, controls_start + 1, 2, "Arrow Keys: Change Az/El")
                safe_addstr(main_win, controls_start + 2, 2, "G: Go (Point telescope)")
                safe_addstr(main_win, controls_start + 3, 2, "S: Stop telescope")
                safe_addstr(main_win, controls_start + 4, 2, "A: Toggle auto-update")
                safe_addstr(main_win, controls_start + 5, 2, "B: Toggle brakes")
                safe_addstr(main_win, controls_start + 6, 2, "C: Calibrate")
                safe_addstr(main_win, controls_start + 7, 2, "1-4: Step size (0.1-10°)")
                safe_addstr(main_win, controls_start + 8, 2, "Q/ESC: Quit")
                
                # Display current message if any
                if self.current_message:
                    # Find a good spot for the message (right side of screen)
                    msg_col = min(width//2, width - len(self.current_message) - 2)
                    if msg_col > 0:
                        safe_addstr(main_win, 8, msg_col, self.current_message, curses.A_BOLD | curses.A_REVERSE)
                
                # Status bar
                status_text = f"Step: {self.step_size}° | Auto: {'ON' if self.auto_update else 'OFF'} | Press Q to quit"
                safe_addstr(status_win, 0, 0, status_text[:width-1], curses.A_REVERSE)
                
            except curses.error:
                # If any drawing fails, just continue
                pass
            
            # Refresh all windows
            main_win.refresh()
            status_win.refresh()
            
            # Handle input
            try:
                key = stdscr.getch()
                if key != -1:
                    self.handle_key(key)
            except curses.error:
                pass
            
            time.sleep(self.refresh_rate)
    
    def run(self):
        """Main run method"""
        print("Connecting to telescope...")
        if not self.connect_telescope():
            print("Failed to connect to telescope. Exiting.")
            return
        
        print("Starting interactive interface...")
        print("Use Ctrl+C to force quit if needed.")
        
        try:
            curses.wrapper(self.draw_interface)
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            self.running = False
            if self.telescope:
                try:
                    self.telescope.cleanup()
                except:
                    pass

    def update_game_status(self):
        """Check if telescope is near any predefined targets and display messages"""
        current_time = time.time()
        
        # Clear expired message
        if self.current_message and (current_time - self.message_display_time) > self.message_duration:
            self.current_message = ""
        
        # Check each predefined target
        for i, (target_az, target_el, message) in enumerate(self.predefined_targets):
            # Skip if already triggered recently
            if i in self.triggered_targets:
                continue
                
            # Calculate distance to target
            az_diff = abs(self.current_az - target_az)
            el_diff = abs(self.current_el - target_el)
            
            # Handle azimuth wrap-around
            if az_diff > 180:
                az_diff = 360 - az_diff
            
            # Check if within tolerance
            if az_diff <= self.position_tolerance and el_diff <= self.position_tolerance:
                self.current_message = message
                self.message_display_time = current_time
                self.triggered_targets.add(i)
                break
        
        # Reset triggered targets if telescope has moved away from all targets
        all_far = True
        for i, (target_az, target_el, _) in enumerate(self.predefined_targets):
            az_diff = abs(self.current_az - target_az)
            el_diff = abs(self.current_el - target_el)
            if az_diff > 180:
                az_diff = 360 - az_diff
            if az_diff <= self.position_tolerance * 2 and el_diff <= self.position_tolerance * 2:
                all_far = False
                break
        
        if all_far:
            self.triggered_targets.clear()
    
    def reset_game(self):
        """Reset game state"""
        self.game_active = False
        self.target_reached = False
        self.message_display_time = 0
        self.target_az = 0.0
        self.target_el = 0.0
    
def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Interactive Telescope Control Interface')
    parser.add_argument('--port', default='COM1', help='Serial port (default: COM1)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    
    args = parser.parse_args()
    
    controller = TelescopeInteractiveController(port=args.port, baudrate=args.baudrate)
    controller.run()

if __name__ == '__main__':
    main()
