# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# import sys, signal
# import os
# import evdev
# from evdev import InputDevice, categorize, ecodes
# from sensor_msgs.msg import Joy
# import asyncio
# import threading

# class Keyboard(Node):
#     def __init__(self):
#         super().__init__("keyboard_node")
#         self.speed_multiplier = 1
#         self.joystick_message_pub = self.create_publisher(Joy, "joy", 10)
#         self.get_logger().info("Keyboard Listener is starting...")
        
#         # Find the keyboard device
#         self.dev = self.find_keyboard_device()
#         if not self.dev:
#             self.get_logger().error("No keyboard device found!")
#             self.list_available_devices()
#             sys.exit(1)
        
#         self.get_logger().info(f"Using keyboard device: {self.dev.name}")
        
#         self.current_joy_message = Joy()
#         self.current_joy_message.axes = [0.0] * 8
#         self.current_joy_message.buttons = [0] * 11
        
#         # Timer to regularly call publish_current_command
#         self.timer = self.create_timer(1.0 / 30.0, self.publish_current_command)  # 30 Hz

#         # Start the event loop in a separate thread
#         self.event_loop = asyncio.new_event_loop()
#         self.event_thread = threading.Thread(target=self.run_event_loop, daemon=True)
#         self.event_thread.start()

#     def find_keyboard_device(self):
#         devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
#         for device in devices:
#             self.get_logger().info(f"Found device: {device.name}, {device.path}")
#             if evdev.ecodes.EV_KEY in device.capabilities():
#                 self.get_logger().info(f"Using device: {device.name}")
#                 return device
#         return None

#     def list_available_devices(self):
#         self.get_logger().info("Available input devices:")
#         devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
#         for device in devices:
#             self.get_logger().info(f"  {device.path}: {device.name}")
#             self.get_logger().info(f"    Capabilities: {device.capabilities(verbose=True)}")

#     async def read_keyboard_events(self):
#         try:
#             async for event in self.dev.async_read_loop():
#                 if event.type == ecodes.EV_KEY:
#                     key_event = categorize(event)
#                     self.process_key_event(key_event)
#         except OSError as e:
#             self.get_logger().error(f"Error reading from device: {e}")
#             self.list_available_devices()

#     def process_key_event(self, key_event):
#         key = key_event.keycode
#         msg = self.current_joy_message

#         if key_event.keystate == key_event.key_down:
#             if key == 'KEY_LEFTSHIFT':
#                 self.speed_multiplier = 2
#             elif key == 'KEY_W':
#                 msg.axes[1] = 0.5 * self.speed_multiplier
#                 self.get_logger().info("KEY_W is pressed")
#             elif key == 'KEY_S':
#                 msg.axes[1] = -0.5 * self.speed_multiplier
#                 self.get_logger().info("KEY_S is pressed")
#             elif key == 'KEY_A':
#                 msg.axes[0] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_D':
#                 msg.axes[0] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_1':
#                 msg.buttons[5] = 1
#             elif key == 'KEY_2':
#                 msg.buttons[0] = 1
#             elif key == 'KEY_BACKSPACE':
#                 msg.buttons[4] = 1
#             elif key == 'KEY_UP':
#                 self.get_logger().info(key)
#                 msg.axes[4] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_DOWN':
#                 self.get_logger().info(key)
#                 msg.axes[4] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_LEFT':
#                 self.get_logger().info(key)
#                 msg.axes[3] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_RIGHT':
#                 self.get_logger().info(key)
#                 msg.axes[3] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_0':
#                 msg.axes[7] = 1
#             elif key == 'KEY_9':
#                 msg.axes[7] = -1
#             elif key == 'KEY_8':
#                 msg.axes[6] = 1
#             elif key == 'KEY_7':
#                 msg.axes[6] = -1
#         elif key_event.keystate == key_event.key_up:
#             if key == 'KEY_LEFTSHIFT':
#                 self.speed_multiplier = 1
#             elif key in ['KEY_W', 'KEY_S']:
#                 msg.axes[1] = 0.0
#             elif key in ['KEY_A', 'KEY_D']:
#                 msg.axes[0] = 0.0
#             elif key == 'KEY_1':
#                 msg.buttons[5] = 0
#             elif key == 'KEY_2':
#                 msg.buttons[0] = 0
#             elif key == 'KEY_BACKSPACE':
#                 msg.buttons[4] = 0
#             elif key in ['KEY_UP', 'KEY_DOWN']:
#                 msg.axes[4] = 0.0
#             elif key in ['KEY_LEFT', 'KEY_RIGHT']:
#                 msg.axes[3] = 0.0
#             elif key in ['KEY_0', 'KEY_9']:
#                 msg.axes[7] = 0
#             elif key in ['KEY_8', 'KEY_7']:
#                 msg.axes[6] = 0

#         self.current_joy_message = msg

#     def publish_current_command(self):
#         self.current_joy_message.header.stamp = self.get_clock().now().to_msg()
#         self.joystick_message_pub.publish(self.current_joy_message)
#         self.get_logger().info("Publishing command")

#     def run_event_loop(self):
#         asyncio.set_event_loop(self.event_loop)
#         self.event_loop.run_until_complete(self.read_keyboard_events())

# def main(args=None):
#     rclpy.init(args=args)
#     keyboard_node = Keyboard()

#     try:
#         rclpy.spin(keyboard_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         keyboard_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# Not shutting down:
# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# import sys, signal
# import os
# import evdev
# from evdev import InputDevice, categorize, ecodes
# from sensor_msgs.msg import Joy
# import asyncio
# import threading

# class Keyboard(Node):
#     def __init__(self):
#         super().__init__("keyboard_node")
#         self.speed_multiplier = 1
#         self.joystick_message_pub = self.create_publisher(Joy, "joy", 10)
#         self.get_logger().info("Keyboard Listener is starting...")
        
#         # Flag to indicate if the node should continue running
#         self.running = True
        
#         # Find the keyboard device
#         self.dev = self.find_keyboard_device()
#         if not self.dev:
#             self.get_logger().error("No keyboard device found!")
#             self.list_available_devices()
#             sys.exit(1)
        
#         self.get_logger().info(f"Using keyboard device: {self.dev.name}")
        
#         self.current_joy_message = Joy()
#         self.current_joy_message.axes = [0.0] * 8
#         self.current_joy_message.buttons = [0] * 11
        
#         # Timer to regularly call publish_current_command
#         self.timer = self.create_timer(1.0 / 30.0, self.publish_current_command)  # 30 Hz

#         # Start the event loop in a separate thread
#         self.event_loop = asyncio.new_event_loop()
#         self.event_thread = threading.Thread(target=self.run_event_loop, daemon=True)
#         self.event_thread.start()

#     def find_keyboard_device(self):
#         devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
#         for device in devices:
#             self.get_logger().info(f"Found device: {device.name}, {device.path}")
#             if evdev.ecodes.EV_KEY in device.capabilities():
#                 self.get_logger().info(f"Using device: {device.name}")
#                 return device
#         return None

#     def list_available_devices(self):
#         self.get_logger().info("Available input devices:")
#         devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
#         for device in devices:
#             self.get_logger().info(f"  {device.path}: {device.name}")
#             self.get_logger().info(f"    Capabilities: {device.capabilities(verbose=True)}")

#     async def read_keyboard_events(self):
#         try:
#             while self.running:
#                 events = await self.event_loop.run_in_executor(None, self.dev.read_one)
#                 if events:
#                     event = evdev.categorize(events)
#                     if event.event.type == ecodes.EV_KEY:
#                         self.process_key_event(event)
#         except OSError as e:
#             self.get_logger().error(f"Error reading from device: {e}")
#             self.list_available_devices()

#     def process_key_event(self, key_event):
#         key = key_event.keycode
#         msg = self.current_joy_message

#         if key_event.keystate == key_event.key_down:
#             if key == 'KEY_LEFTSHIFT':
#                 self.speed_multiplier = 2
#             elif key == 'KEY_W':
#                 msg.axes[1] = 0.5 * self.speed_multiplier
#                 self.get_logger().info("KEY_W is pressed")
#             elif key == 'KEY_S':
#                 msg.axes[1] = -0.5 * self.speed_multiplier
#                 self.get_logger().info("KEY_S is pressed")
#             elif key == 'KEY_A':
#                 msg.axes[0] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_D':
#                 msg.axes[0] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_1':
#                 msg.buttons[5] = 1
#             elif key == 'KEY_2':
#                 msg.buttons[0] = 1
#             elif key == 'KEY_BACKSPACE':
#                 msg.buttons[4] = 1
#             elif key == 'KEY_UP':
#                 self.get_logger().info(key)
#                 msg.axes[4] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_DOWN':
#                 self.get_logger().info(key)
#                 msg.axes[4] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_LEFT':
#                 self.get_logger().info(key)
#                 msg.axes[3] = 0.5 * self.speed_multiplier
#             elif key == 'KEY_RIGHT':
#                 self.get_logger().info(key)
#                 msg.axes[3] = -0.5 * self.speed_multiplier
#             elif key == 'KEY_0':
#                 msg.axes[7] = 1
#             elif key == 'KEY_9':
#                 msg.axes[7] = -1
#             elif key == 'KEY_8':
#                 msg.axes[6] = 1
#             elif key == 'KEY_7':
#                 msg.axes[6] = -1
#         elif key_event.keystate == key_event.key_up:
#             if key == 'KEY_LEFTSHIFT':
#                 self.speed_multiplier = 1
#             elif key in ['KEY_W', 'KEY_S']:
#                 msg.axes[1] = 0.0
#             elif key in ['KEY_A', 'KEY_D']:
#                 msg.axes[0] = 0.0
#             elif key == 'KEY_1':
#                 msg.buttons[5] = 0
#             elif key == 'KEY_2':
#                 msg.buttons[0] = 0
#             elif key == 'KEY_BACKSPACE':
#                 msg.buttons[4] = 0
#             elif key in ['KEY_UP', 'KEY_DOWN']:
#                 msg.axes[4] = 0.0
#             elif key in ['KEY_LEFT', 'KEY_RIGHT']:
#                 msg.axes[3] = 0.0
#             elif key in ['KEY_0', 'KEY_9']:
#                 msg.axes[7] = 0
#             elif key in ['KEY_8', 'KEY_7']:
#                 msg.axes[6] = 0

#         self.current_joy_message = msg

#     def publish_current_command(self):
#         self.current_joy_message.header.stamp = self.get_clock().now().to_msg()
#         self.joystick_message_pub.publish(self.current_joy_message)
#         self.get_logger().info("Publishing command")

#     def run_event_loop(self):
#         asyncio.set_event_loop(self.event_loop)
#         self.event_loop.run_until_complete(self.read_keyboard_events())

#     def shutdown(self):
#         self.running = False
#         self.event_loop.stop()
#         self.event_thread.join()
#         self.get_logger().info("Keyboard node has been shut down.")

# def main(args=None):
#     rclpy.init(args=args)
#     keyboard_node = Keyboard()

#     def signal_handler(sig, frame):
#         keyboard_node.get_logger().info("Interrupt received, shutting down...")
#         keyboard_node.shutdown()
#         rclpy.shutdown()
#         sys.exit(0)

#     signal.signal(signal.SIGINT, signal_handler)

#     try:
#         rclpy.spin(keyboard_node)
#     except KeyboardInterrupt:
#         keyboard_node.shutdown()
#         keyboard_node.destroy_node()
#         rclpy.shutdown()

#     finally:
#         keyboard_node.shutdown()
#         keyboard_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import sys, signal
import os
import evdev
from evdev import InputDevice, categorize, ecodes
from sensor_msgs.msg import Joy
import asyncio
import threading

class Keyboard(Node):
    def __init__(self):
        super().__init__("keyboard_node")
        self.speed_multiplier = 1
        self.joystick_message_pub = self.create_publisher(Joy, "joy", 10)
        self.get_logger().info("Keyboard Listener is starting...")
        
        # Flag to indicate if the node should continue running
        self.running = True
        
        # Find the keyboard device
        self.dev = self.find_keyboard_device()
        if not self.dev:
            self.get_logger().error("No keyboard device found!")
            self.list_available_devices()
            sys.exit(1)
        
        self.get_logger().info(f"Using keyboard device: {self.dev.name}")
        
        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0.0] * 8
        self.current_joy_message.buttons = [0] * 11
        
        # Timer to regularly call publish_current_command
        self.timer = self.create_timer(1.0 / 30.0, self.publish_current_command)  # 30 Hz

        # Start the event loop in a separate thread
        self.event_loop = asyncio.new_event_loop()
        self.event_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.event_thread.start()

    def find_keyboard_device(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(f"Found device: {device.name}, {device.path}")
            print("OK? {}".format(evdev.ecodes.EV_KEY in device.capabilities()))
            print()

        ok=False
        for device in devices:
            self.get_logger().info(f"Found device: {device.name}, {device.path}")
            if (evdev.ecodes.EV_KEY in device.capabilities()) and ('event0' in device.path):
                # if not ok:
                #     ok = not ok
                    # continue
                self.get_logger().info(f"Using device: {device.name} + {device.path}")
                return device
        return None

    def list_available_devices(self):
        self.get_logger().info("Available input devices:")
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            self.get_logger().info(f"  {device.path}: {device.name}")
            self.get_logger().info(f"    Capabilities: {device.capabilities(verbose=True)}")

    async def read_keyboard_events(self):
        try:
            while self.running:
                events = await self.event_loop.run_in_executor(None, self.dev.read_one)
                if events:
                    event = evdev.categorize(events)
                    if event.event.type == ecodes.EV_KEY:
                        self.process_key_event(event)
        except OSError as e:
            self.get_logger().error(f"Error reading from device: {e}")
            self.list_available_devices()

    def process_key_event(self, key_event):
        key = key_event.keycode
        msg = self.current_joy_message

        if key_event.keystate == key_event.key_down:
            if key == 'KEY_LEFTSHIFT':
                self.speed_multiplier = 2
            elif key == 'KEY_W':
                msg.axes[1] = 0.5 * self.speed_multiplier
                self.get_logger().info("KEY_W is pressed")
            elif key == 'KEY_S':
                msg.axes[1] = -0.5 * self.speed_multiplier
                self.get_logger().info("KEY_S is pressed")
            elif key == 'KEY_A':
                msg.axes[0] = 0.5 * self.speed_multiplier
            elif key == 'KEY_D':
                msg.axes[0] = -0.5 * self.speed_multiplier
            elif key == 'KEY_1':
                msg.buttons[5] = 1
            elif key == 'KEY_2':
                msg.buttons[0] = 1
            elif key == 'KEY_BACKSPACE':
                msg.buttons[4] = 1
            elif key == 'KEY_UP':
                self.get_logger().info(key)
                msg.axes[4] = 0.5 * self.speed_multiplier
            elif key == 'KEY_DOWN':
                self.get_logger().info(key)
                msg.axes[4] = -0.5 * self.speed_multiplier
            elif key == 'KEY_LEFT':
                self.get_logger().info(key)
                msg.axes[3] = 0.5 * self.speed_multiplier
            elif key == 'KEY_RIGHT':
                self.get_logger().info(key)
                msg.axes[3] = -0.5 * self.speed_multiplier
            elif key == 'KEY_0':
                msg.axes[7] = 1
            elif key == 'KEY_9':
                msg.axes[7] = -1
            elif key == 'KEY_8':
                msg.axes[6] = 1
            elif key == 'KEY_7':
                msg.axes[6] = -1
        elif key_event.keystate == key_event.key_up:
            if key == 'KEY_LEFTSHIFT':
                self.speed_multiplier = 1
            elif key in ['KEY_W', 'KEY_S']:
                msg.axes[1] = 0.0
            elif key in ['KEY_A', 'KEY_D']:
                msg.axes[0] = 0.0
            elif key == 'KEY_1':
                msg.buttons[5] = 0
            elif key == 'KEY_2':
                msg.buttons[0] = 0
            elif key == 'KEY_BACKSPACE':
                msg.buttons[4] = 0
            elif key in ['KEY_UP', 'KEY_DOWN']:
                msg.axes[4] = 0.0
            elif key in ['KEY_LEFT', 'KEY_RIGHT']:
                msg.axes[3] = 0.0
            elif key in ['KEY_0', 'KEY_9']:
                msg.axes[7] = 0
            elif key in ['KEY_8', 'KEY_7']:
                msg.axes[6] = 0

        self.current_joy_message = msg

    def publish_current_command(self):
        self.current_joy_message.header.stamp = self.get_clock().now().to_msg()
        self.joystick_message_pub.publish(self.current_joy_message)
        self.get_logger().info("Publishing command")

    def run_event_loop(self):
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.run_until_complete(self.read_keyboard_events())

    def shutdown(self):
        self.running = False
        self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        self.event_thread.join()
        self.get_logger().info("Keyboard node has been shut down.")

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = Keyboard()

    def signal_handler(sig, frame):
        keyboard_node.get_logger().info("Interrupt received, shutting down...")
        keyboard_node.shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(keyboard_node)
    except KeyboardInterrupt:
        keyboard_node.shutdown()
        keyboard_node.destroy_node()
        rclpy.shutdown()

    finally:
        keyboard_node.shutdown()
        keyboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
