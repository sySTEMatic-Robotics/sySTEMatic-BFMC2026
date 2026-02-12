import evdev
from evdev import InputDevice, categorize, ecodes

# 1. List devices to find the joystick, or open directly
# Find your device with: evtest
# Example: /dev/input/event0
device_path = '/dev/input/event5' 
gamepad = InputDevice(device_path)
print(f"Connected to: {gamepad.name}")

# 2. Loop to read events
for event in gamepad.read_loop():
    # 3. Filter for Axis (Absolute) or Button (Key) events
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        # 4. Identify axis (e.g., ABS_X is usually Left Stick X)
        if absevent.event.code == ecodes.ABS_X:
            print(f"Left Stick X: {absevent.event.value}")
            
    elif event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        # 5. Identify button (e.g., BTN_A, BTN_B)
        if keyevent.keystate == keyevent.key_down:
            print(f"Button Pressed: {keyevent.keycode}")
        elif keyevent.keystate == keyevent.key_up:
            print(f"Button Released: {keyevent.keycode}")
