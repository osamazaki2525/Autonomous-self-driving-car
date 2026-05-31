import socket
import keyboard

# Replace with the ESP32-CAM's IP address
ESP32_IP = "192.168.1.101"
PORT = 12345

# Create a TCP client
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ESP32_IP, PORT))
    print(f"Connected to {ESP32_IP}:{PORT}")

    # Listen for keypress
    print("Press any key to send (or 'q' to quit)...")
    while True:
        try:
            # Wait for a keypress event
            event = keyboard.read_event()
            if event.event_type == keyboard.KEY_DOWN:  # Only send on keydown
                key = event.name  # Get the key that was pressed
                if key == 'q':  # Exit on 'q' key
                    print("Exiting...")
                    break
                # Send the key to the ESP32-CAM
                s.sendall(key.encode())
                print(f"Sent '{key}' to ESP32-CAM")
        except Exception as e:
            print(f"Error: {e}")
            break