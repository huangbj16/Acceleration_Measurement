import serial
import time
import json

class serial_control:
    def __init__(self, port):
        self.port = port
        print('start a new serial connection')
        self.host = serial.Serial(self.port, 1000000, timeout=1)
        self.buffer = b''  # Buffer for incomplete data
        time.sleep(1)

    def send(self, data):
        self.host.write(bytes(data, encoding='utf-8'))

    def receive(self):
        # Read available data
        data = self.host.read(100)  # Read more bytes to catch complete messages
        if len(data) > 0:
            # Add to buffer and clean null bytes
            self.buffer += data.replace(b'\x00', b'')

        # Try to extract complete JSON objects
        if b'}' not in self.buffer:
            return None

        # Find complete JSON objects
        complete_messages = []
        while b'}' in self.buffer:
            # Find the first }
            end_idx = self.buffer.find(b'}') + 1
            # Look backwards for the matching {
            start_idx = self.buffer.rfind(b'{', 0, end_idx)

            if start_idx >= 0:
                # Extract the complete JSON object
                complete_msg = self.buffer[start_idx:end_idx]
                try:
                    # Verify it's valid JSON
                    test_str = complete_msg.decode('utf-8', errors='ignore').strip()
                    if test_str.startswith('{') and test_str.endswith('}'):
                        # Try to parse it to ensure it's valid JSON
                        json.loads(test_str)  # This will raise if invalid
                        complete_messages.append(complete_msg)
                        # Remove processed data from buffer
                        self.buffer = self.buffer[end_idx:]
                    else:
                        # Not a complete message, keep in buffer
                        break
                except (json.JSONDecodeError, UnicodeDecodeError, ValueError):
                    # Invalid JSON, remove up to the } and try again
                    self.buffer = self.buffer[end_idx:]
            else:
                # No { found before }, remove the } and continue
                self.buffer = self.buffer[end_idx:]

        # Return the first complete message if available
        if complete_messages:
            return complete_messages[0]

        return None

    def finish(self):
        print('serial connection ended')
        self.host.close()

