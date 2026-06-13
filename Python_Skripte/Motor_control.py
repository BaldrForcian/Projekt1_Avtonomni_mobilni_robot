import serial
import sys
import termios
import tty
import time
import json
import threading

PORT = "/dev/ttySC0"
BAUD = 115200
MOTOR_ID = 1
SPEED = 50


# ---------------- SERIAL ----------------
ser = serial.Serial(PORT, BAUD, timeout=0.2)


# ------------- SEND COMMAND -------------
def send_cmd(speed):
    cmd = {
        "T": 10010,
        "id": MOTOR_ID,
        "cmd": speed,
        "act": 3
    }
    ser.write((json.dumps(cmd) + "\n").encode())


# -------- REQUEST STATUS (RPM) ---------
def request_status():
    """
    Ask motor for feedback.
    DDSM115 uses T: 10011 for status query.
    """
    cmd = {
        "T": 10011,
        "id": MOTOR_ID
    }
    ser.write((json.dumps(cmd) + "\n").encode())


# -------- READ RESPONSE LOOP ----------
def read_loop():
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                try:
                    data = json.loads(line)

                    # Common field is "speed" or "rpm"
                    if "speed" in data:
                        print(f"[FEEDBACK] RPM: {data['speed']}")
                    elif "rpm" in data:
                        print(f"[FEEDBACK] RPM: {data['rpm']}")
                    else:
                        print(f"[RAW] {data}")

                except json.JSONDecodeError:
                    print(f"[RAW TEXT] {line}")

        except Exception as e:
            print("Read error:", e)


# -------- KEYBOARD INPUT ----------
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ------------- MAIN ----------------
def main():
    print("DDSM115 Control")
    print("W=forward S=back SPACE=stop Q=quit")

    threading.Thread(target=read_loop, daemon=True).start()

    last_request = time.time()

    try:
        while True:
            key = get_key()

            if key.lower() == "w":
                send_cmd(SPEED)

            elif key.lower() == "s":
                send_cmd(-SPEED)

            elif key == " ":
                send_cmd(0)

            elif key.lower() == "q":
                send_cmd(0)
                break

            # Poll motor feedback every 0.5s
            if time.time() - last_request > 0.5:
                request_status()
                last_request = time.time()

    finally:
        ser.close()


if __name__ == "__main__":
    main()