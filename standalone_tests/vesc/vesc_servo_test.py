#!/usr/bin/env python3

import time
import serial
import pyvesc

from pyvesc import VESCMessage



class SetServoPosition(metaclass=VESCMessage):
    
    """
    VESC servo-position command.

    servo_pos uses a normalized value:
        0.0 = one endpoint
        0.5 = center
        1.0 = opposite endpoint
    """

    id = 12
    fields = [
        ("servo_pos", "h", 1000),
    ]


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

CENTER = 0.500
LEFT = 0.250
RIGHT = 0.750


def send_servo_position(
    connection: serial.Serial,
    position: float,
) -> None:
    if not 0.0 <= position <= 1.0:
        raise ValueError("Servo position must be between 0.0 and 1.0")

    message = SetServoPosition(position)
    packet = pyvesc.encode(message)

    connection.write(packet)
    connection.flush()

    print(f"Sent servo position: {position:.3f}")


def main() -> None:
    print(f"Opening VESC on {SERIAL_PORT}...")

    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.1,
        write_timeout=1.0,
    ) as connection:

        time.sleep(1.0)

        try:
            print("Center")
            send_servo_position(connection, CENTER)
            time.sleep(2.0)

            print("Slight turn")
            send_servo_position(connection, LEFT)
            time.sleep(1.0)

            print("Center")
            send_servo_position(connection, CENTER)
            time.sleep(1.0)

            print("Slight opposite turn")
            send_servo_position(connection, RIGHT)
            time.sleep(1.0)

        finally:
            print("Returning to center")
            send_servo_position(connection, CENTER)
            time.sleep(0.25)

    print("Servo test complete")


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nServo test cancelled")

    except serial.SerialException as error:
        print(f"Serial error: {error}")

    except Exception as error:
        print(f"Servo test failed: {error}")
