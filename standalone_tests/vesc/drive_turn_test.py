#!/usr/bin/env python3

import time

import pyvesc
import serial
from pyvesc import VESCMessage


class SetServoPosition(metaclass=VESCMessage):
    """
    VESC servo-position command.

    The position is normalized:
        0.0 = one steering endpoint
        0.5 = nominal center
        1.0 = opposite steering endpoint
    """

    id = 12

    fields = [
        ("servo_pos", "h", 1000),
    ]


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Keep this low for the initial test.
MOTOR_DUTY = 0.10

# Replace TURN_POSITION with the steering value that worked well
# in your servo test.
CENTER_POSITION = 0.500
TURN_POSITION = 0.250

FIRST_STRAIGHT_TIME = 2.0
TURN_TIME = 2.0
FINAL_STRAIGHT_TIME = 2.0

COMMAND_INTERVAL = 0.10


def send_motor_duty(
    connection: serial.Serial,
    duty: float,
) -> None:
    """Send a normalized motor duty-cycle command."""

    if not -1.0 <= duty <= 1.0:
        raise ValueError(
            "Motor duty must be between -1.0 and 1.0"
        )

    # This installed PyVESC version expects the duty as an integer
    # scaled by 100000.
    scaled_duty = int(round(duty * 100000))

    message = pyvesc.SetDutyCycle(scaled_duty)
    packet = pyvesc.encode(message)

    connection.write(packet)
    connection.flush()


def send_servo_position(
    connection: serial.Serial,
    position: float,
) -> None:
    """Send a normalized steering-servo position."""

    if not 0.0 <= position <= 1.0:
        raise ValueError(
            "Servo position must be between 0.0 and 1.0"
        )

    message = SetServoPosition(position)
    packet = pyvesc.encode(message)

    connection.write(packet)
    connection.flush()


def drive_for_duration(
    connection: serial.Serial,
    duty: float,
    steering_position: float,
    duration: float,
) -> None:
    """
    Repeatedly send motor and steering commands for a fixed time.
    """

    start_time = time.monotonic()

    while time.monotonic() - start_time < duration:
        send_motor_duty(connection, duty)
        send_servo_position(connection, steering_position)

        time.sleep(COMMAND_INTERVAL)


def stop_vehicle(connection: serial.Serial) -> None:
    """
    Repeatedly command zero motor duty and centered steering.
    """

    for _ in range(10):
        send_motor_duty(connection, 0.0)
        send_servo_position(connection, CENTER_POSITION)

        time.sleep(0.05)


def main() -> None:
    print(f"Opening VESC on {SERIAL_PORT}...")

    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.1,
        write_timeout=1.0,
    ) as connection:

        # Allow the USB serial connection to settle.
        time.sleep(1.0)

        try:
            print("Centering steering...")
            send_motor_duty(connection, 0.0)
            send_servo_position(connection, CENTER_POSITION)
            time.sleep(1.0)

            print("Driving straight for 2 seconds...")
            drive_for_duration(
                connection=connection,
                duty=MOTOR_DUTY,
                steering_position=CENTER_POSITION,
                duration=FIRST_STRAIGHT_TIME,
            )

            print("Turning while continuing to drive...")
            drive_for_duration(
                connection=connection,
                duty=MOTOR_DUTY,
                steering_position=TURN_POSITION,
                duration=TURN_TIME,
            )

            print("Returning to straight driving for 2 seconds...")
            drive_for_duration(
                connection=connection,
                duty=MOTOR_DUTY,
                steering_position=CENTER_POSITION,
                duration=FINAL_STRAIGHT_TIME,
            )

        finally:
            print("Stopping motor and centering steering...")
            stop_vehicle(connection)

    print("Drive-and-turn test complete.")


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

    except FileNotFoundError:
        print(
            f"Could not find {SERIAL_PORT}. "
            "Check the VESC USB connection."
        )

    except serial.SerialException as error:
        print(f"Serial communication failed: {error}")

    except Exception as error:
        print(f"Test failed: {error}")
