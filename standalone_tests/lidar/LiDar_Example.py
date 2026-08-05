#!/usr/bin/env python3

import time

import pyvesc
import serial
from pyvesc import VESCMessage


class SetServoPosition(metaclass=VESCMessage):
    """
    VESC servo-position command.

    Normalized position:
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

# Motor speeds.
# Keep these low until the sequence is verified safely.
NORMAL_DUTY = 0.150
CORNER_DUTY = 0.050

# Use the steering values already proven safe on your car.
CENTER_POSITION = 0.500
TURN_POSITION = 0.250

# Timing for each stage.
NORMAL_STRAIGHT_TIME = 2.0
SLOW_APPROACH_TIME = 1.0
CORNER_TIME = 2.0
STRAIGHTEN_TIME = 0.5
FINAL_STRAIGHT_TIME = 3.0

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

    # Your installed PyVESC version expects an integer
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
    """Send a normalized steering position."""

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
    Repeatedly send motor and steering commands
    for the requested duration.
    """

    start_time = time.monotonic()

    while time.monotonic() - start_time < duration:
        send_motor_duty(connection, duty)
        send_servo_position(connection, steering_position)

        time.sleep(COMMAND_INTERVAL)


def stop_vehicle(connection: serial.Serial) -> None:
    """Repeatedly stop the motor and center the steering."""

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

        time.sleep(1.0)

        try:
            print("Centering steering...")
            send_motor_duty(connection, 0.0)
            send_servo_position(connection, CENTER_POSITION)
            time.sleep(1.0)

            print("Driving straight at normal speed for 2 seconds...")
            drive_for_duration(
                connection=connection,
                duty=NORMAL_DUTY,
                steering_position=CENTER_POSITION,
                duration=NORMAL_STRAIGHT_TIME,
            )

            print("Slowing down while still driving straight...")
            drive_for_duration(
                connection=connection,
                duty=CORNER_DUTY,
                steering_position=CENTER_POSITION,
                duration=SLOW_APPROACH_TIME,
            )

            print("Turning at reduced speed...")
            drive_for_duration(
                connection=connection,
                duty=CORNER_DUTY,
                steering_position=TURN_POSITION,
                duration=CORNER_TIME,
            )

            print("Straightening the wheels at reduced speed...")
            drive_for_duration(
                connection=connection,
                duty=CORNER_DUTY,
                steering_position=CENTER_POSITION,
                duration=STRAIGHTEN_TIME,
            )

            print("Increasing speed and driving straight for 3 seconds...")
            drive_for_duration(
                connection=connection,
                duty=NORMAL_DUTY,
                steering_position=CENTER_POSITION,
                duration=FINAL_STRAIGHT_TIME,
            )

        finally:
            print("Stopping motor and centering steering...")
            stop_vehicle(connection)

    print("Slow-corner test complete.")


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
