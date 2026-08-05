#!/usr/bin/env python3

import time
import serial
import pyvesc

from pyvesc import GetValues, SetDutyCycle

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

TEST_DUTY = 0.05
RUN_TIME_SECONDS = 1.0


def send_duty(ser: serial.Serial, duty: float) -> None:
    """Encode and transmit a VESC duty-cycle command."""
    command = SetDutyCycle(int(duty*100000))
    packet = pyvesc.encode(command)
    ser.write(packet)
    ser.flush()


def request_measurements(ser: serial.Serial):
    """Request and decode one VESC telemetry response."""

    # Remove any old bytes currently waiting in the input buffer.
    ser.reset_input_buffer()

    request_packet = pyvesc.encode_request(GetValues)
    ser.write(request_packet)
    ser.flush()

    deadline = time.monotonic() + 1.0
    received = bytearray()

    while time.monotonic() < deadline:
        waiting = ser.in_waiting

        if waiting:
            received.extend(ser.read(waiting))

            try:
                response, consumed = pyvesc.decode(bytes(received))

                if response is not None:
                    return response

            except Exception:
                # The complete packet may not have arrived yet.
                pass

        time.sleep(0.01)

    return None


def print_measurements(values) -> None:
    if values is None:
        print("No telemetry response received.")
        return

    print("Telemetry received:")

    for name, value in vars(values).items():
        print(f"  {name}: {value}")


def stop_motor(ser: serial.Serial) -> None:
    print("Sending stop command...")

    for _ in range(10):
        send_duty(ser, 0.0)
        time.sleep(0.05)


def main() -> None:
    print(f"Opening {SERIAL_PORT}...")

    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.05,
        write_timeout=1.0,
    ) as ser:

        time.sleep(1.0)

        print("Requesting initial VESC telemetry...")
        values = request_measurements(ser)
        print_measurements(values)

        if values is None:
            raise RuntimeError(
                "The VESC did not return telemetry. "
                "Motor command will not be sent."
            )

        print("\nMotor will run at 5% duty for one second.")
        print("Press Ctrl+C now to cancel.")

        for number in (3, 2, 1):
            print(number)
            time.sleep(1)

        try:
            start_time = time.monotonic()

            while time.monotonic() - start_time < RUN_TIME_SECONDS:
                send_duty(ser, TEST_DUTY)
                time.sleep(0.05)

        finally:
            stop_motor(ser)

        print("\nRequesting final telemetry...")
        final_values = request_measurements(ser)
        print_measurements(final_values)

        print("\nTest complete.")


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nTest cancelled.")

    except serial.SerialException as error:
        print(f"Serial-port error: {error}")

    except Exception as error:
        print(f"Test failed: {error}")
