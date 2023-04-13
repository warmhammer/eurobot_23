#!/usr/bin/env python3

import art
import gpiozero
import time
import os
import socket

class ButtonCounter:
    def __init__(self, pin):
        self._button = gpiozero.Button(pin)

        self._prev_state = self._button.is_pressed
        self._state = self._button.is_pressed

        self.count = 0

    def update(self):
        if self._state and not self._prev_state:
            self.count += 1

        self._prev_state = self._state
        self._state = self._button.is_pressed


def main(led_pin=14, host="192.168.1.10", port=65432):
    counter = ButtonCounter(led_pin)

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print('Waiting for server...')

            while True:
                try:
                    s.connect((host, port))
                    break
                except Exception as _:
                    time.sleep(1)


            prev_time = time.time()

            while True:
                counter.update()

                if time.time() - prev_time > 0.5:
                    prev_time = time.time()

                    os.system('clear')
                    art.tprint(str(counter.count), font="tarty9")

                    try:
                        s.sendall(bytes(f"{counter.count}", 'utf-8'))
                    except Exception as _:
                        print('Connection is lost...')
                        break

                time.sleep(1e-5)


if __name__ == "__main__":
    main()
