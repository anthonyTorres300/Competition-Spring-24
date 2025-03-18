import gpiod
import time

CHIP_NAME = 'gpiochip4'
LINE_NUMBERS = [21, 20, 26]  # The GPIO pin numbers you want to control

with gpiod.Chip(CHIP_NAME) as chip:
    lines = chip.get_lines(LINE_NUMBERS)
    lines.request(consumer='example', type=gpiod.LINE_REQ_DIR_OUT)
    print(f"Lines {LINE_NUMBERS} are now controlled by this script.")

    try:
        while True:
            lines.set_values([1, 1, 1])
            time.sleep(1)
            lines.set_values([0, 0, 0])
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting the script.")
