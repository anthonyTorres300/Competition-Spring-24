import lgpio
import time


PIN = 4 #change the pin when using another 

chip = lgpio.gpiochip_open(PIN) # Reading pin from the raspi

lgpio.gpio_claim_input(chip, PIN)
try:
    while True:
        pin_value = lgpio.gpio_read(chip, PIN)
        # print("NO water" if pin_value == 0 else "There is water")
        if pin_value==1:
            print("There is water")
        else:
            print("NO WATER")
        time.sleep(0.1)
finally:    
    lgpio.gpiochip_close(chip)
