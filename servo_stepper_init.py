import time, math, json, network
from machine import Pin, PWM
from servo import Servo
from stepper import Stepper

# arm1 = Servo(13,500,2500,0.0,180.0,50)
# arm2 = Servo(12,500,2500,0.0,180.0,50)
# 
# #arm1.write(180-70) # servo angle is 180-real angle
# arm2.write(32) # servo input 115 deg max,32 deg min

## code to control stepper
# self,step_pin,dir_pin,en_pin=None,steps_per_rev=200,speed_sps=10,invert_dir=False,timer_id=-1
step_pin = 17
dir_pin = 18

ms1 = Pin(19, Pin.OUT, Pin.PULL_DOWN) # different combos of ms1,ms2 states gives microstep sizes. Low Low is 1 step
ms2 = Pin(20, Pin.OUT, Pin.PULL_DOWN)
enable = Pin(21, Pin.OUT, Pin.PULL_DOWN) # when enable is on, stepper is disengaged. Use to keep stepper from overheating from staying on

enable.off() 
ms1.off()
ms2.off()

slider = Stepper(step_pin,dir_pin,steps_per_rev=200,speed_sps=200) # in our system, 200 steps = 4 cm irl motion

slider.overwrite_pos(0) # sets whatever position is curent as 0
slider.target(400) # move 400 steps (8cm) from this position
time.sleep(5)
slider.stop()
enable.on()
