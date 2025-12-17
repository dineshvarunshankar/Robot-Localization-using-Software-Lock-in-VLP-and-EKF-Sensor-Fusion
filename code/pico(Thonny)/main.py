from machine import Pin, PWM, I2C
import sys
import select
import time
import struct

# CONFIGURATION 
L_PWM, L_DIR = 21, 20
R_PWM, R_DIR = 27, 26
ENC_A, ENC_B = 16, 17
I2C_ID, SDA, SCL = 1, 2, 3

# YOUR TRIMS
YELLOW_TRIM = 0.69   
METAL_TRIM  = 0.75  

# HARDWARE 
l_pwm = PWM(Pin(L_PWM)); l_pwm.freq(20000)
l_dir = Pin(L_DIR, Pin.OUT)
r_pwm = PWM(Pin(R_PWM)); r_pwm.freq(20000)
r_dir = Pin(R_DIR, Pin.OUT)

enc_a = Pin(ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B, Pin.IN, Pin.PULL_UP)
ticks = 0

i2c = I2C(I2C_ID, sda=Pin(SDA), scl=Pin(SCL), freq=400000)
try: i2c.writeto_mem(0x68, 0x6B, b'\x00')
except: pass

def encoder_irq(pin):
    global ticks
    if enc_b.value() > 0: ticks += 1
    else: ticks -= 1
enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_irq)

def read_gyro():
    try: return struct.unpack('>h', i2c.readfrom_mem(0x68, 0x47, 2))[0] / 131.0
    except: return 0.0

def set_motor(dir_pin, pwm_pin, speed):
    # 0 = Forward, 1 = Backward 
    if speed >= 0:
        dir_pin.value(0)
        pwm_pin.duty_u16(int(speed * 65535))
    else:
        dir_pin.value(1)
        pwm_pin.duty_u16(int(-speed * 65535))

# MAIN LOOP 
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

target_l = 0.0
target_r = 0.0
last_cmd_time = time.ticks_ms()

while True:
    # 1. READ INPUT
    if poll_obj.poll(0):
        ch = sys.stdin.read(1)
        if ch:
            last_cmd_time = time.ticks_ms()
            # STEERING LOGIC
            if ch == 'w':   # Forward
                target_l, target_r = 0.6, 0.6
            elif ch == 'x': # Backward
                target_l, target_r = -0.6, -0.6
            elif ch == 'a': # Spin Left
                target_l, target_r = -0.5, 0.5
            elif ch == 'd': # Spin Right
                target_l, target_r = 0.5, -0.5
            elif ch == ' ': # Stop
                target_l, target_r = 0.0, 0.0

    # 2. SAFETY WATCHDOG (Stops if key released)
    if time.ticks_diff(time.ticks_ms(), last_cmd_time) > 200:
        target_l, target_r = 0.0, 0.0

    # 3. DRIVE
    # Apply Trims to individual motors
    set_motor(l_dir, l_pwm, target_l * YELLOW_TRIM)
    set_motor(r_dir, r_pwm, target_r * METAL_TRIM)

    # 4. SEND DATA
    print(f"{ticks},{read_gyro()}")
    time.sleep(0.05)