import time
import gpiozero as gp

# motor izquierdo GPIO pins 23(red) 24(black) 25(enable)
# motor derecho GPIO pins 17(red) 27(black) 22(enable)

izq = gp.Motor(23,24,25)

izq.forward(1) # move forward at (0 <= x <= 1) [float]

while(True):
    time.sleep(1)
    izq.reverse() # change direction if moving at same speed
