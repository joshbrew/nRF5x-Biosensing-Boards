#note timing sync may be weird on debug runs, adjust freqMhz as needed.
# Resetting the device (with the stop button if it doesn't quit)


import machine, neopixel, time
from math import floor 
freqMhz = 80
machine.freq(freqMhz*1000000) # machine clock (limit power consumption
timer = machine.Timer() # hardware LED timer loop

uart = machine.UART(0, 115200, tx=machine.Pin(0), rx=machine.Pin(1))
# init with given baudrate

uart.write('Hello from RP2040')

#0 for pulseox mode, rotates thru at a freq defined by timer, todo: have the timer stopped/started with controls for freq
#1 for alert mode, same routine w/ freq but different set of leds
pulseoxleds = [3, -1, 4, -1] # -1 for ambient
alertleds = [3, 4]
ledPins = {}
ledmode = 0

adcFreq = 250 #sps of ADC (250, 500, 1000, 2000) 500 is reasonably the limit

#default in debug mode (2 leds)
nLEDs = 2 #number of LEDs (incl ambient, todo: make struct of specific GPIO to rotate thru
ledCtrMax = nLEDs - 1
cycleFreq = 2 #default to ADC cycle freq

pulseDelayMicros = 50 #delay from start of loop (ideally synced to ADC)
pulseWidthMicros = 200 #pulse width (GPIO "on" time) allow for sufficient rise and fall


#LED pulses: :___|-|________:___|-|_________:...
# between each :: is an ADC sample time, the LED pulse is only on for a
#  partial duration that so LED output doesn't bleed over each reading


import uasyncio as asyncio
async def receiver():
    global nLEDs
    global ledCtrMax
    global cycleFreq
    global adcFreq #we can add more commands for modifying stuff
    
    print("Starting UART poll")
    while True:
        if uart.any() > 0:
            print("data received")

            data = uart.read().decode("utf-8");
            timer.deinit()
            if data == "debug":
                mode = 0
                nLEDs = len(alertleds)
                ledCtrMax = nLEDs - 1
                cycleFreq = adcFreq # e.g. 25 LEDs / 250sps = 10 cycles per sec
            elif data == "ledr1": #this is the sampling routine
                mode = 1
                nLEDs = len(pulseoxleds)
                nLEDs = len(alertleds)
                ledCtrMax = nLEDs - 1
                cycleFreq = adcFreq 
            elif data == "ledr2": #alert routine
                mode = 2
                nLEDs = len(alertleds)
                ledCtrMax = nLEDs - 1
                cycleFreq = 2 # 2Hz

            print(data)
            #asyncio.sleep_ms(100) #e.g. wait a set number of ms for syncing w/ ADC
            timer.init(
                freq=cycleFreq, mode=machine.Timer.PERIODIC, callback=tick
            )
        
        asyncio.sleep_ms(1000)



## todo: implement the protocols for these in the main loop:

for led in pulseoxleds:
    if led in ledPins:
        continue
    elif led > -1:
        pin = machine.Pin(led, machine.Pin.OUT)
        ledPins[led] = pin
        pin.off()

ledCtr = 0
ledCtrMax = nLEDs - 1

# test with onboard RGB
pixel_pin = 16
pixel = neopixel.NeoPixel(machine.Pin(pixel_pin), 1)
pixel[0] = (0, 255, 255) #GRB
pixel.write()

lastTick = time.ticks_cpu()

def tick(timer):
    global ledmode
    global lastTick
    global ledCtr
    global ledCtrMax
    global uartCheckTicks
    global uartTicks
    
    # debug: check consistent timing scheduling
    tick = time.ticks_cpu()
    tickDiff = time.ticks_diff(tick, lastTick)
    lastTick = tick
    print(tickDiff)
    
    time.sleep_us(pulseDelayMicros)
    
    if ledmode == 0: #debug mode, use main LED
        if ledCtr < ledCtrMax:        
            pixel[0] = (255, 0, 0)
        else:
            pixel[0] = (0, 255, 255)
        pixel.write()
        
        time.sleep_us(pulseWidthMicros)
        
        pixel[0] = (0,0,0)
        pixel.write()
        #pin.off()
    elif ledmode == 1:
        pin = ledPins[pulseoxleds[ledCtr]]
        if pin != -1:
            pin.on()
            time.sleep_us(pulseWidthMicros)
            pin.off()
    elif ledmode == 2:
        pin = ledPins[alertleds[ledCtr]]
        if pin != -1:
            pin.on()
            time.sleep_us(pulseWidthMicros)
            pin.off()
    
    
    if ledCtr < ledCtrMax:      
        ledCtr += 1
    else:
        ledCtr = 0
        
   
    #uart.write(bin(pin)) #e.g. we could transmit ctrs


loop = asyncio.get_event_loop()
loop.create_task(receiver())
loop.run_forever()


