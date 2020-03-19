# TinyCircuits ATtiny841 Capacitive Touch Slider Python Package
# Written by: Laverena Wienclaw for TinyCircuits
# Initialized: Mar 2020
# Last updated: Mar 2020

import pigpio
import time

# Registers and associated bit numbers

_T841_ADCSRA =      # 0x20+0x05
_T841_ADEN = 0x80         # 7 = 1000 0000 

_T841_DIDR0   0x60

_T841_PRR     0x70
_T841_PRSPI    = 0x01  # 4 = 0001 0000
_T841_PRUSART0 = 0x02  # 5 = 0010 0000
_T841_PRUSART1 = 0x04  # 6 = 0100 0000


_COMMAND_SET_MODE = 0x00  # write mode- command, register access
# _COMMAND_SERVO_1 = 0x01 # write 16 bit pwm value 1
# _COMMAND_SERVO_2 = 0x02 # write 16 bit pwm value 2
# _COMMAND_SERVO_3 = 0x03 # write 16 bit pwm value 3
# _COMMAND_SERVO_4 = 0x04 # write 16 bit pwm value 4
# _COMMAND_MOTOR_1 = 0x05 # write first two 16 bit pwm values
# _COMMAND_MOTOR_2 = 0x06 # write second two 16 bit pwm values
_COMMAND_ALL_PWM = 0x07 # write four 16 bit pwm values
_COMMAND_TIMER_1 = 0x08 # write 16 bit timer 1 top value
_COMMAND_TIMER_2 = 0x09 # write 16 bit timer 2 top value
_COMMAND_PRESCALER_1 = 0x0A # write timer 1 prescaler
_COMMAND_PRESCALER_2 = 0x0B # write timer 2 prescaler
_COMMAND_CLOCK_PRESCALER = 0x0C # write system clock prescaler
_COMMAND_SET_SLEEP_MODE = 0x0D # set sleep mode
_COMMAND_SLEEP = 0x0E # go to sleep after I2C communication is done
_COMMAND_SET_FAILSAFE_VALUES = 0x0F # set failsafe PWM values - default is 0
_COMMAND_SET_FAILSAFE_PRESCALER = 0x10 # set failsafe timeout
_COMMAND_SET_FAILSAFE_TIMEOUT = 0x11 # set failsafe timeout
_COMMAND_ALL_PWM_8 = 0x12 # write four 8 bit pwm values
_COMMAND_CAP_TOUCH = 0x13 # read cap touch value on pin

_T841_CLOCK_PRESCALER_1 = 0x00
_T841_CLOCK_PRESCALER_2 = 0x01
_T841_CLOCK_PRESCALER_4 = 0x02
_T841_CLOCK_PRESCALER_8 = 0x03
_T841_CLOCK_PRESCALER_16 = 0x04
_T841_CLOCK_PRESCALER_32 = 0x05
_T841_CLOCK_PRESCALER_64 = 0x06
_T841_CLOCK_PRESCALER_128 = 0x07
_T841_CLOCK_PRESCALER_256 = 0x08

_T841_TIMER_PRESCALER_0 = 0x00
_T841_TIMER_PRESCALER_1 = 0x01
_T841_TIMER_PRESCALER_8 = 0x02
_T841_TIMER_PRESCALER_64 = 0x03
_T841_TIMER_PRESCALER_256 = 0x04
_T841_TIMER_PRESCALER_1024 = 0x05

_T841_SLEEP_MODE_IDLE = 0
#_T841_SLEEP_MODE_ADC = _BV(T841_SM0)
#_T841_SLEEP_MODE_PWR_DOWN = _BV(T841_SM1)

_MODE_REGISTER_INC 0xAA
_MODE_REGISTER_DEC 0xAB
_MODE_COMMAND 0xAC

_RETURN_VAL_REG_0 0x1A
_RETURN_VAL_REG_1 0x1B
_RETURN_VAL_REG_2 0x1C
_RETURN_VAL_REG_3 0x1D

_FIRMWARE_REVISION_REG 0x19

_T841_ADDRESS = 0x31
_EXPECTED_CAPTOUCHWIRELING_FIRMWARE = 0x1A



class ATTINY841:
    def __init__(self): # init firmware, returns 0 on success
        self.numSensors = 6
        self.capTouchPins = [ 0, 1, 2, 3, 5, 7]
        self.capTouchCal = [None] * numSensors
        self.capTouchCurrent = [None] * numSensors
        self.overCalCount = [None] * numSensors
        self.currentMaxReading = 0
        self.primary = 0
        self.isTouch = 0
        self.initialTouchPos = 0
        self.finalTouchPos = 0
        self.currentTouchedPositions = 0
        self.allTouchedPositions = 0
        self.touchTimer = 0

    def begin(self):
        pi = pigpio.pi()
        h = pi.i2c_open(1, _T841_ADDRESS)
        pi.i2c_write_byte(h, _COMMAND_SET_MODE)
        pi.i2c_write_byte(h, _MODE_REGISTER_DEC)

        # Make sure there is firmware 
        firmware = pi.i2c_read_byte_data(h, _FIRMWARE_REVISION_REG)
        if firmware != _EXPECTED_CAPTOUCHWIRELING_FIRMWARE:
            return 1 # error

        pi.i2c_write_byte(h, _T841_ADCSRA)
        ADEN = T841_ADEN | 4 | 1 # Does the bitwise OR do what we want with ints?
        pi.i2c_write_byte(h, ADEN)

        pi.i2c_write_byte(h, _T841_DIDR0)
        pi.i2c_write_byte(h, 0xAF)

        pi.i2c_write_byte(h, _T841_PRR)
        PRS = _T841_PRSPI | _T841_PRUSART0 | _T841_PRUSART1 # = 0111 0000 
        pi.i2c_write_byte(h, PRS)

        # send interpreted commands- see header file
        pi.i2c_write_byte(h, _COMMAND_SET_MODE)
        pi.i2c_write_byte(h, _MODE_COMMAND)

        pi.i2c_write_byte(h, _COMMAND_CLOCK_PRESCALER)
        pi.i2c_write_byte(h, _T841_CLOCK_PRESCALER_1)

        pi.i2c_close(h)

        pin = 0
        for pin in range(0,6):
            self.capTouchCal[pin] = self.capTouchRead(pin)
            self.overCalCount[pin] = 0

        return 0


    def capTouchRead(self, pin):
        pi = pigpio.pi()
        h = pi.i2c_open(1, _T841_ADDRESS)

        pi.i2c_write_byte(h, _COMMAND_CAP_TOUCH)
        pi.i2c_write_byte(h, self.capTouchPins[pin])
        pi.i2c_write_byte(h, 5)

        time.sleep(0.1)

        pi.i2c_write_byte(h, _COMMAND_SET_MODE)
        pi.i2c_write_byte(h, _MODE_REGISTER_DEC)

        value = pi.i2c_read_byte_data(h, _RETURN_VAL_REG_0)
        value = value + (pi.i2c_read_byte_data(h, _RETURN_VAL_REG_1) << 8)

        pi.i2c_write_byte(h, _COMMAND_SET_MODE)
        pi.i2c_write_byte(h, _MODE_COMMAND)

        return value

    def getPosition(self, range):
        position = self.primary

        if primary < (numSensors - 1):
            position = position + 
        # TODO : big math equation here

    def getMaxReading(self):
        return self.currentMaxReading

    def getMagnitude(self):
        j = 0
        magnitude = 0

        for j in range(0,6)
        



        










CapTouchWireling(uint8_t addr=0);
    uint8_t begin();
    uint16_t capTouchRead(uint8_t pin);
    int16_t getPosition(int range=100);
    int16_t getMaxReading();
    int16_t getMagnitude();
    uint32_t duration();
    bool update();
    bool isTouched();
    
    # static const uint8_t numSensors = 6;
    # const uint8_t T841_ADDRESS = 0x31;
    # const uint8_t EXPECTED_CAPTOUCHWIRELING_FIRMWARE = 0x1A;
    # const uint8_t capTouchPins[numSensors] = { 0, 1, 2, 3, 5, 7};
    # int16_t capTouchCal[numSensors];
    # int16_t capTouchCurrent[numSensors];
    # uint8_t overCalCount[numSensors];
    # int16_t currentMaxReading= 0;
    # uint8_t primary = 0;
    # bool isTouch = 0;
    # uint8_t initialTouchPos = 0;
    # uint8_t finalTouchPos = 0;
    # uint8_t currentTouchedPositions = 0;
    # uint8_t allTouchedPositions = 0;
    # uint32_t touchTimer = 0;