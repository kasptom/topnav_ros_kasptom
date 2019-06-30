from serial import SerialException

import maestro
import time

servo = None
try:
    print 'trying with /dev/tty/ACM0'
    servo = maestro.Controller('/dev/ttyACM0')
except SerialException:
    print 'trying with /dev/tty/ACM1'
    servo = maestro.Controller('/dev/ttyACM1')
print 'ok'

time.sleep(1.0)

servo.setTarget(0, 0)
servo.setTarget(1, 0)

servo.setAccel(0, 0)
servo.setAccel(1, 0)
servo.setSpeed(0, 60)
servo.setSpeed(1, 60)

print 'after setup'


def test_channel(channel_ids):
    print 'testing servos: %s ...' % str(channel_ids)
    time.sleep(1.0)
    print '... start ...'
    for i in range(1, 10):
        time.sleep(1.0)
        for channel_id in channel_ids:
            if i % 2 == 0:
                servo.setTarget(channel_id, 6400)
            else:
                servo.setTarget(channel_id, 5400)
    print '... end'


test_channel([0])
test_channel([0, 1])

#   servo.setTarget(1, 5400)
# upper = servo.getPosition(2)
#    lower = servo.getPosition(3)
#   print upper, lower

# servo.setTarget(1, 0)
servo.setTarget(0, 0)
servo.setTarget(1, 0)
servo.setTarget(2, 0)
servo.setTarget(3, 0)
