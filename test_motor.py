from adafruit_motorkit import MotorKit

Motor = MotorKit()

for i in range(100):
	Motor.motor1.throttle = 0.0
	Motor.motor2.throttle = 0.0
	Motor.motor3.throttle = 0.0
	Motor.motor4.throttle = 0.5
Motor.motor4.throttle = 0.0
