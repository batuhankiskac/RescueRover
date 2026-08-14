#include "motor.h"

#include "config.h"
#include "pins.h"

namespace {

	MotorStatus		status = {MotionCommand::STOP, 0};
	bool			allMotionInhibited = false;
	bool			forwardInhibited = false;

	void			writeSide(uint8_t pin1, uint8_t pin2, int8_t direction,
							  bool reversed){
		if (reversed) {
			direction = -direction;
		}

		if (direction > 0) {
			digitalWrite(pin1, HIGH);
			digitalWrite(pin2, LOW);
		} else if (direction < 0) {
			digitalWrite(pin1, LOW);
			digitalWrite(pin2, HIGH);
		} else {

			digitalWrite(pin1, LOW);
			digitalWrite(pin2, LOW);
		}
	}

	bool			applyMotion(MotionCommand command, int8_t leftDirection,
								int8_t rightDirection, uint8_t speed) {
		if (allMotionInhibited ||
			(forwardInhibited && command == MotionCommand::FORWARD)) {
			stopMotors();
			return false;
		}



		analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
		writeSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2, leftDirection,
				  MOTOR_LEFT_REVERSED != 0);
		writeSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2, rightDirection,
				  MOTOR_RIGHT_REVERSED != 0);
		analogWrite(Pins::MOTOR_ENABLE_PWM, speed);

		status.motion = command;
		status.speed = speed;
		return true;
	}

}

void
motorBegin()
{
pinMode(Pins::MOTOR_ENABLE_PWM, OUTPUT);
pinMode(Pins::MOTOR_LEFT_IN1, OUTPUT);
pinMode(Pins::MOTOR_LEFT_IN2, OUTPUT);
pinMode(Pins::MOTOR_RIGHT_IN1, OUTPUT);
pinMode(Pins::MOTOR_RIGHT_IN2, OUTPUT);
	stopMotors();
}

bool
moveForward(uint8_t speed)
{
return applyMotion(MotionCommand::FORWARD, 1, 1, speed);
}

bool
moveBackward(uint8_t speed)
{
return applyMotion(MotionCommand::BACKWARD, -1, -1, speed);
}

bool
turnLeft(uint8_t speed)
{
return applyMotion(MotionCommand::LEFT, -1, 1, speed);
}

bool
turnRight(uint8_t speed)
{
return applyMotion(MotionCommand::RIGHT, 1, -1, speed);
}

void
stopMotors()
{
analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
writeSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2, 0, false);
writeSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2, 0, false);
status.motion = MotionCommand::STOP;
	status.speed = 0;
}

void
motorSetSafetyInhibit(bool blockAllMotion, bool blockForward)
{
	allMotionInhibited = blockAllMotion;
	forwardInhibited = blockForward;

	if (allMotionInhibited ||
(forwardInhibited && status.motion == MotionCommand::FORWARD)) {
		stopMotors();
	}
}

const			MotorStatus &
motorGetStatus()
{
	return status;
}

bool
motorIsMoving()
{
return status.motion != MotionCommand::STOP;
}
