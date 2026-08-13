/* Functions for servo control.

Author: Rainer Meyer */


#include "grbl.h"

void gripper_init()
{
	// PE4 (Pin2 on 2560) as output
	GRIPPER_DDR |= (1<<GRIPPER_PWM_BIT);

	// Timer 3 Fast PWM (Mode 14)
	// Clear OC3A on compare match, set at TOP
	TCCR3A = (1<<COM3B1) | (1<<WGM31);
	// WGM Mode 14 + Prescaler = 8
	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS31);

	// Period (TOP value) to 40000 ticks (50Hz)
	ICR3 = SERVO_TOP_TICKS;

	// 1.5ms pulse width by default
	gripper_set(0.5);
}

// Position 
void gripper_set(float position)
{
	if (position < 0.0f) { position = 0.0f; }
	else if (position > 1.0f) { position = 1.0f; }

	uint16_t pulse = (uint16_t)(SERVO_MIN_TICKS + (position * (float)(SERVO_MAX_TICKS - SERVO_MIN_TICKS)));
	// Update Output Compare Register A for Timer 3
	OCR3B = pulse;
}