#ifndef _PWM_H
#define _PWM_H

#include "timer.h"

namespace first_project{

class PWM : public Timer {
	
	private:
		unsigned char can;
	
	public:
		PWM();																				// Constructor.
		void pwmInit(unsigned char tmr, double fre);	// PWM initializer.
		void pinChannel(unsigned char pin);						// Pin configurer.
		void activateChannel(unsigned char can);			// Channel activation.
		void invertPolarity(void);										// Invert polarity.
		void setPwm(unsigned char can, double duty);	// Set PWM duty cycle.
		unsigned int getPtr(unsigned char can);				// Get pointer.
};

}	// namespace first_project

#endif