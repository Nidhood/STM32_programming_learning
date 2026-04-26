#ifndef PIN_H_
#define PIN_H_

#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PIN_MODE_INPUT = 0U,
	PIN_MODE_OUTPUT = 1U,
	PIN_MODE_AF = 2U,
	PIN_MODE_ANALOG = 3U
} Pin_Mode_t;

typedef enum {
	PIN_OUTPUT_PUSH_PULL = 0U,
	PIN_OUTPUT_OPEN_DRAIN = 1U
} Pin_OutputType_t;

typedef enum {
	PIN_SPEED_LOW = 0U,
	PIN_SPEED_MEDIUM = 1U,
	PIN_SPEED_HIGH = 2U,
	PIN_SPEED_VERY_HIGH = 3U
} Pin_Speed_t;

typedef enum {
	PIN_PULL_NONE = 0U,
	PIN_PULL_UP = 1U,
	PIN_PULL_DOWN = 2U
} Pin_Pull_t;

typedef struct {
		PinName_t pin;
		Pin_Mode_t mode;
		Pin_OutputType_t output_type;
		Pin_Speed_t speed;
		Pin_Pull_t pull;
		uint8_t alternate_function;
} Pin_Config_t;

bool Pin_Init( const Pin_Config_t *config );
void Pin_Write( PinName_t pin, bool state );
bool Pin_Read( PinName_t pin );
void Pin_Toggle( PinName_t pin );

#ifdef __cplusplus
}
#endif

#endif /* PIN_H_ */