#include "STServo.h"
#include "Settingator.h"
#include "UARTCommunicator.h"
#include "CustomType.hpp"

Settingator STR(nullptr);
STServoHandler STS(18, 17, 1000000, UART_NUM_2);

STServo	servo(0x01);

STR_UInt16 pos(0, "POSITION");

extern "C" void app_main(void)
{
	STR.begin();
	STR.SetCommunicator(UARTCTR::CreateInstance());

	
	pos.SetCallback([](){
			servo.SetTargetPosition(pos);
			});

	servo.SetTorqueEnable(true);
	servo.SetTargetPosition(4095);

	servo.GetTorqueEnable();
	while (true)
	{
		STR.Update();
		STS.Update();
	}
}
