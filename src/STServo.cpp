#include "STServo.h"
#include "esp_err.h"
#include "hal/uart_types.h"
#include <cstdint>

const char* TAG = "STS";

STServoHandler::STServoHandler(int rxPin, int txPin, int baudrate, uart_port_t uartPort)
				: fRxPin(rxPin), fTxPin(txPin), fBaudrate(baudrate), fUartPort(uartPort)
{
	uart_config_t uart_config = {
		.baud_rate = fBaudrate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_ERROR_CHECK(uart_driver_install(fUartPort, 1024 * 2, 1024 * 2, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(fUartPort, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(fUartPort, fTxPin, fRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

uint8_t STServoHandler::_CalculateChecksum(const uint8_t* packet, size_t length) {
	if (!packet || !length)
		return 0x00;

	uint32_t checksum = 0;
	// Le checksum commence à l'ID (index 2) jusqu'à la fin des paramètres
	for (size_t i = 2; i < length; i++) {
		checksum += packet[i];
	}
	return (uint8_t)(~(checksum & 0xFF));
}

void STServoHandler::Write(uint8_t id, uint8_t address, const uint8_t* data, size_t length)
{
	if (!data || !length)
		return;

	size_t packetSize = 7 + length;
	uint8_t packet[packetSize];

	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = length + 3; // Length = Paramètres + Instruction + Adresse + Checksum
	packet[4] = 0x03;		// Instruction WRITE
	packet[5] = address;
	
	memcpy(&packet[6], data, length);
	packet[packetSize - 1] = _CalculateChecksum(packet, packetSize - 1);

	uart_write_bytes(fUartPort, (const char*)packet, packetSize);
}

void STServoHandler::Write(uint8_t id, uint8_t address, const uint8_t value)
{
	Write(id, address, &value, 1);
}

void STServoHandler::Write(uint8_t id, uint8_t address, const uint16_t value)
{
	const uint8_t data[2] = {
		(uint8_t)(value & 0xFF),
		(uint8_t)((value >> 8) & 0xFF)
	};

	Write(id, address, data, 2);
}

void STServoHandler::Read(uint8_t id, uint8_t address, uint8_t* buffer, size_t length)
{
	if (!buffer || !length)
		return;

	uint8_t packet[8];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = 0x04;	   // Longueur pour une lecture
	packet[4] = 0x02;	   // Instruction Read
	packet[5] = address;
	packet[6] = length;    // Nombre d'octets à lire
	packet[7] = _CalculateChecksum(packet, 7);

	uart_flush(fUartPort);
	uart_write_bytes(fUartPort, (const char*)packet, 8);

	// Lecture de la réponse (Header 5 octets + Data + Checksum)
	if (uart_read_bytes(fUartPort, buffer, length + 6, pdMS_TO_TICKS(100)) == length + 6)
	{
		//Error handlling to do
	}
	else
	{
		ESP_LOGI(TAG, "Message didn't get good response in timeout");
		ESP_LOG_BUFFER_HEX(TAG, packet, 8);
	}
}

void STServoHandler::Read(uint8_t id, uint8_t address, uint8_t* value)
{
	uint8_t buffer[7];

	STS.Read(id, address, buffer, 7);

	*value = buffer[5];
}

void STServoHandler::Read(uint8_t id, uint8_t address, uint16_t* value)
{
	uint8_t buffer[8];

	STS.Read(id, address, buffer, 2);

	*value = buffer[5] + (buffer[6] << 8);
}

void STServoHandler::WriteReg(uint8_t id, uint8_t address, const uint8_t* data, size_t length)
{
	if (!data || !length)
		return;

	// Note: L'instruction 0x04 (REG WRITE) est utilisée pour l'EEPROM/Ecriture différée
	size_t packetSize = 7 + length;
	uint8_t packet[packetSize];

	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = length + 3;
	packet[4] = 0x04; 
	packet[5] = address;

	memcpy(&packet[6], data, length);
	packet[packetSize - 1] = _CalculateChecksum(packet, packetSize - 1);

	int written = uart_write_bytes(fUartPort, (const char*)packet, packetSize);
}

void STServoHandler::Update()
{
	size_t bufferDataLen = 0;

	ESP_ERROR_CHECK(uart_get_buffered_data_len(fUartPort, &bufferDataLen));

	if (bufferDataLen)
	{
		uint8_t buffer[bufferDataLen];

		uart_read_bytes(fUartPort, buffer, bufferDataLen, 0);

		ESP_LOGI(TAG, "Message received");
		ESP_LOG_BUFFER_HEX(TAG, buffer, bufferDataLen);
	}

}

STServo::STServo(uint8_t id) : fId(id)
{}

void STServo::SetTorqueEnable(const bool enabled)
{
	STS.Write(fId, REG_TORQUE_SWITCH, (const uint8_t)enabled);
}

void STServo::SetTargetPosition(const uint16_t position)
{
	STS.Write(fId, REG_TARGET_LOCATION, position);
}

void STServo::SetRunningTime(const uint16_t time)
{
	STS.Write(fId, REG_RUNNING_TIME, time);
}

void STServo::SetRunningSpeed(const uint16_t speed)
{
	STS.Write(fId, REG_RUNNING_SPEED, speed);
}

void STServo::SetAcceleration(const uint8_t acceleration)
{
	STS.Write(fId, REG_ACCELERATION, acceleration);
}

bool STServo::GetTorqueEnable()
{
	uint8_t value;

	STS.Read(fId, REG_TORQUE_SWITCH, &value);

	return value;
}

uint16_t STServo::GetTargetPosition()
{
	uint16_t value;
	STS.Read(fId, REG_TARGET_LOCATION, &value);
	return value;
}

uint16_t STServo::GetRunningTime()
{
	uint16_t value;
	STS.Read(fId, REG_RUNNING_TIME, &value);
	return value;
}

uint16_t STServo::GetRunningSpeed()
{
	uint16_t value;
	STS.Read(fId, REG_RUNNING_SPEED, &value);
	return value;
}

uint8_t STServo::GetAcceleration()
{
	uint8_t value;
	STS.Read(fId, REG_ACCELERATION, &value);
	return value;
}

uint16_t STServo::GetPresentPosition()
{
	uint16_t value;
	STS.Read(fId, REG_PRESENT_LOCATION, &value);
	return value;
}

uint16_t STServo::GetPresentSpeed()
{
	uint16_t value;
	STS.Read(fId, REG_PRESENT_SPEED, &value);
	return value;
}

uint16_t STServo::GetPresentLoad()
{
	uint16_t value;
	STS.Read(fId, REG_PRESENT_LOAD, &value);
	return value;
}

uint8_t STServo::GetPresentVoltage()
{
	uint8_t value;
	STS.Read(fId, REG_PRESENT_VOLTAGE, &value);
	return value;
}

uint8_t STServo::GetPresentTemp()
{
	uint8_t value;
	STS.Read(fId, REG_PRESENT_TEMP, &value);
	return value;
}
