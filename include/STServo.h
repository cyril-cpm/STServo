#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/uart_types.h"
#include <cstdint>
#include <initializer_list>
#include <functional>

/**
 * STS3215 Control Table Defines
 */

// --- EPROM AREA (Persistent) ---
#define REG_FIRMWARE_MAJOR_VER    0x00 // 1 Byte
#define REG_FIRMWARE_SUB_VER      0x01 // 1 Byte
#define REG_SERVO_MAIN_VER        0x03 // 1 Byte
#define REG_SERVO_SUB_VER         0x04 // 1 Byte
#define REG_ID                    0x05 // 1 Byte
#define REG_BAUD_RATE             0x06 // 1 Byte
#define REG_RETURN_DELAY          0x07 // 1 Byte
#define REG_RESPONSE_STATUS_LVL   0x08 // 1 Byte
#define REG_MIN_ANGLE_LIMIT       0x09 // 2 Bytes
#define REG_MAX_ANGLE_LIMIT       0x0B // 2 Bytes
#define REG_MAX_TEMP_LIMIT        0x0D // 1 Byte
#define REG_MAX_INPUT_VOLTAGE     0x0E // 1 Byte
#define REG_MIN_INPUT_VOLTAGE     0x0F // 1 Byte
#define REG_MAX_TORQUE            0x10 // 2 Bytes
#define REG_PHASE                 0x12 // 1 Byte
#define REG_UNLOADING_CONDITION   0x13 // 1 Byte
#define REG_LED_ALARM_CONDITION   0x14 // 1 Byte
#define REG_P_COEFFICIENT         0x15 // 1 Byte
#define REG_D_COEFFICIENT         0x16 // 1 Byte
#define REG_I_COEFFICIENT         0x17 // 1 Byte
#define REG_MIN_STARTUP_FORCE     0x18 // 2 Bytes
#define REG_CW_INSENSITIVE_AREA   0x1A // 1 Byte
#define REG_CCW_INSENSITIVE_AREA  0x1B // 1 Byte
#define REG_PROTECTION_CURRENT    0x1C // 2 Bytes
#define REG_ANGULAR_RESOLUTION    0x1E // 1 Byte
#define REG_POSITION_CORRECTION   0x1F // 2 Bytes
#define REG_OPERATION_MODE        0x21 // 1 Byte
#define REG_PROTECTIVE_TORQUE     0x22 // 1 Byte
#define REG_PROTECTION_TIME       0x23 // 1 Byte
#define REG_OVERLOAD_TORQUE       0x24 // 1 Byte
#define REG_SPEED_CLOSED_LOOP_P   0x25 // 1 Byte
#define REG_OVER_CURRENT_TIME     0x26 // 1 Byte
#define REG_SPEED_CLOSED_LOOP_I   0x27 // 1 Byte

// --- SRAM AREA (Volatile) ---
#define REG_TORQUE_SWITCH         0x28 // 1 Byte
#define REG_ACCELERATION          0x29 // 1 Byte
#define REG_TARGET_LOCATION       0x2A // 2 Bytes
#define REG_RUNNING_TIME          0x2C // 2 Bytes
#define REG_RUNNING_SPEED         0x2E // 2 Bytes
#define REG_TORQUE_LIMIT          0x30 // 2 Bytes
#define REG_WRITE_LOCK            0x37 // 1 Byte
#define REG_PRESENT_LOCATION      0x38 // 2 Bytes
#define REG_PRESENT_SPEED         0x3A // 2 Bytes
#define REG_PRESENT_LOAD          0x3C // 2 Bytes
#define REG_PRESENT_VOLTAGE       0x3E // 1 Byte
#define REG_PRESENT_TEMP          0x3F // 1 Byte
#define REG_ASYNC_WRITE_FLAG      0x40 // 1 Byte
#define REG_SERVO_STATUS          0x41 // 1 Byte
#define REG_MOBILE_SIGN           0x42 // 1 Byte
#define REG_PRESENT_CURRENT       0x45 // 2 Bytes

class STServo;

class  STServoHandler
{
	public:

		STServoHandler(int rxPin, int txPin, int baudrate, uart_port_t uartPort);

		void	Write(uint8_t id, uint8_t address, const uint8_t* data, size_t length);
		void	Read(uint8_t id, uint8_t address, uint8_t* buffer, size_t length);

		void	Write(uint8_t id, uint8_t address, const uint8_t value);
		void	Write(uint8_t id, uint8_t address, const uint16_t value);

		void	Read(uint8_t id, uint8_t address, uint8_t* value);
		void	Read(uint8_t id, uint8_t address, uint16_t* value);

		void	WriteReg(uint8_t id, uint8_t address, const uint8_t value) {
			WriteReg(id, address, &value, 1);
		}

		void	WriteReg(uint8_t id, uint8_t address, const uint16_t value) {
			const uint8_t data[2] = {
				(uint8_t)(value & 0xFF),
				(uint8_t)((value >> 8) & 0xFF)
			};

			WriteReg(id, address, data, 2);
		}

		void	WriteReg(uint8_t id, uint8_t address, const uint8_t* data, size_t length);

		void	SyncWrite(
				std::initializer_list<std::reference_wrapper<STServo>> list,
				uint8_t address,
				const uint8_t* data,
				size_t len
			);

		void	SyncWrite(
				std::initializer_list<std::reference_wrapper<STServo>> list,
				uint8_t address,
				uint8_t value
			) {
			SyncWrite(list, address, &value, 1);
		}

		void	SyncWrite(
				std::initializer_list<std::reference_wrapper<STServo>> list,
				uint8_t address,
				uint16_t value
			) {
			SyncWrite(list, address, (uint8_t*)&value, 2);
		}

		void	SendAction(uint8_t id) {

			uint8_t packet[6];

			packet[0] = 0xFF;
			packet[1] = 0xFF;
			packet[2] = id;
			packet[3] = 2;
			packet[4] = 0x05;
			packet[5] = _CalculateChecksum(packet, 4);

			uart_write_bytes(fUartPort, (const char*)packet, 6);
		}

		void		Update();

	private:

		STServoHandler();
		uint8_t _CalculateChecksum(const uint8_t* packet, size_t length);
		
		int			fRxPin = 0;
		int			fTxPin = 0;
		int			fBaudrate = 1000000;
		uart_port_t	fUartPort = UART_NUM_2;
};

extern STServoHandler STS;

class STServo
{
	public:

		STServo(uint8_t id);

		void		SendAction() {
			STS.SendAction(fId);
		}

		void		SetTorqueEnable(const bool enabled);
		void		SetTargetPosition(const uint16_t position);
		void		SetRunningTime(const uint16_t time);
		void		SetRunningSpeed(const uint16_t speed);
		void		SetAcceleration(const uint8_t acceleration);
		void		SetMaxAngle(const uint16_t angle);
		void		SetMinAngle(const uint16_t angle);
		void		SetOperationMode(const uint8_t mode) {
			STS.Write(fId, REG_OPERATION_MODE, mode);
		}

		void		SetCWArea(const uint8_t area) {
			STS.Write(fId, REG_CW_INSENSITIVE_AREA, area);
		}

		void		SetCCWArea(const uint8_t area) {
			STS.Write(fId, REG_CCW_INSENSITIVE_AREA, area);
		}

		void		SetID(const uint8_t id) {
			STS.Write(fId, REG_ID, id);
		}

		bool		GetTorqueEnable();
		uint16_t	GetTargetPosition();
		uint16_t	GetRunningTime();
		uint16_t	GetRunningSpeed();
		uint8_t		GetAcceleration();
		uint8_t		GetOperationMode() {
			uint8_t value;
			STS.Read(fId, REG_OPERATION_MODE, &value);
			return value;
		}

		uint8_t		GetClockWiseInsensitiveRegion(){
			uint8_t value;
			STS.Read(fId, REG_CW_INSENSITIVE_AREA, &value);
			return value;
		}

		uint8_t GetCounterClockWiseInsensitiveRegion() {
			uint8_t value;
			STS.Read(fId, REG_CCW_INSENSITIVE_AREA, &value);
			return value;
		}

		uint8_t GetID() {
			uint8_t value;
			STS.Read(fId, REG_ID, &value);
			return value;
		}

		uint16_t	GetPresentPosition();
		uint16_t	GetPresentSpeed();
		uint16_t	GetPresentLoad();
		uint8_t		GetPresentVoltage();
		uint8_t		GetPresentTemp();

		void		WriteID(const uint8_t id);
		void		WriteBaudrate(const uint8_t baudrate);

		uint8_t		ReadID();
		uint8_t		ReadBaudrate();
		uint16_t	ReadMinAngleLimit() {
			uint16_t value;

			STS.Read(fId, REG_MIN_ANGLE_LIMIT, &value);
			return value;
		}

		uint16_t	ReadMaxAngleLimit() {
			uint16_t value;

			STS.Read(fId, REG_MAX_ANGLE_LIMIT, &value);
			return value;
		}

		void		LockEPROM() {
			STS.Write(fId, REG_WRITE_LOCK, (uint8_t)1);
		}

		void		UnlockEPROM() {
			STS.Write(fId, REG_WRITE_LOCK, (uint8_t)0);
		}

		uint8_t		id() const { return fId; }


	private:

		STServo();

		uint8_t		fId = 0xFF;
};

