#include <stdint.h>
#include "src\LT_SPI.h"
#include "src\UserInterface.h"
#include "src\LTC681x.h"
#include "src\LTC6812.h"
#include <SPI.h>
#include <freertos\FreeRTOS.h>
#include "driver\can.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2904.h>

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0


/* BLE STUFF */
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic_highv = NULL;
BLECharacteristic* pCharacteristic_lowv = NULL;
BLECharacteristic* pCharacteristic_mode = NULL;
bool deviceConnected = false;
#define SERVICE_UUID        "c2794909-7503-4da2-81cf-89fce41b13bf"
#define CHARACTERISTIC_UUID_HIGHV "665238f7-3cd4-4f18-9fe8-786548a68fdc"
#define CHARACTERISTIC_UUID_LOWV "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_MODE "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/* END BLE STUFF */


void check_error(int error);

/* AFE STUFF */

const uint8_t TOTAL_IC = 1; //!< Number of ICs in the daisy chain

//ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;  //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;  //!< Channel Selection for ADC conversion
const uint8_t NO_OF_REG = REG_ALL; //!< Register Selection

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

cell_asic bms_ic[TOTAL_IC]; //!< Global Battery Variable

bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool gpioBits_a[5] = { false,false,false,false,false }; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool gpioBits_b[4] = { false,false,false,false }; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV = UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool dccBits_a[12] = { false,false,false,false,false,false,false,false,false,false,false,false }; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dccBits_b[7] = { false,false,false,false }; //!< Discharge cell switch //Dcc 0,13,14,15
bool dctoBits[4] = { true,false,true,false }; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool psBits[2] = { false,false }; //!< Digital Redundancy Path Selection//ps-0,1

/* END AFE STUFF */


int8_t fault_timer_highv = 0;
int8_t fault_timer_highv_limit = 5;
float fault_timer_highv_threshold = 4.2;

int8_t fault_timer_lowv = 0;
int8_t fault_timer_lowv_limit = 5;
float fault_timer_lowv_threshold = 3.0;

int8_t fault_timer_sleep_limit = 30;
int8_t fault_timer_sleep = 0;

int64_t deepsleep_time = 10000000; //10s

enum faultflags {
	general_fault =		0b10000000,
	under_temp =		0b01000000,
	over_temp =			0b00100000,
	over_volt =			0b00010000,
	under_volt =		0b00001000,
	over_chargemax =	0b00000100,
	over_discharge =	0b00000010,
	over_charge_dyn =	0b00000001,
	no_fault =			0b00000000
};

enum errorflags {
	error_general = 0b10000000,
	error_can = 0b01000000,
	error_ble = 0b00100000,
	error_none = 0b00000000
};

enum bms_modes {
	mode_run = 2,
	mode_fault = 4,
	mode_startup = 0
};

struct BMS_vars {
	float highv = 0;
	float lowv = 0;
	float packv = 0;
	float cellv[14];
	uint8_t mode = 0; // 0 = sleep, 2 = run, 4 = fault
	uint8_t fault = no_fault;
	uint8_t warning = 0;
	uint8_t error = 0;
	uint8_t soc = 0;
	float pcbtemp = 0;
	float uptime = 0;
};

struct BMS_config_vars {
	int8_t num_cells = 12;
	int16_t bms_refresh_ms = 1000;
};

//int8_t num_cells = 12;

BMS_config_vars BMS_config;
BMS_vars BMS_variable;



class MyServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		deviceConnected = true;
		BLEDevice::startAdvertising();
		Serial.println("--- BLE connect ---");
	};

	void onDisconnect(BLEServer* pServer) {
		deviceConnected = false;
		Serial.println("--- BLE disconnect ---");
	}
};


/*!**********************************************************************
 \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
void setup()
{
	Serial.begin(115200);
	ESP_SPI_init();
	LTC6812_initialise();

	//xTaskCreate(
	//	check_faults,          /* Task function. */
	//	"Check for faults",        /* String with name of task. */
	//	1000,            /* Stack size in bytes. */
	//	NULL,             /* Parameter passed as input of the task */
	//	5,                /* Priority of the task. */
	//	NULL);            /* Task handle. */

	CAN_initialise();

	BLE_initialise();

}

void BLE_initialise() {

	// BLE

	BLEDevice::init("BMS_JL");

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	pCharacteristic_highv = pService->createCharacteristic(
		CHARACTERISTIC_UUID_HIGHV,
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_NOTIFY
	);

	pCharacteristic_lowv = pService->createCharacteristic(
		CHARACTERISTIC_UUID_LOWV,
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_NOTIFY
	);

	pCharacteristic_mode = pService->createCharacteristic(
		CHARACTERISTIC_UUID_MODE,
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_NOTIFY
	);

	// https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
	// Create a BLE Descriptor

	BLE2904 *p2904_Volt_Descriptor = new BLE2904();
	p2904_Volt_Descriptor->setFormat(BLE2904::FORMAT_FLOAT32);
	p2904_Volt_Descriptor->setUnit(0x2728);
	BLE2904 *p2904_Volt_Descriptor2 = new BLE2904();
	p2904_Volt_Descriptor2->setFormat(BLE2904::FORMAT_FLOAT32);
	p2904_Volt_Descriptor2->setUnit(0x2728);
	pCharacteristic_highv->addDescriptor(p2904_Volt_Descriptor);
	pCharacteristic_highv->addDescriptor(new BLE2902());
	pCharacteristic_lowv->addDescriptor(p2904_Volt_Descriptor2);
	pCharacteristic_lowv->addDescriptor(new BLE2902());
	pCharacteristic_mode->addDescriptor(new BLE2902());

	// Start the service
	pService->start();

	// Start advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
	BLEDevice::startAdvertising();
	Serial.println("BLE advertising started...");
}

void CAN_initialise() {
	can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_26, GPIO_NUM_25, CAN_MODE_NORMAL);
	can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
	can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

	//Install CAN driver
	if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		printf("Driver installed\n");
	}
	else {
		printf("Failed to install driver\n");
		return;
	}

	//Start CAN driver
	if (can_start() == ESP_OK) {
		printf("Driver started\n");

		xTaskCreate(
			CAN_output_summary,          /* Task function. */
			"Output summary message",        /* String with name of task. */
			10000,            /* Stack size in bytes. */
			NULL,             /* Parameter passed as input of the task */
			1,                /* Priority of the task. */
			NULL);            /* Task handle. */

	}
	else {
		printf("Failed to start driver\n");
		return;
	}
}

void CAN_output_summary(void * parameter) {
	while (true)
	{
		/*Serial.print("\nHighV: ");
		Serial.print(BMS_variable.highv, 4);
		Serial.println();

		Serial.print("LowV: ");
		Serial.print(BMS_variable.lowv, 4);
		Serial.println();

		Serial.print("Mode: ");
		Serial.print(BMS_variable.mode, 1);
		Serial.println();
		*/
		can_message_t message;
		message.identifier = 0x18FF0600;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = 0;
		message.data[1] = 0;
		message.data[2] = BMS_variable.mode;
		message.data[3] = BMS_variable.fault;
		message.data[4] = (uint16_t) (BMS_variable.highv * 1000) >> 8;
		message.data[5] = (uint16_t) (BMS_variable.highv *1000) & 0xFF;
		message.data[6] = (uint16_t)(BMS_variable.lowv * 1000) >> 8;
		message.data[7] = (uint16_t)(BMS_variable.lowv * 1000) & 0xFF;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}

		message.identifier = 0x18FF0700;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.packv * 10) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.packv * 10) & 0xFF;
		message.data[2] = BMS_variable.soc;
		message.data[3] = 0;
		message.data[4] = 0;
		message.data[5] = 0;
		message.data[6] = 0;
		message.data[7] = 0;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}


		message.identifier = 0x18FF0800;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.pcbtemp * 10) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.pcbtemp * 10) & 0xFF;
		message.data[2] = (uint16_t)(BMS_variable.pcbtemp * 10) >> 8;
		message.data[3] = (uint16_t)(BMS_variable.pcbtemp * 10) & 0xFF;
		message.data[4] = 0;
		message.data[5] = 0;
		message.data[6] = 0;
		message.data[7] = 0;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}

		

		message.identifier = 0x18FF0000;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.cellv[0] * 1000) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.cellv[0] * 1000) & 0xFF;
		message.data[2] = (uint16_t)(BMS_variable.cellv[1] * 1000) >> 8;
		message.data[3] = (uint16_t)(BMS_variable.cellv[1] * 1000) & 0xFF;
		message.data[4] = (uint16_t)(BMS_variable.cellv[2] * 1000) >> 8;
		message.data[5] = (uint16_t)(BMS_variable.cellv[2] * 1000) & 0xFF;
		message.data[6] = (uint16_t)(BMS_variable.cellv[3] * 1000) >> 8;
		message.data[7] = (uint16_t)(BMS_variable.cellv[3] * 1000) & 0xFF;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}


		message.identifier = 0x18FF0100;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.cellv[4] * 1000) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.cellv[4] * 1000) & 0xFF;
		message.data[2] = (uint16_t)(BMS_variable.cellv[5] * 1000) >> 8;
		message.data[3] = (uint16_t)(BMS_variable.cellv[5] * 1000) & 0xFF;
		message.data[4] = (uint16_t)(BMS_variable.cellv[6] * 1000) >> 8;
		message.data[5] = (uint16_t)(BMS_variable.cellv[6] * 1000) & 0xFF;
		message.data[6] = (uint16_t)(BMS_variable.cellv[7] * 1000) >> 8;
		message.data[7] = (uint16_t)(BMS_variable.cellv[7] * 1000) & 0xFF;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}


		message.identifier = 0x18FF0200;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.cellv[8] * 1000) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.cellv[8] * 1000) & 0xFF;
		message.data[2] = (uint16_t)(BMS_variable.cellv[9] * 1000) >> 8;
		message.data[3] = (uint16_t)(BMS_variable.cellv[9] * 1000) & 0xFF;
		message.data[4] = (uint16_t)(BMS_variable.cellv[10] * 1000) >> 8;
		message.data[5] = (uint16_t)(BMS_variable.cellv[10] * 1000) & 0xFF;
		message.data[6] = (uint16_t)(BMS_variable.cellv[11] * 1000) >> 8;
		message.data[7] = (uint16_t)(BMS_variable.cellv[11] * 1000) & 0xFF;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}


		message.identifier = 0x18FF0C00;
		message.flags = CAN_MSG_FLAG_EXTD;
		message.data_length_code = 8;
		message.data[0] = (uint16_t)(BMS_variable.cellv[12] * 1000) >> 8;
		message.data[1] = (uint16_t)(BMS_variable.cellv[12] * 1000) & 0xFF;
		message.data[2] = (uint16_t)(BMS_variable.cellv[13] * 1000) >> 8;
		message.data[3] = (uint16_t)(BMS_variable.cellv[13] * 1000) & 0xFF;
		message.data[4] = (uint16_t)(BMS_variable.cellv[14] * 1000) >> 8;
		message.data[5] = (uint16_t)(BMS_variable.cellv[14] * 1000) & 0xFF;
		message.data[6] = 0;
		message.data[7] = 0;

		//Queue message for transmission
		if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
			//	printf("Message queued for transmission\n");
		}
		else {
			printf("---- CAN error ----\n");
		}

		if (deviceConnected) {
			pCharacteristic_highv->setValue((uint8_t*)&BMS_variable.highv, 4);
			pCharacteristic_highv->notify();
			pCharacteristic_lowv->setValue((uint8_t*)&BMS_variable.lowv, 4);
			pCharacteristic_lowv->notify();
			pCharacteristic_mode->setValue((uint8_t*)&BMS_variable.mode, 4);
			pCharacteristic_mode->notify();
		}


		vTaskDelay(BMS_config.bms_refresh_ms);
	}
	
}

void LTC6812_initialise() {

	LTC6812_clrstat();
	LTC6812_clraux();
	LTC6812_clrcell();
	LTC6812_clrsctrl();

	LTC6812_init_cfg(TOTAL_IC, bms_ic);
	LTC6812_init_cfgb(TOTAL_IC, bms_ic);
	for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
	{
		LTC6812_set_cfgr(current_ic, bms_ic, REFON, ADCOPT, gpioBits_a, dccBits_a, dctoBits, UV, OV);
		LTC6812_set_cfgrb(current_ic, bms_ic, FDRF, DTMEN, psBits, gpioBits_b, dccBits_b);
	}

	LTC6812_reset_crc_count(TOTAL_IC, bms_ic);
	LTC6812_init_reg_limits(TOTAL_IC, bms_ic);


	xTaskCreate(
		LTC6812_read_cell_voltages,          /* Task function. */
		"ReadCellVs",        /* String with name of task. */
		1000,            /* Stack size in bytes. */
		NULL,             /* Parameter passed as input of the task */
		10,                /* Priority of the task. */
		NULL);            /* Task handle. */

}

/*!*********************************************************************
  \brief Main loop
   @return void
***********************************************************************/
void loop()
{


}


void LTC6812_read_cell_voltages(void * parameter)
{
	int8_t error = 0;
	while (1) {
		wakeup_sleep(TOTAL_IC);
		LTC6812_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
		LTC6812_pollAdc();
		wakeup_sleep(TOTAL_IC);
		error = LTC6812_rdcv(NO_OF_REG, TOTAL_IC, bms_ic); // Set to read back all cell voltage registers
		check_error(error);

		wakeup_sleep(TOTAL_IC);
		LTC6812_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
		LTC6812_pollAdc();
		LTC6812_rdstat(0, TOTAL_IC, bms_ic);
		
		BMS_variable.packv = bms_ic[0].stat.stat_codes[0] * 0.0001 * 30;
		
		BMS_variable.pcbtemp = (bms_ic[0].stat.stat_codes[1] * 0.0001 / 0.0076)-276;
		//Serial.println(bms_ic[0].stat.stat_codes[3] * 0.0001, 4);
		//Serial.println(bms_ic[0].stat.stat_codes[4] * 0.0001, 4);
		for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
		{
			Serial.print(bms_ic[0].cells.c_codes[i] * 0.0001, 4);
			BMS_variable.cellv[i] = bms_ic[0].cells.c_codes[i]*0.0001;
			Serial.print(",\t");
		}
		Serial.println();

		float temp = 0;
		for (int i = 0; i < BMS_config.num_cells; i++)
		{
			if (bms_ic[0].cells.c_codes[i] * 0.0001 > temp) {
				temp = bms_ic[0].cells.c_codes[i] * 0.0001;
			}
		}
		BMS_variable.highv = temp;

		for (int i = 0; i < BMS_config.num_cells; i++)
		{
			if (bms_ic[0].cells.c_codes[i] * 0.0001 < temp) {
				temp = bms_ic[0].cells.c_codes[i] * 0.0001;
			}
		}

		BMS_variable.lowv = temp;

		BMS_variable.soc = map((constrain(BMS_variable.lowv * 1000,3300, 4150)), 3300, 4150, 0, 100);

		//esp_deep_sleep(deepsleep_time);
		vTaskDelay(BMS_config.bms_refresh_ms);
	}

}

void check_faults(void * parameter) {
	// high v check
	while (true)
	{
		if (BMS_variable.highv > fault_timer_highv_threshold) {
			fault_timer_highv++;
		}
		else
		{
			fault_timer_highv = 0;
		}
		if (fault_timer_highv >= fault_timer_highv_limit) {
			BMS_variable.mode = 4;
			BMS_variable.fault = BMS_variable.fault | general_fault | over_volt;
			Serial.println("----- HIGH V FAULT ----");
		}


		if (BMS_variable.lowv < fault_timer_lowv_threshold) {
			fault_timer_lowv++;
		}
		else
		{
			fault_timer_lowv = 0;
		}
		if (fault_timer_lowv >= fault_timer_lowv_limit) {
			BMS_variable.mode = 4;
			BMS_variable.fault = BMS_variable.fault | general_fault | under_volt;
			Serial.println("----- LOW V FAULT ----");
		}

		if (BMS_variable.mode == 4) {
			fault_timer_sleep++;
			if (fault_timer_sleep > fault_timer_sleep_limit) {
				Serial.println("----- GOING TO SLEEP ----");
				esp_deep_sleep_start();
			}
		}


		if(BMS_variable.mode != 4)
		{
			BMS_variable.mode = 2;
		}
		BMS_variable.uptime += (1000 / BMS_config.bms_refresh_ms);
		vTaskDelay(BMS_config.bms_refresh_ms);
	}

}

void LTC6812_openwire_check(void * parameter)
{
	while (1) {
		wakeup_sleep(TOTAL_IC);
		delay(9800);
		LTC6812_run_openwire_multi(TOTAL_IC, bms_ic);
	}
}


/*!****************************************************************************
 \brief Function to check error flag and print PEC error message
 @return void
 *****************************************************************************/
void check_error(int error)
{
	if (error == -1)
	{
		Serial.println(F("A PEC error was detected in the received data"));
	}
}