/*
 ===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif
#define TICKRATE_HZ (10)	/* 10 ticks per second */

/* EEPROM Address used for storage */
#define EEPROM_ADDRESS      0x00000100

/* Write count */
#define IAP_NUM_BYTES_TO_READ_WRITE 32

/* Tag for checking if a string already exists in EEPROM */
#define CHKTAG          "NxP"
#define CHKTAG_SIZE     3

/* ASCII ESC character code */
#define ESC_CHAR        27

/* Read/write buffer (32-bit aligned) */
uint32_t buffer[IAP_NUM_BYTES_TO_READ_WRITE / sizeof(uint32_t)];

/* Test string for no DEBUG */
#define TESTSTRING "12345678"

#include <cr_section_macros.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "heap_lock_monitor.h"
#include "retarget_uart.h"
#include "ModbusRegister.h"
#include "DigitalIoPin.h"
#include "LiquidCrystal.h"
#include "string.h"
//static volatile int counter;
static DigitalIoPin SW1(0, 8, DigitalIoPin::input, true);
static DigitalIoPin SW2(1, 8, DigitalIoPin::input, true);

static QueueHandle_t interruptQueue,q1;
static QueueHandle_t xCO2Queue = NULL, xTempQueue = NULL, queue;
static int co2Reading;
uint16_t co2Counter;

static void idle_delay() {
	vTaskDelay(1);
}

typedef enum {
	temperature, humidity, carbonDioxide
} Sensor_t;

typedef struct {
	uint16_t Value;
	Sensor_t SensorType;
} Data;

typedef struct{
	int buffer;
	uint16_t counter;
}encoderCounter;

struct SensorConfig {
	LpcUart *uart;
	Fmutex *mutex;
};

Fmutex *guard = new Fmutex();

extern "C" {
//B INTERRUPT
void PIN_INT0_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	int inc = 1;
	int dec = -inc;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));

	if (Chip_GPIO_GetPinState(LPC_GPIO, 0, 6)) {
		xQueueSendFromISR(queue, (void* )&inc, &xHigherPriorityTaskWoken);
	} else {
		xQueueSendFromISR(queue, (void* )&dec, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
}

//ISR for rotary encoder button
extern "C" {
void PIN_INT1_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));

	int val = 20;

	xQueueSendFromISR(queue, (void* )&val, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
}

void btn_init(void);

/* Sets up system hardware */
static void prvSetupHardware(void) {
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats(void) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
//
void vButtonTask(void *pvParams) {

	queue = xQueueCreate(50, sizeof(int));
	vQueueAddToRegistry(queue, "ButtonsQueue");
	unsigned int counter = 700;
	unsigned int maximum = 1000;
	btn_init();
	encoderCounter encodercounter;

	int buffer = 0;

	while (true) {
		if (xQueueReceive(queue, &buffer, (TickType_t) 5000)) {
			if (buffer == 20) {
				counter = 700;
				co2Counter = 700;
				xQueueSend(interruptQueue, (void* )&encodercounter,
						portMAX_DELAY);

			} else {
				counter += buffer;
				printf("Buffer: %d, count: %d\r\n", buffer, counter);
				encodercounter = { buffer, counter };
				if(counter > maximum){
					counter = maximum;
					co2Counter = maximum;
				}
				else if(counter > 700) {
					co2Counter = counter;
				} else {
					counter = 700;
					co2Counter = 700;
				}
				xQueueSend(interruptQueue, (void* )&encodercounter,
						portMAX_DELAY);
				vTaskDelay(5);
			}
		}

	}
}

static void TempHumidReadingtask(void *params) {
	(void) params;
//	SensorConfig *t = static_cast<SensorConfig *>(params);
	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister RH(&node3, 256, true);
	ModbusRegister TEMP(&node3, 257, true);

	DigitalIoPin sw_a2(1, 8, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a3(0, 5, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a4(0, 6, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a5(0, 7, DigitalIoPin::pullup, true);

	DigitalIoPin *rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
	DigitalIoPin *en = new DigitalIoPin(0, 9, DigitalIoPin::output);
	DigitalIoPin *d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
	DigitalIoPin *d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
	DigitalIoPin *d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
	DigitalIoPin *d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
	LiquidCrystal *lcd = new LiquidCrystal(rs, en, d4, d5, d6, d7);
	// configure display geometry
	lcd->begin(16, 2);
	// set the cursor to column 0, line 1
	// (note: line 1 is the second row, since counting begins with 0):
	lcd->setCursor(0, 0);

	while (true) {

		float rh, temp;
		vTaskDelay(1000);
		guard->lock();
		rh = RH.read() /10 ;
		vTaskDelay(13);
		temp = TEMP.read() /10;
		vTaskDelay(13);
		guard->unlock();
		Data humidData, tempData;
//		printf("DATA : %.1f, %.1f \r\n",rh,temp);
		humidData = { rh, humidity };
		tempData = { temp, temperature };
		// Print a message to the LCD.
		xQueueSend(xTempQueue, (void* ) &humidData, portMAX_DELAY);
		xQueueSend(xTempQueue, (void* ) &tempData, portMAX_DELAY);

	}

}

static void CO2ReadingTask(void *params) {

	ModbusMaster node4(240); // Create modbus object that connects to slave id 241 (HMP60)
	node4.begin(9600); // all nodes must operate at the same speed!
	node4.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister CO2(&node4, 256, true);
	Data co2data;

	while (true) {
		unsigned int co2;
		vTaskDelay(1000);
		guard->lock();
		vTaskDelay(10);
		co2 = CO2.read();
		vTaskDelay(10);
		guard->unlock();
		co2data.Value = co2;
		co2Reading = co2;
		co2data.SensorType = carbonDioxide;
		// Print a message to the LCD.
		DigitalIoPin relay(0, 27, DigitalIoPin::output);
		if(co2Reading > co2Counter || co2Reading > 1000 || co2Reading < 700 ){
			relay.write(0);
			printf("RELAY IS CLOSED \r\n");
		}else if(co2Reading < co2Counter){
			relay.write(1);
			printf("RELAY IS OPENED \r\n");
		}else if(co2Reading == co2Counter){
			relay.write(0);
			printf("RELAY IS OPENED \r\n");
		}
		xQueueSend(xCO2Queue, (void* ) &co2data, portMAX_DELAY);
	}
}

static void ShowString(char *str);
static uint32_t MakeString(uint8_t *str,uint16_t byte);

static void vPrintFromQueue(void *pvParameters) {

	DigitalIoPin *rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
	DigitalIoPin *en = new DigitalIoPin(0, 9, DigitalIoPin::output);
	DigitalIoPin *d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
	DigitalIoPin *d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
	DigitalIoPin *d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
	DigitalIoPin *d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
	LiquidCrystal *lcd = new LiquidCrystal(rs, en, d4, d5, d6, d7);

//	TaskData *t = static_cast<TaskData *>(pvParameters);
//	char buffer[128];
	QueueHandle_t xQueueThatContainsData;
	Data data;
	encoderCounter xcounter;
	while (1) {
		xQueueThatContainsData = (QueueHandle_t) xQueueSelectFromSet(q1,
		portMAX_DELAY);
		if (xQueueThatContainsData == xCO2Queue) {
			xQueueReceive(xQueueThatContainsData, &data, 0);
			if (data.SensorType == carbonDioxide) {
				char buf[64];
				snprintf(buf, 64, "CO2:%d", data.Value);
				lcd->setCursor(0, 1);
				lcd->print(buf);
				printf("%d\n", data.Value);
			}

		} else if (xQueueThatContainsData == xTempQueue) {
			xQueueReceive(xQueueThatContainsData, &data, 0);
			if (data.SensorType == humidity) {
				char buf[64];
				float rH = (float) data.Value;
				snprintf(buf, 64, "rH:%.1f%%", rH);
				lcd->setCursor(0, 0);
				lcd->print(buf);
				printf("%.1f\n", rH);
			} else if (data.SensorType == temperature) {
				char buf[64];
				float temp = (float) data.Value;
				snprintf(buf, 64, "T:%.1fC", temp);
				lcd->setCursor(8, 0);
				lcd->print(buf);
				printf("%.1f\n", temp);
			}

		}else if(xQueueThatContainsData == interruptQueue){
			xQueueReceive(xQueueThatContainsData, &xcounter, 0);
			char str[32];
			int test = xcounter.buffer + 1;
			 // CO2 relay
			printf("%d\r\n",test);
//			if(co2 + xcounter.buffer > 200){
//				if(test == 0){
//					co2 -= 1;
//				}else{
//					co2 += 1;
//				}
//			}

//			snprintf(buf,64,"Buffer: %d, count: %d\r\n", counter.buffer, counter.counter);
			uint8_t ret_code;
			vTaskSuspendAll();
			snprintf(str, 32, "TAR:%d", co2Counter);
//			uint16_t *co2pt = &co2Counter;
//			uint8_t *ptr = (uint8_t *) co2pt;
//			uint8_t *y = (uint8_t) ((co2pt) >> 8 );
//			MakeString(ptr,co2Counter);
//			ret_code = Chip_EEPROM_Write(EEPROM_ADDRESS, ptr, IAP_NUM_BYTES_TO_READ_WRITE);
//			ShowString((char *) ptr);
//			xTaskResumeAll();
			lcd->setCursor(8, 1);

			lcd->print(str);
		}

	}
}
extern "C" {
void vStartSimpleMQTTDemo(void); // ugly - should be in a header
}

extern "C" {
void vStartSimpleMQTTDemo(void); // ugly - should be in a header
}
static void ShowString(char *str);
static uint32_t MakeString(uint8_t *str);

int main(void) {
#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
#endif
#endif
	uint8_t ret_code;
	retarget_init();
	heap_monitor_setup();
	uint8_t *ptr = (uint8_t *) buffer;
	prvSetupHardware();

	heap_monitor_setup();

//	MakeString(ptr);

	/* Data to be written to EEPROM */
//	ret_code = Chip_EEPROM_Write(EEPROM_ADDRESS, ptr, IAP_NUM_BYTES_TO_READ_WRITE);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);

//	ret_code = Chip_EEPROM_Read(EEPROM_ADDRESS, ptr, IAP_NUM_BYTES_TO_READ_WRITE);
//
//		/* Error checking */
//		if (ret_code != IAP_CMD_SUCCESS) {
//			DEBUGOUT("Command failed to execute, return code is: %x\r\n", ret_code);
//		}
//
//		/* Check and display string if it exists */
//		ShowString((char *) ptr);

	xCO2Queue = xQueueCreate(1, sizeof(int));
	xTempQueue = xQueueCreate(1, sizeof(int));
	interruptQueue = xQueueCreate(1, sizeof(int));
	q1 = xQueueCreateSet(5);
	xQueueAddToSet(xCO2Queue, q1);
	xQueueAddToSet(xTempQueue, q1);
	xQueueAddToSet(interruptQueue, q1);

	xTaskCreate(vButtonTask, "Rotary Task",
	configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY +4UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(TempHumidReadingtask, "TempHumidReadingtask",
	configMINIMAL_STACK_SIZE + 256, NULL, (tskIDLE_PRIORITY + 2UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(CO2ReadingTask, "CO2ReadingTask",
	configMINIMAL_STACK_SIZE + 512, NULL, (tskIDLE_PRIORITY + 2UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(vPrintFromQueue, "vPrintFromQueue",
	configMINIMAL_STACK_SIZE + 512, NULL, (tskIDLE_PRIORITY + 3UL),
			(TaskHandle_t*) NULL);

	vStartSimpleMQTTDemo();

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

void btn_init(void) {
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	/* Enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);

	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);

	/* Configure interrupt channel for the GPIO pin in INMUX block */
	Chip_INMUX_PinIntSel(0, 0, 5);
	Chip_INMUX_PinIntSel(1, 1, 8);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0));

	//NVIC_SetPriority(PIN_INT0_IRQn, );
	//NVIC_SetPriority(PIN_INT1_IRQn, );

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);

}

static void ShowString(char *str) {
	int stSize;

	/* Is data tagged with check pattern? */
	if (strncmp(str, CHKTAG, CHKTAG_SIZE) == 0) {
		/* Get next byte, which is the string size in bytes */
		stSize = (uint32_t) str[3];
		if (stSize > 32) {
			stSize = 32;
		}

		/* Add terminator */
		str[4 + stSize] = '\0';

		/* Show string stored in EEPROM */
		DEBUGSTR("Stored string found in EEEPROM\r\n");
		DEBUGSTR("-->");
		DEBUGSTR((char *) &str[4]);
		DEBUGSTR("<--\r\n");
	}
	else {
		DEBUGSTR("No string stored in the EEPROM\r\n");
	}
}

/* Get a string to save from the UART */
static uint32_t MakeString(uint8_t *str, uint16_t byte)
{
	int index;
	char strOut[2];

	/* Get a string up to 32 bytes to write into EEPROM */
	DEBUGSTR("\r\nEnter a string to write into EEPROM\r\n");
	DEBUGSTR("Up to 32 bytes in length, press ESC to accept\r\n");

	/* Setup header */
	strncpy((char *) str, CHKTAG, CHKTAG_SIZE);

#if defined(DEBUG_ENABLE)
	/* Read until escape, but cap at 32 characters */
	index = 0;
	strOut[1] = '\0';
	while ((index < 32) && (byte != ESC_CHAR)) {
		if (byte != EOF) {
			strOut[0] = str[4 + index] = (uint8_t) byte;
			DEBUGSTR(strOut);
			index++;
		}

		byte = DEBUGIN();
	}
#else
	/* Suppress warnings */
	(void) byte;
	(void) strOut;

	/* Debug input not enabled, so use a pre-setup string */
	strcpy((char *) &str[4], TESTSTRING);
	index = strlen(TESTSTRING);
#endif

	str[3] = (uint8_t) index;

	return (uint32_t) index;
}

