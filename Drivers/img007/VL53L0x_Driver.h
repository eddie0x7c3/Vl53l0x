#ifndef VL53L0x_Driver_H_
#define VL53L0x_Driver_H_


#include "vl53l0x_api.h"



//******************************|  interupt types  |*********************************
#define INTERRUPT_NONE 			VL53L0X_GPIOFUNCTIONALITY_OFF
#define INTERRUPT_CROSSED_LOW 	VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW
#define INTERRUPT_CROSSED_HIGH 	VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH
#define INTERRUPT_WINDOW		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT

//******************************|  interupt polarity  |*********************************
#define INTERRUPT_POLARITY_LOW VL53L0X_INTERRUPTPOLARITY_LOW
#define INTERRUPT_POLARITY_HIGH VL53L0X_INTERRUPTPOLARITY_HIGH

//******************************|  Ranging Modes  |*************************************
#define HIGH_ACCURACY  		0x01 // 200ms
#define HIGH_SPEED  		0x02 // 20ms
#define LONG_RANGE  		0x03 // 33ms
#define STANDARD 			0x04 // 30ms

//**************************************************************************************
//******************************|  interupt polarity  |*********************************
void VL53L0x_init(VL53L0X_Dev_t *Device);
void VL53L0x_power_off(VL53L0X_Dev_t *Device);
void VL53L0x_power_on(VL53L0X_Dev_t *Device);
FixPoint1616_t get_reflectance(VL53L0X_Dev_t *Device); //make the return type be floating by dividing by 65535
FixPoint1616_t get_ambient(VL53L0X_Dev_t *Device); //same as above
//******************************|  Data related Functions  |*****************************
uint16_t get_distance_mm(VL53L0X_Dev_t *Device); //******* might have a fractional part
//--------interrupt
void interrupt_config(VL53L0X_Dev_t *Device,uint8_t interrupt_type, uint8_t interrupt_polarity, uint8_t interrupt_low_val, uint8_t interrupt_high_val);//*****need smore parameters


//******************************|  interupt polarity  |*********************************
void clear_interrupt(VL53L0X_Dev_t *Device); //****needs more parameters










#endif

/*

	 sprintf( (char*)buff, "%d, %d, %.2f, %.2f\n\r", RangingData.RangeStatus, RangingData.RangeMilliMeter,
             ( RangingData.SignalRateRtnMegaCps / 65536.0 ), RangingData.AmbientRateRtnMegaCps / 65336.0 );

*/


