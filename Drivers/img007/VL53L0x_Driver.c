#include "VL53L0x_Driver.h"

#define VL53L0X_I2C_ADDR 0x52 ///< Default sensor I2C address





VL53L0X_Version_t Version;
VL53L0X_Version_t *pVersion = &Version;
VL53L0X_DeviceInfo_t DeviceInfo;
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

VL53L0X_RangingMeasurementData_t RangingData;


 
typedef struct {
    VL53L0X_DevData_t Data;               /*!< embed ST Ewok Dev  data as "Data"*/

    /*!< user specific field */
    uint8_t   I2cDevAddr;                /*!< i2c device address user specific field */
    uint8_t   comms_type;                /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
    uint16_t  comms_speed_khz;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */
	I2C_HandleTypeDef *I2cHandle;
	GPIO_TypeDef* power_pin_port;
	uint16_t power_pin;
} VL53L0x_setup;

//******************************|  Data related Functions  |*****************************
uint16_t get_distance_mm(VL53L0X_Dev_t *Device)
{

	VL53L0X_PerformSingleMeasurement(Device); //faster
	VL53L0X_GetRangingMeasurementData(Device, &RangingData);

	return RangingData.RangeMilliMeter;
}
 float get_reflectance(VL53L0X_Dev_t *Device)
 {
	VL53L0X_PerformSingleMeasurement(Device); //faster
	VL53L0X_GetRangingMeasurementData(Device, &RangingData);
	return (RangingData.SignalRateRtnMegaCps / 65536.0) ;
 
 }
 float get_ambient(VL53L0X_Dev_t *Device)
 {
	VL53L0X_PerformSingleMeasurement(Device); //faster
	VL53L0X_GetRangingMeasurementData(Device, &RangingData);
	return (RangingData.AmbientRateRtnMegaCps / 65336.0 );
 
 }
 uint8_t get_status(VL53L0X_Dev_t *Device)
 {
	return (RangingData.RangeStatus) ;
 }
 
//******************************|  interupt   |*******************************************
void clear_interrupt(VL53L0X_Dev_t *Device)
{
	VL53L0X_ClearInterruptMask(Device,0x00);

}
void interrupt_config(VL53L0X_Dev_t *Device,uint8_t interrupt_type, uint8_t interrupt_polarity, uint8_t interrupt_low_val, uint8_t interrupt_high_val) //************************** finish and test
{

	VL53L0X_SetGpioConfig(Device, 0, VL53L0X_DEVICEMODE_SINGLE_RANGING,interrupt_type,interrupt_polarity);
	VL53L0X_SetInterruptThresholds(Device,VL53L0X_DEVICEMODE_SINGLE_RANGING,interrupt_low_val<<16,interrupt_high_val<<16);
}

//******************************|  init / power  |*********************************
void VL53L0x_init(VL53L0X_Dev_t *Device)
{
		//turn on power pin
		HAL_GPIO_WritePin(Device->power_pin_port, Device->power_pin, GPIO_PIN_SET);

		/*	upon reset device has defualt address, so in order to communicate to it and configure
		 *	a new address we have to first talk to it using the default address.
		 *	so ill store the user defined address in a temp variable, and switch the I2cDevAddr
		 *	in the user setup sruct back to default address
		*/
		uint8_t newAddressHolder = Device->I2cDevAddr; //store user address 
		Device->I2cDevAddr = VL53L0X_I2C_ADDR; // switch the setup struct to defualt address


		//********| these init functions must be called in this exact order pg7 of user manual |***************************
		//VL53L0X_WaitDeviceBooted(Device);
		VL53L0X_DataInit(Device);	


		VL53L0X_SetDeviceAddress(Device,newAddressHolder); //now set new address 
		Device->I2cDevAddr = newAddressHolder; //store user address back into the setup struct

		// leave as is
		VL53L0X_GetDeviceInfo( Device, &DeviceInfo );	
		VL53L0X_StaticInit(Device);
		
		//interrupt setup
		interrupt_config(Device,Device->interrupt_type,Device->interrupt_polarity,Device->interrupt_low_val,Device->interrupt_high_val);	
		
		
		VL53L0X_PerformRefSpadManagement(Device, &refSpadCount, &isApertureSpads);
		VL53L0X_PerformRefCalibration( Device, &VhvSettings, &PhaseCal );    
		
		
		
		switch (Device->ranging_mode)
		{
			case HIGH_ACCURACY:
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(18*65536));
				VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Device,200000);				
			break;
			
			case HIGH_SPEED:
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(32*65536));
				VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Device,20000);
			break;
			
			case LONG_RANGE:
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
				VL53L0X_SetLimitCheckValue(Device,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));
				VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Device,33000);
				VL53L0X_SetVcselPulsePeriod(Device,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
				VL53L0X_SetVcselPulsePeriod(Device,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);			
			break;
			
			case STANDARD:
					//deafualt values
			break;
			
			default: 
					//default values			
			break;	
		
		}
		VL53L0X_SetDeviceMode(Device , VL53L0X_DEVICEMODE_SINGLE_RANGING); 
		

}
void VL53L0x_power_off(VL53L0X_Dev_t *Device)
{
    HAL_GPIO_WritePin(Device->power_pin_port, Device->power_pin, GPIO_PIN_RESET);
}
void VL53L0x_power_on(VL53L0X_Dev_t *Device)
{	 
		VL53L0x_init(Device);	
}



