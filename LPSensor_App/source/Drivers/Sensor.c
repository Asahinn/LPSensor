/*
 * Sensor.c
 *
 *  Created on: 21 May 2022
 *      Author: Alican S.
 */

#include <stdbool.h>
#include "fsl_spi.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_swm.h"
#include "fsl_iocon.h"
#include <Drivers/Sensor.h>
#include <Drivers/LpTimer.h>
#include <Bme280/bme280.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define    SENSOR_SPI_MASTER          	SPI0
#define    SENSOR_CLK_SRC             	kCLOCK_MainClk
#define    SENSOR_SPI_MASTER_CLK_FREQ 	CLOCK_GetFreq(SENSOR_CLK_SRC)
#define    SENSOR_SPI_MASTER_BAUDRATE 	10000000U
#define	   SENSOR_SPI_MASTER_SSEL    	kSPI_Ssel0Assert

#define    SENSOR_ICON_INDEX_SPI_MOSI   IOCON_INDEX_PIO0_22
#define    SENSOR_ICON_INDEX_SPI_MISO   IOCON_INDEX_PIO0_21
#define    SENSOR_ICON_INDEX_SPI_SCK    IOCON_INDEX_PIO0_20
#define    SENSOR_ICON_INDEX_SPI_SEL    IOCON_INDEX_PIO0_19

#define    SENSOR_SWM_MOVABLE_SPI_MOSI  kSWM_SPI0_SCK
#define    SENSOR_SWM_MOVABLE_SPI_MISO  kSWM_SPI0_MOSI
#define    SENSOR_SWM_MOVABLE_SPI_SCK   kSWM_SPI0_MISO
#define    SENSOR_SWM_MOVABLE_SPI_SEL   kSWM_SPI0_SSEL0


#define    SENSOR_SWM_PIN_SPI_MOSI  	kSWM_SPI0_SCK
#define    SENSOR_SWM_PIN_SPI_MISO  	kSWM_SPI0_MOSI
#define    SENSOR_SWM_PIN_SPI_SCK   	kSWM_SPI0_MISO
#define    SENSOR_SWM_PIN_SPI_SEL   	kSWM_SPI0_SSEL0

#define    SENSOR_DATARATE_MSEC         1000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int8_t BME280_Sensor_Init				(struct Sensor_TypeStruct *Obj,struct bme280_dev *dev);
static int8_t BME280_Sensor_Set_ForcedMode		(struct bme280_dev *dev);
static int8_t BME280_Sensor_ForcedMode_GetData	(struct bme280_dev *dev,struct Sensor_TypeStruct *Obj);
static void   BME280_Sensor_GetandPrintData		(struct bme280_data *comp_data,struct Sensor_TypeStruct *Obj);
static int8_t Init_SensorChannels				(struct Sensor_TypeStruct *Obj,bool Enable,enum SensorDatatype_TypEnm Datatyp,uint8_t ChNum);
static void   Sensor_PreparePins_SPI			(void);
static void   Sensor_Init_SPI				    (void);
int8_t 		  Sensor_Write_SPI 					(uint8_t RegAddr,const uint8_t *Data,uint16_t Len,void *intf_ptr);
int8_t 		  Sensor_Read_SPI 					(uint8_t RegAddr,uint8_t *Data,uint16_t Len,void *intf_ptr);
void  		  Sensor_Delay_ms					(uint32_t Period, void *intf_ptr);
/*******************************************************************************
 * Variables
 ******************************************************************************/
#define SPI0_BUFFER_SIZE (128)
static uint8_t SPI0_TxBuf[SPI0_BUFFER_SIZE];
static uint8_t SPI0_RxBuf[SPI0_BUFFER_SIZE];
spi_transfer_t xfer = {0};

struct Sensor_TypeStruct  SensorObj;
struct bme280_dev 		  Bme280Obj;
static uint32_t           Conv_Delay;
static uint32_t           Idle_Delay;

/*******************************************************************************
 * Code
 ******************************************************************************/
/***********************************************************
 * Function Name  : LpSensorApp_Loop
 * Description    : Low Power Sensor Application main Loop
 * Input          :
 * Return         :
 ***********************************************************/
void LpSensorApp_Loop(void){
	int8_t  ret =0 ;
	ret = BME280_Sensor_Init(&SensorObj,&Bme280Obj);
	if(ret){
		PRINTF("ERR: BME280 Init Error \r\n");
		return;
	}
	 while(1){
		 BME280_Sensor_Set_ForcedMode(&Bme280Obj);
		 Idle_Delay =  SENSOR_DATARATE_MSEC - Conv_Delay;
		 BME280_Sensor_ForcedMode_GetData(&Bme280Obj,&SensorObj);
		 Delay_ms(Idle_Delay);
	 }
}
/***********************************************************
 * Function Name  : BME280_Sensor_Init
 * Description    : Initialize the Sensor
 * Input          :	Obj : Structure instance of Sensor
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
static int8_t BME280_Sensor_Init(struct Sensor_TypeStruct *Obj,struct bme280_dev *dev){
	int8_t  ret      = BME280_OK;
	uint8_t dev_addr = 0;
	Obj->Enable     = true;
	Obj->Interface  = SENSOR_SPI_INTF;
#ifdef BME280_FLOAT_ENABLE
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_DOUBLE,SENSOR_CH_TEMPERATURE);
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_DOUBLE,SENSOR_CH_HUMIDITY);
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_DOUBLE,SENSOR_CH_PRESSURE);
#else
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_INT,SENSOR_CH_TEMPERATURE);
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_UINT,SENSOR_CH_HUMIDITY);
	ret |= Init_SensorChannels(Obj,true,SENSOR_DATATYPE_UINT,SENSOR_CH_PRESSURE);
#endif
	if(ret != BME280_OK){
		return 1;
	}
	switch(Obj->Interface){
		case SENSOR_SPI_INTF	:
			Sensor_PreparePins_SPI();
			Sensor_Init_SPI();
			break;
		case SENSOR_I2C_INTF	:
			PRINTF("ERR: I2C Interface not implement yet.\r\n");
			return 1;
		case SENSOR_UART_INTF	:
			PRINTF("ERR: UART Interface not support.\r\n");
			return 1;
		case SENSOR_INTF_MAX	:
		default					:
			PRINTF("ERR: Wrong Interface.\r\n");
			return 1;
	}
	ret=BME280_OK;
	dev->intf_ptr 	= &dev_addr;
	dev->intf 		= BME280_SPI_INTF;
	dev->read 		= (bme280_read_fptr_t)Sensor_Read_SPI;
	dev->write 		= (bme280_write_fptr_t)Sensor_Write_SPI;
	dev->delay_us 	= Sensor_Delay_ms;

	ret = bme280_init(dev);
	return ret;

}
/***********************************************************
 * Function Name  : BME280_Sensor_Set_ForcedMode
 * Description    : Initialize the Sensor
 * Input          :	dev : Structure instance of bme280
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
static int8_t BME280_Sensor_Set_ForcedMode(struct bme280_dev *dev){
    int8_t rslt;
    uint8_t settings_sel;
    /* Recommended mode of operation: Weather Monitoring */
    /*
     * Current Consumption 0.16uA @1Hz
     * */
    dev->settings.osr_h 	= BME280_OVERSAMPLING_1X;
    dev->settings.osr_p 	= BME280_OVERSAMPLING_1X;
    dev->settings.osr_t 	= BME280_OVERSAMPLING_1X;
    dev->settings.filter 	= BME280_FILTER_COEFF_OFF;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt 		 = bme280_set_sensor_settings(settings_sel, dev);

	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     * and the oversampling configuration. */
    Conv_Delay = bme280_cal_meas_delay(&dev->settings);
    return rslt;
}
/***********************************************************
 * Function Name  : BME280_Sensor_ForcedMode_GetData
 * Description    :
 * Input          :	dev : Structure instance of bme280
 * 					Obj : Structure instance of Sensor
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
static int8_t BME280_Sensor_ForcedMode_GetData(struct bme280_dev *dev,struct Sensor_TypeStruct *Obj){
    int8_t rslt=0;
    struct bme280_data comp_data;

    bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
    /* Wait for the measurement to complete */
    dev->delay_us(Conv_Delay, dev->intf_ptr);
    rslt=bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

    BME280_Sensor_GetandPrintData(&comp_data,Obj);
    return rslt;
}
/***********************************************************
 * Function Name  : BME280_Sensor_PrintData
 * Description    :
 * Input          :	bme280_data : Structure instance of bme280 data
 * 					Obj         : Structure instance of Sensor
 * Return         :
 ***********************************************************/
static void BME280_Sensor_GetandPrintData(struct bme280_data *comp_data,struct Sensor_TypeStruct *Obj){
	#ifdef BME280_FLOAT_ENABLE
		PRINTF("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
        Obj->Sensor[SENSOR_CH_TEMPERATURE].Data.d32_Val = comp_data->temperature;
        Obj->Sensor[SENSOR_CH_HUMIDITY].Data.d32_Val 	= comp_data->pressure;
        Obj->Sensor[SENSOR_CH_PRESSURE].Data.d32_Val 	= comp_data->humidity;
	#else
        PRINTF("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
        Obj->Sensor[SENSOR_CH_TEMPERATURE].Data.i32_Val = comp_data->temperature;
        Obj->Sensor[SENSOR_CH_HUMIDITY].Data.ui32_Val 	= comp_data->pressure;
        Obj->Sensor[SENSOR_CH_PRESSURE].Data.ui32_Val 	= comp_data->humidity;
	#endif
}
/***********************************************************
 * Function Name  : Init_SensorChannels
 * Description    : Initialize the Sensor Ch
 * Input          :	Obj    : Structure instance of Sensor
 * 					Enable : Ch Enable/Disable
 * 					Datatyp: Type of Sensor Data
 * 					ChNum  : Ch Number
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
static int8_t Init_SensorChannels(struct Sensor_TypeStruct *Obj,bool Enable,enum SensorDatatype_TypEnm Datatyp,uint8_t ChNum){
	if(Obj != NULL){
		PRINTF("ERR: Could not init selected sensor channel : %d. Sensor Object is NULL \r\n",ChNum);
		return 1;
	}
	if(ChNum >= MAX_SENSOR_CH_NUMBER){
		PRINTF("ERR: Could not init selected sensor channel : %d. Selected ch higher than max ch number \r\n",ChNum);
		return 1;
	}
	if(Datatyp >= SENSOR_DATATYPE_MAX){
		PRINTF("ERR: Could not init selected sensor channel : %d. Unknown data type \r\n",ChNum);
		return 1;
	}
	Obj->Sensor[ChNum].Enable 		= Enable;
	Obj->Sensor[ChNum].Type   		= ChNum;
	Obj->Sensor[ChNum].DataType   	= Datatyp;
#ifdef BME280_FLOAT_ENABLE
	Obj->Sensor[ChNum].Data.d32_Val = 0;
#else
	Obj->Sensor[ChNum].Data.ui32_Val = 0;
#endif
	return 0;
}
/***********************************************************
 * Function Name  : Sensor_PreparePins_SPI
 * Description    : Prepare Sensor Pins for SPI
 * Input          :	None
 * Return         : None
 ***********************************************************/
static void Sensor_PreparePins_SPI(void){
    /* Enables clock for IOCON.: enable */
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* Enables clock for switch matrix.: enable */
    CLOCK_EnableClock(kCLOCK_Swm);

    const uint32_t IOCON_INDEX_PIOX_X_config = \
    (
     /* Selects pull-up function */
	 IOCON_PIO_MODE_PULLUP |
	 /* Enable hysteresis */
	 IOCON_PIO_HYS_EN |
	 /* Input not invert */
	 IOCON_PIO_INV_DI |
	 /* Disables Open-drain function */
	 IOCON_PIO_OD_DI |
	 /* Bypass input filter */
	 IOCON_PIO_SMODE_BYPASS |
	 /* IOCONCLKDIV0 */
	 IOCON_PIO_CLKDIV0);

    /* SPI pins is configured as  */
    IOCON_PinMuxSet(IOCON, SENSOR_ICON_INDEX_SPI_SEL, IOCON_INDEX_PIOX_X_config);
    /* SPI pins is configured as  */
    IOCON_PinMuxSet(IOCON, SENSOR_ICON_INDEX_SPI_SCK, IOCON_INDEX_PIOX_X_config);
    /* SPI pins is configured as  */
    IOCON_PinMuxSet(IOCON, SENSOR_ICON_INDEX_SPI_MISO, IOCON_INDEX_PIOX_X_config);
    /* SPI pins is configured as  */
    IOCON_PinMuxSet(IOCON, SENSOR_ICON_INDEX_SPI_MOSI, IOCON_INDEX_PIOX_X_config);

    /* SPI0_SCK connect to P0_20 */
    SWM_SetMovablePinSelect(SWM0, SENSOR_SWM_MOVABLE_SPI_SCK, SENSOR_SWM_PIN_SPI_SCK);
    /* SPI0_MOSI connect to P0_22 */
    SWM_SetMovablePinSelect(SWM0, SENSOR_SWM_MOVABLE_SPI_MOSI, SENSOR_SWM_PIN_SPI_MOSI);
    /* SPI0_MISO connect to P0_21 */
    SWM_SetMovablePinSelect(SWM0, SENSOR_SWM_MOVABLE_SPI_MISO, SENSOR_SWM_PIN_SPI_MISO);
    /* SPI0_SSEL0 connect to P0_19 */
    SWM_SetMovablePinSelect(SWM0, SENSOR_SWM_MOVABLE_SPI_SEL, SENSOR_SWM_PIN_SPI_SEL);

    /* Disable clock for switch matrix. */
    CLOCK_DisableClock(kCLOCK_Swm);
}
/***********************************************************
 * Function Name  : Sensor_Init_SPI
 * Description    :
 * Input          :	None
 * Return         : None
 ***********************************************************/
static void Sensor_Init_SPI (void){
    spi_master_config_t userConfig = {0};

    /* Attach main clock to SPI0. */
    CLOCK_Select(kSPI0_Clk_From_MainClk);
    /* Note: The slave board using interrupt way, slave will spend more time to write data
     *       to TX register, to prevent TX data missing in slave, we will add some delay between
     *       frames and capture data at the second edge, this operation will make the slave
     *       has more time to prapare the data.
     */
    SPI_MasterGetDefaultConfig(&userConfig);

    userConfig.baudRate_Bps           = SENSOR_SPI_MASTER_BAUDRATE;
    userConfig.sselNumber             = SENSOR_SPI_MASTER_SSEL;
    userConfig.clockPolarity		  = kSPI_ClockPolarityActiveHigh;
    userConfig.clockPhase			  = kSPI_ClockPhaseSecondEdge;
    userConfig.direction			  = kSPI_MsbFirst;
    userConfig.dataWidth			  = (uint8_t)kSPI_Data8Bits;

    SPI_MasterInit(SENSOR_SPI_MASTER, &userConfig, SENSOR_SPI_MASTER_CLK_FREQ);
}
/***********************************************************
* Function Name  : Sensor_SendFrame
* Description    : Sending Data to SPI
* Input          :
* Return         :
***********************************************************/
void Sensor_SendFrame (uint16_t SendingBytes){
    /*Start Transfer*/
    xfer.txData      = SPI0_TxBuf;
    xfer.rxData      = SPI0_RxBuf;
    xfer.dataSize    = SendingBytes;
    xfer.configFlags = kSPI_EndOfTransfer;

    SENSOR_SPI_MASTER->TXCTL &= ~SPI_TXCTL_EOT_MASK;
    /* Transfer data in polling mode. */
    SPI_MasterTransferBlocking(SENSOR_SPI_MASTER, &xfer);
}

/***********************************************************
 * Function Name  : Sensor_Read_SPI
 * Description    :
 * Input          :	RegAddr: Register Address of Sensor
 * 					Data   : Data Pointer
 * 					Len    : Length of data
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
int8_t Sensor_Read_SPI (uint8_t RegAddr,uint8_t *Data,uint16_t Len,void *intf_ptr){
	memset(SPI0_TxBuf, 0, SPI0_BUFFER_SIZE);
	SPI0_TxBuf[0] = RegAddr;
	Sensor_SendFrame(Len+1);
	memcpy(Data, &SPI0_RxBuf[1], Len);
	return 0;
}
/***********************************************************
 * Function Name  : Sensor_Write_SPI
 * Description    :
 * Input          :	RegAddr: Register Address of Sensor
 * 					Data   : Data Pointer
 * 					Len    : Length of data
 * Return         : = 0 -> Succes
 * 					> 0 -> Warning
 * 					< 0 -> Fail
 ***********************************************************/
int8_t Sensor_Write_SPI (uint8_t RegAddr,const uint8_t *Data,uint16_t Len,void *intf_ptr){
	memset(SPI0_TxBuf, 0, SPI0_BUFFER_SIZE);
	SPI0_TxBuf[0] = RegAddr;
	SPI0_TxBuf[1] = Data[0];
	Sensor_SendFrame(2);
	return 0;
}
/***********************************************************
 * Function Name  : Sensor_Delay_ms
 * Description    :
 * Input          :	Period : Period of delay
 * Return         : -
 ***********************************************************/
void Sensor_Delay_ms(uint32_t Period, void *intf_ptr){
	Delay_ms(Period);
}
