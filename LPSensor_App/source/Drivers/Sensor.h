/*
 * Sensor.h
 *
 *  Created on: 21 May 2022
 *      Author: Alican
 */

#ifndef DRIVERS_SENSOR_H_
#define DRIVERS_SENSOR_H_

#define  MAX_SENSOR_CH_NUMBER      (3U)
#define  SENSOR_CH_TEMPERATURE     (0U)
#define  SENSOR_CH_HUMIDITY        (1U)
#define  SENSOR_CH_PRESSURE        (2U)

enum SensorType_TypEnm{
	SENSOR_TYPE_TEMP      = 0, /* Temperature    */
	SENSOR_TYPE_HUMID     = 1, /* Humidity       */
	SENSOR_TYPE_PRESSURE  = 2, /* Pressure       */
	SENSOR_TYPE_MAX       = 3, /* Max Check      */
};
enum SensorIntf_TypEnm{
	SENSOR_SPI_INTF  = 0, /* SPI  interface  */
	SENSOR_I2C_INTF  = 1, /* I2C  interface  */
	SENSOR_UART_INTF = 2, /* UART interface  */
	SENSOR_INTF_MAX  = 3, /* Max Check       */
};

enum SensorDatatype_TypEnm{
	SENSOR_DATATYPE_INT     = 0, /* INT     type  */
	SENSOR_DATATYPE_UINT    = 1, /* UINT    type  */
	SENSOR_DATATYPE_DOUBLE  = 2, /* DOUBLE  type  */
	SENSOR_DATATYPE_MAX     = 3, /* Max Check       */
};

union SensorData_TypeUn{
    int32_t   i32_Val;
    uint32_t  ui32_Val;
    double     d32_Val;
};


struct SensorCh_TypeStruct{
	bool  					      Enable;
	enum  SensorType_TypEnm       Type;
	enum  SensorDatatype_TypEnm	  DataType;
	union SensorData_TypeUn       Data;
};


struct Sensor_TypeStruct{
	bool  					       Enable;
	struct SensorCh_TypeStruct     Sensor[MAX_SENSOR_CH_NUMBER];
	enum   SensorIntf_TypEnm       Interface;
};


void LpSensorApp_Loop(void);

#endif /* DRIVERS_SENSOR_H_ */
