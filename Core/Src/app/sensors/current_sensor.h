#ifndef CURRENT_SENSOR_H_
#define CURRENT_SENSOR_H_


// ----------------------------------------------------------------------------------
// ----------- Current Sensor (CS) on VNH5019A-E motor driver  ----------------------
// ----------------------------------------------------------------------------------
#define ADC_RESOLUTION 					4095.0f      		// 12-bit ADC
#define ADC_REF_VOLTAGE 				3.3f        		// Reference voltage (Vref)

// Hardware constants
#define VOLTAGE_DIVIDER_GAIN   			11.0f	 			// (R9 + R10) / R10 = 11
#define R_SENSE 						1000.0f             // 1 kΩ resistor
#define K_SENSE 						7000.0f             // iOUT / iSENSE from driver datasheet (≈7k typical)
#define STALL_CURRENT 					3.0f          		// Stall current threshold (A). In the datasheet ~5.6 A


#endif /* CURRENT_SENSOR_H_ */
