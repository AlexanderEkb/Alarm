#ifndef MEAS_H
#define MEAS_H

void Measurement_Init();
uint32_t Measurement_GetVoltage();
int32_t Measurement_GetTemp();
uint32_t Measurement_GetRPM();

#endif /* MEAS_H */
