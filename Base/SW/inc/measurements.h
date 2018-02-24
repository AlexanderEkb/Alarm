#ifndef MEAS_H
#define MEAS_H

void Measurement_Init();
void Measurement_Task(void *p);
uint32_t Measurement_GetVoltage();
int32_t Measurement_GetTemp();
uint32_t Measurement_GetRPM();

#endif /* MEAS_H */
