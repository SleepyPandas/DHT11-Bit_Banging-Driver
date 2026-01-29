#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include <stdint.h>

int DHT11_Read_Data(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, int *temp,
                    int *hum);

#endif /* DHT11_DRIVER_H */
