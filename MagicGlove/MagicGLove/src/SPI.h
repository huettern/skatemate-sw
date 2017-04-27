/*
* SPI.h
*
* Created: 25.04.2017 22:42:11
*  Author: Farin
*/


#ifndef SPI_H_
#define SPI_H_

void SPI_init(void);
char SPI_Transceive(char);
uint8_t SPI_Read(uint8_t);
void SPI_Write(uint8_t);
void SPI_WriteBuffer(uint8_t *, uint8_t);


#endif /* SPI_H_ */