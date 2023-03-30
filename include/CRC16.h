#ifndef __CRC16_H__
#define __CRC16_H__

// #include <stdio.h>

#define GENERATOR 0x1305

void CalculateTable_CRC16();
unsigned short Compute_CRC16(unsigned char* bytes, const int BYTES_LEN);
#endif // !__CRC16_H__

