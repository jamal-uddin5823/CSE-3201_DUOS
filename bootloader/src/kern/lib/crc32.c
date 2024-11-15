#include<crc32.h>

uint32_t crc32_calculate(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint32_t)data[i] << 24);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return ~crc; // Final XOR value
}


int crc32_verify(const uint8_t *data, size_t length, uint32_t expected_crc) {
    uint32_t calculated_crc = crc32_calculate(data, length);
    return (calculated_crc == expected_crc) ? 1 : 0;
}

