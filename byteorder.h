#ifndef BYTEORDER_H_
#define BYTEORDER_H_

#ifdef __cplusplus
extern "C" {
#endif
    

//! \brief  Converts a byte array to a word value using the big endian format
#define WORDB(bytes)            ((uint16_t) ((bytes[0] << 8) | bytes[1]))

//! \brief  Converts a byte array to a word value using the little endian format
#define WORDL(bytes)            ((uint16_t) ((bytes[1] << 8) | bytes[0]))

//! \brief  Converts a byte array to a dword value using the big endian format
#define DWORDB(bytes)   ((uint32_t) ((bytes[0] << 24) | (bytes[1] << 16) \
                                         | (bytes[2] << 8) | bytes[3]))

//! \brief  Converts a byte array to a dword value using the little endian format
#define DWORDL(bytes)   ((uint32_t) ((bytes[3] << 24) | (bytes[2] << 16) \
                                         | (bytes[1] << 8) | bytes[0]))

//! \brief  Stores a dword value in a byte array, in big endian format
#define STORE_DWORDB(dword, bytes) \
    bytes[0] = (uint8_t) (((dword) >> 24) & 0xFF); \
    bytes[1] = (uint8_t) (((dword) >> 16) & 0xFF); \
    bytes[2] = (uint8_t) (((dword) >> 8) & 0xFF); \
    bytes[3] = (uint8_t) ((dword) & 0xFF);

//! \brief  Stores a word value in a byte array, in big endian format
#define STORE_WORDB(word, bytes) \
    bytes[0] = (uint8_t) (((word) >> 8) & 0xFF); \
    bytes[1] = (uint8_t) ((word) & 0xFF);


#ifdef __cplusplus
}
#endif    
#endif /*BYTEORDER_H_*/

