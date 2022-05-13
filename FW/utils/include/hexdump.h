#ifndef DEBUG_PRINT_H_
#define DEBUG_PRINT_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include <stdint.h>

void hexdump(const uint8_t *buffer, uint16_t bufferLength);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DEBUG_PRINT_H_ */
