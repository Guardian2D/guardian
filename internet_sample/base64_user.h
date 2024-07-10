#ifndef BASE64_USER_H
#define BASE64_USER_H

#include <stdint.h>


int base64_encode(const char *indata, int inlen, char *outdata, int *outlen);
int base64_decode(const char *indata, int inlen, char *outdata, int *outlen);


#endif
