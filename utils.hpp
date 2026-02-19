 #ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <bitset>
#include "typedefs.hpp"

using namespace std;


extern sc_core::sc_time offset;
extern sc_core::sc_time delay;
extern int counter;
// 4 byte = 32 bit (I=16, F=16)

num_t to_fixed(const unsigned char *buf); // const pointer
void to_uchar(unsigned char *buf, num_t value);
//double to_double(unsigned char *buf);



#endif // _UTILS_HPP_

