#include "typedefs.hpp"
#include <cstring>
#include <iostream>


void to_uchar(unsigned char* buf, num_t value) {
    std::memcpy(buf, &value, sizeof(num_t));
}

num_t to_fixed(const unsigned char* buf) {
    num_t val;
    std::memcpy(&val, buf, sizeof(num_t));
    return val;
}


/*
double to_double(unsigned char *buf)
{
  string concated = "";
  for(int i = 0; i<CHARS_AMOUNT; ++i) //concatenate char array into eg. "10101101000"
    concated += bitset<CHAR_LEN>((int)buf[i]).to_string();

  cout << "concated = " << concated << endl;
  
  double multiplier = 1;
  
}
*/
/*#include "utils.hpp"
#include <cstring>
#include <cstdint>

/**
 * Konverzió unsigned char -> num_t (fixpontos), 32 bites bufferből.
 * Stabil, egyszerű verzió.
 */
/*num_t to_fixed (unsigned char *buf)
{
    int32_t raw;
    std::memcpy(&raw, buf, sizeof(raw));
    return num_t(raw) / (1 << 16);
}

/**
 * Konverzió num_t -> unsigned char [4 byte] bufferbe.
 * Stabil, egyszerű verzió.
 */
/*void to_uchar(unsigned char *buf, num_t d)
{
    int32_t raw = static_cast<int32_t>(std::round(d.to_double() * (1 << 16)));
    std::memcpy(buf, &raw, sizeof(raw));
}
*/