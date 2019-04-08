#ifndef __BMP_H__
#define __BMP_H__

#include <string>
#include <vector>
#include "ds/Vec.h"

void mat2bmp(const size_t width, const size_t height, const size_t channel, Vec **mat, const std::string &bmpfile);

#endif
