#include "bmp.h"

using namespace std;

void mat2bmp(const size_t width, const size_t height, const size_t channel, Vec **mat, const string &bmpfile) {
  FILE *f;
  unsigned char *img = NULL;
  int imgsize = channel * width * height;
  int filesize = 54 + imgsize;

  img = (unsigned char *)malloc(imgsize*sizeof(unsigned char));
  memset(img, 0, imgsize);

  unsigned char r, g, b;
  size_t x, y;
  for(int i=0; i<height; i++) {
    for(int j=0; j<width; j++) {
      x = j;
      y = (height-1) - i;
      r = mat[i][j][0] * 255;
      g = mat[i][j][1] * 255;
      b = mat[i][j][2] * 255;
      if (r > 255) r=255;
      if (g > 255) g=255;
      if (b > 255) b=255;
      img[(x+y*width)*channel+2] = (unsigned char)(r);
      img[(x+y*width)*channel+1] = (unsigned char)(g);
      img[(x+y*width)*channel+0] = (unsigned char)(b);
    }
  }

  unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
  unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
  unsigned char bmppad[3] = {0,0,0};

  bmpfileheader[ 2] = (unsigned char)(filesize      );
  bmpfileheader[ 3] = (unsigned char)(filesize >> 8 );
  bmpfileheader[ 4] = (unsigned char)(filesize >> 16);
  bmpfileheader[ 5] = (unsigned char)(filesize >> 24);

  bmpinfoheader[ 4] = (unsigned char)(width       );
  bmpinfoheader[ 5] = (unsigned char)(width >> 8  );
  bmpinfoheader[ 6] = (unsigned char)(width >> 16 );
  bmpinfoheader[ 7] = (unsigned char)(width >> 24 );
  bmpinfoheader[ 8] = (unsigned char)(height      );
  bmpinfoheader[ 9] = (unsigned char)(height >> 8 );
  bmpinfoheader[10] = (unsigned char)(height >> 16);
  bmpinfoheader[11] = (unsigned char)(height >> 24);

  f = fopen(bmpfile.c_str(),"wb");
  fwrite(bmpfileheader,1,14,f);
  fwrite(bmpinfoheader,1,40,f);
  for(int i=0; i<height; i++) {
    fwrite(img+(width*(height-i-1)*channel), channel, width, f);
    fwrite(bmppad,1,(4-(width*channel)%4)%4, f);
  }

  free(img);
  fclose(f);
}
