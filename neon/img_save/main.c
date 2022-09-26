#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<arm_neon.h>

#define ALGN_DATA(a, b)         ((a + b - 1) & (-b))

void imwrite(unsigned char* imgdata, const int w, const int h, const char* filename)
{
    int i, j;
    int l = ALGN_DATA(w, 4);
    int bmi[] = { l*h + 54,0,54,40,l,-h,1 | 3 * 8 << 16,0,l*h,0,0,0,0 };

    FILE *fid = fopen(filename, "wb");
    fprintf(fid, "BM");
    fwrite(&bmi, 52, 1, fid);

    unsigned char ucZero = 0;
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            fwrite(&imgdata[i * w + j + 0], sizeof(unsigned char), 1, fid);
            fwrite(&imgdata[i * w + j + 0], sizeof(unsigned char), 1, fid);
            fwrite(&imgdata[i * w + j + 0], sizeof(unsigned char), 1, fid);
        }
        for (; j < l; j++)
        {
            fwrite(&ucZero, sizeof(unsigned char), 1, fid);
            fwrite(&ucZero, sizeof(unsigned char), 1, fid);
            fwrite(&ucZero, sizeof(unsigned char), 1, fid);
        }
    }
    fclose(fid);

    return;
}

int main()
{
    int w = 640;
    int h = 480;
    unsigned char* imgdata = (unsigned char*)malloc(w * h * sizeof(char));
    memset(imgdata, 100, w*h);

    char filename[255];
    sprintf(filename, "./img.bmp");

    imwrite(imgdata, w, h, filename);

    return 0;
}