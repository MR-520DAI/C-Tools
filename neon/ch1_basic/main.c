#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<arm_neon.h>

void add_int_c(int* dst, int* src1, int* src2, int count)
{
  int i;
  for (i = 0; i < count; i++)
    dst[i] = src1[i] + src2[i];
}

void add_int_neon(int* dst, int* src1, int* src2, int count)
{
    int i;
    for (i = 0; i < count; i += 4)
    {
        int32x4_t in1, in2, out;
        in1 = vld1q_s32(src1);
        src1 += 4;
        in2 = vld1q_s32(src2);
        src2 += 4;
        out = vaddq_s32(in1, in2);
        vst1q_s32(dst, out);
        dst += 4;
    }
}

int main()
{
    /* 数据内存分配，初始化 */
    int size = 5000;
    int *dst = (int*)malloc(size * sizeof(int));
    int *src1 = (int*)malloc(size * sizeof(int));
    int *src2 = (int*)malloc(size * sizeof(int));

    for(int i = 0; i < size; i++)
    {
        src1[i] = i;
        src2[i] = i;
    }

    /* 时间初始化 */
    struct timespec time1_img = {0, 0};
    struct timespec time2_img = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time1_img);

    for(int i = 0; i < size; i++)
    {
        add_int_c(dst, src1, src2, size);
    }
    clock_gettime(CLOCK_REALTIME, &time2_img);
    printf("C time:%d ms\n", (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000);

    printf("dst[0]:%d\n", dst[0]);
    memset(dst, 0, size);

    clock_gettime(CLOCK_REALTIME, &time1_img);
    for(int i = 0; i < size; i++)
    {
        add_int_neon(dst, src1, src2, size);
    }
    clock_gettime(CLOCK_REALTIME, &time2_img);
    printf("Neon time:%d ms\n", (time2_img.tv_sec - time1_img.tv_sec)*1000 + (time2_img.tv_nsec - time1_img.tv_nsec)/1000000);
    printf("dst[0]:%d\n", dst[0]);

    free(dst);
    free(src1);
    free(src2);

    return 0;
}