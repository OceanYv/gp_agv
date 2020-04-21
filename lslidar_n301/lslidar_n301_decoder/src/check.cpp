#include <iostream>
#include <stdio.h>
#include <cmath>
using namespace std;

typedef unsigned long uint64_t;

int main() {

 int BLOCKS_PER_PACKET = 12 ; 
 int FIRINGS_PER_BLOCK = 2;
 int  SCANS_PER_FIRING = 16;
 double DSR_TOFFSET=2.304;
 double FIRING_TOFFSET =55.296;
int RAW_SCAN_SIZE = 3; 
double DISTANCE_RESOLUTION = 0.01;

    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {

        for (size_t blk_fir_idx = 0; blk_fir_idx < FIRINGS_PER_BLOCK; ++blk_fir_idx){
            size_t fir_idx = blk_idx*FIRINGS_PER_BLOCK + blk_fir_idx;

            for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING; ++scan_fir_idx){
                size_t byte_idx = RAW_SCAN_SIZE * (
                            SCANS_PER_FIRING*blk_fir_idx + scan_fir_idx);
                printf("blk_idx=%lu, blk_fir_idx=%lu, fir_idx=%lu, scan_fir_idx=%lu,byte_idx=%lu\n", blk_idx, blk_fir_idx, fir_idx, scan_fir_idx, byte_idx);

            }
        }
    }

    printf("%d\n", 1236%1000);
    uint64_t t1 = 123456;
    uint64_t t2 = 789011;
    double t;
    t = t1*1.0 + t2 * 1e-6;
    printf("timestamp =%f\n", t);

    return 0;
}