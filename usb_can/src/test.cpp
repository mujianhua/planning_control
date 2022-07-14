/**
 * @file
 * @brief
 */

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include "ECanVci.h"

#define msleep(ms) usleep((ms)*1000)
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define MAX_CHANNELS 2
#define CHECK_POINT 200
#define RX_WAIT_TIME 0
#define RX_BUFF_SIZE 1

unsigned int gDevType = 0;
unsigned int gDevIdx = 0;
unsigned int gChMask = 0;
unsigned int gBaud = 0;
unsigned char gTxType = 0;
unsigned int gTxSleep = 0;
unsigned int gTxFrames = 0;

unsigned int s2n(const char *s) {
    unsigned l = strlen(s);
    unsigned v = 0;
    unsigned h = (l > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
    unsigned char c;
    unsigned char t;
    if (!h)
        return atoi(s);
    if (l > 10)
        return 0;
    for (s += 2; c = *s; s++) {
        if (c >= 'A' && c <= 'F')
            c += 32;
        if (c >= '0' && c <= '9')
            t = c - '0';
        else if (c >= 'a' && c <= 'f')
            t = c - 'a' + 10;
        else
            return 0;
        v = (v << 4) | t;
    }
    return v;
}

void generate_frame(CAN_OBJ *can) {
    unsigned i;
    memset(can, 0, sizeof(CAN_OBJ));

    can->SendType = gTxType;

    // can->SendType =0;
    can->DataLen = 1 + (rand() % 8); // random data length: 1~8

    for (i = 0; i < can->DataLen; i++) {
        can->Data[i] = rand() & 0xff; // random data
        can->ID ^= can->Data[i];      // id: bit0~7, checksum of data0~N
    }
    can->ID |= ((unsigned)can->DataLen - 1)
               << 8;              // id: bit8~bit10 = data_length-1
    can->ExternFlag = rand() % 2; // random frame format
    if (!can->ExternFlag)
        return;
    can->ID |= can->ID << 11; // id: bit11~bit21 == bit0~bit10
    can->ID |= can->ID << 11; // id: bit22~bit28 == bit0~bit7
}

int verify_frame(CAN_OBJ *can) {
    printf("CAN DataLen: %d \n", can->DataLen);
    if (can->DataLen > 8)
        return 0; // error: data length
    printf("CAN ID: %d \n", can->ID & 0xffffffff);
    unsigned bcc = 0;
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
        bcc ^= can->Data[i];
    // todo:
    // if ((can->ID & 0xff) != bcc)
    //     return 0; // error: data checksum
    // if (((can->ID >> 8) & 7) != (can->DataLen - 1))
    //     return 0; // error: data length
    // if (!can->ExternFlag)
    //     return 1; // std-frame ok
    // if (((can->ID >> 11) & 0x7ff) != (can->ID & 0x7ff))
    //     return 0; // error: frame id
    // if (((can->ID >> 22) & 0x7f) != (can->ID & 0x7f))
    //     return 0; // error: frame id
    return 1; // ext-frame ok
}

typedef struct {
    unsigned channel; // channel index, 0~3
    unsigned stop;    // stop RX-thread
    unsigned total;   // total received
    unsigned error;   // error(s) detected
} RX_CTX;

void *rx_thread(void *data)

{
    RX_CTX *ctx = (RX_CTX *)data;
    ctx->total = 0; // reset counter

    CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt;                   // current received
    int i;

    unsigned check_point = 0;
    while (!ctx->stop && !ctx->error) {
        cnt = Receive(gDevType, gDevIdx, ctx->channel, can, RX_BUFF_SIZE,
                      RX_WAIT_TIME);
        if (!cnt)
            continue;

        for (i = 0; i < cnt; i++) {
            if (verify_frame(&can[i]))
                continue;
            printf("CAN%d: verify_frame() failed\n", ctx->channel);
            ctx->error = 1;
            break;
        }
        if (ctx->error)
            break;

        ctx->total += cnt;
        if (ctx->total / CHECK_POINT >= check_point) {
            printf("CAN%d: %d frames received & verified\n", ctx->channel,
                   ctx->total);
            check_point++;
        }

        printf("CAN Data: %d, %d, %d, %d, %d, %d, %d, %d\n", can->Data[0],
               can->Data[1], can->Data[2], can->Data[3], can->Data[4],
               can->Data[5], can->Data[6], can->Data[7] & 0xff);
    }

    printf("CAN%d RX thread terminated, %d frames received & verified: %s\n",
           ctx->channel, ctx->total,
           ctx->error ? "error(s) detected" : "no error");

    pthread_exit(0);
}

int test() {
    // ----- init & start -------------------------------------------------

    INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 0;
    config.Mode = 0;
    config.Timing0 = gBaud & 0xff;
    config.Timing1 = gBaud >> 8;

    int i;
    for (i = 0; i < MAX_CHANNELS; i++) {
        if ((gChMask & (1 << i)) == 0)
            continue;

        if (!InitCAN(gDevType, gDevIdx, i, &config)) {
            printf("InitCAN(%d) failed\n", i);
            return 0;
        }

        printf("InitCAN(%d) succeeded\n", i);

        if (!StartCAN(gDevType, gDevIdx, i)) {
            printf("StartCAN(%d) failed\n", i);
            return 0;
        }

        printf("StartCAN(%d) succeeded\n", i);
    }

    // ----- RX-timeout test ----------------------------------------------

    CAN_OBJ can;
    time_t tm1, tm2;
    for (i = 0; i < 3; i++) {
        time(&tm1);
        Receive(gDevType, gDevIdx, 0, &can, 1, (i + 1) * 1000 /*ms*/);
        time(&tm2);
        printf("Receive returned: time ~= %ld seconds\n", tm2 - tm1);
    }

    // ----- create RX-threads --------------------------------------------

    RX_CTX rx_ctx[MAX_CHANNELS];
    pthread_t rx_threads[MAX_CHANNELS];
    for (i = 0; i < MAX_CHANNELS; i++) {
        if ((gChMask & (1 << i)) == 0)
            continue;

        rx_ctx[i].channel = i;
        rx_ctx[i].stop = 0;
        rx_ctx[i].total = 0;
        rx_ctx[i].error = 0;

        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);
    }

    // ----- wait --------------------------------------------------------

    printf("<ENTER> to start TX: %d frames/channel, baud: t0=0x%02x, "
           "t1=0x%02x...\n",
           gTxFrames, config.Timing0, config.Timing1);
    getchar();

    // ----- start transmit -----------------------------------------------

    time(&tm1);
    int err = 0;
    unsigned tx;

    for (tx = 0; !err && tx < gTxFrames; tx++) {
        for (i = 0; i < MAX_CHANNELS; i++) {
            if ((gChMask & (1 << i)) == 0)
                continue;

            generate_frame(&can);
            if (1 != Transmit(gDevType, gDevIdx, i, &can, 1)) {
                printf("CAN%d TX failed: ID=%08x\n", i, can.ID);
                err = 1;
                break;
            }
        }
        if (gTxSleep)
            msleep(gTxSleep);
    }
    time(&tm2);

    // ----- stop TX & RX -------------------------------------------------

    msleep(1000);
    printf("TX stopped, <ENTER> to terminate RX-threads...\n");
    getchar();

    for (i = 0; i < MAX_CHANNELS; i++) {
        if ((gChMask & (1 << i)) == 0)
            continue;

        rx_ctx[i].stop = 1;

        pthread_join(rx_threads[i], NULL);

        if (rx_ctx[i].error)
            err = 1;
    }

    // ----- report -------------------------------------------------------

    if (err) {
        printf("error(s) detected, test failed\n");
        return 0;
    }

    printf(
        "\n ***** %d frames/channel transferred, %ld seconds elapsed *****\n",
        gTxFrames, tm2 - tm1);
    if (tm2 - tm1)
        printf("        performance: %ld frames/channel/second\n",
               gTxFrames / (tm2 - tm1));

    return 1;
}

int main(int argc, char *argv[]) {
    if (argc < 7) {
        printf("test [DevType] [DevIdx] [ChMask] [Baud] [TxType] [TxSleep] "
               "[TxFrames]\n"
               "    example: test 16 0 3 0x1400 0 1 1000\n"
               "                  |  | | |      | | |\n"
               "                  |  | | |      | | |1000 frames / channel\n"
               "                  |  | | |      | |\n"
               "                  |  | | |      | |tx > sleep(3ms) > tx > "
               "sleep(3ms) ....\n"
               "                  |  | | |      |\n"
               "                  |  | | |      |0-normal, 1-single, "
               "2-self_test, 3-single_self_test, 4-single_no_wait....\n"
               "                  |  | | |\n"
               "                  |  | | |0x1400-1M, 0x1c03-125K, ....\n"
               "                  |  | |\n"
               "                  |  | |bit0-CAN1, bit1-CAN2, bit2-CAN3, "
               "bit3-CAN4, 3=CAN1+CAN2, 7=CAN1+CAN2+CAN3\n"
               "                  |  |\n"
               "                  |  |Card0\n"
               "                  |\n"
               "                  |1-usbcan,  ....\n");
        return 0;
    }

    gDevType = s2n(argv[1]);
    gDevIdx = s2n(argv[2]);
    gChMask = s2n(argv[3]);
    gBaud = s2n(argv[4]);
    gTxType = s2n(argv[5]);
    gTxSleep = s2n(argv[6]);
    gTxFrames = s2n(argv[7]);
    printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, "
           "TxSleep=%d, TxFrames=0x%08x(%d)\n",
           gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames,
           gTxFrames);

    if (!OpenDevice(gDevType, gDevIdx, 0)) {
        printf("OpenDevice failed\n");
        return 0;
    }
    printf("OpenDevice succeeded\n");

    test();

    CloseDevice(gDevType, gDevIdx);
    printf("CloseDevice\n");
    return 0;
}
