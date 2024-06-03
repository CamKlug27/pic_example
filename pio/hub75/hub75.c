/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hub75.pio.h"

#include "mountains_128x64_rgb565.h"



#define F_CPU            250000000
#define DATA_BASE_PIN    0
#define DATA_N_PINS      6
#define ROWSEL_BASE_PIN  6
#define ROWSEL_N_PINS    5
#define CLK_PIN          11
#define STROBE_PIN       14
#define OEN_PIN          15

#define ROW_NUMBER        1
#define PANNEL_BY_ROW     1
#define PIXEL_BY_PANNEL   128
#define WIDTH (PIXEL_BY_PANNEL*PANNEL_BY_ROW*ROW_NUMBER)
#define HEIGHT 64

#define PANNEL_WIDTH (WIDTH*2) 


#if (ROWSEL_N_PINS == 5)
#define ADDR_MSK            0xFFFF383F
#elif (ROWSEL_N_PINS == 4)
#define ADDR_MSK            0xFFFF3C3F
#elif (ROWSEL_N_PINS == 3)
#define ADDR_MSK            0xFFFF3E3F
#elif (ROWSEL_N_PINS == 2)
#define ADDR_MSK            0xFFFF3F3F
#endif

#define WRITE_ADDR(VALUE)                             (sio_hw->gpio_out = ((sio_hw->gpio_out & (0x1FFE3F)) | ((uint32_t)(VALUE) << 6)))
#define WRITE_ROW(ADDR_VALUE, OE_VALUE, LAT_VALUE)    (sio_hw->gpio_out = ((sio_hw->gpio_out & (ADDR_MSK)) | (((uint32_t)(ADDR_VALUE) << 6) | ((uint32_t)((LAT_VALUE) | ((OE_VALUE) << 1)) << 14))))
#define WRITE_RGB(VALUE)                              (sio_hw->gpio_out = ((sio_hw->gpio_out & (0x1FFFC0)) | ((uint32_t)(VALUE))))
#define WRITE_CLK(VALUE)                              (sio_hw->gpio_out = ((sio_hw->gpio_out & (0x1FF7FF)) | ((uint32_t)(VALUE << 11))))
#define LAT_ENABLE                                    1
#define LAT_DISABLE                                   0
#define OE_ENABLE                                     0
#define OE_DISABLE                                    1

 
#define OFFSET                                          0
#define PIO_DEF                                         1
#define BCM_DEF                                         0
#define BRIGTHNESS                                      100
#define XFER_POLLING                                    0
#define XFER_DMA                                        1
#define XFER_PIO_RGB6                                   2
#define XFER_MODE                                       XFER_DMA

#define FRAME_LEN                                       WIDTH
#define QUEUE_LEN                                       (8)

#define TRIGGER_DMA                                     true
                  

//********************************************************************

typedef enum eMatrixState_tTag
{
    MATRIX_INIT_GAMMA = 0,
    MATRIX_SEND_DATA,
    MATRIX_WAIT_DMA_END_XFER,
    MATRIX_WAIT_STATE_MACHINE_STALL,
    MATRIX_ENABLE_ROW,
    MATRIX_WAIT_SM_ROW_STALL,
    MATRIX_BCM_WAIT,

    MATRIX_STATE_MAX,
} eMatrixState_t;

typedef struct sFrame_tTag
{
    #if (XFER_MODE == XFER_POLLING)
    uint32_t u32gammaData[2][FRAME_LEN];
    #elif (XFER_MODE == XFER_DMA)
    uint32_t u32gammaData[PANNEL_WIDTH + 2];
    #endif
} sFrame_t;

typedef struct sQueue_tTag
{
  sFrame_t sFrame[QUEUE_LEN];
  uint16_t u16ReadIndex;
  uint16_t u16WriteIndex;
} sQueue_t;

static int dma_chan[3] = {0};
static volatile sQueue_t sMatrixData;
static eMatrixState_t eMatrixState = MATRIX_INIT_GAMMA;
const uint16_t *img = (const uint16_t*)mountains_128x64;
static uint8_t u8Row =0, u8Col = 0, u8Bit = 0, u8Row_m = 0;
static uint8_t u8Index = 0;
static uint16_t u16RdIdx = 0, u16WrIdx = 0;
static uint32_t *u32Ptr1 = NULL; 
static uint32_t *u32Ptr2 = NULL; 
uint32_t txstall_mask = 0;

const static PIO pio = pio0;
const static uint sm_data = 0;
const static uint sm_row = 1;
const static uint16_t sm_row_msk  = (PIO_FDEBUG_TXSTALL_LSB + sm_row);
const static uint16_t sm_data_msk = (PIO_FDEBUG_TXSTALL_LSB + sm_data);
//********************************************************************


typedef  union sPixel_tTag
{
    uint32_t u32All;
    struct 
    {
        uint32_t r : 8;
        uint32_t g : 8;
        uint32_t b : 8;
        uint32_t   : 8;
    };
    
} sPixel_t;

sPixel_t sRGBPixel;

static inline uint32_t gamma_correct_565_888(uint16_t pix) {
    uint32_t r_gamma = pix & 0xf800u;
    r_gamma *= r_gamma;
    uint32_t g_gamma = pix & 0x07e0u;
    g_gamma *= g_gamma;
    uint32_t b_gamma = pix & 0x001fu;
    b_gamma *= b_gamma;
    return (b_gamma >> 2 << 16) | (g_gamma >> 14 << 8) | (r_gamma >> 24 << 0);
}

static inline bool isQueueFull(void)
{
    return (sMatrixData.u16WriteIndex == QUEUE_LEN);
}

static void computeNextData(void)
{
    u16WrIdx = sMatrixData.u16WriteIndex;
    u32Ptr2 = (uint32_t *)&sMatrixData.sFrame[u16WrIdx].u32gammaData[0];

    if (!isQueueFull())
    {
        u32Ptr2[u8Index]     = gamma_correct_565_888(img[u8Row_m * WIDTH + u8Col]);
        u32Ptr2[u8Index + 1] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + u8Row_m) * WIDTH + u8Col]);

        // increment column and buffer index
        u8Index += 2;
        u8Col++;

        // check if we are fetched all pixels of the selected line
        if (u8Col == WIDTH)
        {
            sMatrixData.u16WriteIndex = u16WrIdx + 1;

            u8Row_m = (u8Row_m + 1) % (1u << ROWSEL_N_PINS);

            u8Index = 0;
            u8Col = 0;
        }
    }
}

int main() {
    set_sys_clock_khz(F_CPU / 1000, true);
    stdio_init_all();


    #if PIO_DEF == 1
    uint data_prog_offs = pio_add_program(pio, &hub75_data_rgb888_program);
    uint row_prog_offs = pio_add_program(pio, &hub75_row_program);
    hub75_data_rgb888_program_init(pio, sm_data, data_prog_offs, DATA_BASE_PIN, CLK_PIN);
    hub75_row_program_init(pio, sm_row, row_prog_offs, ROWSEL_BASE_PIN, ROWSEL_N_PINS, STROBE_PIN);

    #if (XFER_MODE == XFER_DMA)
    /* set DMA channels */
    dma_chan[0] = dma_claim_unused_channel(true);
    dma_chan[1] = dma_claim_unused_channel(true);

    /* configuration of DMA channels */
    dma_channel_config config1 = dma_channel_get_default_config(dma_chan[0]);
    dma_channel_config config2 = dma_channel_get_default_config(dma_chan[1]);

    /* configuration of the firt channel */
    channel_config_set_transfer_data_size(&config1, DMA_SIZE_32);
    channel_config_set_read_increment(&config1, true);
    channel_config_set_write_increment(&config1, false);
    channel_config_set_dreq(&config1, pio_get_dreq(pio, sm_data, true));

    /* configuration of the second channel */
    channel_config_set_transfer_data_size(&config2, DMA_SIZE_32);
    channel_config_set_read_increment(&config2, false);
    channel_config_set_write_increment(&config2, false);
    channel_config_set_dreq(&config2, pio_get_dreq(pio, sm_row, true));

    /* start the first channel */
    dma_channel_configure(dma_chan[0], &config1, &pio->txf[sm_data], NULL, (PANNEL_WIDTH + 2), false);

    /* start the second channel */
    dma_channel_configure(dma_chan[1], &config2, &pio->txf[sm_row], NULL, 1, false);
    #endif
    #endif

    #if PIO_DEF  == 0
    for(uint8_t i = DATA_BASE_PIN ; i < DATA_N_PINS; i++)
    {
        gpio_init(i);
        gpio_set_dir(i, true);
    }

    for(uint8_t i = ROWSEL_BASE_PIN ; i < (ROWSEL_N_PINS + ROWSEL_BASE_PIN); i++)
    {
        gpio_init(i);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_4MA);
        gpio_set_dir(i, true);
    }

    gpio_init(CLK_PIN);
    gpio_set_drive_strength(CLK_PIN, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_dir(CLK_PIN, true);

    gpio_init(STROBE_PIN);
    gpio_set_drive_strength(STROBE_PIN, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_dir(STROBE_PIN, true);

    gpio_init(OEN_PIN);
    gpio_set_drive_strength(OEN_PIN, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_dir(OEN_PIN, true);

    WRITE_RGB(0);

    #endif


    #if (XFER_MODE == XFER_POLLING)
    static uint32_t gc_row[2][WIDTH];
    #elif (XFER_MODE == XFER_DMA)
    static uint32_t gc_row[PANNEL_WIDTH + 2];
    #endif
   
    memset((void*)&sMatrixData, 0x0, sizeof(sMatrixData));

    #if (XFER_MODE == XFER_DMA)
    for (uint8_t u8Index = 0; u8Index < QUEUE_LEN; u8Index++)
    {
        sMatrixData.sFrame[u8Index].u32gammaData[256] = 0xFF << 8;
        sMatrixData.sFrame[u8Index].u32gammaData[257] = 0xFF << 8;
    }
    #endif

    while (1) {
        #if PIO_DEF == 1
        #if (XFER_MODE == XFER_POLLING)
        for (int rowsel = 0; rowsel < (1 << ROWSEL_N_PINS); ++rowsel) {
            for (int x = 0; x < WIDTH; ++x) {
                gc_row[0][x] = gamma_correct_565_888(img[rowsel * WIDTH + x]);
                gc_row[1][x] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + rowsel) * WIDTH + x]);
            }
            for (int bit = 0; bit < 8; ++bit) {
                hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, bit);
                for (int x = 0; x < WIDTH; ++x) {
                    pio_sm_put_blocking(pio, sm_data, gc_row[0][x]);
                    pio_sm_put_blocking(pio, sm_data, gc_row[1][x]);
                }
                // Dummy pixel per lane
                pio_sm_put_blocking(pio, sm_data, 0);
                pio_sm_put_blocking(pio, sm_data, 0);
                // SM is finished when it stalls on empty TX FIFO
                hub75_wait_tx_stall(pio, sm_data);
                // Also check that previous OEn pulse is finished, else things can get out of sequence
                hub75_wait_tx_stall(pio, sm_row);

                // Latch row data, pulse output enable for new row.
                pio_sm_put_blocking(pio, sm_row, rowsel | ((BRIGTHNESS) * (1u << bit) << 5));
            }
        }

        #elif (XFER_MODE == XFER_DMA)
        switch(eMatrixState)
        {
            case MATRIX_INIT_GAMMA :

                u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[0].u32gammaData[0]; 

                for (int x = 0; x < PANNEL_WIDTH; x+=2) 
                {
                    u32Ptr1[x]   = gamma_correct_565_888(img[u8Row_m * WIDTH + u8Col]);
                    u32Ptr1[x+1] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + u8Row_m) * WIDTH + u8Col]);

                    u8Col++;
                }
                // clear colonne index
                u8Col = 0;

                // increment write index
                sMatrixData.u16WriteIndex++;

                u8Row_m++;

                // switch to the next state
                eMatrixState = MATRIX_SEND_DATA;
                break;
            
            case MATRIX_SEND_DATA :

                u16RdIdx = sMatrixData.u16ReadIndex; 
                u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[0]; 

                hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, u8Bit);
                /* send RGB datas */
                dma_channel_set_read_addr(dma_chan[0], u32Ptr1, TRIGGER_DMA);

                // switch to the next state
                eMatrixState = MATRIX_WAIT_DMA_END_XFER;
                break;

            case MATRIX_WAIT_DMA_END_XFER :

                pio->fdebug  = 1 << sm_data_msk;

                // wait for DMA completed tranfer
                // if(dma_channel_is_busy(dma_chan[0]) || (((pio->fdebug >> sm_data_msk) & (0x01)) == 0))
                if(((pio->fdebug >> sm_data_msk) & (0x01)) == 0)
                {
                    computeNextData();
                }
                else
                {
                    // switch to the next state
                    eMatrixState = MATRIX_ENABLE_ROW;
                }
                break;
            
            case MATRIX_ENABLE_ROW :

                // Latch row data, pulse output enable for new row.
                uint32_t u32Temp = ((uint32_t)u8Row | ((BRIGTHNESS) * (1u << u8Bit) << 5));

                // pio_sm_put_blocking(pio, sm_row, u8Row | ((BRIGTHNESS) * (1u << u8Bit) << 5));
                dma_channel_set_read_addr(dma_chan[1], &u32Temp, TRIGGER_DMA);

                // switch to the next state
                eMatrixState = MATRIX_WAIT_SM_ROW_STALL;
                break;

            case MATRIX_WAIT_SM_ROW_STALL :

                pio->fdebug = 1 << sm_row_msk;

                // check if SM row is stalled
                if (((pio->fdebug >> sm_row_msk) & (0x01)) == 0)
                {
                    if (u8Bit >= 5)
                    {
                        computeNextData();
                    }
                }
                else
                {
                    // switch to the next state
                    eMatrixState = MATRIX_BCM_WAIT;
                }
                break;

            case MATRIX_BCM_WAIT :

                // increment bit BCM
                u8Bit += 1;

                if (u8Bit == 8)
                {
                    //  clear bit BCM
                    u8Bit = 0;

                    // go to the next row
                    u8Row = (u8Row + 1) % (1u << ROWSEL_N_PINS);

                    // increment read index
                    sMatrixData.u16ReadIndex = (u16RdIdx + 1) % QUEUE_LEN; 

                    // check if we are reached the last element of the queue
                    if (isQueueFull() && (sMatrixData.u16ReadIndex == (QUEUE_LEN - 1)))
                    {
                        sMatrixData.u16WriteIndex = 0;
                    }
                }

                // switch to the next state
                eMatrixState = MATRIX_SEND_DATA;
                break;

            default :
                break;
        }
        #elif (XFER_MODE == XFER_RGB6)
        #endif

        #endif

        #if PIO_DEF == 0
        #if BCM_DEF == 0

        for (int rowsel = 0; rowsel < (1 << ROWSEL_N_PINS); ++rowsel) 
        {
            for (int x = 0; x < WIDTH; ++x) 
            {
                gc_row[0][x] = gamma_correct_565_888(img[rowsel * WIDTH + x]);
                gc_row[1][x] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + rowsel) * WIDTH + x]);
            }

            for (int bit = 0; bit < 8; ++bit) 
            {
                for (int x = 0; x < WIDTH; ++x)
                {
                    sPixel_t Data[2];

                    uint8_t r[2], g[2], b[2];
                    uint8_t rgbData = 0;

                    /* fetch all pixel */
                    Data[0].u32All = gc_row[0][x];
                    Data[1].u32All = gc_row[1][x];

                    r[0] = (Data[0].r >> bit) & 0x01;
                    g[0] = (Data[0].g >> bit) & 0x01;
                    b[0] = (Data[0].b >> bit) & 0x01;

                    r[1] = (Data[1].r >> bit) & 0x01;
                    g[1] = (Data[1].g >> bit) & 0x01;
                    b[1] = (Data[1].b >> bit) & 0x01;

                    rgbData = (r[0] | (g[0] << 1) | (b[0] << 2) | (r[1] << 3) | (g[1] << 4) | (b[1] << 5));

                    /* generate pulse of clock and write RGD pixel in shift registers */
                    WRITE_CLK(0);
                    WRITE_RGB(rgbData);
                    WRITE_CLK(1);
                }
                
                /* generate pulse of LAT pin */
                WRITE_ROW(rowsel, OE_DISABLE, LAT_DISABLE); 
                WRITE_ROW(rowsel, OE_DISABLE, LAT_ENABLE); 

                /* enable OE pin */
                WRITE_ROW(rowsel, OE_ENABLE, LAT_DISABLE);

                uint16_t oeDelay = (BRIGTHNESS * (1u << bit)) + 1;

                /* enable OE pin during OE delay time */
                while(oeDelay--);
                // WRITE_ROW(rowsel, OE_DISABLE, LAT_DISABLE); 
            }
        }

        #else
        //  switch(eMatrixState)
        // {
        //     case MATRIX_INIT_GAMMA :
        //         u16RdIdx = sMatrixData.u16ReadIndex; 
        //         u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[0]; 
        //         u32Ptr2 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[1]; 

        //         for (int x = 0; x < WIDTH; ++x) 
        //         {
        //             u32Ptr1[x] = gamma_correct_565_888(img[u8Row * WIDTH + x]);
        //             u32Ptr2[x] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + u8Row) * WIDTH + x]);
        //         }

        //         //  clear bit BCM
        //         u8Bit = 0;

        //         // switch to the next state
        //         eMatrixState = MATRIX_SEND_DATA;
        //         break;
            
        //     case MATRIX_SEND_DATA :

        //         // u16RdIdx = sMatrixData.u16ReadIndex; 
        //         // u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[0]; 
        //         // u32Ptr2 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[1]; 

        //         hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, u8Bit);
        //         for (int x = 0; x < WIDTH; ++x) {
        //             pio_sm_put_blocking(pio, sm_data, u32Ptr1[x]);
        //             pio_sm_put_blocking(pio, sm_data, u32Ptr2[x]);
        //         }

        //         // Dummy pixel per lane
        //         pio_sm_put_blocking(pio, sm_data, 0);
        //         pio_sm_put_blocking(pio, sm_data, 0);

        //         // SM is finished when it stalls on empty TX FIFO
        //         hub75_wait_tx_stall(pio, sm_data);
        //         // Also check that previous OEn pulse is finished, else things can get out of sequence
        //         hub75_wait_tx_stall(pio, sm_row);

        //         // // increment read index
        //         // sMatrixData.u16ReadIndex = u16RdIdx + 1; 

        //         // switch to the next state
        //         eMatrixState = MATRIX_BCM_WAIT;
        //         break;
            
        //     case MATRIX_BCM_WAIT :
        //         // Latch row data, pulse output enable for new row.
        //         pio_sm_put_blocking(pio, sm_row, u8Row | ((BRIGTHNESS) * (1u << u8Bit) << 5));

        //         // increment bit BCM
        //         u8Bit += 1;

        //         if (u8Bit == 8)
        //         {
        //             // go to the next row
        //             u8Row = (u8Row + 1) % (1u << ROWSEL_N_PINS);

        //             // switch to the next state
        //             eMatrixState = MATRIX_INIT_GAMMA;
        //         }
        //         else
        //         {
        //             // switch to the next state
        //             eMatrixState = MATRIX_SEND_DATA;
        //         }

        //         break;

        //     default :
        //         break;
        // }

        switch(eMatrixState)
        {
            case MATRIX_INIT_GAMMA :
                u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[0].u32gammaData[0]; 
                u32Ptr2 = (uint32_t *)&sMatrixData.sFrame[0].u32gammaData[1]; 

                for (int x = 0; x < WIDTH; ++x) 
                {
                    u32Ptr1[x] = gamma_correct_565_888(img[u8Row * WIDTH + x]);
                    u32Ptr2[x] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + u8Row) * WIDTH + x]);
                }

                // increment write index
                sMatrixData.u16WriteIndex++;

                u8Row_m++;

                // switch to the next state
                eMatrixState = MATRIX_SEND_DATA;
                break;
            
            case MATRIX_SEND_DATA :

                u16RdIdx = sMatrixData.u16ReadIndex; 
                u32Ptr1 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[0]; 
                u32Ptr2 = (uint32_t *)&sMatrixData.sFrame[u16RdIdx].u32gammaData[1];

                hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, u8Bit);
                for (int x = 0; x < WIDTH; ++x) {
                    pio_sm_put_blocking(pio, sm_data, u32Ptr1[x]);
                    pio_sm_put_blocking(pio, sm_data, u32Ptr2[x]);
                }

                // Dummy pixel per lane
                pio_sm_put_blocking(pio, sm_data, 0);
                pio_sm_put_blocking(pio, sm_data, 0);

                // SM is finished when it stalls on empty TX FIFO
                hub75_wait_tx_stall(pio, sm_data);

                // Latch row data, pulse output enable for new row.
                pio_sm_put_blocking(pio, sm_row, u8Row | ((BRIGTHNESS) * (1u << u8Bit) << 5));

                // switch to the next state
                eMatrixState = MATRIX_WAIT_STATE_MACHINE_STALL;
                break;
            
            case MATRIX_WAIT_STATE_MACHINE_STALL :
                uint32_t txstall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_row);
                uint16_t u16WrIdx = sMatrixData.u16WriteIndex;
                pio->fdebug = txstall_mask;

                if (!(pio->fdebug & txstall_mask))
                {
                    if ((u8Bit > 4) && (!isQueueFull()))
                    {
                        sMatrixData.sFrame[u16WrIdx].u32gammaData[0][u8Col] = gamma_correct_565_888(img[u8Row_m * WIDTH + u8Col]);
                        sMatrixData.sFrame[u16WrIdx].u32gammaData[1][u8Col] = gamma_correct_565_888(img[((1u << ROWSEL_N_PINS) + u8Row_m) * WIDTH + u8Col]);

                        // increment column
                        u8Col++;

                        // check if we are fetched all pixel of the selected line
                        if (u8Col == WIDTH)
                        {
                            sMatrixData.u16WriteIndex = u16WrIdx + 1;

                            u8Row_m = (u8Row_m + 1) % (1u << ROWSEL_N_PINS);

                            u8Col = 0;
                        }
                    }
                }
                else
                {
                    // switch to the next state
                    eMatrixState = MATRIX_BCM_WAIT;
                }

                break;

            case MATRIX_BCM_WAIT :
                {
                    // increment bit BCM
                    u8Bit += 1;

                    if (u8Bit == 8)
                    {
                        //  clear bit BCM
                        u8Bit = 0;

                        // go to the next row
                        u8Row = (u8Row + 1) % (1u << ROWSEL_N_PINS);

                        // increment read index
                        sMatrixData.u16ReadIndex = (u16RdIdx + 1) % QUEUE_LEN; 

                        // check if we are reached the last element of the queue
                        if (isQueueFull() && (sMatrixData.u16ReadIndex == (QUEUE_LEN - 1)))
                        {
                            sMatrixData.u16WriteIndex = 0;
                        }
                    }

                    // switch to the next state
                    eMatrixState = MATRIX_SEND_DATA;
                }

                break;

            default :
                break;
        }
        #endif;

        #endif
    }

}
