/*
 * LimeMicroSystem transceiver driver
 * Copyright (C) 2015-2020 Amarisoft/LimeMicroSystems
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <getopt.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <lime/LimeSuite.h>

extern "C" {
#include "trx_driver.h"
};

#define CALIBRATE_FILTER    2
#define CALIBRATE_IQDC      1
#define MAX_NUM_CH 2
using namespace std;
typedef struct TRXLmsState          TRXLmsState;

struct TRXLmsState {
    lms_device_t *device;
    lms_stream_t rx_stream[MAX_NUM_CH];
    lms_stream_t tx_stream[MAX_NUM_CH];
    int tcxo_calc;          /* values from 0 to 255*/
    int dec_inter;
    int started;
    int sample_rate;
    int tx_channel_count;
    int rx_channel_count;
    int calibrate;
    int ini_file;
    float rx_power;
    float tx_power;
    int tdd_trx_start_delay;
    int tdd_trx_stop_delay;
    int tdd_trx_switch_mode;
    int tdd_trx_switch_dir;
    int tdd_mode_reg;
    bool rx_power_available;
    bool tx_power_available;
    bool tdd_tx_en_ctrl;
    bool tdd_tx_en_dir;
    bool tdd_rx_en_ctrl;
    bool tdd_rx_en_dir;
};


int16_t* tx_buffers[MAX_NUM_CH] = {0};
int16_t* rx_buffers[MAX_NUM_CH] = {0};

static int64_t trx_lms_t0 = 0;

static int64_t get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + (ts.tv_nsec / 1000U) - trx_lms_t0;
}

void LogHandler(int lvl, const char *msg)
{
    if (lvl <= LMS_LOG_ERROR) {
        fprintf(stderr, "%s\n", msg);
        fprintf(stderr, "Received error from LimeSuite library. Terminating..\n");
        std::terminate();
    }
    printf("%s\n", msg);
}

static void trx_lms7002m_end(TRXState *s1)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    for (int ch = 0; ch < s->rx_channel_count; ch++)
	LMS_StopStream(&s->rx_stream[ch]);

    for (int ch = 0; ch < s->tx_channel_count; ch++)
	LMS_StopStream(&s->tx_stream[ch]);

    for (int ch = 0; ch < s->rx_channel_count; ch++)
	LMS_DestroyStream(s->device,&s->rx_stream[ch]);

    for (int ch = 0; ch < s->tx_channel_count; ch++)
	LMS_DestroyStream(s->device,&s->tx_stream[ch]);

    LMS_Close(s->device);
    for (int ch = 0; ch < s->rx_channel_count; ch++)
        if (rx_buffers[ch]) delete [] rx_buffers[ch];
    for (int ch = 0; ch < s->tx_channel_count; ch++)
        if (tx_buffers[ch]) delete [] tx_buffers[ch];
    free(s);
}

static void trx_lms7002m_write(TRXState *s1, trx_timestamp_t timestamp,
                               const void **samples, int count, int flags,
                               int rf_port_index)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;

    // Nothing to transmit
    if (!samples)
        return;
    lms_stream_meta_t meta;
    meta.waitForTimestamp = true;
    meta.flushPartialPacket = (flags&TRX_WRITE_FLAG_END_OF_BURST);
    meta.timestamp = timestamp;

    for (int ch = 0; ch < s->tx_channel_count; ch++)
    	LMS_SendStream(&s->tx_stream[ch],(const void*)samples[ch],count,&meta,30);
}

static int trx_lms7002m_read(TRXState *s1, trx_timestamp_t *ptimestamp, void **psamples, int count, int port)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    lms_stream_meta_t meta;
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;
    // First shot ?
    if (!s->started) {
    	for (int ch = 0; ch < s->rx_channel_count; ch++)
		LMS_StartStream(&s->rx_stream[ch]);
    	for (int ch = 0; ch < s->tx_channel_count; ch++)
		LMS_StartStream(&s->tx_stream[ch]);
        s->started = 1;
        printf("START\n");
    }

    int ret;
    for (int ch = 0; ch < s->rx_channel_count; ch++)
    	ret = LMS_RecvStream(&s->rx_stream[ch],psamples[ch],count,&meta,30);

    *ptimestamp = meta.timestamp;

    return ret;
}

void trx_lms7002m_write2(TRXState *s, trx_timestamp_t timestamp, const void **samples, int count, int port, TRXWriteMetadata *md)
{
    trx_lms7002m_write(s, timestamp, samples, count, md->flags, port);
}
int trx_lms7002m_read2(TRXState *s, trx_timestamp_t *ptimestamp, void **samples, int count, int port, TRXReadMetadata *md)
{
    return trx_lms7002m_read(s, ptimestamp, samples, count, port);
}


static void trx_lms7002m_write_int(TRXState *s1, trx_timestamp_t timestamp,
                               const void **samples, int count, int flags,
                               int rf_port_index)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    const float maxValue = s->tx_stream->dataFmt == lms_stream_t::LMS_FMT_I12 ? 2047.0f : 32767.0f;
    // Nothing to transmit
    if (!samples)
        return;
    lms_stream_meta_t meta;
    meta.waitForTimestamp = true;
    meta.flushPartialPacket = false;
    meta.timestamp = timestamp;

    for (int ch = 0; ch < s->tx_channel_count; ch++)
    	for (int i = 0; i < count*2; i++)
	    tx_buffers[ch][i] = ((const float*)samples[ch])[i]*maxValue;

    for (int ch = 0; ch < s->tx_channel_count; ch++)
    	LMS_SendStream(&s->tx_stream[ch],(const void*)tx_buffers[ch],count,&meta,30);
}

static int trx_lms7002m_read_int(TRXState *s1, trx_timestamp_t *ptimestamp, void **psamples, int count, int port)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    const float maxValue = s->rx_stream->dataFmt == lms_stream_t::LMS_FMT_I12 ? 2048.0f : 32768.0f;
    lms_stream_meta_t meta;
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;

    // First shot ?
    if (!s->started) {
        for (int ch = 0; ch < s->rx_channel_count; ch++)
            rx_buffers[ch] = new int16_t[count*2];
        for (int ch = 0; ch < s->tx_channel_count; ch++)
            tx_buffers[ch] = new int16_t[count*2];
    	for (int ch = 0; ch < s->rx_channel_count; ch++)
		LMS_StartStream(&s->rx_stream[ch]);
    	for (int ch = 0; ch < s->tx_channel_count; ch++)
		LMS_StartStream(&s->tx_stream[ch]);
        s->started = 1;
        printf("START\n");
    }

    int ret;
    for (int ch = 0; ch < s->rx_channel_count; ch++)
    	ret = LMS_RecvStream(&s->rx_stream[ch],rx_buffers[ch],count,&meta,30);

    for (int ch = 0; ch < s->rx_channel_count; ch++)
    	for (int i = 0; i < count*2; i++)
	    ((float*)psamples[ch])[i] = rx_buffers[ch][i]/maxValue;

    *ptimestamp = meta.timestamp;

    return ret;
}

void trx_lms7002m_write_int2(TRXState *s, trx_timestamp_t timestamp, const void **samples, int count, int port, TRXWriteMetadata *md)
{
    trx_lms7002m_write_int(s, timestamp, samples, count, md->flags, port);
}
int trx_lms7002m_read_int2(TRXState *s, trx_timestamp_t *ptimestamp, void **samples, int count, int port, TRXReadMetadata *md)
{
    return trx_lms7002m_read_int(s, ptimestamp, samples, count, port);
}

static int trx_lms7002m_get_sample_rate(TRXState *s1, TRXFraction *psample_rate,
                                     int *psample_rate_num, int sample_rate_min)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;

    // sample rate not specified, align on 1.92Mhz
    if (s->sample_rate <= 0)
    {
        if (!s->ini_file)
        {
            int i, n;
            static const char sample_rate_tab[] = {1,2,4,8,12,16};
            for(i = 0; i < sizeof(sample_rate_tab); i++)
            {
                n = sample_rate_tab[i];
                if (sample_rate_min <= n * 1920000)
                {
                    *psample_rate_num = n;
                    psample_rate->num = n * 1920000;
                    psample_rate->den = 1;
                    s->sample_rate = psample_rate->num;
                    printf("Automatic sample rate: %f MSps\n", (double)s->sample_rate/1e6);
                    return 0;
                }
            }
        }
        else
        {
            double srate;
            LMS_GetSampleRate(s->device, LMS_CH_RX, 0, &srate, nullptr);
	    printf("Use sample rate from INI file %f MSps\n", srate/1e6);
            psample_rate->num = int(srate);
            psample_rate->den = 1;
            *psample_rate_num = 0;
       	    return 0;
        }

    }
    else
    {
        printf("Sample rate: %f MSps\n", (double)s->sample_rate/1e6);
        psample_rate->num = s->sample_rate;
        psample_rate->den = 1;
        *psample_rate_num = 0;
        return 0;
    }
    return -1;
}

static int trx_lms7002m_get_tx_samples_per_packet_func(TRXState *s1)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;

    return (s->tx_stream->dataFmt == lms_stream_t::LMS_FMT_I12 ? 1360 : 1020)/s->tx_channel_count;
}

 static int trx_lms7002m_get_abs_rx_power_func(TRXState *s1, float *presult, int channel_num)
 {
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    if (s->rx_power_available)
    {
        *presult = s->rx_power;
        printf("rx power %f\n", *presult);
        return 0;
    }
    return -1;
 }

 static int trx_lms7002m_get_abs_tx_power_func(TRXState *s1, float *presult, int channel_num)
 {
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    if (s->tx_power_available)
    {
        *presult = s->tx_power;
        printf("tx power %f\n", *presult);
        return 0;
    }
    return -1;
 }



//min gain 0
//max gain ~70-76 (higher will probably degrade signal quality to much)
static void trx_lms7002m_set_tx_gain_func(TRXState *s1, double gain, int channel_num)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    if (LMS_SetGaindB(s->device, LMS_CH_TX, channel_num, gain)!=0)
        fprintf(stderr, "Failed to set Tx gain\n");
}

//min gain 0
//max gain Rx: 73
static void trx_lms7002m_set_rx_gain_func(TRXState *s1, double gain, int channel_num)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;
    if (LMS_SetGaindB(s->device, LMS_CH_RX, channel_num, gain)!=0)
        fprintf(stderr, "Failed to set Rx gain\n");
}

static int trx_lms7002m_start(TRXState *s1, const TRXDriverParams *p)
{
    TRXLmsState *s = (TRXLmsState*)s1->opaque;

    if (p->rf_port_count != 1) {
        fprintf(stderr, "Only one port allowed\n");
        return -1;
    }

    LMS_WriteFPGAReg(s->device,0x18, s->tdd_mode_reg);
    LMS_WriteFPGAReg(s->device,0x10, s->tdd_trx_start_delay);
    LMS_WriteFPGAReg(s->device,0x11, s->tdd_trx_stop_delay);

    LMS_WriteFPGAReg(s->device,0xCC, s->tdd_trx_switch_mode);
    LMS_WriteFPGAReg(s->device,0xCD, s->tdd_trx_switch_dir);
    LMS_WriteFPGAReg(s->device,0x17, 0x0011);

    s->tx_channel_count = p->tx_channel_count;
    s->rx_channel_count = p->rx_channel_count;

    if (s->ini_file == 0)
    {
        for(int ch=0; ch< s->rx_channel_count; ++ch)
        {
            printf("Set CH%d gains: rx %1.0f; tx %1.0f\n",ch+1, p->rx_gain[ch], p->tx_gain[ch]);
            LMS_EnableChannel(s->device,LMS_CH_RX,ch,true);
            LMS_EnableChannel(s->device,LMS_CH_TX,ch,true);
            LMS_SetGaindB(s->device,LMS_CH_RX,ch,(int)(p->rx_gain[ch]+0.5));
            LMS_SetGaindB(s->device,LMS_CH_TX,ch,(int)(p->tx_gain[ch]+0.5));
        }
    }
    else
    {
        for(int ch=0; ch< s->rx_channel_count; ++ch)
        {
	    int ant = LMS_GetAntenna(s->device, LMS_CH_RX, ch);
	    LMS_SetAntenna(s->device, LMS_CH_RX, ch, ant);
	}
        for(int ch=0; ch< s->tx_channel_count; ++ch)
        {
	    int ant = LMS_GetAntenna(s->device, LMS_CH_TX, ch);
	    LMS_SetAntenna(s->device, LMS_CH_TX, ch, ant);
	}
    }

    printf ("CH RX %d; TX %d\n",s->rx_channel_count,s->tx_channel_count);
    printf("SR:   %.3f MHz\n", (float)p->sample_rate[0].num / p->sample_rate[0].den/ 1e6);
    if (s->sample_rate || (!s->ini_file))
    {
        s->sample_rate = p->sample_rate[0].num / p->sample_rate[0].den;
        printf("DEC/INT: %d\n", s->dec_inter);
        if ((LMS_SetSampleRateDir(s->device, LMS_CH_RX, s->sample_rate,s->dec_inter)!=0)
         || (LMS_SetSampleRateDir(s->device, LMS_CH_TX, s->sample_rate,s->dec_inter)!=0))
        {
            fprintf(stderr, "Failed to set sample rate\n");
            return -1;
        }
    }
    printf ("CH RX %d; TX %d\n",s->rx_channel_count,s->tx_channel_count);

    for(int ch=0; ch< s->rx_channel_count; ++ch)
    {
	    printf ("setup RX stream %d\n",ch);
	    s->rx_stream[ch].channel = ch;
	    s->rx_stream[ch].fifoSize = 256*1024;
	    s->rx_stream[ch].throughputVsLatency = 0.3;
	    s->rx_stream[ch].isTx = false;
    	    if (LMS_SetupStream(s->device, &s->rx_stream[ch])!=0)
                return -1;
    }

    for(int ch=0; ch< s->tx_channel_count; ++ch)
    {
	    printf ("setup TX stream %d\n",ch);
	    s->tx_stream[ch].channel = ch;
	    s->tx_stream[ch].fifoSize = 256*1024;
	    s->tx_stream[ch].throughputVsLatency = 0.3;
	    s->tx_stream[ch].isTx = true;
	    if (LMS_SetupStream(s->device, &s->tx_stream[ch])!=0)
                return -1;
    }

    if (LMS_SetLOFrequency(s->device,LMS_CH_RX, 0, (double)p->rx_freq[0])!=0)
    {
        fprintf(stderr, "Failed to Set Rx frequency\n");
        return -1;
    }

    if (LMS_SetLOFrequency(s->device,LMS_CH_TX, 0,(double)p->tx_freq[0])!=0)
    {
        fprintf(stderr, "Failed to Set Tx frequency\n");
        return -1;
    }

    if (s->rx_channel_count > 2)
    {
	    if (LMS_SetLOFrequency(s->device,LMS_CH_RX, 2, (double)p->rx_freq[0])!=0)
	    {
		fprintf(stderr, "Failed to Set Rx frequency\n");
		return -1;
	    }
     }

    if (s->tx_channel_count > 2)
    {
	    if (LMS_SetLOFrequency(s->device,LMS_CH_TX, 2,(double)p->tx_freq[0])!=0)
	    {
		fprintf(stderr, "Failed to Set Tx frequency\n");
		return -1;
	    }
    }

    uint16_t reg13;
    LMS_ReadFPGAReg(s->device,0x13, &reg13);
    reg13 &= ~((1<<7)|(1 <<15)|(1<<8)|(1<<14));
    LMS_WriteFPGAReg(s->device,0x13, reg13 | 0x60);
    if (s->calibrate & CALIBRATE_FILTER)
    {
        for(int ch=0; ch< s->tx_channel_count; ++ch)
        {
            printf("Configuring Tx LPF for ch %i\n", ch);
            unsigned gain = p->tx_gain[ch];
            LMS_GetGaindB(s->device, LMS_CH_TX, ch, &gain);
            if (LMS_SetLPFBW(s->device, LMS_CH_TX, ch,(double)(p->tx_bandwidth[0]>5e6 ? p->tx_bandwidth[0] : 5e6))!=0)
                fprintf(stderr, "Failed set TX LPF\n");
	    LMS_SetGaindB(s->device, LMS_CH_TX, ch, gain);
        }

        for(int ch=0; ch< s->rx_channel_count; ++ch)
        {
            printf("Configuring Rx LPF for ch %i\n", ch);
            if (LMS_SetLPFBW(s->device, LMS_CH_RX, ch,(double)p->rx_bandwidth[0])!=0)
                fprintf(stderr, "Failed to set RX LPF\n");
        }
    }

    if (s->calibrate & CALIBRATE_IQDC)
    {
        for(int ch=0; ch< s->tx_channel_count; ++ch)
        {
            printf("Calibrating Tx channel :%i\n", ch);
            if (LMS_Calibrate(s->device, LMS_CH_TX, ch,(double)p->tx_bandwidth[0],0)!=0)
                fprintf(stderr, "Failed to calibrate Tx\n");
        }

        for(int ch=0; ch< s->rx_channel_count; ++ch)
        {
            printf("Calibrating Rx channel :%i\n", ch);
            if (LMS_Calibrate(s->device, LMS_CH_RX, ch,(double)p->rx_bandwidth[0],0)!=0)
                fprintf(stderr, "Failed to calibrate Rx\n");
        }
    }
    LMS_WriteFPGAReg(s->device,0x13, reg13|(s->tdd_tx_en_ctrl<<7)|(s->tdd_tx_en_dir<<14)|(s->tdd_rx_en_ctrl<<8)|(s->tdd_rx_en_dir<<15));
    LMS_RegisterLogHandler(LogHandler);
    printf("Running\n");
    return 0;
}

struct TDDModeParam
{
    std::string name;
    uint8_t offset;
    uint16_t mask;
};

const TDDModeParam tdd_params[] = {{"TX2_TOGGLE_INV", 13, 1},{"TX2_TOGGLE_EN",12,1},
    {"TX1_TOGGLE_INV", 9, 1},{"TX1_TOGGLE_EN",8,1},{"RX2_PATH_SEL",6,3},{"RX2_TOGGLE_INV",5,1},
    {"RX2_TOGGLE_EN", 4,1}, {"RX1_PATH_SEL", 2,3}, {"RX1_TOGGLE_INV", 1, 1}, {"RX1_TOGGLE_EN", 0,1}};

/* Driver initialization called at eNB startup */
int trx_driver_init(TRXState *s1)
{
    double val;
    char *configFile, *configFile1;
    int lms7002_index;
    int stream_index;
    TRXLmsState *s;
    lms_info_str_t list[16]={0};

    if (s1->trx_api_version != TRX_API_VERSION) {
        fprintf(stderr, "ABI compatibility mismatch between LTEENB and TRX driver (LTEENB ABI version=%d, TRX driver ABI version=%d)\n",
            s1->trx_api_version, TRX_API_VERSION);
        return -1;
    }

    s = (TRXLmsState*)malloc(sizeof(TRXLmsState));
    memset(s, 0, sizeof(*s));

    /* Few parameters */
    s->sample_rate = 0;
    if (trx_get_param_double(s1, &val, "sample_rate") >= 0)
        s->sample_rate = val*1e6;

    s->dec_inter = 0;
    if (trx_get_param_double(s1, &val, "dec_inter") >= 0)
        s->dec_inter = val;

    /* Get device index */
    lms7002_index = 0;
    if (trx_get_param_double(s1, &val, "lms7002_index") >= 0)
        lms7002_index = val;

    // Open LMS7002 port
    int n = LMS_GetDeviceList(list);

    if (n <= 0) {
        fprintf(stderr, "No LMS7002 board found: %d\n", n);
        return -1;
    }

    if (n <= lms7002_index)
    {
        printf("Requested device index (%d) not available\ndefaulting to using 1st device\n", lms7002_index);
        lms7002_index = 0;
    }

    if (LMS_Open(&(s->device),list[lms7002_index],stream_index>=0?list[stream_index]:nullptr)!=0) {
        fprintf(stderr, "Can't open lms port\n");
        return -1;
    }

    LMS_Program(s->device, nullptr, 0, "FX3 Reset", nullptr);
    LMS_Close(s->device);
    sleep(1);
    // Open LMS7002 port
    n= LMS_GetDeviceList(list);

    if (n <= lms7002_index || lms7002_index < 0) {
        fprintf(stderr, "No LMS7002 board found: %d\n", n);
        return -1;
    }

    if (LMS_Open(&(s->device),list[lms7002_index],stream_index>=0?list[stream_index]:nullptr)!=0) {
        fprintf(stderr, "Can't open lms port\n");
        return -1;
    }

    s->tcxo_calc = -1;
    if (trx_get_param_double(s1, &val, "tcxo_calc") >= 0)
    {
        s->tcxo_calc = val;
        LMS_WriteCustomBoardParam(s->device, 0, val, "");
	printf("DAC WRITE %d\n", s->tcxo_calc);
    }

    s->rx_power = 0.0;
    s->tx_power = 0.0;
    s->rx_power_available = false;
    s->tx_power_available = false;
    if (trx_get_param_double(s1, &val, "rx_power") >= 0)
    {
        s->rx_power = val;
        s->rx_power_available = true;
        printf("rx power %.1f dBm\n", s->rx_power);
    }
    if (trx_get_param_double(s1, &val, "tx_power") >= 0)
    {
        s->tx_power = val;
        s->tx_power_available = true;
        printf("tx power %.1f dBm\n", s->tx_power);
    }

    if (trx_get_param_double(s1, &val, "TDD_TRX_EN_START_DELAY") >= 0)
    {
        s->tdd_trx_start_delay = val;
        printf("TDD_TRX_EN_START_DELAY %d\n", s->tdd_trx_start_delay);
    }

    if (trx_get_param_double(s1, &val, "TDD_TRX_EN_STOP_DELAY") >= 0)
    {
        s->tdd_trx_stop_delay = val;
        printf("TDD_TRX_EN_STOP_DELAY %d\n", s->tdd_trx_stop_delay);
    }

    if (trx_get_param_double(s1, &val, "TDD_TRX_SWITCH_MODE") >= 0)
    {
        s->tdd_trx_switch_mode = val;
        printf("tx TDD_TRX_SWITCH_MODE %d\n", s->tdd_trx_switch_mode);
    }

    if (trx_get_param_double(s1, &val, "TDD_TRX_SWITCH_DIR") >= 0)
    {
        s->tdd_trx_switch_dir = val;
        printf("tx TDD_TRX_SWITCH_MODE %d\n", s->tdd_trx_switch_dir);
    }

    if (trx_get_param_double(s1, &val, "TDD_TX_EN_CTRL") >= 0)
    {
        s->tdd_tx_en_ctrl = val;
        printf("tx TDD_TX_EN_CTRL %d\n", s->tdd_tx_en_ctrl);
    }

    if (trx_get_param_double(s1, &val, "TDD_TX_EN_DIR") >= 0)
    {
        s->tdd_tx_en_dir = val;
        printf("tx TDD_TX_EN_DIR %d\n", s->tdd_tx_en_dir);
    }

    if (trx_get_param_double(s1, &val, "TDD_RX_EN_CTRL") >= 0)
    {
        s->tdd_rx_en_ctrl = val;
        printf("TDD_RX_EN_CTRL %d\n", s->tdd_rx_en_ctrl);
    }

    if (trx_get_param_double(s1, &val, "TDD_RX_EN_DIR") >= 0)
    {
        s->tdd_rx_en_dir = val;
        printf("TDD_RX_EN_DIR %d\n", s->tdd_rx_en_dir);
    }

    for (const auto& param : tdd_params){
        if (trx_get_param_double(s1, &val, param.name.c_str()) >= 0)
        {
            s->tdd_mode_reg &= ~(param.mask<<param.offset);
            s->tdd_mode_reg |= (param.mask & uint16_t(val)) << param.offset;
            printf("%s %d\n", param.name.c_str(), int(val));
        }
    }
    printf("TDD mode register (0x18) value %04X\n", s->tdd_mode_reg);

    //Configuration INI file
    configFile = trx_get_param_string(s1, "config_file");
    if (configFile)
    {
        configFile1 = (char*)malloc(strlen(s1->path) + strlen(configFile) + 2);
        sprintf(configFile1, "%s/%s", s1->path, configFile);

        fprintf(stderr, "Config file: %s\n", configFile1);
        if  (LMS_LoadConfig(s->device,configFile1)!=0) //load registers configuration from file
        {
            fprintf(stderr, "Can't open %s\n", configFile1);
            return -1;
        }
        free(configFile1);
        s->ini_file = 1;
    }
    else
    {
        if ( LMS_Init(s->device)!=0)
        {
            fprintf(stderr, "LMS Init failed\n");
            return -1;
        }
        s->ini_file = 0;
    }

    /* Auto calibration */
    char* calibration;
    LMS_EnableCache(s->device,false);
    calibration = trx_get_param_string(s1, "calibration");
    s->calibrate = CALIBRATE_FILTER;
    if (calibration)
    {
	if (!strcasecmp(calibration, "none"))
	    s->calibrate = 0;
	else if ((!strcasecmp(calibration, "force")) || (!strcasecmp(calibration, "all")))
            s->calibrate = CALIBRATE_FILTER | CALIBRATE_IQDC;
        else if (!strcasecmp(calibration, "filter"))
            s->calibrate = CALIBRATE_FILTER;
        else if (!strcasecmp(calibration, "iq_dc"))
            s->calibrate = CALIBRATE_IQDC;
        free(calibration);
    }
    /*sample format*/
    for (int i =0; i< MAX_NUM_CH; i++)
        s->rx_stream[i].dataFmt = s->tx_stream[i].dataFmt = lms_stream_t::LMS_FMT_F32;

    char* sampleFmt = trx_get_param_string(s1, "sample_format");
    if (sampleFmt) {
        if (strstr(sampleFmt,"16")!=nullptr){
            for (int i =0; i< MAX_NUM_CH; i++)
                s->rx_stream[i].dataFmt = s->tx_stream[i].dataFmt = lms_stream_t::LMS_FMT_I16;
        }
        else if (strstr(sampleFmt,"12")!=nullptr){
            for (int i =0; i< MAX_NUM_CH; i++)
                s->rx_stream[i].dataFmt = s->tx_stream[i].dataFmt = lms_stream_t::LMS_FMT_I12;
        }
        free(sampleFmt);
    }

    /* Set callbacks */
    s1->opaque = s;
    s1->trx_end_func = trx_lms7002m_end;
    s1->trx_write_func = s->rx_stream->dataFmt==lms_stream_t::LMS_FMT_F32 ? trx_lms7002m_write : trx_lms7002m_write_int;
    s1->trx_read_func = s->rx_stream->dataFmt==lms_stream_t::LMS_FMT_F32 ? trx_lms7002m_read : trx_lms7002m_read_int;
    s1->trx_write_func2 = s->rx_stream->dataFmt==lms_stream_t::LMS_FMT_F32 ? trx_lms7002m_write2 : trx_lms7002m_write_int2;
    s1->trx_read_func2 = s->rx_stream->dataFmt==lms_stream_t::LMS_FMT_F32 ? trx_lms7002m_read2 : trx_lms7002m_read_int2;
    s1->trx_start_func = trx_lms7002m_start;
    s1->trx_get_sample_rate_func = trx_lms7002m_get_sample_rate;
    s1->trx_get_tx_samples_per_packet_func = trx_lms7002m_get_tx_samples_per_packet_func;
    s1->trx_get_abs_rx_power_func = trx_lms7002m_get_abs_rx_power_func;
    s1->trx_get_abs_tx_power_func = trx_lms7002m_get_abs_tx_power_func;
    s1->trx_set_tx_gain_func = trx_lms7002m_set_tx_gain_func;
    s1->trx_set_rx_gain_func = trx_lms7002m_set_rx_gain_func;
    return 0;
}
