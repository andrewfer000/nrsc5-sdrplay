#include <assert.h>
#include <string.h>

#include "private.h"

#ifdef __MINGW32__
#define NRSC5_API __declspec(dllexport)
#else
#define NRSC5_API
#endif


static void stream_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
                            unsigned int numSamples, unsigned int reset, void *cbContext)
{
    nrsc5_t *st = (nrsc5_t *) cbContext;

    unsigned int num_loops = numSamples / 4;
    for (unsigned int i = 0; i < num_loops; i++) {
        st->samples_buf[8*i+0] = xi[4*i];
        st->samples_buf[8*i+2] = xi[4*i+1];
        st->samples_buf[8*i+4] = xi[4*i+2];
        st->samples_buf[8*i+6] = xi[4*i+3];
        st->samples_buf[8*i+1] = xq[4*i];
        st->samples_buf[8*i+3] = xq[4*i+1];
        st->samples_buf[8*i+5] = xq[4*i+2];
        st->samples_buf[8*i+7] = xq[4*i+3];
    }
    switch (numSamples % 4) {
        case 0:
            break;
        case 1:
            st->samples_buf[numSamples*2-2] = xi[numSamples-1];
            st->samples_buf[numSamples*2-1] = xq[numSamples-1];
            break;
        case 2:
            st->samples_buf[numSamples*2-4] = xi[numSamples-2];
            st->samples_buf[numSamples*2-2] = xi[numSamples-1];
            st->samples_buf[numSamples*2-3] = xq[numSamples-2];
            st->samples_buf[numSamples*2-1] = xq[numSamples-1];
            break;
        case 3:
            st->samples_buf[numSamples*2-6] = xi[numSamples-3];
            st->samples_buf[numSamples*2-4] = xi[numSamples-2];
            st->samples_buf[numSamples*2-2] = xi[numSamples-1];
            st->samples_buf[numSamples*2-5] = xq[numSamples-3];
            st->samples_buf[numSamples*2-3] = xq[numSamples-2];
            st->samples_buf[numSamples*2-1] = xq[numSamples-1];
            break;
    }
    input_push_cs16(&st->input, st->samples_buf, numSamples*2);
    return;
}

static void event_callback(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner,
                           sdrplay_api_EventParamsT *params, void *cbContext)
{
    nrsc5_t *st = (nrsc5_t *) cbContext;

    /* display warning for overloads */
    switch (eventId) {
    case sdrplay_api_PowerOverloadChange:
        /* send ack back for overload events */
        switch (params->powerOverloadParams.powerOverloadChangeType) {
        case sdrplay_api_Overload_Detected:
            log_warn("overload detected - please increase gain reduction");
            break;
        case sdrplay_api_Overload_Corrected:
            log_warn("overload corrected");
            break;
        }
        sdrplay_api_Update(st->dev.dev, st->dev.tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck, sdrplay_api_Update_Ext1_None);
        break;
    /* these case are not handled (yet) */
    case sdrplay_api_GainChange:
    case sdrplay_api_DeviceRemoved:
    case sdrplay_api_RspDuoModeChange:
        break;
    }
    return;
}

static void nrsc5_init(nrsc5_t *st)
{
    st->closed = 0;
    st->stopped = 1;
    st->gain = 0;
    st->freq = NRSC5_SCAN_BEGIN;
    st->mode = NRSC5_MODE_FM;
    st->callback = NULL;

    output_init(&st->output, st);
    input_init(&st->input, st, &st->output);
}

static nrsc5_t *nrsc5_alloc()
{
    nrsc5_t *st = calloc(1, sizeof(*st));
    return st;
}

NRSC5_API int nrsc5_open(nrsc5_t **result, char *device_serial, char *antenna)
{
    sdrplay_api_ErrT err;
    nrsc5_t *st = nrsc5_alloc();

    if ((err = sdrplay_api_Open()) != sdrplay_api_Success)
        goto error_init;

    unsigned int n_devs = 0;

    sdrplay_api_LockDeviceApi();
    sdrplay_api_DeviceT devs[SDRPLAY_MAX_DEVICES];
    sdrplay_api_GetDevices(devs, &n_devs, SDRPLAY_MAX_DEVICES);

    unsigned int dev_idx = 0;
    if (device_serial) {
        int device_serial_length = strlen(device_serial);
        if (device_serial_length == 1) {
            sscanf(device_serial, "%u", &dev_idx);
        } else if (device_serial_length > 1) {
            for (dev_idx = 0; dev_idx < n_devs; ++dev_idx) {
                if (strcmp(devs[dev_idx].SerNo, device_serial) == 0)
                    break;
            }
        }
    }
    if (dev_idx >= n_devs)
        goto error_release_api_lock;
    st->dev = devs[dev_idx];
    if (st->dev.hwVer == SDRPLAY_RSPduo_ID) {
        if (st->dev.rspDuoMode & sdrplay_api_RspDuoMode_Single_Tuner) {
            st->dev.rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
            if (antenna && (strcmp(antenna, "Tuner 2") == 0 || strncmp(antenna, "Tuner 2 ", 8) == 0)) {
                st->dev.tuner = sdrplay_api_Tuner_B;
            } else {
                st->dev.tuner = sdrplay_api_Tuner_A;
            } 
        } else {
            goto error_release_api_lock;
        }
    }

    if ((err = sdrplay_api_SelectDevice(&st->dev)) != sdrplay_api_Success)
        goto error_release_api_lock;
    sdrplay_api_UnlockDeviceApi();
    if ((err = sdrplay_api_GetDeviceParams(st->dev.dev, &st->dev_params)) != sdrplay_api_Success)
        goto error;
    st->ch_params = st->dev.tuner == sdrplay_api_Tuner_B ? st->dev_params->rxChannelB : st->dev_params->rxChannelA;

    /* default settings:
     *   - rf: 200MHz
     *   - fs: 2MHz
     *   - decimation: off
     *   - IF: 0kHz (zero IF)
     *   - bw: 200kHz
     *   - attenuation: 50dB
     *   - LNA state: 0
     *   - AGC: 50Hz
     *   - DC correction: on
     *   - IQ balance: on
    */

    int decimation = 4;
    st->ch_params->ctrlParams.decimation.enable = 1;
    st->ch_params->ctrlParams.decimation.decimationFactor = decimation;
#if 0
    st->ch_params->ctrlParams.decimation.wideBandSignal = 1;
#endif
    st->dev_params->devParams->fsFreq.fsHz = SAMPLE_RATE * decimation / 2;
    /* increase bandwidth since NRSC5 requires about 400kHz */
    st->ch_params->tunerParams.bwType = sdrplay_api_BW_0_600;

    nrsc5_init(st);

    *result = st;
    return 0;

error:
    log_error("nrsc5_open error: %d", err);
    sdrplay_api_LockDeviceApi();
    sdrplay_api_ReleaseDevice(&st->dev);
error_release_api_lock:
    sdrplay_api_UnlockDeviceApi();
    sdrplay_api_Close();
error_init:
    free(st);
    *result = NULL;
    return 1;
}

NRSC5_API int nrsc5_open_pipe(nrsc5_t **result)
{
    nrsc5_t *st = nrsc5_alloc();
    nrsc5_init(st);

    *result = st;
    return 0;
}

NRSC5_API void nrsc5_close(nrsc5_t *st)
{
    if (!st)
        return;

    st->closed = 1;

    if (st->dev.SerNo[0]) {
        sdrplay_api_LockDeviceApi();
        sdrplay_api_ReleaseDevice(&st->dev);
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
    }

    input_free(&st->input);
    output_free(&st->output);
    free(st);
}

NRSC5_API void nrsc5_start(nrsc5_t *st)
{
    sdrplay_api_CallbackFnsT callbacks;
    callbacks.StreamACbFn = stream_callback;
    callbacks.StreamBCbFn = 0;
    callbacks.EventCbFn = event_callback;

/* fv - debug info */
if (st->dev.hwVer == SDRPLAY_RSP2_ID) {
    fprintf(stderr, "antenna: %d\n", st->ch_params->rsp2TunerParams.antennaSel);
    fprintf(stderr, "AM port: %d\n", st->ch_params->rsp2TunerParams.amPortSel);
} else if (st->dev.hwVer == SDRPLAY_RSPduo_ID) {
    fprintf(stderr, "tuner: %d\n", st->dev.tuner);
    fprintf(stderr, "AM port: %d\n", st->ch_params->rspDuoTunerParams.tuner1AmPortSel);
} else if (st->dev.hwVer == SDRPLAY_RSPdx_ID) {
    fprintf(stderr, "antenna: %d\n", st->dev_params->devParams->rspDxParams.antennaSel);
}
fprintf(stderr, "LNA state: %d\n", st->ch_params->tunerParams.gain.LNAstate);
fprintf(stderr, "IF gain reduction: %d\n", st->ch_params->tunerParams.gain.gRdB);
fprintf(stderr, "IF AGC: %d\n", st->ch_params->ctrlParams.agc.enable);
/* fv - end debug info */

    sdrplay_api_ErrT err;
    if ((err = sdrplay_api_Init(st->dev.dev, &callbacks, (void *)st)) != sdrplay_api_Success) {
        log_error("sdrplay_api_Init failed");
        return;
    }
    st->stopped = 0;
}

NRSC5_API void nrsc5_stop(nrsc5_t *st)
{
    sdrplay_api_ErrT err;
    if ((err = sdrplay_api_Uninit(st->dev.dev)) != sdrplay_api_Success) {
        log_error("sdrplay_api_Uninit failed");
        return;
    }
    st->stopped = 1;
}

NRSC5_API int nrsc5_set_mode(nrsc5_t *st, int mode)
{
    if (mode == NRSC5_MODE_FM || mode == NRSC5_MODE_AM)
    {
        st->mode = mode;
        input_set_mode(&st->input);
        return 0;
    }
    return 1;
}

NRSC5_API int nrsc5_set_antenna(nrsc5_t *st, char *antenna)
{
    if (!antenna)
        return 0;

    if (st->ch_params)
    {
        if (st->dev.hwVer == SDRPLAY_RSP2_ID) {
            if (strcmp(antenna, "Antenna A") == 0) {
                st->ch_params->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
                st->ch_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
            } else if (strcmp(antenna, "Antenna B") == 0) {
                st->ch_params->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
                st->ch_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
            } else if (strcmp(antenna, "Hi-Z") == 0) {
                st->ch_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_1;
            }
        } else if (st->dev.hwVer == SDRPLAY_RSPduo_ID) {
            if (strcmp(antenna, "Tuner 1 50 ohm") == 0) {
                st->dev.tuner = sdrplay_api_Tuner_A;
                st->ch_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
            } else if (strcmp(antenna, "Tuner 2 50 ohm") == 0) {
                st->dev.tuner = sdrplay_api_Tuner_B;
                st->ch_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
            } else if (strcmp(antenna, "Tuner 1 Hi-Z") == 0) {
                st->dev.tuner = sdrplay_api_Tuner_A;
                st->ch_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
            }
        }
    }

    if (st->dev_params)
    {
        if (st->dev.hwVer == SDRPLAY_RSPdx_ID) {
            if (strcmp(antenna, "Antenna A") == 0) {
                st->dev_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_A;
            } else if (strcmp(antenna, "Antenna B") == 0) {
                st->dev_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_B;
            } else if (strcmp(antenna, "Antenna C") == 0) {
                st->dev_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_C;
            }
        }
    }

    return 0;
}

NRSC5_API int nrsc5_set_freq_correction(nrsc5_t *st, int ppm_error)
{
    if (st->dev_params)
    {
        st->dev_params->devParams->ppm = ppm_error;
    }
    return 0;
}

NRSC5_API void nrsc5_get_frequency(nrsc5_t *st, float *freq)
{
    if (st->ch_params)
        *freq = st->ch_params->tunerParams.rfFreq.rfHz;
    else
        *freq = st->freq;
}

NRSC5_API int nrsc5_set_frequency(nrsc5_t *st, float freq)
{
    if (st->freq == freq)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->ch_params)
        st->ch_params->tunerParams.rfFreq.rfHz = freq;

    input_reset(&st->input);
    output_reset(&st->output);

    st->freq = freq;
    return 0;
}

NRSC5_API void nrsc5_get_gain(nrsc5_t *st, float *gain)
{
    if (st->ch_params) {
        float rfgr = st->ch_params->tunerParams.gain.LNAstate;
        float ifgr = st->ch_params->ctrlParams.agc.enable == sdrplay_api_AGC_DISABLE ? st->ch_params->tunerParams.gain.gRdB : 0;
        *gain = rfgr + ifgr / 100.0f;
    } else {
        *gain = st->gain;
    }
}

NRSC5_API int nrsc5_set_gain(nrsc5_t *st, float gain)
{
    if (st->gain == gain)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->ch_params) {
        int rfgr = (int) gain;
        int ifgr = (int) ((gain - rfgr) * 100.0f + 0.00001);
        st->ch_params->tunerParams.gain.LNAstate = (unsigned char) rfgr;
        if (ifgr == 0) {
            st->ch_params->ctrlParams.agc.setPoint_dBfs = -60;
            st->ch_params->ctrlParams.agc.enable = sdrplay_api_AGC_50HZ;
        } else {
            st->ch_params->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
            st->ch_params->tunerParams.gain.gRdB = ifgr;
        }
    }

    st->gain = gain;
    return 0;
}

NRSC5_API void nrsc5_set_callback(nrsc5_t *st, nrsc5_callback_t callback, void *opaque)
{
    st->callback = callback;
    st->callback_opaque = opaque;
}

NRSC5_API int nrsc5_pipe_samples_cu8(nrsc5_t *st, uint8_t *samples, unsigned int length)
{
    input_push_cu8(&st->input, samples, length);

    return 0;
}

NRSC5_API int nrsc5_pipe_samples_cs16(nrsc5_t *st, int16_t *samples, unsigned int length)
{
    input_push_cs16(&st->input, samples, length);

    return 0;
}
