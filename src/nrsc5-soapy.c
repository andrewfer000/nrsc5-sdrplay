#include <assert.h>
#include <string.h>

#include "private.h"

#ifdef __MINGW32__
#define NRSC5_API __declspec(dllexport)
#else
#define NRSC5_API
#endif


#if 0
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
#endif

static void *worker_thread(void *arg)
{
    nrsc5_t *st = arg;

    pthread_mutex_lock(&st->worker_mutex);
    while (!st->closed)
    {
        if (st->stopped && !st->worker_stopped)
        {
            st->worker_stopped = 1;
            pthread_cond_broadcast(&st->worker_cond);
        }
        else if (!st->stopped && st->worker_stopped)
        {
            st->worker_stopped = 0;
            pthread_cond_broadcast(&st->worker_cond);
        }

        if (st->stopped)
        {
            // wait for a signal
            pthread_cond_wait(&st->worker_cond, &st->worker_mutex);
        }
        else
        {
            int err = 0;

            pthread_mutex_unlock(&st->worker_mutex);

            if (st->dev)
            {
                void *buffs[] = {st->samples_buf}; //array of buffers
                int flags; //flags set by receive operation
                long long timeNs; //timestamp for receive buffer
                err = SoapySDRDevice_readStream(st->dev, st->rx_stream, buffs, 128 * 256 / 2, &flags, &timeNs, 100000);
                if (err >= 0) {
                    input_push_cs16(&st->input, st->samples_buf, err*2);
                    err = 0;
                } else {
                    log_error("SoapySDRDevice_readStream failed");
                }
            }

            pthread_mutex_lock(&st->worker_mutex);

            if (err)
            {
                st->stopped = 1;
                nrsc5_report_lost_device(st);
            }
        }
    }

    pthread_mutex_unlock(&st->worker_mutex);
    return NULL;
}

static void nrsc5_init(nrsc5_t *st)
{
    st->closed = 0;
    st->stopped = 1;
    st->worker_stopped = 1;
    st->gain_settings[0] = '\0';
    st->freq = NRSC5_SCAN_BEGIN;
    st->mode = NRSC5_MODE_FM;
    st->callback = NULL;

    output_init(&st->output, st);
    input_init(&st->input, st, &st->output);

    // Create worker thread
    pthread_mutex_init(&st->worker_mutex, NULL);
    pthread_cond_init(&st->worker_cond, NULL);
    pthread_create(&st->worker, NULL, worker_thread, st);
}

static nrsc5_t *nrsc5_alloc()
{
    nrsc5_t *st = calloc(1, sizeof(*st));
    return st;
}

NRSC5_API int nrsc5_open(nrsc5_t **result, char *device_args)
{
    int err;
    nrsc5_t *st = nrsc5_alloc();

    if (!(st->dev = SoapySDRDevice_makeStrArgs(device_args)))
        goto error_init;

    err = SoapySDRDevice_setSampleRate(st->dev, SOAPY_SDR_RX, 0, (double) SAMPLE_RATE / 2.0);
    if (err) goto error;
    /* increase bandwidth since NRSC5 requires about 400kHz */
    err = SoapySDRDevice_setBandwidth(st->dev, SOAPY_SDR_RX, 0, 600e3);
    if (err) goto error;

    nrsc5_init(st);

    *result = st;
    return 0;

error:
    log_error("nrsc5_open error: %d", err);
    SoapySDRDevice_unmake(st->dev);
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

    if (st->dev) {
        SoapySDRDevice_unmake(st->dev);
    }

    input_free(&st->input);
    output_free(&st->output);
    free(st);
}

NRSC5_API void nrsc5_start(nrsc5_t *st)
{
    st->rx_stream = SoapySDRDevice_setupStream(st->dev, SOAPY_SDR_RX, SOAPY_SDR_CS16, NULL, 0, NULL);
    if (st->rx_stream == NULL) {
        log_error("SoapySDRDevice_setupStream failed");
        return;
    }
    SoapySDRDevice_activateStream(st->dev, st->rx_stream, 0, 0, 0);

    // signal the worker to start
    pthread_mutex_lock(&st->worker_mutex);
    st->stopped = 0;
    pthread_cond_broadcast(&st->worker_cond);
    pthread_mutex_unlock(&st->worker_mutex);
}

NRSC5_API void nrsc5_stop(nrsc5_t *st)
{
    // signal the worker to stop
    pthread_mutex_lock(&st->worker_mutex);
    st->stopped = 1;
    pthread_cond_broadcast(&st->worker_cond);
    pthread_mutex_unlock(&st->worker_mutex);

    // wait for worker to stop
    pthread_mutex_lock(&st->worker_mutex);
    while (st->stopped != st->worker_stopped)
        pthread_cond_wait(&st->worker_cond, &st->worker_mutex);
    pthread_mutex_unlock(&st->worker_mutex);

    SoapySDRDevice_deactivateStream(st->dev, st->rx_stream, 0, 0);
    SoapySDRDevice_closeStream(st->dev, st->rx_stream);
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

    int err = SoapySDRDevice_setAntenna(st->dev, SOAPY_SDR_RX, 0, antenna);
    if (err)
        return 1;

    return 0;
}

NRSC5_API int nrsc5_set_freq_correction(nrsc5_t *st, int ppm_error)
{
    int err = SoapySDRDevice_setFrequencyCorrection(st->dev, SOAPY_SDR_RX, 0, ppm_error);
    if (err)
        return 1;

    return 0;
}

NRSC5_API void nrsc5_get_frequency(nrsc5_t *st, float *freq)
{
    if (st->dev)
        *freq = SoapySDRDevice_getFrequency(st->dev, SOAPY_SDR_RX, 0);
    else
        *freq = st->freq;
}

NRSC5_API int nrsc5_set_frequency(nrsc5_t *st, float freq)
{
    if (st->freq == freq)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->dev && SoapySDRDevice_setFrequency(st->dev, SOAPY_SDR_RX, 0, freq, NULL) != 0)
        return 1;

    input_reset(&st->input);
    output_reset(&st->output);

    st->freq = freq;
    return 0;
}

NRSC5_API void nrsc5_get_gain(nrsc5_t *st, char **gain_settings)
{
    static char gain_settings_store[MAX_GAIN_SETTINGS_SIZE];
    if (st->dev) {
        int offset = 0;
        if (SoapySDRDevice_hasGainMode(st->dev, SOAPY_SDR_RX, 0)) {
            offset += snprintf(&gain_settings_store[offset], MAX_GAIN_SETTINGS_SIZE - 1 - offset, "AGC=%d,", SoapySDRDevice_getGainMode(st->dev, SOAPY_SDR_RX, 0) ? 1 : 0);
        }
        size_t n_gains;
        char **gain_names = SoapySDRDevice_listGains(st->dev, SOAPY_SDR_RX, 0, &n_gains);
        for (int i = 0; i < n_gains; i++) {
            offset += snprintf(&gain_settings_store[offset], MAX_GAIN_SETTINGS_SIZE - 1 - offset, "%s=%f,",
                               gain_names[i],  SoapySDRDevice_getGainElement(st->dev, SOAPY_SDR_RX, 0, gain_names[i]));
        }
        gain_settings_store[offset] = '\0';
        *gain_settings = gain_settings_store;
    } else {
        *gain_settings = st->gain_settings;
    }
}

NRSC5_API int nrsc5_set_gain(nrsc5_t *st, char *gain_settings)
{
    if (strcmp(st->gain_settings, gain_settings) == 0)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->dev)
    {
        int len = strlen(gain_settings);
        int offset;
        for (offset = 0; offset < len; ) {
            char gain_name[21];
            float gain_value;
            int consumed;
            int n = sscanf(&gain_settings[offset], "%20[^=]=%f%n", gain_name, &gain_value, &consumed);
            if (n != 2)
                return 1;
            if (strcmp(gain_name, "AGC") == 0 && SoapySDRDevice_hasGainMode(st->dev, SOAPY_SDR_RX, 0)) {
                SoapySDRDevice_setGainMode(st->dev, SOAPY_SDR_RX, 0, gain_value != 0);
            } else {
                SoapySDRDevice_setGainElement(st->dev, SOAPY_SDR_RX, 0, gain_name, gain_value);
            }
            offset += consumed;
            offset += strspn(&gain_settings[offset], " /,");
        }
    }

    strncpy(st->gain_settings, gain_settings, MAX_GAIN_SETTINGS_SIZE - 1);
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
