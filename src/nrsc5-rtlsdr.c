#include <assert.h>
#include <string.h>

#include "private.h"

#ifdef __MINGW32__
#define NRSC5_API __declspec(dllexport)
#else
#define NRSC5_API
#endif

static int snr_callback(void *arg, float snr)
{
    nrsc5_t *st = arg;
    st->auto_gain_snr_ready = 1;
    st->auto_gain_snr = snr;
    return 1;
}

static int get_tuner_gains(nrsc5_t *st, int *gains)
{
    if (st->dev)
        return rtlsdr_get_tuner_gains(st->dev, gains);
    assert(st->rtltcp);
    return rtltcp_get_tuner_gains(st->rtltcp, gains);
}

static int set_tuner_gain(nrsc5_t *st, int gain)
{
    if (st->dev)
        return rtlsdr_set_tuner_gain(st->dev, gain);
    assert(st->rtltcp);
    return rtltcp_set_tuner_gain(st->rtltcp, gain);
}

static int do_auto_gain(nrsc5_t *st)
{
    int gain_count, best_gain = 0, ret = 1;
    int *gain_list = NULL;
    float best_snr = 0;

    input_set_snr_callback(&st->input, snr_callback, st);

    gain_count = get_tuner_gains(st, NULL);
    if (gain_count < 0)
        goto error;

    gain_list = malloc(gain_count * sizeof(*gain_list));
    if (!gain_list)
        goto error;

    gain_count = get_tuner_gains(st, gain_list);
    if (gain_count < 0)
        goto error;

    for (int i = 0; i < gain_count; i++)
    {
        int gain = gain_list[i];

        if (set_tuner_gain(st, gain_list[i]) != 0)
            continue;

        if (st->rtltcp)
        {
            // there is no good way to wait for samples after the new gain was applied
            // dump 250ms of samples and hope for the best
            rtltcp_reset_buffer(st->rtltcp, (SAMPLE_RATE / 4) * 2);
        }

        st->auto_gain_snr_ready = 0;
        while (!st->auto_gain_snr_ready)
        {
            int len = sizeof(st->samples_buf);

            if (st->dev)
            {
                if (rtlsdr_read_sync(st->dev, st->samples_buf, len, &len) != 0)
                    goto error;
            }
            else
            {
                assert(st->rtltcp);
                if (rtltcp_read(st->rtltcp, st->samples_buf, len) != len)
                    goto error;
            }

            input_push_cu8(&st->input, st->samples_buf, len);
        }
        log_debug("Gain: %.1f dB, CNR: %.1f dB", gain / 10.0f, 10 * log10f(st->auto_gain_snr));
        if (st->auto_gain_snr > best_snr)
        {
            best_snr = st->auto_gain_snr;
            best_gain = gain;
        }
        input_reset(&st->input);
    }

    log_debug("Best gain: %.1f dB, CNR: %.1f dB", best_gain / 10.0f, 10 * log10f(best_snr));
    st->gain = best_gain;
    set_tuner_gain(st, best_gain);
    ret = 0;

error:
    free(gain_list);
    input_set_snr_callback(&st->input, NULL, NULL);
    return ret;
}

static void worker_cb(uint8_t *buf, uint32_t len, void *arg)
{
    nrsc5_t *st = arg;

    if (st->stopped && st->dev)
        rtlsdr_cancel_async(st->dev);
    else
        input_push_cu8(&st->input, buf, len);
}

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

            if (st->dev && rtlsdr_reset_buffer(st->dev) != 0)
                log_error("rtlsdr_reset_buffer failed");

            if (st->dev || st->rtltcp)
            {
                if (st->auto_gain && st->gain < 0 && do_auto_gain(st) != 0)
                {
                    st->stopped = 1;
                    continue;
                }
            }
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
                err = rtlsdr_read_async(st->dev, worker_cb, st, 8, 512 * 1024);
            }
            else if (st->rtltcp)
            {
                err = rtltcp_read(st->rtltcp, st->samples_buf, sizeof(st->samples_buf));
                if (err >= 0)
                {
                    // a short read is possible when EOF and may not be aligned
                    input_push_cu8(&st->input, st->samples_buf, err & ~3);
                    if (err == sizeof(st->samples_buf))
                        err = 0;
                }
            }
            else if (st->iq_file)
            {
                int count = fread(st->samples_buf, 4, sizeof(st->samples_buf) / 4, st->iq_file);
                if (count > 0)
                    input_push_cu8(&st->input, st->samples_buf, count * 4);
                if (feof(st->iq_file) || ferror(st->iq_file))
                    err = 1;
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
    st->auto_gain = 1;
    st->gain = -1;
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

NRSC5_API int nrsc5_open(nrsc5_t **result, int device_index)
{
    int err;
    nrsc5_t *st = nrsc5_alloc();

    if (rtlsdr_open(&st->dev, device_index) != 0)
        goto error_init;

    err = rtlsdr_set_sample_rate(st->dev, SAMPLE_RATE);
    if (err) goto error;
    err = rtlsdr_set_tuner_gain_mode(st->dev, 1);
    if (err) goto error;
    err = rtlsdr_set_offset_tuning(st->dev, 1);
    if (err && err != -2) goto error;

    nrsc5_init(st);

    *result = st;
    return 0;

error:
    log_error("nrsc5_open error: %d", err);
    rtlsdr_close(st->dev);
error_init:
    free(st);
    *result = NULL;
    return 1;
}

NRSC5_API int nrsc5_open_file(nrsc5_t **result, FILE *fp)
{
    nrsc5_t *st = nrsc5_alloc();
    st->iq_file = fp;
    nrsc5_init(st);

    *result = st;
    return 0;
}

NRSC5_API int nrsc5_open_pipe(nrsc5_t **result)
{
    nrsc5_t *st = nrsc5_alloc();
    nrsc5_init(st);

    *result = st;
    return 0;
}

NRSC5_API int nrsc5_open_rtltcp(nrsc5_t **result, int socket)
{
    int err;
    nrsc5_t *st = nrsc5_alloc();

    st->rtltcp = rtltcp_open(socket);
    if (st->rtltcp == NULL)
        goto error;

    err = rtltcp_set_sample_rate(st->rtltcp, SAMPLE_RATE);
    if (err) goto error;
    err = rtltcp_set_tuner_gain_mode(st->rtltcp, 1);
    if (err) goto error;
    err = rtltcp_set_offset_tuning(st->rtltcp, 1);
    if (err) goto error;

    nrsc5_init(st);

    *result = st;
    return 0;
error:
    free(st);
    *result = NULL;
    return 1;
}

NRSC5_API void nrsc5_close(nrsc5_t *st)
{
    if (!st)
        return;

    // signal the worker to exit
    pthread_mutex_lock(&st->worker_mutex);
    st->closed = 1;
    pthread_cond_broadcast(&st->worker_cond);
    pthread_mutex_unlock(&st->worker_mutex);

    // wait for worker to finish
    pthread_join(st->worker, NULL);

    if (st->dev)
        rtlsdr_close(st->dev);
    if (st->iq_file)
        fclose(st->iq_file);
    if (st->rtltcp)
        rtltcp_close(st->rtltcp);

    input_free(&st->input);
    output_free(&st->output);
    free(st);
}

NRSC5_API void nrsc5_start(nrsc5_t *st)
{
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

NRSC5_API int nrsc5_set_bias_tee(nrsc5_t *st, int on)
{
    if (st->dev)
    {
        int err = rtlsdr_set_bias_tee(st->dev, on);
        if (err)
            return 1;
    }
    else if (st->rtltcp)
    {
        int err = rtltcp_set_bias_tee(st->rtltcp, on);
        if (err)
            return 1;
    }
    return 0;
}

NRSC5_API int nrsc5_set_direct_sampling(nrsc5_t *st, int on)
{
    if (st->dev)
    {
        int err = rtlsdr_set_direct_sampling(st->dev, on);
        if (err)
            return 1;
    }
    else if (st->rtltcp)
    {
        int err = rtltcp_set_direct_sampling(st->rtltcp, on);
        if (err)
            return 1;
    }
    return 0;
}

NRSC5_API int nrsc5_set_freq_correction(nrsc5_t *st, int ppm_error)
{
    if (st->dev)
    {
        int err = rtlsdr_set_freq_correction(st->dev, ppm_error);
        if (err && err != -2)
            return 1;
    }
    else if (st->rtltcp)
    {
        int err = rtltcp_set_freq_correction(st->rtltcp, ppm_error);
        if (err)
            return 1;
    }
    return 0;
}

NRSC5_API void nrsc5_get_frequency(nrsc5_t *st, float *freq)
{
    if (st->dev)
        *freq = rtlsdr_get_center_freq(st->dev);
    else
        *freq = st->freq;
}

NRSC5_API int nrsc5_set_frequency(nrsc5_t *st, float freq)
{
    if (st->freq == freq)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->dev && rtlsdr_set_center_freq(st->dev, freq) != 0)
        return 1;
    if (st->rtltcp && rtltcp_set_center_freq(st->rtltcp, freq) != 0)
        return 1;

    if (st->auto_gain)
        st->gain = -1;
    input_reset(&st->input);
    output_reset(&st->output);

    st->freq = freq;
    return 0;
}

NRSC5_API void nrsc5_get_gain(nrsc5_t *st, float *gain)
{
    if (st->dev)
        *gain = rtlsdr_get_tuner_gain(st->dev) / 10.0f;
    else
        *gain = st->gain;
}

NRSC5_API int nrsc5_set_gain(nrsc5_t *st, float gain)
{
    if (st->gain == gain)
        return 0;
    if (!st->stopped)
        return 1;

    if (st->dev && rtlsdr_set_tuner_gain(st->dev, gain * 10) != 0)
        return 1;
    if (st->rtltcp && rtltcp_set_tuner_gain(st->rtltcp, gain * 10) != 0)
        return 1;

    st->gain = gain;
    return 0;
}

NRSC5_API void nrsc5_set_auto_gain(nrsc5_t *st, int enabled)
{
    st->auto_gain = enabled;
    st->gain = -1;
}

NRSC5_API void nrsc5_set_callback(nrsc5_t *st, nrsc5_callback_t callback, void *opaque)
{
    pthread_mutex_lock(&st->worker_mutex);
    st->callback = callback;
    st->callback_opaque = opaque;
    pthread_mutex_unlock(&st->worker_mutex);
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
