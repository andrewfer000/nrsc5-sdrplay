/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ao/ao.h>
#include <getopt.h>
#include <math.h>
#include <nrsc5.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#ifdef __MINGW32__
#include <conio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <netdb.h>
#include <sys/socket.h>
#include <termios.h>
#endif

#include "bitwriter.h"
#include "log.h"
#include "main.h"

static ao_sample_format sample_format = {
    16,
    44100,
    2,
    AO_FMT_NATIVE,
    "L,R"
};

ao_device *open_ao_live()
{
    return ao_open_live(ao_default_driver_id(), &sample_format, NULL);
}

ao_device *open_ao_wav(const char *name)
{
    return ao_open_file(ao_driver_id("wav"), name, 1, &sample_format, NULL);
}

static void reset_audio_buffers(state_t *st)
{
    audio_buffer_t *b;

    // find the end of the head list
    for (b = st->head; b && b->next; b = b->next) { }

    // if the head list is non-empty, prepend to free list
    if (b != NULL)
    {
        b->next = st->free;
        st->free = st->head;
    }

    st->head = NULL;
    st->tail = NULL;
}

static void push_audio_buffer(state_t *st, unsigned int program, const int16_t *data, size_t count)
{
    audio_buffer_t *b;
    struct timespec ts;
    struct timeval now;

    gettimeofday(&now, NULL);
    ts.tv_sec = now.tv_sec;
    ts.tv_nsec = (now.tv_usec + 100000) * 1000;
    if (ts.tv_nsec >= 1000000000)
    {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec += 1;
    }

    pthread_mutex_lock(&st->mutex);
    if (program != st->program)
        goto unlock;

    while (st->free == NULL)
    {
        if (pthread_cond_timedwait(&st->cond, &st->mutex, &ts) == ETIMEDOUT)
        {
            log_warn("Audio output timed out, dropping samples");
            reset_audio_buffers(st);
        }
    }
    b = st->free;
    st->free = b->next;
    pthread_mutex_unlock(&st->mutex);

    assert(AUDIO_DATA_LENGTH == count * sizeof(data[0]));
    memcpy(b->data, data, count * sizeof(data[0]));

    pthread_mutex_lock(&st->mutex);
    if (program != st->program)
    {
        b->next = st->free;
        st->free = b;
        goto unlock;
    }

    b->next = NULL;
    if (st->tail)
        st->tail->next = b;
    else
        st->head = b;
    st->tail = b;

    if (st->audio_ready < AUDIO_THRESHOLD)
        st->audio_ready++;

    pthread_cond_signal(&st->cond);

unlock:
    pthread_mutex_unlock(&st->mutex);
}

void init_audio_buffers(state_t *st)
{
    st->head = NULL;
    st->tail = NULL;
    st->free = NULL;

    for (int i = 0; i < AUDIO_BUFFERS; ++i)
    {
        audio_buffer_t *b = malloc(sizeof(audio_buffer_t));
        b->next = st->free;
        st->free = b;
    }

    pthread_cond_init(&st->cond, NULL);
    pthread_mutex_init(&st->mutex, NULL);
}

static void write_adts_header(FILE *fp, unsigned int len)
{
    uint8_t hdr[7];
    bitwriter_t bw;

    bw_init(&bw, hdr);
    bw_addbits(&bw, 0xFFF, 12); // sync word
    bw_addbits(&bw, 0, 1); // MPEG-4
    bw_addbits(&bw, 0, 2); // Layer
    bw_addbits(&bw, 1, 1); // no CRC
    bw_addbits(&bw, 1, 2); // AAC-LC
    bw_addbits(&bw, 7, 4); // 22050 HZ
    bw_addbits(&bw, 0, 1); // private bit
    bw_addbits(&bw, 2, 3); // 2-channel configuration
    bw_addbits(&bw, 0, 1);
    bw_addbits(&bw, 0, 1);
    bw_addbits(&bw, 0, 1);
    bw_addbits(&bw, 0, 1);
    bw_addbits(&bw, len + 7, 13); // frame length
    bw_addbits(&bw, 0x7FF, 11); // buffer fullness (VBR)
    bw_addbits(&bw, 0, 2); // 1 AAC frame per ADTS frame

    fwrite(hdr, 7, 1, fp);
}

static void dump_hdc(FILE *fp, const uint8_t *pkt, unsigned int len)
{
    write_adts_header(fp, len);
    fwrite(pkt, len, 1, fp);
    fflush(fp);
}

static void dump_aas_file(state_t *st, const nrsc5_event_t *evt)
{
#if defined(WIN32) || defined(_WIN32)
#define PATH_SEPARATOR "\\"
#else
#define PATH_SEPARATOR "/"
#endif
    char fullpath[strlen(st->aas_files_path) + strlen(evt->lot.name) + 16];
    FILE *fp;

    sprintf(fullpath, "%s" PATH_SEPARATOR "%d_%s", st->aas_files_path, evt->lot.lot, evt->lot.name);
    fp = fopen(fullpath, "wb");
    if (fp == NULL)
    {
        log_warn("Failed to open %s (%d)", fullpath, errno);
        return;
    }
    fwrite(evt->lot.data, 1, evt->lot.size, fp);
    fclose(fp);
}

static void dump_ber(float cber)
{
    static float min = 1, max = 0, sum = 0, count = 0;
    sum += cber;
    count += 1;
    if (cber < min) min = cber;
    if (cber > max) max = cber;
    log_info("BER: %f, avg: %f, min: %f, max: %f", cber, sum / count, min, max);
}

static void done_signal(state_t *st)
{
    pthread_mutex_lock(&st->mutex);
    st->done = 1;
    pthread_cond_signal(&st->cond);
    pthread_mutex_unlock(&st->mutex);
}

static void change_program(state_t *st, unsigned int program)
{
    pthread_mutex_lock(&st->mutex);

    // reset audio buffers
    st->audio_ready = 0;
    if (st->tail)
    {
        st->tail->next = st->free;
        st->free = st->head;
        st->head = st->tail = NULL;
    }
    // update current program
    st->program = program;

    pthread_mutex_unlock(&st->mutex);
}

void callback(const nrsc5_event_t *evt, void *opaque)
{
    state_t *st = opaque;
    nrsc5_sig_service_t *sig_service;
    nrsc5_sig_component_t *sig_component;
    nrsc5_sis_asd_t *audio_service;
    nrsc5_sis_dsd_t *data_service;

    switch (evt->event)
    {
    case NRSC5_EVENT_LOST_DEVICE:
        done_signal(st);
        break;
    case NRSC5_EVENT_BER:
        dump_ber(evt->ber.cber);
        break;
    case NRSC5_EVENT_MER:
        log_info("MER: %.1f dB (lower), %.1f dB (upper)", evt->mer.lower, evt->mer.upper);
        break;
    case NRSC5_EVENT_IQ:
        if (st->iq_file)
            fwrite(evt->iq.data, 1, evt->iq.count, st->iq_file);
        break;
    case NRSC5_EVENT_HDC:
        if (evt->hdc.program == st->program)
        {
            if (st->hdc_file)
                dump_hdc(st->hdc_file, evt->hdc.data, evt->hdc.count);

            st->audio_packets++;
            st->audio_bytes += evt->hdc.count * sizeof(evt->hdc.data[0]);
            if (st->audio_packets >= 32) {
                log_info("Audio bit rate: %.1f kbps", (float)st->audio_bytes * 8 * 44100 / 2048 / st->audio_packets / 1000);
                st->audio_packets = 0;
                st->audio_bytes = 0;
            }
        }
        break;
    case NRSC5_EVENT_AUDIO:
        push_audio_buffer(st, evt->audio.program, evt->audio.data, evt->audio.count);
        break;
    case NRSC5_EVENT_SYNC:
        log_info("Synchronized");
        st->audio_ready = 0;
        break;
    case NRSC5_EVENT_LOST_SYNC:
        log_info("Lost synchronization");
        break;
    case NRSC5_EVENT_ID3:
        if (evt->id3.program == st->program)
        {
            if (evt->id3.title)
                log_info("Title: %s", evt->id3.title);
            if (evt->id3.artist)
                log_info("Artist: %s", evt->id3.artist);
            if (evt->id3.album)
                log_info("Album: %s", evt->id3.album);
            if (evt->id3.genre)
                log_info("Genre: %s", evt->id3.genre);
            if (evt->id3.ufid.owner)
                log_info("Unique file identifier: %s %s", evt->id3.ufid.owner, evt->id3.ufid.id);
            if (evt->id3.xhdr.param >= 0)
                log_info("XHDR: %d %08X %d", evt->id3.xhdr.param, evt->id3.xhdr.mime, evt->id3.xhdr.lot);
        }
        break;
    case NRSC5_EVENT_SIG:
        for (sig_service = evt->sig.services; sig_service != NULL; sig_service = sig_service->next)
        {
            log_info("SIG Service: type=%s number=%d name=%s",
                     sig_service->type == NRSC5_SIG_SERVICE_AUDIO ? "audio" : "data",
                     sig_service->number, sig_service->name);

            for (sig_component = sig_service->components; sig_component != NULL; sig_component = sig_component->next)
            {
                if (sig_component->type == NRSC5_SIG_SERVICE_AUDIO)
                {
                    log_info("  Audio component: id=%d port=%04X type=%d mime=%08X", sig_component->id,
                             sig_component->audio.port, sig_component->audio.type, sig_component->audio.mime);
                }
                else if (sig_component->type == NRSC5_SIG_SERVICE_DATA)
                {
                    log_info("  Data component: id=%d port=%04X service_data_type=%d type=%d mime=%08X",
                             sig_component->id, sig_component->data.port, sig_component->data.service_data_type,
                             sig_component->data.type, sig_component->data.mime);
                }
            }
        }
        break;
    case NRSC5_EVENT_LOT:
        if (st->aas_files_path)
            dump_aas_file(st, evt);
        log_info("LOT file: port=%04X lot=%d name=%s size=%d mime=%08X", evt->lot.port, evt->lot.lot, evt->lot.name, evt->lot.size, evt->lot.mime);
        break;
    case NRSC5_EVENT_SIS:
        if (evt->sis.country_code)
            log_info("Country: %s, FCC facility ID: %d", evt->sis.country_code, evt->sis.fcc_facility_id);
        if (evt->sis.name)
            log_info("Station name: %s", evt->sis.name);
        if (evt->sis.slogan)
            log_info("Slogan: %s", evt->sis.slogan);
        if (evt->sis.message)
            log_info("Message: %s", evt->sis.message);
        if (evt->sis.alert)
            log_info("Alert: %s", evt->sis.alert);
        if (!isnan(evt->sis.latitude))
            log_info("Station location: %f, %f, %dm", evt->sis.latitude, evt->sis.longitude, evt->sis.altitude);
        for (audio_service = evt->sis.audio_services; audio_service != NULL; audio_service = audio_service->next)
        {
            const char *name = NULL;
            nrsc5_program_type_name(audio_service->type, &name);
            log_info("Audio program %d: %s, type: %s, sound experience %d",
                     audio_service->program,
                     audio_service->access == NRSC5_ACCESS_PUBLIC ? "public" : "restricted",
                     name, audio_service->sound_exp);
        }
        for (data_service = evt->sis.data_services; data_service != NULL; data_service = data_service->next)
        {
            const char *name = NULL;
            nrsc5_service_data_type_name(data_service->type, &name);
            log_info("Data service: %s, type: %s, MIME type %03x",
                     data_service->access == NRSC5_ACCESS_PUBLIC ? "public" : "restricted",
                     name, data_service->mime_type);
        }
        break;
    }
}

#ifndef __MINGW32__
static void restore_termios(void *arg)
{
    tcsetattr(STDIN_FILENO, TCSANOW, arg);
}
#endif

void *input_main(void *arg)
{
    state_t *st = arg;

    if (!isatty(STDIN_FILENO))
        return NULL;

#ifndef __MINGW32__
    struct termios prev_termios, t;

    // disable terminal canonical mode
    tcgetattr(STDIN_FILENO, &prev_termios);
    pthread_cleanup_push(restore_termios, &prev_termios);
    t = prev_termios;
    t.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
#endif

    while (!st->done)
    {
        int ch;

#ifdef __MINGW32__
        ch = _getch();
#else
        if (read(STDIN_FILENO, &ch, 1) != 1)
            break;
#endif

        switch (ch)
        {
        case 'q':
            done_signal(st);
            // user wants to immediately exit, so reset audio buffer
            change_program(st, -1);
            break;
        case '0':
        case '1':
        case '2':
        case '3':
            change_program(st, ch - '0');
            break;
        }
    }

#ifndef __MINGW32__
    // restore terminal settings
    pthread_cleanup_pop(1);
#endif

    return NULL;
}

void log_lock(void *udata, int lock)
{
    pthread_mutex_t *mutex = udata;
    if (lock)
        pthread_mutex_lock(mutex);
    else
        pthread_mutex_unlock(mutex);
}

void cleanup(state_t *st)
{
    reset_audio_buffers(st);
    while (st->free)
    {
        audio_buffer_t *b = st->free;
        st->free = b->next;
        free(b);
    }

    if (st->hdc_file)
        fclose(st->hdc_file);
    if (st->iq_file)
        fclose(st->iq_file);

    free(st->input_name);
    free(st->aas_files_path);

    if (st->dev)
        ao_close(st->dev);
}
