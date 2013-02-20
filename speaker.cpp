/*
    Copyright (c) 2013 Yaniv Kamay,
    All rights reserved.

    Source code is provided for evaluation purposes only. Modification or use in
    source code for any other purpose is prohibited.

    Binary code (i.e. the binary form of source code form) is allowed to use for
    evaluation purposes only. Modification or use in binary code for any other
    purpose is prohibited.

    Redistribution, in source form or in binary form, with or without modification,
    are not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTOR BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <math.h>

#include "speaker.h"
#include "application.h"
#include "nox_vm.h"
#include "pit.h"

#define SPEAKER_Hz 44100
#define SPEAKER_NANO_PER_SAMPLE (1000 * 1000 * 1000 / SPEAKER_Hz)
#define SPEAKER_NANO_PER_FRAME (SPEAKER_NANO_PER_SAMPLE * SPEAKER_SAMPLES)
#define SPEAKER_UNCHANGED_TO_STOP (SPEAKER_Hz / SPEAKER_SAMPLES) // ~1 sec
#define SPEAKER_NUM_PLAYER_FRAMES 4
#define SPEAKER_HIGH_VAL ((sizeof(speaket_sample_t) == 1) ? 160 : 1000)
#define SPEAKER_LOW_VAL ((sizeof(speaket_sample_t) == 1) ? 128 : 0)
#define SPEAKER_PIT_CLOCK_Hz 1193181.8181
#define SPEAKER_PIT_CLOCK_RATIO (SPEAKER_PIT_CLOCK_Hz / SPEAKER_Hz)


Speaker::Speaker(VMPart& owner)
    : VMPart("speaker", owner)
    , _ready (false)
    , _terminate (false)
    , _timer (application->create_timer((void_callback_t)&Speaker::timer_proc, this))
    , _frame_end (_frame + SPEAKER_SAMPLES)
    , _thread (new Thread((Thread::start_proc_t)&Speaker::player_main, this))
{
    // for now
    for (int i = 0; i < SPEAKER_NUM_PLAYER_FRAMES; i++) {
        _free_frames.push_front(new speaket_sample_t[SPEAKER_SAMPLES]);
    }

    _sample_val[0] = SPEAKER_HIGH_VAL;
    _sample_val[1] = SPEAKER_LOW_VAL;

    get_nox().get_pit().set_state_callback(2, (PIT::state_cb_t)&Speaker::pit_cb, this);

    reset();

    Lock lock(_player_mutex);
    while (!_ready) {
        _ready_condition.wait(_player_mutex);
    }
}


Speaker::~Speaker()
{
    _timer->destroy();
    Lock lock(_player_mutex);
    _terminate = true;
    _player_condition.signal();
    lock.unlock();
    _thread->join();
    delete _thread;
}


void Speaker::set_level(uint8_t data, uint8_t gate)
{
    Lock lock(_mutex);

    if (data == _data_level && gate == _gate_level) {
        return;
    }

    _unchanged_count = 0;

    fill_samples();

    _data_level = data;
    _gate_level = gate;

    start_play();
}


static uint conver_samples_run(double n)
{
    n /= SPEAKER_PIT_CLOCK_RATIO;
    return MAX(round(n), 1);
}


void Speaker::pit_cb(uint mode, uint counter)
{
    if (mode != 3) {
        W_MESSAGE_SOME(10, "timer mode %u is not implemented", mode);
    }

    Lock lock(_mutex);

    fill_samples();
    _timer_mode = mode;
    _timer_counter = counter;

    _samples_run = conver_samples_run(_timer_counter / 2);
    _sample_val_selector = 0;
    _samples_left = 0;
}


void Speaker::start_play()
{
    if (_started) {
        return;
    }

    // transmit one frame as if we never stop palying
    for (_frame_pos = _frame; _frame_pos < _frame_end; ++_frame_pos) {
        *_frame_pos = _last_output;
    }

    transmit_frame();

    _frame_pos = _frame;
    _frame_start_time = get_monolitic_time();
    _frame_pos_time = _frame_start_time;
    _timer->arm(SPEAKER_NANO_PER_FRAME, true);

    _started = true;
}


void Speaker::stop_play()
{
    _timer->disarm();
    _started = false;
    _frame_start_time = 0;
    _frame_pos_time = 0;
    _frame_pos = _frame;
    _unchanged_count = 0;
}


void Speaker::timer_proc()
{
    Lock lock(_mutex);

    finalize_frame();
    transmit_frame();

    if (!_gate_level && ++_unchanged_count == SPEAKER_UNCHANGED_TO_STOP) {
        stop_play();
    } else {
        nox_time_t now = get_monolitic_time();
        nox_time_t delta = now - _frame_start_time;
        nox_time_t duration = MIN(int64_t(SPEAKER_NANO_PER_FRAME) * 2 - delta,
                                SPEAKER_NANO_PER_FRAME);
        _timer->modify(duration);
        _frame_start_time = now;
        _frame_pos_time = _frame_start_time;
    }
}


void Speaker::finalize_frame()
{
    fill_samples(_frame_end - _frame_pos);
    _frame_pos = _frame;
}


void Speaker::fill_samples_by_pit(uint n)
{
    if (_timer_mode != 3) {
        for (speaket_sample_t* end = _frame_pos + n; _frame_pos < end; ++_frame_pos) {
            *_frame_pos = SPEAKER_LOW_VAL;
        }

        return;
    }

    uint samples = _samples_left ? _samples_left : _samples_run;
    while (n) {
        uint run = MIN(samples, n);
        n -= run;

        _last_output = _sample_val[_sample_val_selector];

        for (speaket_sample_t* end = _frame_pos + run; _frame_pos < end; ++_frame_pos) {
            *_frame_pos = _last_output;
        }

        if (run == samples) {
            _sample_val_selector ^= 1;
            _samples_left = 0;
            samples = _samples_run;
        } else {
            _samples_left = samples - run;
        }
    }
}


void Speaker::fill_samples(uint n)
{
    if (_gate_level && _data_level) {
        fill_samples_by_pit(n);
    } else {
        _last_output = _data_level ? SPEAKER_HIGH_VAL : SPEAKER_LOW_VAL;

        for (speaket_sample_t* end = _frame_pos + n; _frame_pos < end; ++_frame_pos) {
            *_frame_pos = _last_output;
        }
    }
}


void Speaker::fill_samples()
{
    if (!_started) {
        return;
    }

    nox_time_t now = get_monolitic_time();
    nox_time_t duration = now - _frame_pos_time;
    _frame_pos_time = now;

    fill_samples(MIN(duration / SPEAKER_NANO_PER_SAMPLE, _frame_end - _frame_pos));
}


void Speaker::transmit_frame()
{
    Lock lock(_player_mutex);

    if (_free_frames.empty()) {
        D_MESSAGE("drop");
        return;
    }

    speaket_sample_t* frame = _free_frames.front();
    _free_frames.pop_front();
    memcpy(frame, _frame, sizeof(_frame));

    _pending_frames.push_back(frame);

    _player_condition.signal();
}


void Speaker::reset()
{
    _started = false;
    _unchanged_count = 0;
    _data_level = 0;
    _gate_level = 0;
    _last_output = SPEAKER_LOW_VAL;
    _frame_start_time = 0;
    _frame_pos_time = 0;
    _suspend_time = 0;
    _frame_pos = _frame;
}


bool Speaker::start()
{
    if (_started) {
        nox_time_t suspend_duration = get_monolitic_time() - _suspend_time;
        _suspend_time = 0;
        _frame_pos_time += suspend_duration;
        _frame_start_time += suspend_duration;
        _timer->resume();
    }

    return true;
}


bool Speaker::stop()
{
    if (_started) {
        _suspend_time = get_monolitic_time();
        _timer->suspend();
    }

    return true;
}


#include <pulse/simple.h>
#include <pulse/error.h>

void Speaker::player_main()
{
    pa_simple *s = NULL;
    int error;

    pa_sample_spec ss;

    ss.format = (sizeof(speaket_sample_t) == 1) ? PA_SAMPLE_U8 : PA_SAMPLE_S16LE;
    ss.rate = SPEAKER_Hz;
    ss.channels = 1;

     s = pa_simple_new(NULL, "V",  PA_STREAM_PLAYBACK, NULL, "play", &ss, NULL, NULL, &error);

     if (!s) {
        W_MESSAGE("simple new failed %s", pa_strerror(error));
        return;
     }

     Lock lock(_player_mutex);

     _ready = true;
     _ready_condition.signal();

     for (;;) {
        if (_pending_frames.empty()) {
            _player_condition.wait(_player_mutex);

            if (_terminate) {
                break;
            }
        }

        speaket_sample_t* frame = _pending_frames.front();
        _pending_frames.pop_front();
        lock.unlock();

        if (pa_simple_write(s, frame, sizeof(_frame), &error) < 0) {
            W_MESSAGE("play failed %s", pa_strerror(error));
        }

        lock.lock();
        _free_frames.push_front(frame);
    }

    pa_simple_free(s);
}

