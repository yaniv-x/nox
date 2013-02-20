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

#ifndef _H_SPEAKER
#define _H_SPEAKER

#include "vm_part.h"
#include "threads.h"

#define SPEAKER_SAMPLES 1024 // 44100Hz mono => 23 mili

class Timer;

typedef uint8_t speaket_sample_t;

class Speaker: public VMPart {
public:
    Speaker(VMPart& owner);
    virtual ~Speaker();

    void set_level(uint8_t data, uint8_t gate);

    virtual void reset();
    virtual bool start();
    virtual bool stop();
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

private:
    void start_play();
    void stop_play();
    void timer_proc();
    void finalize_frame();
    void transmit_frame();
    void fill_samples_by_pit(uint n);
    void fill_samples(uint n);
    void fill_samples();
    void pit_cb(uint mode, uint counter);

    // for now
    void player_main();

private:
    Mutex _mutex;
    bool _ready;
    Condition _ready_condition;
    bool _started;
    bool _terminate;
    nox_time_t _frame_start_time;
    nox_time_t _frame_pos_time;
    nox_time_t _suspend_time;
    Timer* _timer;
    speaket_sample_t _frame[SPEAKER_SAMPLES];
    speaket_sample_t* const _frame_end;
    speaket_sample_t* _frame_pos;
    uint _unchanged_count;

    uint8_t _data_level;
    uint8_t _gate_level;
    speaket_sample_t _last_output;
    uint _timer_mode;
    uint _timer_counter;
    uint _samples_run;
    uint _samples_left;
    uint _sample_val_selector;
    speaket_sample_t _sample_val[2];

    // for now
    Mutex _player_mutex;
    Condition _player_condition;
    Thread* _thread;
    std::list<speaket_sample_t*> _free_frames;
    std::list<speaket_sample_t*> _pending_frames;
};

#endif

