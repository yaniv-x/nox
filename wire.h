/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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

#ifndef _H_WIRE
#define _H_WIRE


#include "non_copyable.h"
#include "threads.h"


class Wire: public NonCopyable {
public:
    Wire(VMPart& owner)
        : _owner (owner)
        , _output (0)
        , _dest (NULL)
        , _opaque (NULL)
        , _raised_cb (NULL)
        , _droped_cb (NULL)
        , _detach_cb (NULL)
    {
    }

    virtual ~Wire()
    {
        detach_dest(false);
    }

    void set_dest(VMPart& part, void* opaque, void_callback_t raised, void_callback_t droped,
                  void_callback_t detach)
    {
        detach_dest(true);

        Lock lock(_mutex);
        _dest = &part;
        _opaque = opaque;
        _raised_cb = raised;
        _droped_cb = droped;
        _detach_cb = detach;

        if (output()) {
            _raised_cb(_opaque);
        }
    }

    void detach_dest(bool may_drop)
    {
        Lock lock(_mutex);

        if (may_drop && output()) {
            _droped_cb(_opaque);
        }

        if (_detach_cb) {
            _detach_cb(_opaque);
        }

        _dest = NULL;
        _opaque = NULL;
        _raised_cb = NULL;
        _droped_cb = NULL;
        _detach_cb = NULL;
    }

    void raise()
    {
        Lock lock(_mutex);
        _raise();
    }

    void drop()
    {
        Lock lock(_mutex);
        _drop();
    }

    void spike()
    {
        Lock lock(_mutex);
        _drop();
        _raise();
    }

    void set_level(uint level)
    {
        Lock lock(_mutex);

        if (level) {
            _raise();
        } else {
            _drop();
        }
    }

    void reset() { _output = 0;}
    uint output() { return _output;}

private:
    void _raise()
    {
        if (_output) {
            return;
        }

        _output = !_output;

        if (_raised_cb) {
            _raised_cb(_opaque);
        }
    }

    void _drop()
    {
        if (!_output) {
            return;
        }

        _output = !_output;

        if (_droped_cb) {
            _droped_cb(_opaque);
        }
    }

private:
    Mutex _mutex;
    VMPart& _owner;
    uint _output;
    VMPart* _dest;
    void* _opaque;
    void_callback_t _raised_cb;
    void_callback_t _droped_cb;
    void_callback_t _detach_cb;
};

#endif

