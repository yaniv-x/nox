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

#ifndef _H_DISPLAY
#define _H_DISPLAY

#include <X11/Xlib.h>

#include "common.h"
#include "non_copyable.h"
#include "vga.h"
#include "run_loop.h"

class Thread;
class VGA;

class NoxDisplay: public RunLoop, public VGAFrontEnd {
public:
    NoxDisplay(VGA& vga);

private:
    virtual void on_size_changed(uint32_t width, uint32_t hight);
    virtual void invalid();

    void update_area(Window window, int x, int y, int width, int height);
    void x11_handler();
    void update();
    void* main();

private:
    VGA& _vga;
    VGABackEnd* _back_end;
    std::auto_ptr<Thread> _thread;
    Display* _display;
    Window _window;

    uint32_t _width;
    uint32_t _height;
    const uint8_t* _fb;
};

#endif

