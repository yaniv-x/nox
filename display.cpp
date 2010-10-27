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

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/XKBlib.h>
#include "display.h"
#include "threads.h"
#include "keyboard.h"


NoxDisplay::NoxDisplay(VGA& vga, KbdController& kbd)
    : _vga (vga)
    , _back_end (_vga.attach_front_end(this))
    , _thread (new Thread((Thread::start_proc_t)&NoxDisplay::main, this))
    , _window (None)
    , _kbd (kbd)
    , _evdev (false)
{

}


void NoxDisplay::update_area(Window window, int x, int y, int width, int height)
{
    XImage ximage = {0};

    ximage.width = _width;
    ximage.height = _height;
    ximage.data = (char*)_fb;
    ximage.format = ZPixmap;

    ximage.depth = 24;
    ximage.byte_order = ximage.bitmap_bit_order = LSBFirst;
    ximage.bitmap_unit = ximage.bitmap_pad = 32;
    ximage.bytes_per_line = _width * 4;
    ximage.bits_per_pixel = 32;
    ximage.red_mask = 0x00ff0000;
    ximage.green_mask = 0x0000ff00;
    ximage.blue_mask = 0x000000ff;

    if (!XInitImage(&ximage)) {
        THROW("init failed");
    }

    XGCValues values;
    values.function = GXcopy;
    GC gc = XCreateGC(_display, window, GCFunction, &values);
    XPutImage(_display, window, gc, &ximage, x, y, x, y, width, height);
    XFreeGC(_display, gc);
}


void NoxDisplay::x11_handler()
{
    while (XPending(_display)) {
        XEvent event;

        XNextEvent(_display, &event);

        switch (event.type) {
        case Expose:
            update_area(event.xexpose.window, event.xexpose.x, event.xexpose.y,
                        event.xexpose.width, event.xexpose.height);
            break;
        case KeyPress:
            on_key_press(event.xkey.keycode);
            break;
        case KeyRelease:
            on_key_release(event.xkey.keycode);
            break;
        case ClientMessage: {
            Atom wm_protocol_atom = XInternAtom(_display, "WM_PROTOCOLS", False);
            Atom wm_delete_window_atom = XInternAtom(_display, "WM_DELETE_WINDOW", False);

            if (event.xclient.message_type == wm_protocol_atom) {
                if (event.xclient.data.l[0] == wm_delete_window_atom) {
                    //XDestroyWindow(display, win);
                    //XCloseDisplay(display);
                    D_MESSAGE("WM_DELETE_WINDOW");
                }
            }
            break;
        }
        case DestroyNotify:
            //XDestroyWindow(display, win);
            //XCloseDisplay(display);
            D_MESSAGE("DestroyNotify");
        }
    }
}

void NoxDisplay::update()
{
    if (_window != None) {
        update_area(_window, 0, 0, _width, _height);
    }
}


void NoxDisplay::query_input_driver()
{
    XkbDescPtr keyboard = XkbGetKeyboard(_display, XkbAllComponentsMask, XkbUseCoreKbd);
    char* str;

    if (!keyboard) {
        W_MESSAGE("failed");
        return;
    }

    if ((str = XGetAtomName(_display, keyboard->names->keycodes))) {
        _evdev = strstr(str, "evdev") != NULL;
        XFree(str);
    } else {
        W_MESSAGE("failed");
    }

    XkbFreeClientMap(keyboard, XkbAllComponentsMask, True);
}


void* NoxDisplay::main()
{
    XInitThreads();
    _display = XOpenDisplay(NULL);

    if (!_display) {
        return NULL;
    }

    query_input_driver();

    XSetWindowAttributes win_attributes;

    unsigned long mask = CWBorderPixel | CWEventMask;
    win_attributes.border_pixel = 1;
    win_attributes.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask |
                                KeyReleaseMask;


    _back_end->get_size(&_width, &_height);
    _fb = _back_end->get_fb();

    _window = XCreateWindow(_display, DefaultRootWindow(_display), 100, 100,
                               _width, _height, 0, CopyFromParent, InputOutput,
                               CopyFromParent, mask, &win_attributes);

    if (_window == None) {
        return NULL;
    }

    Atom wm_delete_window_atom = XInternAtom(_display, "WM_DELETE_WINDOW", False);

    XSetWMProtocols(_display, _window, &wm_delete_window_atom, 1);
    XMapWindow(_display, _window);

    create_fd_event(ConnectionNumber(_display), (void_callback_t)&NoxDisplay::x11_handler, this);
    Timer* timer = create_timer((void_callback_t)&NoxDisplay::update, this);
    timer->arm(1000 * 1000 * 1000 / 30, true);
    try {
        RunLoop::run();
    } catch (...) {
        D_MESSAGE("unhandled exception");
    }

    return NULL;
}


void NoxDisplay::on_size_changed(uint32_t width, uint32_t hight)
{
    _width = width;
    _height = hight;

    if (_window != None) {
        XResizeWindow(_display, _window, _width, _height);
        XSizeHints *hints = XAllocSizeHints();

        if (!hints) {
            throw OOMException();
        }
        hints->flags = PMinSize | PMaxSize;
        hints->max_width = hints->min_width = _width;
        hints->max_height = hints->min_height = _height;
        XSetWMNormalHints(_display, _window, hints);
        XFree(hints);
    }

    D_MESSAGE("%u %u", width, hight);
}

void NoxDisplay::invalid()
{
    D_MESSAGE("implement me");
}


void NoxDisplay::on_key_press(unsigned int keycode)
{
    if (!_evdev) {
        XBell(_display, 0);
        return;
    }

    _kbd.key_down((NoxKey)keycode);
}


void NoxDisplay::on_key_release(unsigned int keycode)
{
    if (!_evdev) {
        XBell(_display, 0);
        return;
    }

    _kbd.key_up((NoxKey)keycode);
}

