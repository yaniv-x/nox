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

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/XKBlib.h>
#include "display.h"
#include "threads.h"
#include "keyboard.h"


NoxDisplay::NoxDisplay(const char* vm_name, VGA& vga, KbdController& kbd)
    : _vm_name (vm_name)
    , _vga (vga)
    , _back_end (NULL)
    , _thread (new Thread((Thread::start_proc_t)&NoxDisplay::main, this))
    , _window (None)
    , _invisible_cursor (None)
    , _update_timer (NULL)
    , _gc (NULL)
    , _kbd (kbd)
    , _evdev (false)
    , _tracking (false)
    , _width (640)
    , _height (480)
    , _valid_x_image (false)
{

}

NoxDisplay::~NoxDisplay()
{
    run_break();
    _thread->join();
}


void NoxDisplay::update_area(Window window, int x, int y, int width, int height)
{
    if (!_valid_x_image) {
        XFillRectangle(_display, window, _gc, x, y, width, height);
    } else {
        XPutImage(_display, window, _gc, &_x_image, x, y, x, y, width, height);
    }
}


void NoxDisplay::x11_handler()
{
    while (XPending(_display)) {
        XEvent event;

        XNextEvent(_display, &event);

        switch (event.type) {
        case MotionNotify:
            on_motion(event.xmotion.x, event.xmotion.y);
            break;
        case Expose: {
            Lock lock(_mutex);
            update_area(event.xexpose.window, event.xexpose.x, event.xexpose.y,
                        event.xexpose.width, event.xexpose.height);
            break;
        }
        case ButtonPress:
            on_button_press(event.xbutton.button);
            break;
        case ButtonRelease:
            on_button_release(event.xbutton.button);
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
    Lock lock(_mutex);
    ASSERT(_window != None);
    update_area(_window, 0, 0, _width, _height);
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

    XColor x_color;
    uint8_t pixmap_data[1];
    Pixmap cursur_pixmap = XCreateBitmapFromData(_display, DefaultRootWindow(_display),
                                                 (char*)pixmap_data, 1, 1);
    _invisible_cursor = XCreatePixmapCursor(_display, cursur_pixmap, cursur_pixmap,
                                            &x_color, &x_color, 0, 0);
    XFreePixmap(_display, cursur_pixmap);

    FDEvent* x_event = create_fd_event(ConnectionNumber(_display),
                                       (void_callback_t)&NoxDisplay::x11_handler,
                                       this);

    create_window();

    _update_timer = create_timer((void_callback_t)&NoxDisplay::update, this);

    _back_end = _vga.attach_front_end(this);

    try {
        RunLoop::run();
    } catch (...) {
        D_MESSAGE("unhandled exception");
    }

    _update_timer->destroy();
    _back_end->detach();
    x_event->destroy();
    XCloseDisplay(_display);

    return NULL;
}


void NoxDisplay::create_window()
{
    ASSERT(_window == None);
    XSetWindowAttributes win_attributes;

    unsigned long mask = CWBorderPixel | CWEventMask;
    win_attributes.border_pixel = 1;
    win_attributes.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask |
                                KeyReleaseMask | ButtonPressMask | ButtonReleaseMask |
                                PointerMotionMask;

    _window = XCreateWindow(_display, DefaultRootWindow(_display), 100, 100,
                            _width, _height, 0, CopyFromParent, InputOutput,
                            CopyFromParent, mask, &win_attributes);

    if (_window == None) {
        THROW("create window failed");
    }

    Atom wm_name_atom = XInternAtom(_display, "_NET_WM_NAME", False);
    Atom utf8_str_atom = XInternAtom(_display, "UTF8_STRING", False);
    std::string win_title;

    sprintf(win_title, "Nox - %s", _vm_name.c_str());

    XChangeProperty(_display, _window, wm_name_atom, utf8_str_atom, 8 /* list of 8-bit*/,
                    PropModeReplace, (const unsigned char*)win_title.c_str(),
                    win_title.size());

    Atom wm_delete_window_atom = XInternAtom(_display, "WM_DELETE_WINDOW", False);

    XSetWMProtocols(_display, _window, &wm_delete_window_atom, 1);

    XGCValues values;
    values.function = GXcopy;
    values.foreground = BlackPixel(_display, DefaultScreen(_display));
    values.fill_style = FillSolid;
    _gc = XCreateGC(_display, _window, GCFunction | GCForeground | GCFillStyle, &values);

    if (!_gc) {
        THROW("create gc failed");
    }

    XMapWindow(_display, _window);
}


void NoxDisplay::prepare_window()
{
    if (_window == None) {
        PANIC("unexpected");
    }

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


void NoxDisplay::prepare_x_image()
{
    _valid_x_image = false;

    memset(&_x_image, 0, sizeof(_x_image));

    _x_image.width = _width;
    _x_image.height = _height;
    _x_image.data = (char*)_fb->get();
    _x_image.format = ZPixmap;

    _x_image.depth = 24;
    _x_image.byte_order = _x_image.bitmap_bit_order = LSBFirst;
    _x_image.bitmap_unit = _x_image.bitmap_pad = 32;
    _x_image.bytes_per_line = _stride;
    _x_image.bits_per_pixel = 32;
    _x_image.red_mask = 0x00ff0000;
    _x_image.green_mask = 0x0000ff00;
    _x_image.blue_mask = 0x000000ff;

    if (!XInitImage(&_x_image)) {
        THROW("init failed");
    }

    _valid_x_image = true;
}


void NoxDisplay::set(SharedBuf* fb, uint32_t width, uint32_t height, int32_t stride)
{
    Lock lock(_mutex);

    D_MESSAGE("%u %u", width, height);

    _width = width;
    _height = height;
    _stride = stride;
    _fb.reset(fb->ref());

    prepare_window();
    prepare_x_image();

    _update_timer->arm(1000 * 1000 * 1000 / 30, true);
}


void NoxDisplay::on_key_press(unsigned int keycode)
{
    if (!_evdev) {
        XBell(_display, 0);
        return;
    }

    if (keycode == NOX_KEY_SCROLLLOCK) {
        cancel_tracking();
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

    if (keycode == NOX_KEY_SCROLLLOCK) {
        return;
    }

    _kbd.key_up((NoxKey)keycode);
}


void NoxDisplay::on_motion(int x, int y)
{
    if (!_tracking || (x == 100 && y == 100)) {
        return;
    }

    _kbd.mouse_motion(x - 100, y - 100);
    XWarpPointer(_display, None, _window, 0, 0, 0, 0, 100, 100);
}


void NoxDisplay::on_button_press(unsigned int x_button)
{
    if (!_tracking) {
        return;
    }

    switch (x_button) {
    case Button1:
        _kbd.mouse_button_press(KbdController::MOUSE_LEFT_BUTTON);
        break;
    case Button2:
        _kbd.mouse_button_press(KbdController::MOUSE_MIDDLE_BUTTON);
        break;
    case Button3:
        _kbd.mouse_button_press(KbdController::MOUSE_RIGHT_BUTTON);
        break;
    case Button4:
        _kbd.mouse_z_motion(-1);
        break;
    case Button5:
        _kbd.mouse_z_motion(1);
        break;
    }
}


void NoxDisplay::cancel_tracking()
{
    if (!_tracking) {
        return;
    }

    XUngrabPointer(_display, CurrentTime);
    XUndefineCursor(_display, _window);
    _tracking = false;
}


void NoxDisplay::on_button_release(unsigned int x_button)
{
    if (!_tracking) {
        if (x_button == Button1) {
            _tracking = XGrabPointer(_display, _window, True, 0, GrabModeAsync, GrabModeAsync,
                                     None, None, CurrentTime) == GrabSuccess;
            XWarpPointer(_display, None, _window, 0, 0, 0, 0, 100, 100);
            XDefineCursor(_display, _window, _invisible_cursor);
        }
        return;
    }

    switch (x_button) {
    case Button1:
        _kbd.mouse_button_release(KbdController::MOUSE_LEFT_BUTTON);
        break;
    case Button2:
        _kbd.mouse_button_release(KbdController::MOUSE_MIDDLE_BUTTON);
        break;
    case Button3:
        _kbd.mouse_button_release(KbdController::MOUSE_RIGHT_BUTTON);
        break;
    }
}

