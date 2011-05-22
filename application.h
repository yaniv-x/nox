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

#ifndef _H_APPLICATION
#define _H_APPLICATION

#include "common.h"
#include "run_loop.h"

class NoxVM;
class AdminServer;

class Application: public RunLoop {
public:
    Application();
    ~Application();

    AdminServer* get_admin() { return _admin_server.get();}

    static ErrorCode main(int argc, const char** argv);
    static const std::string& get_nox_dir();
    void quit();

private:
    void init_signals();
    void restore_signals();
    bool init(int argc, const char** argv);
    void continue_quitting(bool ok);
    void quit_handler();

    static void sig_int_handler(int sig);
    static void sig_term_handler(int sig);

private:
    std::auto_ptr<AdminServer> _admin_server;
    std::auto_ptr<NoxVM> _vm;
    Event* _quit_event;
    bool _quitting;
    struct sigaction _prev_term_act;
    struct sigaction _prev_int_act;
};

extern Application* application;

#endif

