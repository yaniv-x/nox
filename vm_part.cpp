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

#include "vm_part.h"
#include "nox_vm.h"
#include "io_bus.h"


VMPart::VMPart(const char* name, VMPart& container)
    : _name (name)
    , _container (&container)
    , _state (INIT)
{
    if (_container) {
        _container->add_part(this);
    }
}

VMPart::VMPart(const char* name)
    : _name (name)
    , _container (NULL)
    , _state (INIT)
{
}


VMPart::~VMPart()
{
    unregister_regions();

    if (_container) {
         _container->remove_part(this);
    }

    ASSERT(_parts.empty());
}


void VMPart::add_part(VMPart* part)
{
    _parts.push_back(part);
}


void VMPart::remove_part(VMPart* part)
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        if ((*iter) == part) {
            _parts.erase(iter);
            return;
        }
    }

    PANIC("not found");
}


void VMPart::add_io_region(IORegion* region)
{
    _regions.push_back(region);
}


void VMPart::remap_io_regions()
{
    std::list<IORegion*>::iterator iter =  _regions.begin();

    for (; iter != _regions.end(); iter++) {
        get_nox().get_io_bus().remap_region(*iter);
    }
}


void VMPart::unregister_regions()
{
    while (!_regions.empty()) {
        get_nox().get_io_bus().unregister_region(*_regions.begin());
        _regions.pop_front();
    }
}


void VMPart::reset_all()
{
    _state = RESETING;

    reset();

    _state = READY;

    reset_childrens();
}


void VMPart::down_all()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->down_all();
    }

    down();

    _state = DOWN;
}


void VMPart::freeze_all()
{
    VMParts::reverse_iterator iter = _parts.rbegin();

    for (; iter != _parts.rend(); iter++) {
        (*iter)->freeze_all();
    }

    _state = FREEZING;

    freeze();

    _state = FREEZED;
}


void VMPart::unfreeze_all()
{
    _state = BACK_TO_SLEEP;

    unfreeze();

    _state = SLEEPING;

    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->unfreeze_all();
    }
}


void VMPart::debug_all()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->debug_all();
    }

    _state = DEBUGGING;
}


void VMPart::set_state_all(State state)
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->set_state_all(state);
    }

    _state = state;
}


void VMPart::reset_childrens()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->reset_all();
    }
}


void VMPart::transition_done()
{
    switch (_state) {
    case STARTING:
        _state = RUNNING;
        break;
    case ABOUT_TO_SLEEP:
        _state = SLEEPING;
        break;
    case FREEZING:
        _state = FREEZED;
        break;
    default:
        PANIC("unexpected state");
    }

    get_nox().resume_mode_change();
}


bool VMPart::start_all()
{
    switch (_state) {
    case RUNNING:
        break;
    case SLEEPING:
    case FREEZED:
    case READY:
        _state = STARTING;
        if (start()) {
            _state = RUNNING;
            break;
        }
    case STARTING:
        return false;
    default:
        D_MESSAGE("unexpected %u", get_state());
        return false;
    }

    return start_childrens();
}


bool VMPart::start_childrens()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        if (!(*iter)->start_all()) {
            return false;
        }
    }

    return true;
}


bool VMPart::stop_all(State pre, State post)
{
    if (!stop_childrens(pre, post)) {
        return false;
    }

    if (_state == RUNNING) {
        _state = pre;

        if (stop()) {
            _state = post;
        }
    }

    ASSERT(_state == pre || _state == post);

    return _state == post;
}


bool VMPart::stop_childrens(State pre, State post)
{
    VMParts::reverse_iterator iter = _parts.rbegin();

    for (; iter != _parts.rend(); iter++) {
        if (!(*iter)->stop_all(pre, post)) {
            return false;
        }
    }

    return true;
}

