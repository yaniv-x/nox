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


void VMPart::reset_childrens()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->reset_all();
    }
}


void VMPart::start_all()
{
    _state = STARTING;

    start();

    _state = RUNNING;

    start_childrens();
}


void VMPart::start_childrens()
{
    VMParts::iterator iter = _parts.begin();

    for (; iter != _parts.end(); iter++) {
        (*iter)->start_all();
    }
}


void VMPart::stop_all()
{
    stop_childrens();

    _state = STOPPING;

    stop();

    _state = STOPPED;
}


void VMPart::stop_childrens()
{
    VMParts::reverse_iterator iter = _parts.rbegin();

    for (; iter != _parts.rend(); iter++) {
        (*iter)->stop_all();
    }
}

