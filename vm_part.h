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

#ifndef _H_VM_PART
#define _H_VM_PART

#include "common.h"

class OutStream;
class InStream;
class IORegion;
class NoxVM;


class VMPart: private NonCopyable {
public:
    VMPart(const char* name, VMPart& container);
    virtual ~VMPart();

    void add_part(VMPart* chaild);
    void remove_part(VMPart* chaild);

    const char* get_name() { return _name.c_str();}
    VMPart* get_container() { return _container;}
    NoxVM& get_nox();

    virtual void reset() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void power() = 0;
    virtual void save(OutStream& stream) = 0;
    virtual void load(InStream& stream) = 0;

    void add_io_region(IORegion* region);

    /*enum State {
        todo add states values
        + set state
        + get state
    };*/

private:
    VMPart(const char* name);
    void unregister_regions();

private:
    typedef std::list<VMPart*> VMParts;

    std::string _name;
    VMPart* _container;
    VMParts _parts;
    std::list<IORegion*> _regions;

    friend class NoxVM;
};

class ResetException: public std::exception {
public:
    virtual const char* what() const throw () {return "reset exception: implement me!!!";}

};

inline NoxVM& VMPart::get_nox()
{
    if (!_container) {
        return *(NoxVM*)this;
    }

    return get_container()->get_nox();
}

#endif

