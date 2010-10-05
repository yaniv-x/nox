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

#ifndef _H_RING
#define _H_RING

class RingItem {
public:
    RingItem()
        : prev (NULL)
        , next (NULL)
    {
    }

    RingItem* prev;
    RingItem* next;
};


class Ring {
public:

    Ring()
    {
        _ring.next = &_ring;
        _ring.prev = &_ring;
    }

    bool empty()
    {
        return _ring.next == &_ring;
    }

    static bool is_linked(RingItem& item)
    {
        return !!item.next;
    }

    RingItem* head()
    {
        if (empty()) {
            return NULL;
        }

        return _ring.next;
    }

    RingItem* tail()
    {
        if (empty()) {
            return NULL;
        }

        return _ring.next;
    }

    RingItem* next(RingItem& item)
    {
        ASSERT(valid_linked(item));

        if (item.next == &_ring) {
            return NULL;
        }

        return item.next;
    }

    RingItem* prev(RingItem& item)
    {
        ASSERT(valid_linked(item));

        if (item.prev == &_ring) {
            return NULL;
        }

        return item.prev;
    }

    static bool valid_linked(RingItem& item)
    {
        return item.next && item.prev && item.next->prev == &item && item.prev->next == &item;
    }

    static bool valid_not_linked(RingItem& item)
    {
        return item.next == NULL && item.prev == NULL;
    }

    static void remove(RingItem& item)
    {
        ASSERT(valid_linked(item));
        item.prev->next = item.next;
        item.next->prev = item.prev;
        item.next = NULL;
        item.prev = NULL;
    }

    void push_front(RingItem& item)
    {
        ASSERT(valid_not_linked(item));
        item.next = _ring.next;
        item.prev = &_ring;

        _ring.next->prev = &item;
        _ring.next = &item;
    }

    void push_back(RingItem& item)
    {
        ASSERT(valid_not_linked(item));
        item.next = &_ring;
        item.prev = _ring.prev;

        _ring.prev->next = &item;
        _ring.prev = &item;
    }

    static void insert_before(RingItem& item, RingItem& pos)
    {
        ASSERT(valid_not_linked(item));
        ASSERT(valid_linked(pos));
        item.next = &pos;
        item.prev = pos.prev;
        pos.prev = &item;
        item.prev->next = &item;
    }

private:
    RingItem _ring;
};

#endif

