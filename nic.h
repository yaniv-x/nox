/*
    Copyright (c) 2014 Yaniv Kamay,
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

#ifndef _H_NIC
#define _H_NIC

#include "pci_device.h"
#include "run_loop.h"

class NoxVM;
struct LegacyTxDescriptor;
struct TxDataDescriptor;


enum {
    NUM_RECEIVE_ADDR = 16,
    MULTICAST_TABLE_SIZE = 128,
    NIC_RSS_KEY_SIZE = 40,
    NIC_REDIRECTION_TABLE_SIZE = 128,
    NUM_STATISTIC_REGS = 65,
};


typedef struct __attribute__ ((__packed__)) TxContextDescriptor {
    uint8_t ipcss;
    uint8_t ipcso;
    uint16_t ipcse;
    uint8_t tucss;
    uint8_t tucso;
    uint16_t tucse;
    union {
        uint32_t mix;
        uint8_t mix_v[4];
    };
    uint8_t status;
    uint8_t hdrlen;
    uint16_t mss;
} TxContextDescriptor;


class NIC: public PCIDevice {
public:
    NIC(NoxVM& nox);
    virtual ~NIC();

protected:
    virtual void reset();
    virtual bool start();
    virtual bool stop();
    virtual void down();

private:
    class Queue {
    public:
        void reset(uint32_t qx_cause_mask);
        void begin();

        void set_addr_low(uint32_t val);
        void set_addr_high(uint32_t val);
        void set_length(uint32_t val) { _length = val; begin();}
        void set_public_head(uint32_t head) { _public_head = head;}
        void set_tail(uint32_t tail) { _tail = tail;}
        void set_descriptor_ctrl(uint32_t descriptor_ctrl) { _descriptor_ctrl = descriptor_ctrl;}
        uint32_t get_tail() { return _tail;}
        uint32_t get_public_head() { return _public_head;}
        uint32_t get_descriptor_ctrl() { return _descriptor_ctrl;}

        bool is_empty() { return _private_head == _tail;}
        void pop() {  _private_head = (_private_head + 1) % _q_size;}
        void relase_used() { _public_head = _private_head;}
        uint64_t head_address() { return _address + (_private_head << 4);}
        uint32_t get_qx_cause_mask() { return _qx_cause_mask;}
        uint get_wb_threshold() { return  (_descriptor_ctrl >> 16) & 0x3f;}
        uint get_low_Threshold() { return  _descriptor_ctrl >> 25;}
        uint get_wb_count() { return delta(_public_head, _private_head);}
        uint get_unused_count() { return delta(_private_head, _tail);}
        uint32_t get_private_head() { return _private_head;}
        uint64_t get_base_address() { return _address;}
        uint num_items() { return _q_size;}

    private:
        uint32_t delta(uint32_t start, uint32_t end)
        {
            uint32_t ret = end - start;
            return (ret & (1 << 31)) ? _q_size + ret : ret;
        }

    private:
        uint64_t _address;
        uint32_t _length;
        uint32_t _public_head;
        uint32_t _tail;
        uint32_t _descriptor_ctrl;

        uint32_t _private_head;
        uint32_t _qx_cause_mask;
        uint32_t _q_size;
    };

    class NicTimer {
    public:
        NicTimer(const char* name, Timer* a, Timer* b);
        ~NicTimer();

        void reset();
        void arm();
        void disarm();
        void resume();
        void suspend();

        void set_val(uint32_t val);
        void set_abs_val(uint32_t val);

        uint32_t private_delay_val() { return _delay_val;}

    private:
        std::string _name;
        Timer* _timer;
        Timer* _abs_timer;
        bool _abs_timer_armed;
        uint32_t _delay_val;
        uint32_t _abs_delay_val;
    };

private:
    void tx_timer_handler();
    void tx_write_back(Queue& queue);
    void tx_write_back();
    void rx_timer_handler();
    void rx_write_back(Queue& queue, uint32_t cause);
    void rx_write_back(uint32_t cause);
    void tr_cmd(uint cmd);
    void tr_wait(uint cmd);
    void tr_set_state(int state);
    void handle_legacy_tx(LegacyTxDescriptor& descriptor);
    void handle_tx_data(TxDataDescriptor& descriptor);
    void handle_tx_context(TxContextDescriptor& descriptor);
    bool do_tx(Queue& queue);
    void tx_trigger_handler();
    void rx_trigger_handler();
    void interface_event_handler();
    void* thread_main();
    bool ether_filter(uint8_t* address);
    void push(uint8_t* packet, uint length);
    void drop(uint8_t *buf, ssize_t n);
    bool recive_data();
    void trancive();

    void csr_read(uint64_t src, uint64_t length, uint8_t* dest);
    void csr_write(const uint8_t* src, uint64_t length, uint64_t dest);
    uint32_t io_read_dword(uint16_t port);
    void io_write_dword(uint16_t port, uint32_t val);

    void mdi_set_data(uint16_t val);
    void mdi_read_common(uint reg);
    void mdi_page0_read(uint reg);
    void mdi_read(uint reg);

    void mdi_write_common(uint reg);
    void mdi_page0_write(uint reg);
    void mdi_write(uint reg);

    void common_reset();
    void init_eeprom();
    void nv_load();
    void phy_reset_interface(const char* interface_name);
    void phy_reset_common();
    void phy_reset();
    void phy_soft_reset();
    uint phy_speed_select();
    void phy_update_link_state();
    bool is_auto_negotiation();
    bool is_phy_power_up();
    void update_interrupt_level();
    void interrupt(uint32_t cause);
    void soft_reset();
    void eeprom_reset();
    void speed_detection();
    void auto_negotiation();
    void mac_update_link_state();
    bool auto_sens_speed(uint& speed, bool& duplex);
    void phy_manual_config();
    bool phy_link_is_up() { return _interface.is_valid();}

    void tx_trigger();
    void rx_trigger();

private:
    Mutex _mutex;
    uint8_t _in_buf[16384 /*max long packet size*/];
    int _tr_state;
    int _tr_command;
    uint32_t _io_address;
    AutoFD _interface;
    RunLoop _transceiver;
    Event* _tx_trigger;
    Event* _rx_trigger;
    FDEvent* _interface_event;
    NicTimer _tx_timer;
    NicTimer _rx_timer;
    Thread _thread;
    Mutex _tr_cmd_lock;
    Condition _tr_cmd_condition;
    Mutex _tr_state_lock;
    Condition _tr_state_condition;
    TxContextDescriptor _seg_context;
    TxContextDescriptor _offload_context;

    uint32_t _ctrl;
    uint32_t _status;
    uint32_t _int_mask;
    uint32_t _int_cause;
    uint32_t _auto_mask;
    uint32_t _int_throttling;
    uint32_t _soft_sem;
    uint32_t _ext_conf_ctrl;
    uint32_t _mdi_ctrl;

    uint32_t _rx_ctrl;
    uint32_t _rx_checksum_ctrl;
    uint32_t _rx_filter_ctrl;
    Queue _rx_queue[2];

    uint32_t _tx_ctrl;
    uint32_t _tx_arbitration_count_0;
    uint32_t _tx_arbitration_count_1;
    Queue _tx_queue[2];

    uint32_t _rom_read;
    uint32_t _packet_buff_alloc;
    uint32_t _packet_buff_size;
    uint32_t _multi_rq_command;

    uint32_t _ext_ctrl;
    uint32_t _nvm_ctrl;

    uint32_t _wakeup_ctrl;
    uint32_t _wakeup_filter;
    uint32_t _led_ctrl;

    uint32_t _3gio_ctrl_1;
    uint32_t _3gio_ctrl_2;

    uint32_t _vlan_ether_type;
    uint32_t _adaptive_IFS_throttle;

    uint32_t _time_sync_rx_ctrl;
    uint32_t _time_sync_tx_ctrl;
    uint32_t _time_sync_mess_type;
    uint32_t _time_sync_udp_port;

    enum {
        VLAN_FILTER_TABLE_SIZE = 128,
    };

    uint32_t _vlan_filter_table[VLAN_FILTER_TABLE_SIZE];
    uint32_t _receive_addr_table[NUM_RECEIVE_ADDR * 2];
    uint32_t _rx_rss_key[NIC_RSS_KEY_SIZE / 4];
    uint32_t _redirection_table[NIC_REDIRECTION_TABLE_SIZE / 4];
    uint32_t _multicast_table[MULTICAST_TABLE_SIZE];
    uint32_t _statistic[NUM_STATISTIC_REGS];

    enum {
        EEPROM_WORDS = 64 * 2,
    };

    uint16_t _rom[EEPROM_WORDS];

    uint16_t _phy_ctrl;
    uint16_t _phy_status;
    uint16_t _phy_id_2;
    uint16_t _phy_page;
    uint16_t _phy_auto_nego;
    uint16_t _phy_partner_ability;
    uint16_t _phy_1000_ctrl;
    uint16_t _phy_1000_status;
    uint16_t _phy_0_copper_ctrl_1;
    uint16_t _phy_0_copper_status_1;
    uint16_t _phy_0_bias_1;
    uint16_t _phy_0_bias_2;
    uint16_t _phy_0_oem_bits;
};

#endif

