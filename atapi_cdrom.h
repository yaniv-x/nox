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

#ifndef _H_ATAPI_CDROM
#define _H_ATAPI_CDROM

#include "ata_device.h"
#include "ata_host.h"

class AdminReplyContext;
class BlockDevice;

class ATAPICdrom: public ATADevice {
public:
    ATAPICdrom(VMPart& owner, Wire& wire, const std::string& file_name);
    virtual ~ATAPICdrom();

    virtual uint8_t io_read(uint16_t port);

protected:
    virtual void set_signature();
    virtual void do_command(uint8_t command);
    virtual void reset(bool cold);

private:
    void _packet_cmd_done(uint sense, uint sense_add);
    void packet_cmd_abort(uint sens, uint sens_add);
    void packet_cmd_chk(uint sens, uint sens_add);
    void packet_cmd_sucess();
    void read_formatted_toc(uint8_t* packet);
    void read_raw_toc(uint8_t* packet);

    void handle_packet(uint8_t* packet);
    uint max_pio_transfer_bytes();

    void mmc_read_capacity(uint8_t* packet);
    void mmc_read(uint8_t* packet);
    void mmc_read_toc(uint8_t* packet);
    void mmc_get_configuration(uint8_t* packet);
    void mmc_prevent_allow_removal(uint8_t* packet);
    void mmc_get_event_status_notification(uint8_t* packet);
    void mmc_mechanisim_status(uint8_t* packet);
    void mmc_start_stop_unit(uint8_t* packet);
    void scsi_request_sens(uint8_t* packet);
    void scsi_test_unit_ready(uint8_t* packet);
    void scsi_inquiry(uint8_t* packet);
    void scsi_mode_sense(uint8_t* packet);
    void do_packet_command();
    void do_set_features();
    void do_identify_packet_device();
    void do_device_reset();

    void set_media_command(AdminReplyContext* context, const char* name);
    void eject_command(AdminReplyContext* context);
    void register_admin_commands();
    void get_media() { _media_refs.inc();}
    void put_media() { _media_refs.dec();}
    bool is_tray_locked();
    bool is_tray_open();
    void _open_tray();
    void _close_tray();
    void open_tray();
    void eject_button_press();
    void set_media(const std::string& file_name);
    uint get_not_present_sens_add();
    bool handle_attention_condition();

private:
    std::auto_ptr<BlockDevice> _media;
    BlockDevice* _mounted_media;
    Mutex _media_lock;
    Atomic _media_refs;

    uint _sense;
    uint _sense_add;
    uint _cdrom_state;

    friend class CDIdentifyTask;
    friend class PacketTask;
    friend class CDGenericTransfer;
    friend class CDReadTask;
    friend class DeferSetMedia;
};


class ATAPICdromFactory: public ATADeviceFactory {
public:
    ATAPICdromFactory(const std::string& file_mame);
    virtual ATADevice* creat_device(VMPart &owner, Wire &wire);

private:
    std::string _file_name;
};


#endif

