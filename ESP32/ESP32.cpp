/* ESP32 Example
 * Copyright (c) 2015 ARM Limited
 * Copyright (c) 2017 Renesas Electronics Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if DEVICE_SERIAL && defined(MBED_CONF_EVENTS_PRESENT) && defined(MBED_CONF_NSAPI_PRESENT) && defined(MBED_CONF_RTOS_PRESENT)
#include "ESP32.h"

#define ESP32_DEFAULT_BAUD_RATE   115200
#define ESP32_ALL_SOCKET_IDS      -1

using namespace mbed;
using namespace rtos;

ESP32 * ESP32::instESP32 = NULL;

ESP32 * ESP32::getESP32Inst(PinName en, PinName io0, PinName tx, PinName rx, bool debug,
                            PinName rts, PinName cts, int baudrate)
{
    if (instESP32 == NULL) {
        instESP32 = new ESP32(en, io0, tx, rx, debug, rts, cts, baudrate);
    } else {
        if (debug) {
            instESP32->debugOn(debug);
        }
    }
    return instESP32;
}

ESP32::ESP32(PinName en, PinName io0, PinName tx, PinName rx, bool debug,
    PinName rts, PinName cts, int baudrate)
    : _p_wifi_en(NULL), _p_wifi_io0(NULL), init_end(false)
    , _serial(tx, rx, ESP32_DEFAULT_BAUD_RATE), _parser(&_serial, "\r\n")
    , _packets(0), _packets_end(&_packets)
    , _id_bits(0), _id_bits_close(0), _server_act(false)
    , _wifi_status(STATUS_DISCONNECTED)
    , _wifi_status_cb(NULL)
{
    if ((int)en != NC) {
        _p_wifi_en = new DigitalOut(en);
    }
    if ((int)io0 != NC) {
        _p_wifi_io0 = new DigitalOut(io0);
    }

    _wifi_mode = WIFIMODE_STATION;
    _baudrate = baudrate;
    memset(_ids, 0, sizeof(_ids));
    memset(_cbs, 0, sizeof(_cbs));

    _rts = rts;
    _cts = cts;

    if ((_rts != NC) && (_cts != NC)) {
        _flow_control = 3;
    } else if (_rts != NC) {
        _flow_control = 1;
    } else if (_cts != NC) {
        _flow_control = 2;
    } else {
        _flow_control = 0;
    }

    _serial.set_baud(ESP32_DEFAULT_BAUD_RATE);
    debugOn(debug);

    _parser.oob("+IPD", callback(this, &ESP32::_packet_handler));
    _parser.oob("0,CONNECT", callback(this, &ESP32::_connect_handler_0));
    _parser.oob("1,CONNECT", callback(this, &ESP32::_connect_handler_1));
    _parser.oob("2,CONNECT", callback(this, &ESP32::_connect_handler_2));
    _parser.oob("3,CONNECT", callback(this, &ESP32::_connect_handler_3));
    _parser.oob("4,CONNECT", callback(this, &ESP32::_connect_handler_4));
    _parser.oob("0,CLOSED", callback(this, &ESP32::_closed_handler_0));
    _parser.oob("1,CLOSED", callback(this, &ESP32::_closed_handler_1));
    _parser.oob("2,CLOSED", callback(this, &ESP32::_closed_handler_2));
    _parser.oob("3,CLOSED", callback(this, &ESP32::_closed_handler_3));
    _parser.oob("4,CLOSED", callback(this, &ESP32::_closed_handler_4));
    _parser.oob("WIFI ", callback(this, &ESP32::_connection_status_handler));

    _serial.sigio(Callback<void()>(this, &ESP32::event));

    setTimeout();
}

void ESP32::debugOn(bool debug)
{
    _parser.debug_on((debug) ? 1 : 0);
}

int ESP32::get_firmware_version()
{
    int version;

    _smutex.lock();
    startup();
    bool done = _parser.send("AT+GMR")
           && _parser.recv("SDK version:%d", &version)
           && _parser.recv("OK");
    _smutex.unlock();

    if(done) {
        return version;
    } else { 
        // Older firmware versions do not prefix the version with "SDK version: "
        return -1;
    }
}

bool ESP32::startup()
{
    if (init_end) {
        return true;
    }

    if (_p_wifi_io0 != NULL) {
        _p_wifi_io0->write(1);
    }
    if (_p_wifi_en != NULL) {
        _p_wifi_en->write(0);
        ThisThread::sleep_for(10);
        _p_wifi_en->write(1);
        _parser.recv("ready");
    } else {
        setTimeout(100);
        _parser.recv("ready");
    }

    reset();
    bool success = _parser.send("AT+CWMODE=%d", _wifi_mode)
                && _parser.recv("OK")
                && _parser.send("AT+CIPMUX=1")
                && _parser.recv("OK")
                && _parser.send("AT+CWAUTOCONN=0")
                && _parser.recv("OK")
                && _parser.send("AT+CWQAP")
                && _parser.recv("OK");
    if (success) {
        init_end = true;
    }

    return success;
}

bool ESP32::restart()
{
    bool success;

    _smutex.lock();
    if (!init_end) {
        success = startup();
    } else {
        reset();
        success = _parser.send("AT+CWMODE=%d", _wifi_mode)
               && _parser.recv("OK")
               && _parser.send("AT+CIPMUX=1")
               && _parser.recv("OK");
    }
    _smutex.unlock();

    return success;
}

bool ESP32::set_mode(int mode)
{
    //only 3 valid modes
    if (mode < 1 || mode > 3) {
        return false;
    }
    if (_wifi_mode != mode) {
        _wifi_mode = mode;
        return restart();
    }
    return true;
}

bool ESP32::cre_server(int port)
{
    if (_server_act) {
        return false;
    }
    _smutex.lock();
    startup();
    if (!(_parser.send("AT+CIPSERVER=1,%d", port)
        && _parser.recv("OK"))) {
        _smutex.unlock();
        return false;
    }
    _server_act = true;
    _smutex.unlock();
    return true;
}

bool ESP32::del_server()
{
    _smutex.lock();
    startup();
    if (!(_parser.send("AT+CIPSERVER=0")
        && _parser.recv("OK"))) {
        _smutex.unlock();
        return false;
    }
    _server_act = false;
    _smutex.unlock();
    return true;
}

void ESP32::socket_handler(bool connect, int id)
{
    _cbs[id].Notified = 0;
    if (connect) {
        _id_bits |= (1 << id);
        if (_server_act) {
            _accept_id.push_back(id);
        }
    } else {
        _id_bits &= ~(1 << id);
        _id_bits_close |= (1 << id);
        if (_server_act) {
            for (size_t i = 0; i < _accept_id.size(); i++) {
                if (id == _accept_id[i]) {
                    _accept_id.erase(_accept_id.begin() + i);
                }
            }
        }
    }
}

bool ESP32::accept(int * p_id)
{
    bool ret = false;

    while (!ret) {
        if (!_server_act) {
            break;
        }

        _smutex.lock();
        startup();
        if (!_accept_id.empty()) {
            ret = true;
        } else {
            _parser.process_oob(); // Poll for inbound packets
            if (!_accept_id.empty()) {
                ret = true;
            }
        }
        if (ret) {
            *p_id = _accept_id[0];
            _accept_id.erase(_accept_id.begin());
        }
        _smutex.unlock();
        if (!ret) {
            ThisThread::sleep_for(5);
        }
    }

    if (ret) {
        for (int i = 0; i < 50; i++) {
            if ((_id_bits_close & (1 << *p_id)) == 0) {
                break;
            }
            ThisThread::sleep_for(10);
        }
    }

    return ret;
}

bool ESP32::reset(void)
{
    for (int i = 0; i < 2; i++) {
        if (_parser.send("AT+RST")
            && _parser.recv("OK")) {
            _serial.set_baud(ESP32_DEFAULT_BAUD_RATE);
#if DEVICE_SERIAL_FC
            _serial.set_flow_control(SerialBase::Disabled);
#endif
            _parser.recv("ready");
            _clear_socket_packets(ESP32_ALL_SOCKET_IDS);

            if (_parser.send("AT+UART_CUR=%d,8,1,0,%d", _baudrate, _flow_control)
                && _parser.recv("OK")) {
                _serial.set_baud(_baudrate);
#if DEVICE_SERIAL_FC
                switch (_flow_control) {
                    case 1:
                        _serial.set_flow_control(SerialBase::RTS, _rts);
                        break;
                    case 2:
                        _serial.set_flow_control(SerialBase::CTS, _cts);
                        break;
                    case 3:
                        _serial.set_flow_control(SerialBase::RTSCTS, _rts, _cts);
                        break;
                    case 0:
                    default:
                        // do nothing
                        break;
                }
#endif
            }

            return true;
        }
    }

    return false;
}

bool ESP32::dhcp(bool enabled, int mode)
{
    //only 3 valid modes
    if (mode < 0 || mode > 2) {
        return false;
    }

    _smutex.lock();
    startup();
    bool done = _parser.send("AT+CWDHCP=%d,%d", enabled?1:0, mode)
       && _parser.recv("OK");
    _smutex.unlock();

    return done;
}

bool ESP32::connect(const char *ap, const char *passPhrase)
{
    bool ret;

    _wifi_status = STATUS_DISCONNECTED;

    _smutex.lock();
    startup();

    setTimeout(ESP32_CONNECT_TIMEOUT);
    ret = _parser.send("AT+CWJAP=\"%s\",\"%s\"", ap, passPhrase)
       && _parser.recv("OK");
    setTimeout();
    _smutex.unlock();
    return ret;
}

bool ESP32::config_soft_ap(const char *ap, const char *passPhrase, uint8_t chl, uint8_t ecn)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CWSAP=\"%s\",\"%s\",%hhu,%hhu", ap, passPhrase, chl, ecn)
       && _parser.recv("OK");
    _smutex.unlock();
    return ret;
}

bool ESP32::get_ssid(char *ap)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CWJAP?")
       && _parser.recv("+CWJAP:\"%33[^\"]\",", ap)
       && _parser.recv("OK");
    _smutex.unlock();
    return ret;
}

bool ESP32::disconnect(void)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CWQAP") && _parser.recv("OK");
    _smutex.unlock();
    return ret;
}

const char *ESP32::getIPAddress(void)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIFSR")
       && _parser.recv("+CIFSR:STAIP,\"%15[^\"]\"", _ip_buffer)
       && _parser.recv("OK");
    _smutex.unlock();
    if (!ret) {
        return 0;
    }
    return _ip_buffer;
}

const char *ESP32::getIPAddress_ap(void)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIFSR")
       && _parser.recv("+CIFSR:APIP,\"%15[^\"]\"", _ip_buffer_ap)
       && _parser.recv("OK");
    _smutex.unlock();
    if (!ret) {
        return 0;
    }
    return _ip_buffer_ap;
}

const char *ESP32::getMACAddress(void)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIFSR")
       && _parser.recv("+CIFSR:STAMAC,\"%17[^\"]\"", _mac_buffer)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _mac_buffer;
}

const char *ESP32::getMACAddress_ap(void)
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIFSR")
       && _parser.recv("+CIFSR:APMAC,\"%17[^\"]\"", _mac_buffer_ap)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _mac_buffer_ap;
}

const char *ESP32::getGateway()
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIPSTA?")
       && _parser.recv("+CIPSTA:gateway:\"%15[^\"]\"", _gateway_buffer)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _gateway_buffer;
}

const char *ESP32::getGateway_ap()
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIPAP?")
       && _parser.recv("+CIPAP:gateway:\"%15[^\"]\"", _gateway_buffer_ap)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _gateway_buffer_ap;
}

const char *ESP32::getNetmask()
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIPSTA?")
       && _parser.recv("+CIPSTA:netmask:\"%15[^\"]\"", _netmask_buffer)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _netmask_buffer;
}

const char *ESP32::getNetmask_ap()
{
    bool ret;

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CIPAP?")
       && _parser.recv("+CIPAP:netmask:\"%15[^\"]\"", _netmask_buffer_ap)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }
    return _netmask_buffer_ap;
}

int8_t ESP32::getRSSI()
{
    bool ret;
    int8_t rssi;
    char ssid[33];
    char bssid[18];

    _smutex.lock();
    startup();
    ret = _parser.send("AT+CWJAP?")
       && _parser.recv("+CWJAP:\"%32[^\"]\",\"%17[^\"]\"", ssid, bssid)
       && _parser.recv("OK");
    if (!ret) {
        _smutex.unlock();
        return 0;
    }

    ret = _parser.send("AT+CWLAP=\"%s\",\"%s\"", ssid, bssid)
       && _parser.recv("+CWLAP:(%*d,\"%*[^\"]\",%hhd,", &rssi)
       && _parser.recv("OK");
    _smutex.unlock();

    if (!ret) {
        return 0;
    }

    return rssi;
}

int ESP32::scan(WiFiAccessPoint *res, unsigned limit)
{
    unsigned cnt = 0;
    nsapi_wifi_ap_t ap;

    if (!init_end) {
        _smutex.lock();
        startup();
        _smutex.unlock();
        ThisThread::sleep_for(1500);
    }

    _smutex.lock();
    setTimeout(5000);
    if (!_parser.send("AT+CWLAP")) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    while (recv_ap(&ap)) {
        if (cnt < limit) {
            res[cnt] = WiFiAccessPoint(ap);
        }

        cnt++;
        if ((limit != 0) && (cnt >= limit)) {
            break;
        }
        setTimeout(500);
    }
    setTimeout(10);
    _parser.recv("OK");
    setTimeout();
    _smutex.unlock();

    return cnt;
}

bool ESP32::isConnected(void)
{
    return getIPAddress() != 0;
}

bool ESP32::open(const char *type, int id, const char* addr, int port, int opt)
{
    bool ret;

    if (id >= SOCKET_COUNT) {
        return false;
    }
    _cbs[id].Notified = 0;

    _smutex.lock();
    startup();
    setTimeout(500);
    if (opt != 0) {
        ret = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d, %d", id, type, addr, port, opt)
           && _parser.recv("OK");
    } else {
        ret = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, type, addr, port)
           && _parser.recv("OK");
    }
    setTimeout();
    _clear_socket_packets(id);
    _smutex.unlock();

    return ret;
}

bool ESP32::send(int id, const void *data, uint32_t amount)
{
    int send_size;
    bool ret;
    int error_cnt = 0;
    int index = 0;

    _cbs[id].Notified = 0;
    if (amount == 0) {
        return true;
    }

    //May take a second try if device is busy
    _smutex.lock();
    while (error_cnt < 2) {
        if (((_id_bits & (1 << id)) == 0)
         || ((_id_bits_close & (1 << id)) != 0)) {
            _smutex.unlock();
            return false;
        }
        send_size = amount;
        if (send_size > 2048) {
            send_size = 2048;
        }
        startup();
        setTimeout(ESP32_SEND_TIMEOUT);
        ret = _parser.send("AT+CIPSEND=%d,%d", id, send_size)
           && _parser.recv(">")
           && (_parser.write((char*)data + index, (int)send_size) >= 0)
           && _parser.recv("SEND OK");
        setTimeout();
        if (ret) {
            amount -= send_size;
            index += send_size;
            error_cnt = 0;
            if (amount == 0) {
                _smutex.unlock();
                return true;
            }
        } else {
            error_cnt++;
        }
    }
    _smutex.unlock();

    return false;
}

void ESP32::_packet_handler()
{
    int id;
    int amount;
    uint32_t tmp_timeout;

    // parse out the packet
    if (!_parser.recv(",%d,%d:", &id, &amount)) {
        return;
    }

    struct packet *packet = (struct packet*)malloc(
            sizeof(struct packet) + amount);
    if (!packet) {
        return;
    }

    packet->id = id;
    packet->len = amount;
    packet->next = 0;
    packet->index = 0;

    tmp_timeout = last_timeout_ms;
    setTimeout(500);
    if (!(_parser.read((char*)(packet + 1), amount))) {
        free(packet);
        setTimeout(tmp_timeout);
        return;
    }

    // append to packet list
    *_packets_end = packet;
    _packets_end = &packet->next;
}

int32_t ESP32::recv(int id, void *data, uint32_t amount, uint32_t timeout)
{
    struct packet **p;
    uint32_t idx = 0;

    _cbs[id].Notified = 0;

    _smutex.lock();
    setTimeout(timeout);
    if (_rts == NC) {
        while (_parser.process_oob()); // Poll for inbound packets
    } else {
        _parser.process_oob(); // Poll for inbound packets
    }
    setTimeout();

    // check if any packets are ready for us
    p = &_packets;
    while (*p) {
        if ((*p)->id == id) {
            struct packet *q = *p;

            if (q->len <= amount) { // Return and remove full packet
                memcpy(&(((uint8_t *)data)[idx]), (uint8_t*)(q+1) + q->index, q->len);
                if (_packets_end == &(*p)->next) {
                    _packets_end = p;
                }
                *p = (*p)->next;
                idx += q->len;
                amount -= q->len;
                free(q);
            } else { // return only partial packet
                memcpy(&(((uint8_t *)data)[idx]), (uint8_t*)(q+1) + q->index, amount);
                q->len -= amount;
                q->index += amount;
                idx += amount;
                break;
            }
        } else {
            p = &(*p)->next;
        }
    }
    _smutex.unlock();

    if (idx > 0) {
        return idx;
    } else if (((_id_bits & (1 << id)) == 0) || ((_id_bits_close & (1 << id)) != 0)) {
        return -2;
    } else {
        return -1;
    }
}

void ESP32::_clear_socket_packets(int id)
{
    struct packet **p = &_packets;

    while (*p) {
        if ((*p)->id == id || id == ESP32_ALL_SOCKET_IDS) {
            struct packet *q = *p;

            if (_packets_end == &(*p)->next) {
                _packets_end = p; // Set last packet next field/_packets
            }
            *p = (*p)->next;

            free(q);
        } else {
            // Point to last packet next field
            p = &(*p)->next;
        }
    }
}

bool ESP32::close(int id, bool wait_close)
{
    if (wait_close) {
        _smutex.lock();
        for (int j = 0; j < 2; j++) {
            if (((_id_bits & (1 << id)) == 0)
             || ((_id_bits_close & (1 << id)) != 0)) {
                _id_bits_close &= ~(1 << id);
                _ids[id] = false;
                _clear_socket_packets(id);
                _smutex.unlock();
                return true;
            }
            startup();
            setTimeout(500);
            _parser.process_oob(); // Poll for inbound packets
            setTimeout();
        }
        _smutex.unlock();
    }

    //May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        _smutex.lock();
        if ((_id_bits & (1 << id)) == 0) {
            _id_bits_close &= ~(1 << id);
            _ids[id] = false;
            _clear_socket_packets(id);
            _smutex.unlock();
            return true;
        }
        startup();
        setTimeout(500);
        if (_parser.send("AT+CIPCLOSE=%d", id)
            && _parser.recv("OK")) {
            setTimeout();
            _clear_socket_packets(id);
            _id_bits_close &= ~(1 << id);
            _ids[id] = false;
            _smutex.unlock();
            return true;
        }
        setTimeout();
        _smutex.unlock();
    }

    _ids[id] = false;
    return false;
}

void ESP32::setTimeout(uint32_t timeout_ms)
{
    last_timeout_ms = timeout_ms;
    _parser.set_timeout(timeout_ms);
}

bool ESP32::readable()
{
    return _serial.FileHandle::readable();
}

bool ESP32::writeable()
{
    return _serial.FileHandle::writable();
}

void ESP32::socket_attach(int id, void (*callback)(void *), void *data)
{
    _cbs[id].callback = callback;
    _cbs[id].data = data;
    _cbs[id].Notified = 0;
}

bool ESP32::recv_ap(nsapi_wifi_ap_t *ap)
{
    int sec;
    bool ret = _parser.recv("+CWLAP:(%d,\"%32[^\"]\",%hhd,\"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\",%hhu)", &sec, ap->ssid,
                            &ap->rssi, &ap->bssid[0], &ap->bssid[1], &ap->bssid[2], &ap->bssid[3], &ap->bssid[4],
                            &ap->bssid[5], &ap->channel);

    ap->security = sec < 5 ? (nsapi_security_t)sec : NSAPI_SECURITY_UNKNOWN;

    return ret;
}

void ESP32::_connect_handler_0() { socket_handler(true, 0);  }
void ESP32::_connect_handler_1() { socket_handler(true, 1);  }
void ESP32::_connect_handler_2() { socket_handler(true, 2);  }
void ESP32::_connect_handler_3() { socket_handler(true, 3);  }
void ESP32::_connect_handler_4() { socket_handler(true, 4);  }
void ESP32::_closed_handler_0()  { socket_handler(false, 0); }
void ESP32::_closed_handler_1()  { socket_handler(false, 1); }
void ESP32::_closed_handler_2()  { socket_handler(false, 2); }
void ESP32::_closed_handler_3()  { socket_handler(false, 3); }
void ESP32::_closed_handler_4()  { socket_handler(false, 4); }

void ESP32::_connection_status_handler()
{
    char status[13];
    if (_parser.recv("%12[^\"]\n", status)) {
        if (strcmp(status, "CONNECTED\n") == 0) {
            _wifi_status = STATUS_CONNECTED;
        } else if (strcmp(status, "GOT IP\n") == 0) {
            _wifi_status = STATUS_GOT_IP;
        } else if (strcmp(status, "DISCONNECT\n") == 0) {
            _wifi_status = STATUS_DISCONNECTED;
        } else {
            return;
        }

        if(_wifi_status_cb) {
            _wifi_status_cb(_wifi_status);
        }
    }
}

int ESP32::get_free_id()
{
    // Look for an unused socket
    int id = -1;

    for (int i = 0; i < SOCKET_COUNT; i++) {
        if ((!_ids[i]) && ((_id_bits & (1 << i)) == 0)) {
            id = i;
            _ids[i] = true;
            break;
        }
    }

    return id;
}

void ESP32::event() {
    for (int i = 0; i < SOCKET_COUNT; i++) {
        if ((_cbs[i].callback) && (_cbs[i].Notified == 0)) {
            _cbs[i].callback(_cbs[i].data);
            _cbs[i].Notified = 1;
        }
    }
}

bool ESP32::set_network(const char *ip_address, const char *netmask, const char *gateway)
{
    bool ret;

    if (ip_address == NULL) {
        return false;
    }

    _smutex.lock();
    if ((netmask != NULL) && (gateway != NULL)) {
        ret = _parser.send("AT+CIPSTA=\"%s\",\"%s\",\"%s\"", ip_address, gateway, netmask)
           && _parser.recv("OK");
    } else {
        ret = _parser.send("AT+CIPSTA=\"%s\"", ip_address)
           && _parser.recv("OK");
    }
    _smutex.unlock();

    return ret;
}

bool ESP32::set_network_ap(const char *ip_address, const char *netmask, const char *gateway)
{
    bool ret;

    if (ip_address == NULL) {
        return false;
    }

    _smutex.lock();
    if ((netmask != NULL) && (gateway != NULL)) {
        ret = _parser.send("AT+CIPAP=\"%s\",\"%s\",\"%s\"", ip_address, gateway, netmask)
           && _parser.recv("OK");
    } else {
        ret = _parser.send("AT+CIPAP=\"%s\"", ip_address)
           && _parser.recv("OK");
    }
    _smutex.unlock();

    return ret;
}

void ESP32::attach_wifi_status(mbed::Callback<void(int8_t)> status_cb)
{
    _wifi_status_cb = status_cb;
}

int8_t ESP32::get_wifi_status() const
{
    return _wifi_status;
}
#endif

