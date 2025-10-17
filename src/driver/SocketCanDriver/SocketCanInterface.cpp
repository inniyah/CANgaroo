/*

  Copyright (c) 2015, 2016 Hubert Denkmair

  This file is part of cangaroo.

  cangaroo is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  cangaroo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with cangaroo.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "SocketCanInterface.h"

#include <core/Backend.h>
#include <core/MeasurementInterface.h>
#include <core/CanMessage.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <QString>
#include <QStringList>
#include <QProcess>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/netlink.h>
#include <linux/sockios.h>
#include <netlink/version.h>
#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

SocketCanInterface::SocketCanInterface(SocketCanDriver *driver, int index, QString name)
  : CanInterface((CanDriver *)driver),
	_idx(index),
    _isOpen(false),
	_fd(0),
    _name(name),
    _ts_mode(ts_mode_SIOCSHWTSTAMP)
{
}

SocketCanInterface::~SocketCanInterface() {
}

QString SocketCanInterface::getName() const {
	return _name;
}

void SocketCanInterface::setName(QString name) {
    _name = name;
}

QList<CanTiming> SocketCanInterface::getAvailableBitrates()
{
    QList<CanTiming> retval;
    QList<unsigned> bitrates({10000, 20000, 50000, 83333, 100000, 125000, 250000, 500000, 800000, 1000000});
    QList<unsigned> samplePoints({500, 625, 750, 875});

    unsigned i=0;
    foreach (unsigned br, bitrates) {
        foreach (unsigned sp, samplePoints) {
            retval << CanTiming(i++, br, 0, sp);
        }
    }

    return retval;
}

QString SocketCanInterface::buildIpRouteCmd(const MeasurementInterface &mi)
{
    QStringList cmd;
    cmd.append("ip");
    cmd.append("link");
    cmd.append("set");
    cmd.append(getName());
    cmd.append("up");
    cmd.append("type");
    cmd.append("can");

    cmd.append("bitrate");
    cmd.append(QString().number(mi.bitrate()));
    cmd.append("sample-point");
    cmd.append(QString().number((float)mi.samplePoint()/1000.0, 'f', 3));

    if (mi.isCanFD()) {
        cmd.append("dbitrate");
        cmd.append(QString().number(mi.fdBitrate()));
        cmd.append("dsample-point");
        cmd.append(QString().number((float)mi.fdSamplePoint()/1000.0, 'f', 3));
        cmd.append("fd");
        cmd.append("on");
    }

    cmd.append("restart-ms");
    if (mi.doAutoRestart()) {
        cmd.append(QString().number(mi.autoRestartMs()));
    } else {
        cmd.append("0");
    }

    return cmd.join(' ');
}

QStringList SocketCanInterface::buildCanIfConfigArgs(const MeasurementInterface &mi)
{
    QStringList args;
    args << "-d";
    args << "-i" << getName();
    args << "-b" << QString::number(mi.bitrate());
    args << "-p" << QString::number(mi.samplePoint());
    args << "-u";
    return args;
}


void SocketCanInterface::applyConfig(const MeasurementInterface &mi)
{
    if (!mi.doConfigure()) {
        log_info(QString("interface %1 not managed by cangaroo, not touching configuration").arg(getName()));
        return;
    }

    log_info(QString("calling canifconfig to reconfigure interface %1").arg(getName()));
    QStringList sl = buildCanIfConfigArgs(mi);
    sl.prepend("canifconfig");
    log_info(sl.join(" "));

    QProcess canIfConfig;
    canIfConfig.start("canifconfig", buildCanIfConfigArgs(mi));
    if (!canIfConfig.waitForFinished()) {
        log_error(QString("timeout waiting for canifconfig"));
        return;
    }

    if (canIfConfig.exitStatus()!=QProcess::NormalExit) {
        log_error(QString("canifconfig crashed"));
        return;
    }

    if (canIfConfig.exitCode() != 0) {
        log_error(QString("canifconfig failed: ") + QString(canIfConfig.readAllStandardError()).trimmed());
        return;
    }

}

#if (LIBNL_CURRENT<=216)
#warning we need at least libnl3 version 3.2.22 to be able to get link status via netlink
int rtnl_link_can_state(struct rtnl_link *link, uint32_t *state) {
    (void) link;
    (void) state;
    return -1;
}
#endif

bool SocketCanInterface::updateStatus()
{
    bool retval = false;

    struct nl_sock *sock = nl_socket_alloc();
    struct nl_cache *cache;
    struct rtnl_link *link;
    uint32_t state;

    _status.can_state = state_unknown;

    nl_connect(sock, NETLINK_ROUTE);
    if (rtnl_link_alloc_cache(sock, AF_UNSPEC, &cache) >= 0) {
        if (rtnl_link_get_kernel(sock, _idx, 0, &link) == 0) {

            _status.rx_count = rtnl_link_get_stat(link, RTNL_LINK_RX_PACKETS);
            _status.rx_overruns = rtnl_link_get_stat(link, RTNL_LINK_RX_OVER_ERR);
            _status.tx_count = rtnl_link_get_stat(link, RTNL_LINK_TX_PACKETS);
            _status.tx_dropped = rtnl_link_get_stat(link, RTNL_LINK_TX_DROPPED);

            if (rtnl_link_is_can(link)) {
                if (rtnl_link_can_state(link, &state)==0) {
                    _status.can_state = state;
                }
                _status.rx_errors = rtnl_link_can_berr_rx(link);
                _status.tx_errors = rtnl_link_can_berr_tx(link);
            } else {
                _status.rx_errors = 0;
                _status.tx_errors = 0;
            }
            retval = true;
        }
    }

    nl_cache_free(cache);
    nl_close(sock);
    nl_socket_free(sock);

    return retval;
}

bool SocketCanInterface::readConfig()
{
    bool retval = false;

    struct nl_sock *sock = nl_socket_alloc();
    struct nl_cache *cache;
    struct rtnl_link *link;

    nl_connect(sock, NETLINK_ROUTE);
    int result = rtnl_link_alloc_cache(sock, AF_UNSPEC, &cache);

    if (result>=0) {
        if (rtnl_link_get_kernel(sock, _idx, 0, &link) == 0) {
            retval = readConfigFromLink(link);
        }
    }

    nl_cache_free(cache);
    nl_close(sock);
    nl_socket_free(sock);

    return retval;
}

bool SocketCanInterface::readConfigFromLink(rtnl_link *link)
{
    _config.state = state_unknown;
    _config.supports_canfd = (rtnl_link_get_mtu(link) >= CANFD_MTU);
    _config.supports_timing = rtnl_link_is_can(link);
    if (_config.supports_timing) {
        rtnl_link_can_freq(link, &_config.base_freq);
        rtnl_link_can_get_ctrlmode(link, &_config.ctrl_mode);
        rtnl_link_can_get_bittiming(link, &_config.bit_timing);
        rtnl_link_can_get_sample_point(link, &_config.sample_point);
        rtnl_link_can_get_restart_ms(link, &_config.restart_ms);
    } else {
        // maybe a vcan interface?
    }
    return true;
}

bool SocketCanInterface::supportsTimingConfiguration()
{
    return _config.supports_timing;
}

bool SocketCanInterface::supportsCanFD()
{
    return _config.supports_canfd;
}

bool SocketCanInterface::supportsTripleSampling()
{
    return false;
}

unsigned SocketCanInterface::getBitrate() {
    if (readConfig()) {
        return _config.bit_timing.bitrate;
    } else {
        return 0;
    }
}

uint32_t SocketCanInterface::getCapabilities()
{
    uint32_t retval =
        CanInterface::capability_config_os |
        CanInterface::capability_listen_only |
        CanInterface::capability_auto_restart;

    if (supportsCanFD()) {
        retval |= CanInterface::capability_canfd;
    }

    if (supportsTripleSampling()) {
        retval |= CanInterface::capability_triple_sampling;
    }

    return retval;
}

bool SocketCanInterface::updateStatistics()
{
    return updateStatus();
}

uint32_t SocketCanInterface::getState()
{
    switch (_status.can_state) {
        case CAN_STATE_ERROR_ACTIVE: return state_ok;
        case CAN_STATE_ERROR_WARNING: return state_warning;
        case CAN_STATE_ERROR_PASSIVE: return state_passive;
        case CAN_STATE_BUS_OFF: return state_bus_off;
        case CAN_STATE_STOPPED: return state_stopped;
        default: return state_unknown;
    }
}

int SocketCanInterface::getNumRxFrames()
{
    return _status.rx_count;
}

int SocketCanInterface::getNumRxErrors()
{
    return _status.rx_errors;
}

int SocketCanInterface::getNumTxFrames()
{
    return _status.tx_count;
}

int SocketCanInterface::getNumTxErrors()
{
    return _status.tx_errors;
}

int SocketCanInterface::getNumRxOverruns()
{
    return _status.rx_overruns;
}

int SocketCanInterface::getNumTxDropped()
{
    return _status.tx_dropped;
}

int SocketCanInterface::getIfIndex() {
    return _idx;
}

const char *SocketCanInterface::cname()
{
    return _name.toStdString().c_str();
}

void SocketCanInterface::open() {
    if((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        _isOpen = false;
        return;
    }

    // Enable CAN-FD frames on this socket
    int enable = 1;
    if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
        perror("Error enabling CAN FD on socket");
        // don't return; you can still work in classic CAN if the iface isn't FD
    }

    struct ifreq ifr;
    struct sockaddr_can addr;
    strncpy(ifr.ifr_name, _name.toStdString().c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX failed");
        _isOpen = false;
        return;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        _isOpen = false;
        return;
    }

    _isOpen = true;
}

bool SocketCanInterface::isOpen()
{
    return _isOpen;
}

void SocketCanInterface::close() {
	::close(_fd);
    _isOpen = false;
}

#include <linux/can.h>
#include <linux/can/raw.h>

// ...
void SocketCanInterface::sendMessage(const CanMessage &msg) {
    if (msg.isFD()) {
        struct canfd_frame fdf;
        memset(&fdf, 0, sizeof(fdf));

        fdf.can_id = msg.getId();
        if (msg.isExtended())    fdf.can_id |= CAN_EFF_FLAG;
        if (msg.isRTR())         fdf.can_id |= CAN_RTR_FLAG;   // (rare with FD)
        if (msg.isErrorFrame())  fdf.can_id |= CAN_ERR_FLAG;

        uint8_t len = msg.getLength();
        if (len > CANFD_MAX_DLEN) len = CANFD_MAX_DLEN;

        fdf.len = len;                   // note: 'len' for FD, not can_dlc
        if (msg.isBRS()) fdf.flags |= CANFD_BRS;  // bit-rate switching
        // (ESI if you track it): if (msg.isESI()) fdf.flags |= CANFD_ESI;

        for (int i = 0; i < len; ++i) fdf.data[i] = msg.getByte(i);

        ::write(_fd, &fdf, CANFD_MTU);
        return;
    }

    // classic CAN fallback
    struct can_frame cf;
    memset(&cf, 0, sizeof(cf));

    cf.can_id = msg.getId();
    if (msg.isExtended())    cf.can_id |= CAN_EFF_FLAG;
    if (msg.isRTR())         cf.can_id |= CAN_RTR_FLAG;
    if (msg.isErrorFrame())  cf.can_id |= CAN_ERR_FLAG;

    uint8_t len = msg.getLength();
    if (len > CAN_MAX_DLEN) len = CAN_MAX_DLEN;

    cf.can_dlc = len;
    for (int i = 0; i < len; ++i) cf.data[i] = msg.getByte(i);

    ::write(_fd, &cf, CAN_MTU);
}

bool SocketCanInterface::readMessage(QList<CanMessage> &msglist, unsigned int timeout_ms) {
    // Buffer large enough for an FD frame
    alignas(struct canfd_frame) uint8_t buf[CANFD_MTU];

    struct timespec ts_rcv;
    struct timeval  tv_rcv;
    struct timeval  timeout;
    fd_set          fdset;

    timeout.tv_sec  = timeout_ms / 1000;
    timeout.tv_usec = 1000 * (timeout_ms % 1000);

    FD_ZERO(&fdset);
    FD_SET(_fd, &fdset);

    int rv = ::select(_fd + 1, &fdset, nullptr, nullptr, &timeout);
    if (rv <= 0) {
        // timeout or error -> behave like original and just return false
        return false;
    }

    // Read up to CANFD_MTU; the size returned tells us the frame type
    ssize_t n = ::read(_fd, buf, sizeof(buf));
    if (n < 0) {
        return false;
    }

    CanMessage msg;

    // Timestamping mode (same logic as original, just before decoding)
    if (_ts_mode == ts_mode_SIOCSHWTSTAMP) {
        // TODO: implement hardware timestamp config; fall back for now
        _ts_mode = ts_mode_SIOCGSTAMPNS;
    }

    if (_ts_mode == ts_mode_SIOCGSTAMPNS) {
        if (::ioctl(_fd, SIOCGSTAMPNS, &ts_rcv) == 0) {
            msg.setTimestamp(ts_rcv.tv_sec, ts_rcv.tv_nsec / 1000);
        } else {
            _ts_mode = ts_mode_SIOCGSTAMP;
        }
    }

    if (_ts_mode == ts_mode_SIOCGSTAMP) {
        if (::ioctl(_fd, SIOCGSTAMP, &tv_rcv) == 0) {
            msg.setTimestamp(tv_rcv.tv_sec, tv_rcv.tv_usec);
        }
    }

    // Decode classic vs FD by the read size
    if (n == CAN_MTU) {
        const struct can_frame *cf = reinterpret_cast<const struct can_frame *>(buf);

        msg.setId(cf->can_id);
        msg.setExtended((cf->can_id & CAN_EFF_FLAG) != 0);
        msg.setRTR((cf->can_id & CAN_RTR_FLAG) != 0);
        msg.setErrorFrame((cf->can_id & CAN_ERR_FLAG) != 0);
        msg.setInterfaceId(getId());
        msg.setFD(false);
        msg.setBRS(false);

        uint8_t len = cf->can_dlc;
        if (len > CAN_MAX_DLEN) len = CAN_MAX_DLEN;

        msg.setLength(len);
        for (int i = 0; i < static_cast<int>(len); ++i) {
            msg.setByte(i, cf->data[i]);
        }
    } else if (n == CANFD_MTU) {
        const struct canfd_frame *fdf = reinterpret_cast<const struct canfd_frame *>(buf);

        msg.setId(fdf->can_id);
        msg.setExtended((fdf->can_id & CAN_EFF_FLAG) != 0);
        msg.setRTR((fdf->can_id & CAN_RTR_FLAG) != 0);       // rarely used with FD
        msg.setErrorFrame((fdf->can_id & CAN_ERR_FLAG) != 0);
        msg.setInterfaceId(getId());
        msg.setFD(true);
        msg.setBRS((fdf->flags & CANFD_BRS) != 0);

        uint8_t len = fdf->len;                               // note: 'len' is data length for FD
        if (len > CANFD_MAX_DLEN) len = CANFD_MAX_DLEN;

        msg.setLength(len);
        for (int i = 0; i < static_cast<int>(len); ++i) {
            msg.setByte(i, fdf->data[i]);
        }
    } else {
        // Unexpected size (e.g., not classic or FD). Ignore frame.
        return false;
    }

    msglist.append(msg);
    return true;
}
