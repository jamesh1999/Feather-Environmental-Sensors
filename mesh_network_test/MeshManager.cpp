#include "MeshManager.h"
#include <RH_RF69.h>

#ifdef STATUS_LED
    REGISTER32_T PM_APBCMASK    = REGISTER32(0x40000400 + 0x20);

    REGISTER8_T  PORT_PMUXA8    = REGISTER8(0x41004400 + 0x38);
    REGISTER8_T  PORT_PINCFGA17 = REGISTER8(0x41004400 + 0x51);

    REGISTER16_T GCLK_CLKCTRL   = REGISTER16(0x40000C00 + 0x2);
    REGISTER32_T GCLK_GENDIV    = REGISTER32(0x40000C00 + 0x8);
    REGISTER32_T GCLK_GENCTRL   = REGISTER32(0x40000C00 + 0x4);

    REGISTER32_T TCC0_CTRLA     = REGISTER32(0x42002000 + 0x00);
    REGISTER32_T TCC0_SYNCBUSY  = REGISTER32(0x42002000 + 0x08);
    REGISTER32_T TCC0_WEXCTRL   = REGISTER32(0x42002000 + 0x14);
    REGISTER32_T TCC0_WAVE      = REGISTER32(0x42002000 + 0x3C);
    REGISTER32_T TCC0_PER       = REGISTER32(0x42002000 + 0x40);
    REGISTER32_T TCC0_CC0       = REGISTER32(0x42002000 + 0x44);
#endif

//New nodes will always join on addr = 254
MeshManager::MeshManager(RHGenericDriver& driver, uint8_t* serial) : RHDatagram(driver, 254)
{
    memcpy(m_serial, serial, 16);
    m_tmpPacket.data = m_tmpMsg + sizeof(MeshPacketHeader);

#ifdef STATUS_LED
    //Enable TCC0 clock gating region
    PM_APBCMASK |= 0x100;

    //Bind generic clock 3 to TCC0/1 Freq: XTAL/1
    GCLK_GENDIV  = 0x103;
    GCLK_GENCTRL = 0x10003;
    GCLK_CLKCTRL = 0x431A;

    //Enable peripheral F (TCC0/WO[7]) on pin 26 (PA17)
    PORT_PMUXA8    |= 0x5 << 4;
    PORT_PINCFGA17 |= 1;

    //Set up TCC0 for NPWM & OM to use compare/capture channel 0
    TCC0_CTRLA &= ~0x2;
    while(TCC0_SYNCBUSY);
    TCC0_WAVE |= 0x2;
    TCC0_WEXCTRL = 0x2;
    while(TCC0_SYNCBUSY);
    TCC0_CTRLA |= 0x2;

    TCC0_PER = 0x800000;
    TCC0_CC0 = 0x080000;
#endif
}

bool MeshManager::init()
{
    return RHDatagram::init();
}

void MeshManager::becomeServer(bool force)
{
    m_force = force;

    if(m_force)
    {
        //DHCP_CLEAR - Tell other nodes that this is forced server
        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_CLEAR;
        m_tmpPacket.header.source = thisAddress();
        m_tmpPacket.len = 17;
        m_tmpPacket.data[0] = 16;
        memcpy(m_tmpPacket.data + 1, m_serial, 16);
        m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
        m_tmpPacket.flags = MeshPacket::Flags::FLOOD;
        sendMsg();
        delay(MESSAGE_DELAYS);
    }

    m_server = true;
    setThisAddress(0);
    m_lastCheck = millis() - DHCP_PING_INTERVAL;

#ifdef DEBUG
    Serial.println("Became server");
#endif
#ifdef STATUS_LED
    TCC0_PER = 0xFFFFFF;
    TCC0_CC0 = 0xFFFFFF;
#endif

    //Clear lease table
    uint64_t now = millis();
    for(int i = 0; i < 253; ++i)
        m_leases[i].end = now;

    //DHCP_RENEW - Call for nodes to renew their leases
    m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_RENEW;
    m_tmpPacket.len = 1;
    m_tmpPacket.data[0] = 0;
    m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
    m_tmpPacket.flags = MeshPacket::Flags::FLOOD;
    sendMsg();
}

bool MeshManager::isServer()
{
    return m_server;
}

bool MeshManager::peekMsg()
{
    uint8_t length = MAX_MESSAGE + sizeof(MeshPacketHeader);
    if(!recvfrom(m_tmpMsg, &length, &m_tmpPacket.addr, NULL, &m_tmpPacket.id, &m_tmpPacket.flags)) return false;
    
    memcpy(&m_tmpPacket.header, m_tmpMsg, sizeof(MeshPacketHeader));
    m_tmpPacket.len = length - sizeof(MeshPacketHeader);

#ifdef DEBUG
    Serial.print("\033[0;34m[<=] Received: ");
    Serial.print(m_packetNames[static_cast<int>(m_tmpPacket.header.type)]);
    if(m_tmpPacket.flags & MeshPacket::Flags::ACK)
        Serial.print(" ACK");
    if(m_tmpPacket.flags & MeshPacket::Flags::FLOOD)
        Serial.print(" FLOOD");
    Serial.print(" ");
    Serial.print(m_tmpPacket.id);
    Serial.print(" from ");
    Serial.print(m_tmpPacket.addr);
    Serial.println("\033[0;0m");
#endif

    //Ring buffer is full
    if((m_buffWritePos + 1) % MAX_BUFFERED == m_buffReadPos) return true;

    memcpy(m_bufferedPacket + m_buffWritePos, &m_tmpPacket, sizeof(MeshPacket));
    memcpy(m_bufferedMsg[m_buffWritePos], m_tmpPacket.data, MAX_MESSAGE);

    ++m_buffWritePos;
    if(m_buffWritePos == MAX_BUFFERED) m_buffWritePos = 0;

    return true;
}

bool MeshManager::recvMsg()
{
    //Read a new message if there is space in the ring buffer
    if((m_buffWritePos + 1) % MAX_BUFFERED != m_buffReadPos) peekMsg();

    //No messages in ring buffer
    if(m_buffReadPos == m_buffWritePos) return false;

    //memcpy(&m_tmpPacket, m_bufferedPacket + m_buffReadPos, sizeof(MeshPacket));
    //memcpy(m_tmpPacket.data, m_bufferedMsg[m_buffReadPos], MAX_MESSAGE);
    
    ++m_buffReadPos;
    if(m_buffReadPos == MAX_BUFFERED) m_buffReadPos = 0;

#ifdef DEBUG
    Serial.print("Read buffered: ");
    Serial.print(m_packetNames[static_cast<int>(m_tmpPacket.header.type)]);
    if(m_tmpPacket.flags & MeshPacket::Flags::ACK)
        Serial.print(" ACK");
    if(m_tmpPacket.flags & MeshPacket::Flags::FLOOD)
        Serial.print(" FLOOD");
    Serial.print(" ");
    Serial.print(m_tmpPacket.id);
    Serial.print(" from ");
    Serial.println(m_tmpPacket.addr);
#endif

    return true;
}

bool MeshManager::sendMsg(bool usePacketID)
{
    setHeaderId(usePacketID 
                ? m_tmpPacket.id
                : m_messageID++);
    setHeaderFlags(m_tmpPacket.flags, 0xff);
    memcpy(m_tmpMsg, &m_tmpPacket.header, sizeof(MeshPacketHeader));

#ifdef DEBUG
    Serial.print("\033[0;32m[=>] Sent: ");
    Serial.print(m_packetNames[static_cast<int>(m_tmpPacket.header.type)]);
    if(m_tmpPacket.flags & MeshPacket::Flags::ACK)
        Serial.print(" ACK");
    if(m_tmpPacket.flags & MeshPacket::Flags::FLOOD)
        Serial.print(" FLOOD");
    Serial.print(" ");
    Serial.print(m_messageID - 1);
    Serial.print(" to ");
    Serial.print(m_tmpPacket.addr);
    Serial.println("\033[0;0m");
#endif

    return sendto(m_tmpMsg, m_tmpPacket.len + sizeof(MeshPacketHeader), m_tmpPacket.addr) && _driver.waitPacketSent();
}

bool MeshManager::sendMsgAck()
{
    int attempts = 0;
    m_tmpPacket.id = m_messageID++;

    //Buffer message
    m_ackPacket = m_tmpPacket;
    memcpy(m_ackMsg, m_tmpPacket.data, m_tmpPacket.len);

    while(attempts < DEFAULT_MAX_ATTEMPTS) 
    {
        //Restore buffer
        m_tmpPacket = m_ackPacket;
        memcpy(m_tmpPacket.data, m_ackMsg, m_tmpPacket.len);

        if(!sendMsg(true)) continue;
        uint64_t start = millis();
        while(millis() - start < DEFAULT_TIMEOUT)
        {
            waitAvailableTimeout(DEFAULT_TIMEOUT - (millis() - start));

            //Do we have an ACK corresponding to this message
            if(peekMsg()
                && m_tmpPacket.flags & MeshPacket::Flags::ACK
                && m_tmpPacket.id == m_messageID - 1)
                return true;
        }
        ++attempts;

#ifdef DEBUG
        Serial.print("\033[31;0mNo ACK on attempt ");
        Serial.print(attempts);
        Serial.println("\033[0;0m");
#endif
    }

    return false;
}

void MeshManager::ackMsg()
{
    m_tmpPacket.flags |= MeshPacket::Flags::ACK;
    sendMsg(true);
}

bool MeshManager::acquireLease()
{
    setThisAddress(254); //Free address
    m_messageID = 1; //Reset message ID

#ifdef STATUS_LED
    TCC0_PER = 0x400000;
    TCC0_CC0 = 0x200000;
#endif

    int attempts = 0;
    bool done = false;
    uint8_t addr = 0;
    uint64_t end = 0;
    while(attempts < DHCP_MAX_ATTEMPTS && !done)
    {
        ++attempts; //Counter for lease attempts w/ no response

#ifdef DEBUG
        Serial.print("DHCP attempt ");
        Serial.println(attempts);
#endif

        //DHCP_DISCOVER - Ping server for address
        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_DISCOVER;
        m_tmpPacket.header.source = thisAddress();
        m_tmpPacket.header.dest = 0;
        memcpy(m_tmpPacket.data, m_serial, 16);
        m_tmpPacket.len = 16;
        m_tmpPacket.addr = 0;
        m_tmpPacket.flags = 0;
        sendMsg();

        //DHCP_OFFER - Server replies with an offered address
        done = false;
        uint64_t start = millis();
        while(millis() - start < DEFAULT_TIMEOUT && !done)
        {
            waitAvailableTimeout(DEFAULT_TIMEOUT - (millis() - start));
            done = recvMsg()
                && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_OFFER
                && !memcmp(m_serial, m_tmpPacket.data, 16);
        }

        if(done) 
            attempts = 0; //Reset counter because communication succeeded
        else
            continue;

        //DHCP_REQUEST - Client accepts first offered lease
        addr = m_tmpPacket.data[16];
        memcpy(&end, m_tmpPacket.data + 17, 8);
        end += millis();

        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_REQUEST;
        m_tmpPacket.header.source = thisAddress();
        m_tmpPacket.len = 16;
        sendMsg();
        
        //DHCP_ACK - Wait for server ACK to complete process
        done = false;
        start = millis();
        while(millis() - start < DEFAULT_TIMEOUT && !done)
        {
            waitAvailableTimeout(DEFAULT_TIMEOUT - (millis() - start));
            done = recvMsg()
                && m_tmpPacket.flags & MeshPacket::Flags::ACK
                && !memcmp(m_serial, m_tmpPacket.data, 16);
        }
    }

#ifdef STATUS_LED
    TCC0_PER = 0x1;
    TCC0_CC0 = 0x0;
#endif

    if(!done) return false;

#ifdef DEBUG
    Serial.println("Successful lease");
#endif

    //Assume new lease
    setThisAddress(addr);
    m_leaseAddr = addr;
    m_leaseEnd = end;

    return true;
}

//Acquire a lease or become new DHCP server
void MeshManager::renewDHCP()
{
#ifdef DEBUG
    Serial.println("\033[0;31mDHCP lease expired: renewing\033[0;0m");
#endif

    if(!acquireLease())
        becomeServer();
}

int MeshManager::sendtoWaitPacket()
{
    //DHCP lease has expired
    if(m_leaseEnd < millis() && !m_server)
        renewDHCP();

    int idx = getRouteIdx(m_tmpPacket.header.dest);
    if(idx != -1) m_tmpPacket.addr = m_routeCache[idx].hop;

    //No route: heal mesh
    if(idx != -1 && sendMsgAck()) return MESH_SUCCESS;

#ifdef DEBUG
    Serial.print("\033[0;31mNo route to ");
    Serial.println(m_tmpPacket.header.dest);
    Serial.println("\033[0;0mSearching for new route");
#endif
#ifdef STATUS_LED
    TCC0_PER = 0x100000;
    TCC0_CC0 = 0x80000;
#endif
    
    //Store app message
    m_appPacket = m_tmpPacket;
    memcpy(m_appMsg, m_tmpPacket.data, m_tmpPacket.len);

    //Broadcast ROUTE_DISCOVER packet
    m_tmpPacket.header.type = MeshPacketHeader::Type::ROUTE_DISCOVER;
    m_tmpPacket.header.source = thisAddress();

    m_tmpPacket.len = 1;
    m_tmpPacket.data[0] = thisAddress();
    m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
    m_tmpPacket.flags = 0;
    sendMsg();

    //Wait for reply containing route
    uint64_t start = millis();
    bool done = false;
    while(millis() - start < DEFAULT_TIMEOUT && !done)
    {
        waitAvailableTimeout(DEFAULT_TIMEOUT - (millis() - start));
        done = peekMsg()
            && m_tmpPacket.header.type == MeshPacketHeader::Type::ROUTE_FOUND
            && m_tmpPacket.header.source == m_appPacket.header.dest;
    }

#ifdef STATUS_LED
    if(m_server)
    {
        TCC0_PER = 0xFFFFFF;
        TCC0_CC0 = 0xFFFFFF;
    }
    else
    {
        TCC0_PER = 0x1;
        TCC0_CC0 = 0x0;
    }
#endif

    //No route could be found
    if(!done) return MESH_ERROR_NO_ROUTE;

    //Ack route packet & add route
    ackMsg();

    idx = getRouteIdx(m_appPacket.header.dest);

    if(idx == -1)
    {
        //Make space for new route
        memcpy(m_routeCache + 1, m_routeCache, sizeof(Route) * (ROUTE_COUNT - 1));
        idx = 0;
    }

    m_routeCache[idx].dest = m_appPacket.header.dest;
    m_routeCache[idx].hop = m_tmpPacket.data[m_tmpPacket.len - 1];
    m_routeCache[idx].valid = true;

#ifdef DEBUG
    Serial.print("Route found: ");
    nclude <cfloat>

//Radio libraries
//#include "RF69Driver.h"
//#include "MeshManager.h"
//
////Sensor libraries
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>
//#include <Adafruit_MAX31856.h>
//
////#define DEBUGSerial.print(m_routeCache[idx].dest);
    Serial.print(" --> ");
    Serial.println(m_routeCache[idx].hop);
#endif

    //Restore application message
    m_tmpPacket = m_appPacket;
    memcpy(m_tmpPacket.data, m_appMsg, m_appPacket.len);

    if(sendMsgAck()) return MESH_SUCCESS;

    return MESH_NO_ACK;
}

int MeshManager::sendtoWait(uint8_t* msg, uint8_t len, uint8_t addr)
{
    //Header for APPLICATION packet
    m_tmpPacket.header.type = MeshPacketHeader::Type::APPLICATION;
    m_tmpPacket.header.dest = addr;
    m_tmpPacket.header.source = thisAddress();

    //Payload (must clear everything)
    m_tmpPacket.len = len;
    memcpy(m_tmpPacket.data, msg, len);
    m_tmpPacket.addr = addr;
    m_tmpPacket.flags = 0;

    return sendtoWaitPacket();
}

int MeshManager::getRouteIdx(uint8_t dest)
{
    for(int i = 0; i < ROUTE_COUNT; ++i)
        if(m_routeCache[i].dest == dest && m_routeCache[i].valid)
            return i;
    return -1;
}

void MeshManager::setThisAddress(uint8_t addr)
{
#ifdef DEBUG
    Serial.print("Address: ");
    Serial.println(addr);
#endif

    RHDatagram::setThisAddress(addr);
}

bool MeshManager::recvfromAck(uint8_t* msg, uint8_t* len, uint8_t* source, uint8_t* dest, uint8_t* id, uint8_t* flags)
{
    //DHCP lease has expired
    if(m_leaseEnd < millis() && !m_server)
        renewDHCP();

    //DHCP check for other servers
    if(millis() - m_lastCheck > DHCP_PING_INTERVAL && m_server)
    {
        //DHCP_DISCOVER - Signal sent by servers to resolve conflicts
        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_DISCOVER;
        m_tmpPacket.header.source = thisAddress();
        m_tmpPacket.len = 18;
        m_tmpPacket.data[0] = 17;
        memcpy(m_tmpPacket.data + 1, m_serial, 16);
        m_tmpPacket.data[17] = m_force ? 1 : 0;
        m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
        m_tmpPacket.flags = MeshPacket::Flags::FLOOD;
        sendMsg();

        m_lastCheck = millis();
    }
    
    //No available messages
    if(!recvMsg()) return false;

    //Stray ACKs
    if(m_tmpPacket.flags & MeshPacket::Flags::ACK)
        return false;
    //Prevent duplicate ACKed messages
    if(m_tmpPacket.header.type == MeshPacketHeader::Type::ROUTE_FOUND
        || m_tmpPacket.header.type == MeshPacketHeader::Type::APPLICATION)
    {
        if(static_cast<uint8_t>(m_seenMessages[m_tmpPacket.addr] - headerId()) < 127)
        {
#ifdef DEBUG
            Serial.println("\033[0;31mRejected duplicate message\033[0;0m");
#endif

            ackMsg();
            return false;
        }

        m_seenMessages[m_tmpPacket.addr] = headerId();
        ackMsg();
    }

    //Routing messages
    //Add route & relay packet
    if(m_tmpPacket.header.type == MeshPacketHeader::Type::ROUTE_FOUND)
    {
        int idx = getRouteIdx(m_tmpPacket.header.source);
        
        if(idx == -1)
        {
            memcpy(m_routeCache + 1, m_routeCache, sizeof(Route) * (ROUTE_COUNT - 1));
            idx = 0;
        }

        m_routeCache[idx].dest = m_tmpPacket.header.source;
        m_routeCache[idx].hop = m_tmpPacket.data[m_tmpPacket.len - 1];
        m_routeCache[idx].valid = true;

        --m_tmpPacket.len;
        m_tmpPacket.addr = m_tmpPacket.data[m_tmpPacket.len - 2];
        sendMsgAck();

        return false;
    }
    if(m_tmpPacket.header.type == MeshPacketHeader::Type::ROUTE_DISCOVER)
    {
        //ROUTE_FOUND if this is the destination node
        if(m_tmpPacket.header.dest == thisAddress())
        {
            m_tmpPacket.header.type = MeshPacketHeader::Type::ROUTE_FOUND;
            m_tmpPacket.header.dest = m_tmpPacket.header.source;
            m_tmpPacket.header.source = thisAddress();

            ++m_tmpPacket.len;
            m_tmpPacket.data[m_tmpPacket.len - 1] = thisAddress();
            m_tmpPacket.addr = m_tmpPacket.data[m_tmpPacket.len - 2];
            m_tmpPacket.flags = 0;
            sendMsgAck();
        }
        //Add self to nodes visited then rebroadcast
        else if(m_tmpPacket.len < ROUTE_MAX_HOPS)
        {
            //Ignore if already visited
            for(int i = 0; i < m_tmpPacket.len; ++i)
                if(m_tmpPacket.data[i] == thisAddress())
                    return false;

            ++m_tmpPacket.len;
            m_tmpPacket.data[m_tmpPacket.len - 1] = thisAddress();
            m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
            sendMsg();
        }
        return false;
    }

    //Messages to reroute
    //Network flood
    if(m_tmpPacket.flags & MeshPacket::Flags::FLOOD)
    {
        uint8_t addressLen = m_tmpPacket.len - m_tmpPacket.data[0] - 1;
        if(addressLen >= ROUTE_MAX_HOPS) return false;

        //Ignore if already visited
        for(int i = m_tmpPacket.data[0] + 1; i < m_tmpPacket.len; ++i)
            if(m_tmpPacket.data[i] == thisAddress())
                return false;

        ++m_tmpPacket.len;
        m_tmpPacket.data[m_tmpPacket.len - 1] = thisAddress();
        m_tmpPacket.addr = RH_BROADCAST_ADDRESS;

        sendMsg();
    }
    //Point to point
    else if(m_tmpPacket.header.dest != thisAddress())
    {
        sendtoWaitPacket();
        return false;
    }

    //DHCP messages
    //Discover from server: Compare to see who is server
    if(m_server && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_DISCOVER && m_tmpPacket.header.source == 0)
    {
        //Packet originated from this node
        if(memcmp(m_tmpPacket.data + 1, m_serial, 16) == 0) return false;

#ifdef DEBUG 
        Serial.print("Discover from ");
        char buff[3];
        for(int i = 0; i < 16; ++i)
        {
            sprintf(buff, "%02X", m_tmpPacket.data[i + 1]);
            Serial.print(buff);
        }
        if(m_tmpPacket.data[17]) Serial.print(" FORCED");
        Serial.println("");
#endif

        //Schedule check immediately
        //Slight delay necessary otherwise message will be missed
        m_lastCheck = millis() - DHCP_PING_INTERVAL + MESSAGE_DELAYS;

        if(m_tmpPacket.data[17] == 0)
        {
            if(m_force)
            {
#ifdef DEBUG
                Serial.println("Remaining server: forced");
#endif
                return false;
            }
            if(memcmp(m_tmpPacket.data + 1, m_serial, 16) < 0)
            {
#ifdef DEBUG
                Serial.println("Remaining server: Greater SID");
#endif
                return false;
            }
        }
    
        //DHCP_RENEW - Tell all nodes to renew their leases
        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_RENEW;
        m_tmpPacket.len = 1;
        m_tmpPacket.data[0] = 0;
        m_tmpPacket.addr = RH_BROADCAST_ADDRESS;
        m_tmpPacket.flags = MeshPacket::Flags::FLOOD;
        sendMsg();

#ifdef DEBUG
        Serial.println("No longer server");
#endif
#ifdef STATUS_LED
        TCC0_PER = 0x1;
        TCC0_CC0 = 0x0;
#endif

        m_force = false;
        m_server = false;
        setThisAddress(254);
        m_leaseEnd = millis();
        return false;
    }
    //Discover: Reply with offer
    if(m_server && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_DISCOVER)
    {//TODO: Make DHCP messages routable could use FLOOD or too much traffic?
        uint64_t now = millis();
        if(m_leases[m_leasePos].end - now > (1 << 32))
        {
                //Reserve address for this node
                memcpy(m_leases[m_leasePos].serialID, m_tmpPacket.data, 16);

                //Submit DHCP_OFFER
                m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_OFFER;

                m_tmpPacket.len = 25;
                m_tmpPacket.data[16] = m_leasePos + 1;
                memcpy(m_tmpPacket.data + 17, &DHCP_LEASE_DURATION, 8);
                m_tmpPacket.addr = 254;
                m_tmpPacket.flags = 0;

#ifdef DEBUG
                Serial.print("Leasing ");
                Serial.println(m_leasePos + 1, DEC);
#endif

                sendMsg();

                ++m_leasePos;
                if(m_leasePos == 253) m_leasePos = 0;
        }
        return false;
    }
    //Request: ACK and record lease
    if(m_server && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_REQUEST)
    {
        uint8_t addr = m_tmpPacket.data[16];
        if(!memcmp(m_leases[addr - 1].serialID, m_tmpPacket.data, 16))
        {
            //Log lease finish time
            m_leases[addr - 1].end = millis() + DHCP_LEASE_DURATION;
            
            //Reset message ID
            m_seenMessages[addr] = 0;

            //Submit ACK
            m_tmpPacket.flags |= MeshPacket::Flags::ACK;
            sendMsg();
        }
        return false;
    }
    //Renew: Client - Renew lease
    if(!m_server && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_RENEW)
    {
        m_leaseEnd = millis();
        return false;
    }
    if(m_server && m_tmpPacket.header.type == MeshPacketHeader::Type::DHCP_CLEAR)
    {
#ifdef DEBUG
        Serial.print("Cleared by ");
        char buff[3];
        for(int i = 0; i < 16; ++i)
        {
            sprintf(buff, "%02X", m_tmpPacket.data[i + 1]);
            Serial.print(buff);
        }
        Serial.println("");
#endif
        
        if(!memcmp(m_tmpPacket.data + 1, m_serial, 16)) return false;
        m_force = false;
        return false;
    }

#ifdef VERIFY_DHCP
    //May as well verify the lease is valid if we are the server
    //Less useful for other applications but makes sure server only
    //passes correct information here
    if(m_tmpPacket.header.source
        && m_tmpPacket.header.source < 254
        && m_server
        && m_leases[m_tmpPacket.header.source - 1].end < millis())
    {
        m_tmpPacket.header.dest == m_tmpPacket.header.source;
        m_tmpPacket.header.type = MeshPacketHeader::Type::DHCP_RENEW;
        m_tmpPacket.header.source = thisAddress();
        m_tmpPacket.len = 0;
        m_tmpPacket.flags = 0;
        sendtoWaitPacket();
        
        return false;
    }
#endif

    //Application messages
    if(m_tmpPacket.header.type == MeshPacketHeader::Type::APPLICATION)
    {
        memcpy(msg, m_tmpPacket.data, m_tmpPacket.len);
        *len = m_tmpPacket.len;
        if(source) *source = m_tmpPacket.header.source;
        if(dest) *dest = m_tmpPacket.header.dest;
        if(id) *id = headerId();
        if(flags) *flags = m_tmpPacket.flags;

        return true;
    }

    return false;
}

void MeshManager::printSerial(uint8_t addr)
{
    char buff[3];
    for(int i = 0; i < 16; ++i)
    {
        sprintf(buff, "%02X", m_leases[addr - 1].serialID[i]);
        Serial.print(buff);
    }
    Serial.println("");
}
