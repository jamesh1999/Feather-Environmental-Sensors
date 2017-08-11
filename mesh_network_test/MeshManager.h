#ifndef __MESH_MANAGER_INCLUDED__
#define __MESH_MANAGER_INCLUDED__

#include <RHDatagram.h>

//Useful macros for talking to registers (should be in a Utils.h)
#define REGISTER32(x) (*reinterpret_cast<volatile uint32_t*>(x))
#define REGISTER16(x) (*reinterpret_cast<volatile uint16_t*>(x))
#define REGISTER8(x) (*reinterpret_cast<volatile uint8_t*>(x))

#define REGISTER32_T volatile uint32_t&
#define REGISTER16_T volatile uint16_t&
#define REGISTER8_T volatile uint8_t&

//Display packet sent/received messages etc
#define DEBUG

//Status LED modes:
//Continuous off - client
//Continuous on - server
//Slow flash - renewing DHCP
//Fast flash - route finding
//Quick on, long off - not connected
#define STATUS_LED

//Server will ignore packets coming from invalid leases
//Ensures only valid data is received
#define VERIFY_DHCP

//Maximum message size (bytes)/ size of the received ring buffer (messages)
const int MAX_MESSAGE = 50;
const int MAX_BUFFERED = 5;

//Default times to wait for a response / number of retries
const int DEFAULT_TIMEOUT = 1000;
const int DEFAULT_MAX_ATTEMPTS = 3;

//Delay added between some messages to allow for processing of the previous packets
const int MESSAGE_DELAYS = 66;

//Number of retries when performing a DHCP request
//Recommended to be above DEFAULT_MAX_ATTEMPTS in case server is stuck in an ACK/route finding loop
const int DHCP_MAX_ATTEMPTS = 5;
//Duration of a lease (ms)
const uint64_t DHCP_LEASE_DURATION = 60000;
//Time between server DHCP_DISCOVER messages (ms)
const uint64_t DHCP_PING_INTERVAL = 10000;

//Number of routes to store / maximum length of a route
const int ROUTE_COUNT = 15;
const int ROUTE_MAX_HOPS = 10;

//sendtoWait response codes
const int MESH_SUCCESS = 0;
const int MESH_ERROR_NO_ROUTE = 1;
const int MESH_NO_ACK = 2;


class MeshManager : private RHDatagram
{

/* 
 * Communication overview:
 *
 * ==================
 * Address allocation:
 * ==================
 * 0       - Server
 * [1:253] - Other nodes
 * 254     - Joining node
 * 255     - RH_BROADCAST_ADDRESS
 *
 * =================
 * Joining procedure:
 * =================
 * Use address 254 and broadcast DHCP_DISCOVER containing SID
 * Server responds w/ DHCP_OFFER
 * Client " DHCP_REQUEST
 * Server ACKs
 *
 * ===============
 * Message routing:
 * ===============
 * Default:
 * Sender stores dest, next hop pairs in a route table
 * Sender tries to send to next hop & next hop ACKs
 * Any failure along the chain will result in route finding starting from that node:
 * Node broadcasts ROUTE_DISCOVER with the destination address
 * All listeners add their address to the packet and rebroadcast
 * If the destination receives: reply back along chain with ROUTE_FOUND
 *
 * Flood:
 * Sender broadcasts packet
 * All receiving nodes add their address & rebroadcast
 * Nodes are not guaranteed to receive the packet only once
 * Format bytes:
 * 0:   Message length
 * 1-n: Message
 * ...: Addresses received
 *
 * ==================
 * Assigning a server:
 * ==================
 * Whenever a DHCP lease fails a node will become a server
 * Servers send DHCP_DISCOVER periodically
 * If a DHCP_DISCOVER from a server arrives at another server that server will become a client if:
 * The sender has been forced
 * The SID of the sender is greater than its own
 * Whenever a new node becomes a server or an old server stops, DHCP_RENEW FLOOD will be sent
 * This triggers all leases to expire
 * To force a node to be a server call becomeServer(true)
 * This sends a DHCP_CLEAR FLOOD telling all other nodes they are no longer forced
 */

private:

    //Node information
    uint8_t m_serial[16];
    bool m_server = false;
    bool m_force = false;
    uint64_t m_lastCheck = 0;

    //Packet structure
    typedef struct
    {
        enum Type { APPLICATION,
                    DHCP_DISCOVER, DHCP_OFFER, DHCP_REQUEST, DHCP_RENEW, DHCP_CLEAR,
                    ROUTE_DISCOVER, ROUTE_FOUND };

        Type type;
        uint8_t dest;
        uint8_t source;
    } MeshPacketHeader;

#ifdef DEBUG
    const char* m_packetNames[8] = 
    {
        "APPLICATION",
        "DHCP_DISCOVER",
        "DHCP_OFFER",
        "DHCP_REQUEST",
        "DHCP_RENEW",
        "DHCP_CLEAR",
        "ROUTE_DISCOVER",
        "ROUTE_FOUND"
    };
#endif

    typedef struct
    {
        MeshPacketHeader header;
        uint8_t len;
        uint8_t* data;

        uint8_t addr;
        uint8_t id;
        enum Flags { FLOOD = 0x01, ACK = 0x80};
        uint8_t flags;
    } MeshPacket;

    //IO packet buffer
    uint8_t m_tmpMsg[MAX_MESSAGE + sizeof(MeshPacketHeader)];
    MeshPacket m_tmpPacket;
    //Buffer for ACKed messages while receiving ACKs
    MeshPacket m_ackPacket;
    uint8_t m_ackMsg[MAX_MESSAGE];
    //Extra buffer in case we need to send routing packets before application
    uint8_t m_appMsg[MAX_MESSAGE];
    MeshPacket m_appPacket;

    //TODO Fix buffer mess:
    //Would be less error prone to pass a const MeshPacket& to sendMsg that you create at the time

    //Ring buffer to hold messages
    uint8_t m_bufferedMsg[MAX_MESSAGE][MAX_BUFFERED];
    MeshPacket m_bufferedPacket[MAX_BUFFERED];
    uint8_t m_buffReadPos = 0;
    uint8_t m_buffWritePos = 0;
    
    //Message IDs to determine if messages are duplicates
    uint8_t m_seenMessages[255];
    uint8_t m_messageID = 1;

    //Lease information
    uint8_t m_leaseAddr;
    uint32_t m_leaseEnd;

    typedef struct
    {
        uint8_t serialID[16];
        uint64_t end;
    } Lease;
    Lease m_leases[253];
    uint8_t m_leasePos = 0;

    //Route information
    typedef struct
    {
        uint8_t dest;
        uint8_t hop;
        bool valid = false;
    } Route;
    Route m_routeCache[ROUTE_COUNT];

    //Internal methods for reading messages
    //Both read from the radio to the ring buffer if available
    //Recv returns oldest from ring buffer and pops it off
    //Peek returns newest and the message stays
    bool recvMsg();
    bool peekMsg();

    //Send messages with/without an ACK being required
    //usePacketID - Whether m_tmpPacket.id should be used or m_messageID should be used and incremented
    bool sendMsg(bool usePacketID = false);
    bool sendMsgAck();

    //Ack the previous message (must still be in m_tmpPacket)
    void ackMsg();

    //DHCP helper methods
    bool acquireLease();
    void renewDHCP();

    //sendtoWait reading from m_tmpPacket
    //Packets must be ACKed
    int sendtoWaitPacket();

    //Helper lookup method for the route table
    int getRouteIdx(uint8_t);

    //Overriden for debugging messages
    void setThisAddress(uint8_t);
    

public:

    MeshManager(RHGenericDriver&, uint8_t*);

/*
 *  Sets up the radio for communication
 */
    bool init();

/*
 *  Becomes the DHCP server for a network
 *  force - Signals to existing servers that this node has priority
 */
    void becomeServer(bool force = false);

/*
 *  Returns whether this node is the current server
 */
    bool isServer();

/*
 *  Sends a message on the network
 *  Message will be ACKed at each hop in the route
 */
    int sendtoWait(uint8_t* msg, uint8_t len, uint8_t addr);

/*
 *  Receives a message (non-blocking) if available
 *  Handles non-application messages:
 *  Rerouting/flooding of packets
 *  Route finding
 *  DHCP renewal / requests
 */
    bool recvfromAck(uint8_t* buff, uint8_t* len, uint8_t* source = nullptr, uint8_t* dest = nullptr, uint8_t* id = nullptr, uint8_t* flags = nullptr);

/*
 *  Prints the serial number associated with a given address (to be called by a server node)
 */
    void printSerial(uint8_t addr);

};

#endif
