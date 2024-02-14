// Copyright (c) 2010 Georgia Institute of Technology
// Copyright (c) 2022 CTTC
// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "traffic-generator.h"

#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include "src/internet/model/udp-socket-impl.h"

#include "ns3/inet-socket-address.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TrafficGenerator");
NS_OBJECT_ENSURE_REGISTERED(TrafficGenerator);

std::ofstream TrafficGenerator::m_TGDropFile;
std::string TrafficGenerator::m_TGDropFileName;

//std::ofstream UdpL4Protocol::m_udpDropFile;
//std::string UdpL4Protocol::m_udpDropFileName;

uint16_t TrafficGenerator::m_tgIdCounter = 0;

//static const uint32_t MAX_IPV4_UDP_DATAGRAM_SIZE = 65507;
//N Teste mit Payload Size aus UDP Header
static const uint32_t MAX_IPV4_UDP_DATAGRAM_SIZE = 62820;

//static const uint32_t MAX_IPV4_UDP_DATAGRAM_SIZE = 32768;

TypeId
TrafficGenerator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TrafficGenerator").SetParent<Application>().SetGroupName("Applications");
    return tid;
}

TrafficGenerator::TrafficGenerator()
    : Application(),
      m_socket(0),
      m_connected(false),
      m_currentBurstTotBytes(0),
      m_currentBurstTotPackets(0),
      m_totBytes(0),
      m_totPackets(0),
      m_stopped(false)
{
    NS_LOG_FUNCTION(this);
    m_tgId = m_tgIdCounter++;
}

TrafficGenerator::~TrafficGenerator()
{
    NS_LOG_FUNCTION(this);
}

uint64_t
TrafficGenerator::GetTotalBytes() const
{
    NS_LOG_FUNCTION(this);
    return m_totBytes;
}

uint64_t
TrafficGenerator::GetTotalPackets() const
{
    NS_LOG_FUNCTION(this);
    return m_totPackets;
}

Ptr<Socket>
TrafficGenerator::GetSocket() const
{
    NS_LOG_FUNCTION(this);
    return m_socket;
}

void
TrafficGenerator::GenerateNextPacketBurstSize()
{
    NS_LOG_FUNCTION(this);
    m_packetBurstSizeInBytes = 0;
    m_packetBurstSizeInPackets = 0;
}

void
TrafficGenerator::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_socket = 0;
    // chain up
    Application::DoDispose();
}

void
TrafficGenerator::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    Application::DoInitialize();
}

bool
TrafficGenerator::SendPacketBurst()
{
    NS_LOG_FUNCTION(this);

    m_waitForNextPacketBurst = false;

    if (m_stopped)
    {
        NS_LOG_INFO("Ignore SendPacketBurst because the application is stopped.");
        return false;
    }

    m_currentBurstTotBytes = 0;
    m_currentBurstTotPackets = 0;

    if (m_socket)
    {
        NS_LOG_FUNCTION("Socket exists");
    }
    else
    {
        m_socket = Socket::CreateSocket(GetNode(), m_tid);
        if (Inet6SocketAddress::IsMatchingType(m_peer))
        {
            m_socket->Bind6();
        }
        else if (InetSocketAddress::IsMatchingType(m_peer))
        {
            m_socket->Bind();
        }
        else
        {
            NS_LOG_UNCOND("Could not bind the socket.");
        }
        int connectRes = m_socket->Connect(m_peer);
        if (DynamicCast<UdpSocketImpl>(m_socket))
        {
            Ptr<UdpSocketImpl> udpSocket = DynamicCast<UdpSocketImpl>(m_socket);
            InetSocketAddress transport = InetSocketAddress::ConvertFrom(m_peer);
            uint32_t daddr = transport.GetIpv4().Get();
            udpSocket->SetDAddr(daddr);
        }
        else
        {
            std::cout << "Socket not castable" << std::endl;
        }
        NS_ABORT_MSG_UNLESS(connectRes == 0,
                            "Error in connecting the socket to the peer address:" << m_peer);
        m_socket->ShutdownRecv();
        m_socket->SetConnectCallback(MakeCallback(&TrafficGenerator::ConnectionSucceeded, this),
                                     MakeCallback(&TrafficGenerator::ConnectionFailed, this));
        m_socket->SetSendCallback(MakeCallback(&TrafficGenerator::SendNextPacketIfConnected, this));
        m_socket->SetCloseCallbacks(MakeCallback(&TrafficGenerator::CloseSucceeded, this),
                                    MakeCallback(&TrafficGenerator::CloseFailed, this));
    }

    if (m_connected || m_tid == UdpSocketFactory::GetTypeId())
    {
        GenerateNextPacketBurstSize();

        if (m_packetBurstSizeInBytes != 0)
        {
            NS_LOG_LOGIC("Starting transfer of packet burst of size " << m_packetBurstSizeInBytes);
        }
        else
        {
            NS_LOG_LOGIC(
                "Starting transfer of packet burst of unknown size, that will contain at least: "
                << m_packetBurstSizeInPackets << " packets");
        }

        // If the event is running cancel it since we call directly the first packet of the packet
        // burst
        if (m_eventIdSendNextPacket.IsRunning())
        {
            m_eventIdSendNextPacket.Cancel();
            NS_LOG_WARN("Canceling next packet send");
        }
        SendNextPacket();
    }
    else
    {
        NS_LOG_UNCOND(this << " Not connected yet. Expected if you are using TCP socket because "
                              "TCP handshake needs to complete...");
    }

    return true;
}

// Application Methods
void
TrafficGenerator::StartApplication() // Called at time specified by Start
{
    NS_LOG_FUNCTION(this);
}

void
TrafficGenerator::StopApplication() // Called at time specified by Stop
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC("TrafficGenerator closing socket");

    if (!(m_connected || m_tid == UdpSocketFactory::GetTypeId()))
    {
        NS_LOG_WARN("Stopping the application that never started. Which could happen if the "
                    "protocol used is TCP and the connection never got established.");
    }

    // so that if any event is being scheduled to cancel it
    m_stopped = true;

    if (m_socket == 0)
    {
        NS_LOG_WARN("TrafficGenerator found null socket to close in StopApplication");
        return;
    }

    if (m_tid == UdpSocketFactory::GetTypeId())
    {
        m_socket = 0;
    }
    else
    {
        m_socket->Close();
        m_connected = false;
        m_socket = 0;
    }

    NS_LOG_INFO("Sent packets: " << m_totPackets << " and the total bytes: " << m_totBytes);
}

void
TrafficGenerator::SetPacketBurstSizeInBytes(uint32_t burstSize)
{
    m_packetBurstSizeInBytes = burstSize;
}

void
TrafficGenerator::SetPacketBurstSizeInPackets(uint32_t burstSize)
{
    m_packetBurstSizeInPackets = burstSize;
}

uint32_t
TrafficGenerator::GetPacketBurstSizeInBytes() const
{
    return m_packetBurstSizeInBytes;
}

uint32_t
TrafficGenerator::GetPacketBurstSizeInPackets() const
{
    return m_packetBurstSizeInPackets;
}

void TrafficGenerator::SendSplitPacket(uint32_t remainingSize, uint32_t iteration)
{
    if (m_stopped)
    {
        NS_LOG_WARN("Ignore SendNextPacket because the application is stopped.");
        return;
    }
    if (m_socket == 0)
    {
        NS_LOG_DEBUG("Socket closed. Ignoring the call for send next packet.");
        return;
    }

    int actual = 0;

    // maximum datagram size hit
    if (remainingSize > MAX_IPV4_UDP_DATAGRAM_SIZE)
    {
        if (m_socket->GetTxAvailable() >= MAX_IPV4_UDP_DATAGRAM_SIZE)
        {
            // split data in multiple packets
            Ptr<Packet> packet = Create<Packet>(MAX_IPV4_UDP_DATAGRAM_SIZE);
            if (DynamicCast<UdpSocketImpl>(m_socket))
            {
                Ptr<UdpSocketImpl> udpSocket = DynamicCast<UdpSocketImpl>(m_socket);
                packet->SetPort(udpSocket->GetPort());
                packet->SetDAddr(udpSocket->GetDAddr());
            }else{
                std::cout << "Socket not castable" << std::endl;
            }
            m_txTrace(packet);
            actual = m_socket->Send(packet);
            //std::cout << "To send: " << MAX_IPV4_UDP_DATAGRAM_SIZE << std::endl;
            //NS_ASSERT(actual == (int)(MAX_IPV4_UDP_DATAGRAM_SIZE));
            if(actual != (int)(MAX_IPV4_UDP_DATAGRAM_SIZE)){
                std::cout << "Not everything could be sent" << std::endl;
                Simulator::Schedule(MilliSeconds(1), &TrafficGenerator::SendSplitPacket, this, remainingSize - actual, iteration + 1);
            }else{
                Simulator::Schedule(MilliSeconds(1), &TrafficGenerator::SendSplitPacket, this, remainingSize - MAX_IPV4_UDP_DATAGRAM_SIZE, iteration + 1);
            }
            logTGDrop(actual);
        }else{
            // buffer still too small, try to reschedule
            Simulator::Schedule(MilliSeconds(1), &TrafficGenerator::SendSplitPacket, this, remainingSize, iteration + 1);
        }
    }else{
        // instant send
        if (m_socket->GetTxAvailable() > remainingSize)
        {
            Ptr<Packet> packet = Create<Packet>(remainingSize);
            if (DynamicCast<UdpSocketImpl>(m_socket))
            {
                Ptr<UdpSocketImpl> udpSocket = DynamicCast<UdpSocketImpl>(m_socket);
                packet->SetPort(udpSocket->GetPort());
                packet->SetDAddr(udpSocket->GetDAddr());
            }
            else
            {
                std::cout << "Socket not castable" << std::endl;
            }
            m_txTrace(packet);
            actual = m_socket->Send(packet);
            //std::cout << "To send: " << remainingSize << std::endl;
            NS_ASSERT(actual == (int)(remainingSize));
            logTGDrop(actual);
            //N Letztes Split Packet erfolgreich gesendet

            Time nextPacketTime = GetNextPacketTime() - MilliSeconds(iteration);
            NS_ASSERT(nextPacketTime.GetSeconds() >= 0);
            m_eventIdSendNextPacket = Simulator::Schedule(nextPacketTime, &TrafficGenerator::SendNextPacket, this);
        }
        else
        {
            // it may happen that the buffer is full
            NS_LOG_WARN("Unable to send packet; actual " << actual << " size " << remainingSize
                                                         << "; caching for later attempt");
            std::cout << "UDP tx buffer full. packet dropped" << std::endl;
        }
    }
}

void TrafficGenerator::logTGDrop(int size)
{
    if (!m_TGDropFile.is_open())
    {
        m_TGDropFileName = "TGDrop.txt";
        m_TGDropFile.open(m_TGDropFileName.c_str());
        m_TGDropFile << "Time"
                      << "\t"
                      << "size"
                      << std::endl;

        if (!m_TGDropFile.is_open())
        {
            NS_FATAL_ERROR("Could not open tracefile");
        }
    }

    m_TGDropFile << Simulator::Now().GetSeconds()
                 << "\t" << size << std::endl;
}

void
TrafficGenerator::SendNextPacket()
{
    if (m_stopped)
    {
        NS_LOG_WARN("Ignore SendNextPacket because the application is stopped.");
        return;
    }
    if (m_socket == 0)
    {
        NS_LOG_DEBUG("Socket closed. Ignoring the call for send next packet.");
        return;
    }

    if (m_packetBurstSizeInBytes == 0 && m_packetBurstSizeInPackets == 0)
    {
        GenerateNextPacketBurstSize();
    }

    NS_ASSERT(m_packetBurstSizeInBytes || m_packetBurstSizeInPackets);

    // Time to send more
    uint32_t toSend = GetNextPacketSize();
    // Make sure we don't send too many
    if ((m_packetBurstSizeInBytes > 0) and
        (m_packetBurstSizeInBytes - m_currentBurstTotBytes > toSend))
    {
        toSend = std::min(toSend, m_packetBurstSizeInBytes - m_currentBurstTotBytes);
        NS_LOG_INFO("Sending a packet at " << Simulator::Now() << " of size:" << toSend);
        //<< ", and left to send: " << (m_packetBurstSizeInBytes - m_currentBurstTotBytes));
    }
    NS_ASSERT(toSend);
    //std::cout << "To send: " << toSend << std::endl;
    int actual = 0;
    /*
    XrHeader xrHeader (GetTrafficType());
    if (m_socket->GetTxAvailable() > toSend + xrHeader.GetSerializedSize ())
    */
    if (m_socket->GetTxAvailable() > toSend)
    {
        Ptr<Packet> packet = Create<Packet>(toSend);
        if (DynamicCast<UdpSocketImpl>(m_socket))
        {
            Ptr<UdpSocketImpl> udpSocket = DynamicCast<UdpSocketImpl>(m_socket);
            packet->SetPort(udpSocket->GetPort());
            packet->SetDAddr(udpSocket->GetDAddr());
        }
        else
        {
            std::cout << "Socket not castable" << std::endl;
        }
        m_txTrace(packet);
        actual = m_socket->Send(packet);
        AddressValue remoteAddress(InetSocketAddress("7.0.0.7", 4004));
        if(m_peer==remoteAddress.Get()){
            //std::cout << "TG sent packet" << std::endl;
        }
        //std::cout << "To send: " << toSend << std::endl;
        // NS_ASSERT (actual == (int) (toSend + xrHeader.GetSerializedSize ()));
        NS_ASSERT(actual == (int)(toSend));
        logTGDrop(actual);
    }
    else
    {
        // it may happen that the buffer is full
        NS_LOG_WARN("Unable to send packet; actual " << actual << " size " << toSend
                                                     << "; caching for later attempt");

        // N Maximale Datagram Size ausgereizt
        if (toSend > MAX_IPV4_UDP_DATAGRAM_SIZE)
        {
            // N Splitte Daten in mehrere Packets
            Ptr<Packet> packet = Create<Packet>(MAX_IPV4_UDP_DATAGRAM_SIZE);
            if (DynamicCast<UdpSocketImpl>(m_socket))
            {
                Ptr<UdpSocketImpl> udpSocket = DynamicCast<UdpSocketImpl>(m_socket);
                packet->SetPort(udpSocket->GetPort());
                packet->SetDAddr(udpSocket->GetDAddr());
            }
            else
            {
                std::cout << "Socket not castable" << std::endl;
            }
            m_txTrace(packet);
            actual = m_socket->Send(packet);
            //std::cout << "To send: " << MAX_IPV4_UDP_DATAGRAM_SIZE << std::endl;
            //NS_ASSERT(actual == (int)(MAX_IPV4_UDP_DATAGRAM_SIZE));
            if(actual != (int)(MAX_IPV4_UDP_DATAGRAM_SIZE)){
                std::cout << "Not everything could be sent" << std::endl;
                SendSplitPacket(toSend - actual, 0);
            }else{
                SendSplitPacket(toSend - MAX_IPV4_UDP_DATAGRAM_SIZE, 0);
            }
            logTGDrop(actual);
            return;
            //splitBlock = true;
        }else{
            std::cout << "UDP tx buffer full. packet dropped" << std::endl;
        }
    }
    NS_LOG_INFO("Sent data: " << actual << " bytes.");

    if ((actual < (int)toSend))
    {
        // We exit this loop when actual < toSend as the send side
        // buffer is full. The "DataSent" callback will pop when
        // some buffer space has freed ip.
        NS_LOG_DEBUG("Send buffer is full.");
        return;
    }
    else
    {
        m_currentBurstTotBytes += actual;
        m_totBytes += actual;
        m_currentBurstTotPackets++;
        m_totPackets++;
        NS_LOG_INFO("Sending " << actual
                               << " bytes. "
                                  "Total bytes: "
                               << m_currentBurstTotBytes
                               << ", and packetBurstSize: " << m_packetBurstSizeInBytes);
    }

    if (m_currentBurstTotBytes < m_packetBurstSizeInBytes ||
        m_currentBurstTotPackets < m_packetBurstSizeInPackets || m_packetBurstSizeInPackets == 1)
    {
        Time nextPacketTime = GetNextPacketTime();
        NS_ASSERT(nextPacketTime.GetSeconds() >= 0);
        m_eventIdSendNextPacket =
            Simulator::Schedule(nextPacketTime, &TrafficGenerator::SendNextPacket, this);
    }
    else // we finished transmitting this packet burst
    {
        m_currentBurstTotBytes = 0;
        m_currentBurstTotPackets = 0;
        m_eventIdSendNextPacket.Cancel();
        m_waitForNextPacketBurst = true;
        PacketBurstSent();
    }
}

void
TrafficGenerator::PacketBurstSent()
{
    NS_LOG_FUNCTION(this);
}

Time
TrafficGenerator::GetNextPacketTime() const
{
    NS_LOG_FUNCTION(this);
    return MilliSeconds(0);
}

void
TrafficGenerator::ConnectionSucceeded(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_LOG_UNCOND(this << " TrafficGenerator Connection succeeded");
    m_connected = true;

    // Only if the event is not running scheule it
    if (!m_eventIdSendNextPacket.IsRunning() && !m_waitForNextPacketBurst)
    {
        Time nextPacketTime = GetNextPacketTime();
        m_eventIdSendNextPacket =
            Simulator::Schedule(nextPacketTime, &TrafficGenerator::SendNextPacket, this);
    }
}

void
TrafficGenerator::ConnectionFailed(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_LOG_LOGIC("TrafficGenerator Connection failed");
    m_socket->Close();
}

void
TrafficGenerator::CloseSucceeded(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_LOG_LOGIC("TrafficGenerator Close succeeded");
    m_socket = 0;
}

void
TrafficGenerator::CloseFailed(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    NS_LOG_LOGIC("TrafficGenerator Close failed");
    m_socket = 0;
}

void
TrafficGenerator::SendNextPacketIfConnected(Ptr<Socket>, uint32_t)
{
    NS_LOG_FUNCTION(this);
    if (m_socket)
    {
        if (m_connected || m_tid == UdpSocketFactory::GetTypeId())
        { // Only send new data if the connection has completed
            NS_LOG_LOGIC("TrafficGenerator SendNextPacketIfConnected callback triggers new "
                         "SendNextPacket call");
            // Only if the event is not running scheule it
            if (!m_eventIdSendNextPacket.IsRunning() && !m_waitForNextPacketBurst)
            {
                Time nextPacketTime = GetNextPacketTime();
                m_eventIdSendNextPacket =
                    Simulator::Schedule(nextPacketTime, &TrafficGenerator::SendNextPacket, this);
            }
        }
    }
}

void
TrafficGenerator::SetRemote(Address remote)
{
    m_peer = remote;
}

void
TrafficGenerator::SetProtocol(TypeId protocol)
{
    m_tid = protocol;
}

uint16_t
TrafficGenerator::GetTgId()
{
    return m_tgId;
}

Address
TrafficGenerator::GetPeer() const
{
    return m_peer;
}

} // Namespace ns3
