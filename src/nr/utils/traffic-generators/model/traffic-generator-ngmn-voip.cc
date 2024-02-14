// Copyright (c) 2022 CTTC
// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "traffic-generator-ngmn-voip.h"

#include "ns3/address.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
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

#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TrafficGeneratorNgmnVoip");
NS_OBJECT_ENSURE_REGISTERED(TrafficGeneratorNgmnVoip);

TypeId
TrafficGeneratorNgmnVoip::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TrafficGeneratorNgmnVoip")
            .SetParent<TrafficGenerator>()
            .SetGroupName("Applications")
            .AddConstructor<TrafficGeneratorNgmnVoip>()
            .AddAttribute("EncoderFrameLength",
                          "The encoder frame length in milliseconds. It is used for "
                          "the calculation of transition probabilities based on configured "
                          "voice activity factor (VAF).",
                          UintegerValue(20),
                          //UintegerValue(20000),
                          MakeUintegerAccessor(&TrafficGeneratorNgmnVoip::m_encoderFrameLength),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MeanTalkSpurtDuration",
                          "Mean talk spurt duration in the "
                          "number of milliseconds.",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&TrafficGeneratorNgmnVoip::m_meanTalkSpurtDuration),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("VoiceActivityFactor",
                          "Voice activity factor, determines "
                          "the ratio of active versus inactive state. Expressed as the ratio.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&TrafficGeneratorNgmnVoip::m_voiceActivityFactor),
                          MakeDoubleChecker<double>(0.0, 0.99))
            .AddAttribute("VoicePayload",
                          "The voice packet payload in number of bytes.",
                          UintegerValue(40),
                          MakeUintegerAccessor(&TrafficGeneratorNgmnVoip::m_activePayload),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SIDPeriodicity",
                          "The periodicity of SIDs is 160 ms during silence",
                          UintegerValue(160),
                          MakeUintegerAccessor(&TrafficGeneratorNgmnVoip::m_SIDPeriodicity),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SIDPayload",
                          "The payload of SIDs.",
                          UintegerValue(15),
                          MakeUintegerAccessor(&TrafficGeneratorNgmnVoip::m_SIDPayload),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("Remote",
                          "The address of the destination",
                          AddressValue(),
                          MakeAddressAccessor(&TrafficGenerator::SetRemote),
                          MakeAddressChecker())
            .AddAttribute("Protocol",
                          "The type of protocol to use.",
                          TypeIdValue(TcpSocketFactory::GetTypeId()),
                          MakeTypeIdAccessor(&TrafficGenerator::SetProtocol),
                          MakeTypeIdChecker())
            .AddTraceSource("Tx",
                            "A new packet is created and is sent",
                            MakeTraceSourceAccessor(&TrafficGenerator::m_txTrace),
                            "ns3::TrafficGenerator::TxTracedCallback");
    return tid;
}

TrafficGeneratorNgmnVoip::TrafficGeneratorNgmnVoip()
    : TrafficGenerator()
{
    NS_LOG_FUNCTION(this);
}

TrafficGeneratorNgmnVoip::~TrafficGeneratorNgmnVoip()
{
    NS_LOG_FUNCTION(this);
}

void
TrafficGeneratorNgmnVoip::StartApplication()
{
    /*double currentTime = Simulator::Now().GetSeconds();
    std::cout << currentTime << std::endl;*/
    NS_LOG_FUNCTION(this);
    // calculate variables needed for the generation of the active and inactive
    // probabilities
    // The probability of transitioning from the active to the inactive state
    m_a = ((double)m_encoderFrameLength / (double)m_meanTalkSpurtDuration);
    // The probability of transitioning from the inactive to the active state
    m_c = (m_a * m_voiceActivityFactor) / (1 - m_voiceActivityFactor);

    UpdateState();
    SendPacketBurst();
}

void
TrafficGeneratorNgmnVoip::StopApplication()
{
    NS_LOG_FUNCTION(this);
    TrafficGenerator::StopApplication();
    NS_ASSERT(m_updateState.IsRunning());
    m_updateState.Cancel();
}

void
TrafficGeneratorNgmnVoip::UpdateState()
{
    NS_LOG_FUNCTION(this);
    // check what is the next state
    if (m_state == INACTIVE_STATE)
    {
        double randomValue = m_fromInactiveToActive->GetValue();
        // throw a coin and check if lower than the probability of transmition from inactive to
        // active
        //, switch to active state
        if (randomValue < m_c)
        {
            // switch to active state
            m_state = ACTIVE_STATE;
        }
    }
    else if (m_state == ACTIVE_STATE)
    {
        double randomValue = m_fromActiveToInactive->GetValue();
        // throw a coin and check if lower than the probability of transitions from active to
        // inactive
        if (randomValue < m_a)
        {
            // switch to inactive state
            m_state = INACTIVE_STATE;
        }
    }
    // The model is assumed updated at the speech encoder frame rate R=1/T, where T
    //  is the encoder frame duration (typically, 20ms)
    m_updateState = Simulator::Schedule(MilliSeconds(m_encoderFrameLength),
                                        &TrafficGeneratorNgmnVoip::UpdateState,
                                        this);
}

void
TrafficGeneratorNgmnVoip::GenerateNextPacketBurstSize()
{
    NS_LOG_FUNCTION(this);
    SetPacketBurstSizeInBytes(UINT32_MAX);
}

uint32_t
TrafficGeneratorNgmnVoip::GetNextPacketSize() const
{
    NS_LOG_FUNCTION(this);
    if (m_state == ACTIVE_STATE)
    {
        return m_activePayload;
    }
    else
    {
        return m_SIDPayload;
    }
}

Time
TrafficGeneratorNgmnVoip::GetNextPacketTime() const
{
    NS_LOG_FUNCTION(this);
    if (m_state == INACTIVE_STATE)
    {
        return MilliSeconds(m_SIDPeriodicity);
        //return Seconds(20);
    }
    else
    {
        return Seconds((double)(m_activePayload * 8) / 12200);
        //return Seconds(20);
    }
}

void
TrafficGeneratorNgmnVoip::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_fromActiveToInactive = 0;
    m_fromInactiveToActive = 0;
    // chain up
    TrafficGenerator::DoDispose();
}

void
TrafficGeneratorNgmnVoip::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    m_fromActiveToInactive = CreateObject<UniformRandomVariable>(); //N Uniform distribution for switching states
    m_fromActiveToInactive->SetAttribute("Min", DoubleValue(0));
    m_fromActiveToInactive->SetAttribute("Max", DoubleValue(1));
    m_fromInactiveToActive = CreateObject<UniformRandomVariable>();
    m_fromInactiveToActive->SetAttribute("Min", DoubleValue(0));
    m_fromInactiveToActive->SetAttribute("Max", DoubleValue(1));
    // chain up
    TrafficGenerator::DoInitialize();
}

} // Namespace ns3
