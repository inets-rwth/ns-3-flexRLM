#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/nr-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/log.h"
#include <ns3/buildings-module.h>
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/lte-helper.h"
#include "ns3/nr-module.h"
#include "ns3/three-gpp-http-helper.h"

#include <string>
#include <vector>
#include <dirent.h>
#include <fstream>

#include <sys/stat.h>

/*
**********************************************************************************************************
*   The following ns-3 code is based on the proposed work in the paper:
*   [1] A. Ichkov, A. Schott, P. Mähönen and L. Simić, 'flexRLM: Flexible Radio Link Monitoring
*       for Multi-User Downlink Millimeter-Wave Networks', to be presented in Proc. IEEE INFOCOM 2023.
*
*   This code enables multi-user downlink millimeter-wave end-to-end network simulations utilizing
*   5G-NR beam management downlink operations. It is based on the flexRLM framework, 
*   a coordinator-based flexible radio link monitoring (RLM) framework that enables joint 
*   beam management and low complexity load-balancing, as proposed in [1].
*
*   This code also enables single-user network simulations based on the proposed work in the paper:
*   [2] A. Ichkov, O. Atasoy, P. Mähönen and L. Simić, "Full-Stack ns-3 Framework for the 
*       Evaluation of 5G-NR Beam Management in Non-Standalone Downlink Millimeter-Wave Networks," 
*       in Proc. IEEE WoWMoM 2022, Belfast UK, June 2023. (https://ieeexplore.ieee.org/document/9842765)
*
**********************************************************************************************************
*   The following parameters are the most important for setting up the network simulations:
*
*     - simTime:      Duration of simulation in seconds
*     - numOfUes:     Due to the use of a ray-tracing channel model, for each UE there is
*                     #walkID and #walk-path information, which is stored in 
*                     /src/nr/model/Raytracing_UE_set. Additional UE sets can be found in
*                     /src/nr/model/UE_sets.
*                     The walk information, i.e. (X,Y) coordinates and walking speed, are 
*                     loaded during simulation initialization. We note that the number of UEs 
*                     further impacts the currently static assignment and scheduling of 
*                     Channel State Information-Reference Signals (CSI-RS), as detailed below.
*     - loadBalancing:  enables low complexity load blancing
*
*   A variety of parameters are introduced to configure the beam management operation, 
*   represented by three beam management strategies, as shown in Table I. 
*
*     - Ideal:          Ideal (instantaneous) beamforming/beam scanning and initial access, i.e. 
*                       no exchange on any beam management control messages. Exhaustive beam search 
*                       of candidate beam pair links (BPLs) during initial access is instantaneous 
*                       (completeSSBDuration=0.0) or set to a fixed value (completeSSBDuration=20.0).
*     - Default 5G-NR:  Time-frequency resource scheduling for transmission and reception
*                       of Synchronization Signal Blocks (SSBs) for the purpose of realistic beam
*                       sweep updates for initial access and Radio Link Failure (RLF) recovery.
*                       In addition to SSBs, CSI-RS control signals are scheduled for monitoring
*                       of alternative BPLs from the serving gNB to enable faster beam switching
*                       via Radio Link Monitoring (RLM). The monitored CSI-RSs are sent periodically 
*                       by the serving gNB over different BPLs, which are determined after a 
*                       successful SSB beam sweep and thus can become stale over time.
*                       Using the variable ssbRlmOn, we implement an additional update of the monitored 
*                       CSI-RSs based on the periodic SSB transmissions sent by the serving gNB.  
*                       Each connected UE performs beam sweeps during the periodic SSB transmissions to  
*                       keep track of long-term channel dynamics. The SSB measurement reports are then 
*                       used to update the monitored CSI-RSs for RLM. The CSI-RSs, which are transmitted 
*                       more frequently than the SSBs are thus used to capture short-term channel dynamics.
*     - flexRLM:        Instead of solely monitoring candidate BPLs from the serving gNB, flexRLM
*                       allows flexible configuration of the minimum number of BPLs to be monitored
*                       via CSI-RS control signals from the serving gNB (via minCSIRSFromServiceGnb).
*                       The remaining CSI-RS resources can be utilized to monitor candidate BPLs from 
*                       other candidate gNB to improve link stability and facilitate handover decisions.
*                       Additionally, the CSI-RS BPL information from alternative gNBs can be leveraged
*                       for load-balancing purpsoses via the central coordinator (via loadBalancing).
*
*
*                             Table I. Beam management strategies and configuration parameters
*                              _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
*                             |                  Beam management strategy                     |
*     Associated variables    |   Ideal   |    Default 5G-NR      |         flexRLM           |
*     ----------------------------------------------------------------------------------------|
*     realisticIA             |   false   |         true          |          true             |
*     rlmOn                   |   false   |         true          |          true             |
*     ssbRlmOn                |   false   |      true / false     |          true             |
*     completeSSBDuration (ms)|   0 / 20  |           -           |           -               |
*     minCSIRSFromServiceGnb  |   -       |           4           |          0-3              |
*     loadBalancing           |   -       |           -           |      true / false         |
*
*
*   flexRLM implements a threshold based operations using two signal-to-noise ratio (SNR)
*   thresholds for Radio Link Failure (γ_RLF = -5 dB, via the variable RLFThreshold) and 
*   maximum achievable data rate based on the highest supported modulation and coding scheme 
*   (γ_MR = 22.7 dB, via the variable MRThreshold), which dictate the beam management events 
*   triggered directly by the UE or facilitated by the coordinator. 
*
*   The static assignment of CSI-RS resources for a given number of users is as follows 
*   (please refer to the reference paper in [1] for further information):
*
*      1 UE:    csiRSPeriodicity = 1
*     20 UEs:   csiRSPeriodicity = 23
*     50 UEs:   csiRSPeriodicity = 55
* 
*       with:   csiRSOffset = 3, 
*               maxCSIRSResourcesPerFrame = 2, 
*               noOfBeamsTbRLM = maxCSIRSResourcesPerFrame*2, 
*               ssbRLMTXDirections = 10; // same as number of gNBs
*
**********************************************************************************************************
*/

using namespace ns3;

// LOAD LOCATION INFORMATION OF ENBS
void
LoadEnbLocations (Ptr<ListPositionAllocator> enbPositionAlloc)
{
  std::string input_folder = "src/nr/model/Raytracing/";
  std::string enbFile = input_folder + "enb_locations.txt";
  std::ifstream file1;
  file1.open (enbFile.c_str (), std::ifstream::in);
  NS_ASSERT_MSG (file1.good (), "File containing ENB locations not found");

  std::string line;
  std::string token;
  while (std::getline (file1, line))
    {
      doubleVector_t lineElements;
      std::istringstream stream (line);

      while (getline (stream, token, ','))
        {
          double sigma = 0.00;
          std::stringstream stream (token);
          stream >> sigma;
          lineElements.push_back (sigma);
        }
      enbPositionAlloc->Add (
          Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
      NS_LOG_UNCOND (Vector (lineElements.at (0), lineElements.at (1), lineElements.at (2)));
    }
}

// IP THROUGHPUT AND DELAY LOGGING
std::vector<double> totalBytesReceived;
std::vector<double> totalTimeElapsed;
std::vector<int> numPacketsReceived;
std::vector<double> totalDelay;

static void
GetThroughput (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon,
               std::vector<Ptr<OutputStreamWrapper>> vectorOfStream)
{
  flowMon->CheckForLostPackets ();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats ();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier ());

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin ();
       stats != flowStats.end (); ++stats)
    {
      Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);

      if (fiveTuple.sourceAddress != "1.0.0.2")
        {
          continue;
        }
      //change latter IP address to allow for more users
      uint16_t streamIndex; 
      if(Ipv4Address("7.0.0.1") < fiveTuple.destinationAddress && fiveTuple.destinationAddress < Ipv4Address("7.0.0.102"))
      {
        uint8_t buf[4];
        fiveTuple.destinationAddress.Serialize(buf);
        streamIndex = buf[3]-2;
      }else{
        NS_LOG_UNCOND("Stream Index out of range");
      }  

      *(vectorOfStream.at(streamIndex))->GetStream () << Simulator::Now ().GetSeconds () << "\t"
                                            << (stats->second.rxBytes - totalBytesReceived[streamIndex]) * 8.0 /
                                                (stats->second.timeLastRxPacket.GetSeconds () - totalTimeElapsed[streamIndex])
                                            << "\t"
                                            << (stats->second.delaySum.GetSeconds () - totalDelay[streamIndex]) /
                                                (stats->second.rxPackets - numPacketsReceived[streamIndex])
                                            << std::endl;
      totalBytesReceived[streamIndex] = stats->second.rxBytes;
      totalDelay[streamIndex] = stats->second.delaySum.GetSeconds ();
      numPacketsReceived[streamIndex] = stats->second.rxPackets;
      totalTimeElapsed[streamIndex] = stats->second.timeLastRxPacket.GetSeconds ();
    }

  Simulator::Schedule (Seconds (0.04), &GetThroughput, fmhelper, flowMon, vectorOfStream);
}

// IP LATENCY LOGGING
static void
GetIpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p , Ptr<Ipv4> ipv4, uint32_t interface)
{
  Ptr<Packet> packet = p->Copy ();
  Ipv4Header ipHeader;
  packet->PeekHeader (ipHeader);
  // Logging from the IP header: payloadSize, identification, TOS, TTL, protocol, flags, source IP, dst IP, checksum, headerSize
  IpTimestampTag ipTimestampTag;
  packet->RemovePacketTag (ipTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - ipTimestampTag.GetIpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << ipHeader.GetIdentification() << " \t" << perPacketLatency << std::endl;
}

// UDP LATENCY LOGGING
static void
GetUdpLatency (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy ();
  //Logging from the UDP header: payloadSize, source Port, dst Port, source IP, dst IP, protocol, checksum
  UdpTimestampTag udpTimestampTag;
  packet->RemovePacketTag (udpTimestampTag);
  // get per packet latency in us.
  uint32_t perPacketLatency = (uint32_t)((Simulator::Now().GetSeconds() - udpTimestampTag.GetUdpTimestamp())*1e6); 
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " \t" << perPacketLatency << std::endl;
}

// UDP THROUGHPUT LOGGING
std::vector<double> totalUdpPacketsReceived;
std::vector<double> totalTimElapsedUdp;

static void
GetUdpThroughput (ApplicationContainer appContainer, std::vector<Ptr<OutputStreamWrapper>> vectorOfStream)
{
  for (uint32_t n = 0; n < appContainer.GetN(); n++)
  {
    auto tempReceivedPackets = (double)DynamicCast<UdpServer>(appContainer.Get(n))->GetReceivedBytes();
    auto lastPacketReceived = DynamicCast<UdpServer>(appContainer.Get(n))->GetLastReceivedTime().GetSeconds();

    *(vectorOfStream.at(n))->GetStream() << Simulator::Now().GetSeconds() << "\t"
                                         << (tempReceivedPackets - totalUdpPacketsReceived[n]) * 8.0 /
                                                (lastPacketReceived - totalTimElapsedUdp[n])
                                         << std::endl;

    totalUdpPacketsReceived[n] = tempReceivedPackets;
    totalTimElapsedUdp[n] = lastPacketReceived;
  }

  Simulator::Schedule (Seconds (0.04), &GetUdpThroughput, appContainer, vectorOfStream);
}

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void SetParameters();

/*
*****************************************    SIMULATION PARAMETER CONFIGURATION    *****************************************
*/

// 5G-NR numerology, bandwidth and txPower
uint32_t num = 3; // 5G-NR numerology mu
double centerFreq = 28e9; // Carrier frequency
                          // Available ray-tracing channel data includes 28 GHz and 60 GHz traces
double bandwidth = 400e6; // System Bandwidth 
double txPower = 15.0;    // txPower = 30 dBm for ray-tracing channel input
                          // (total txPower setting is 15 dBm + 15 dBm (from the ray-tracing simulations)
uint32_t packetSize = 1400; // Packet size in bytes
DataRate dataRate = DataRate("400Mb/s"); // Requested data rate per user in Mbps

/*
 *   The simulation parameters are set according to our paper 'flexRLM: Flexible Radio Link
 *   Monitoring for Multi-User Downlink Millimeter-Wave Networks'. 5G-NR numerology 3 
 *   (120kHz subcarriervspacing) at a central frequency of 28 GHz, with a total system bandwidth 
 *   of 400 MHz and a transmit power of 30 dBm is used. The maximum number of SSBs that can be
 *   transmitted is set to 64. For RLM, we set the number of monitored and reported CSI-RSs to 4.
 *   We consider CBR downlink UE traffic at 400 Mbps, and TDMA round-robin as our scheuduler.
 *   The total duration of the multi-user network simulation is 240 s.
*/
    
double simTime = 240; // in seconds
uint8_t numOfUes = 20;

/*
 *                           Table I. Beam management strategies and configuration parameters
 *                              _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
 *                             |                  Beam management strategy                     |
 *     Associated variables    |   Ideal   |    Default 5G-NR      |         flexRLM           |
 *     ----------------------------------------------------------------------------------------|
 *     realisticIA             |   false   |         true          |          true             |
 *     rlmOn                   |   false   |         true          |          true             |
 *     ssbRlmOn                |   false   |      true / false     |          true             |
 *     completeSSBDuration (ms)|   0 / 20  |           -           |           -               |
 *     minCSIRSFromServiceGnb  |   -       |           4           |          0-3              |
 *     loadBalancing           |   -       |           -           |      true / false         |
 *
*/
bool realisticIA = true;
bool rlmOn = true;
bool ssbRlmOn = true;
Time completeSSBDuration = MilliSeconds (0.0);
uint8_t minCSIRSFromServingGnb = 2;
bool loadBalancing = true;  // optional for flexRLM, not available in Ideal or Default 5G-NR
bool adaptiveBF = true;     // set this to FALSE for using the Ideal beam management!!! (leave to true for the others)
bool BFdelay = false;       // leave this to FALSE
bool omniFallback = false;  // leave this to FALSE

// CSI-RS configuration (default settings for numOfUes = 20; please refer to the details above for other cases)
uint16_t csiRSPeriodicity = 23;
uint16_t csiRSOffset = 3;
uint16_t maxCSIRSResourcesPerFrame = 2;
uint8_t noOfBeamsTbRLM = maxCSIRSResourcesPerFrame*2;
uint16_t ssbRLMTXDirections = 10; // same as number of gNBs

// antenna configuraton
std::string antennaConfig = "AntennaConfigInets";
        //AntennaConfigDefault: gNB and UE Elevation range is from 60 to 120 degrees.
        //Angular step in Elevation can be adjusted from variables gNBVerticalBeamStep and ueVerticalBeamStep 
        //gNB and UE Azimuth range is from 0 to 180 degrees.
        //Angular step in Azimuth cannot be adjusted. They depend on the number of rows in antenna arrays of gNB and UE
        //AntennaConfigInets: gNB Elevation: 90, 120, 150; UE Elevation 30, 60, 90
uint8_t ueNumRows = 4;
uint8_t ueNumColumns = 4;
uint8_t gNBNumRows = 8;
uint8_t gNBNumColumns = 8;
double gNBVerticalBeamStep; 
double ueVerticalBeamStep;
double gNBHorizontalBeamStep;
double ueHorizontalBeamStep;

// flexRLM implements a threshold based operations using two signal-to-noise ratio (SNR):
double RLFThreshold = -5.0; // RLF (Outage) threshold in dB
double MRThreshold = 22.7; // Maximum rate threshold in dB

// UE walk parameters
std::vector<uint16_t> walkId;
std::vector<double> ueX;
std::vector<double> ueY;

// Per-packet latency logging
bool ipLatencyLogging  = true;
bool udpLatencyLogging = true;

// other parameters
AsciiTraceHelper asciiTraceHelper;
std::string path = "./out/test/"; // default path for saving simulation output results
bool harqEnabled = true;          // enable/disable hybrid automatic repeat request (HARQ)


/*
*****************************************    START OF MAIN FUNCTION   *****************************************
*/

int
main (int argc, char *argv[])
{
  // antenna configuraton
  if (antennaConfig == "AntennaConfigDefault")
  {
    gNBVerticalBeamStep = 10.0;
    ueVerticalBeamStep = 20.0;
  }
  else if (antennaConfig == "AntennaConfigInets")
  {
    gNBVerticalBeamStep = 30.0;
    ueVerticalBeamStep = 30.0;
    gNBHorizontalBeamStep = 9.0;
    ueHorizontalBeamStep = 18.0;
  }
  else
  {
    NS_ABORT_MSG ("Undefined Antenna Configuration");
  }
  
  // UE walk parameters, load UE information from folder:
  std::string input_raytracing_folder = "src/nr/model/Raytracing_UE_set/";

  struct dirent *entry;
  DIR *dir = opendir("src/nr/model/Raytracing_UE_set/");

  while((entry = readdir(dir)) != NULL)
  {
    std::string filename = entry->d_name;
    if(filename.find("_cords.txt") != std::string::npos)
    {
      walkId.push_back(stoi(filename));

      std::ifstream infile("src/nr/model/Raytracing_UE_set/"+filename);
      std::string sLine;
      getline(infile, sLine);

      size_t pos = sLine.find(",");
      ueX.push_back(std::stof(sLine.substr(0, pos)));
      sLine.erase(0, pos+1);

      pos = sLine.find(",");
      ueY.push_back(std::stof(sLine.substr(0, pos)));
    }
  }
  closedir(dir);

  mkdir(path.c_str(),S_IRWXU);
  SetParameters();
  
  //Good way to display what parameters are tried to be set
  NS_LOG_UNCOND ("Path: " << path);
  NS_LOG_UNCOND ("Simulation parameter<s:");
  NS_LOG_UNCOND ("sim Time: " << simTime);
  NS_LOG_UNCOND ("dataRate: " << dataRate);
  NS_LOG_UNCOND ("numerology: " << num);
  NS_LOG_UNCOND ("bandwidth: " << bandwidth);
  NS_LOG_UNCOND ("adaptiveBF: " << adaptiveBF);
  NS_LOG_UNCOND ("BFdelay: " << BFdelay);
  NS_LOG_UNCOND ("CSI-RS from serving gNB: " << minCSIRSFromServingGnb);
  NS_LOG_UNCOND ("Load balancing: " << loadBalancing);
  
  // setup the Nr simulation
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();

  nrHelper->SetEpcHelper (epcHelper);
  nrHelper->SetIdealBeamformingHelper (idealBeamformingHelper);
  lteHelper->SetEpcHelper (epcHelper);
  nrHelper->SetAttribute ("ChannelModel", StringValue ("ns3::ThreeGppChannelModel"));
  nrHelper->SetHarqEnabled (harqEnabled);
  nrHelper->SetSchedulerTypeId (NrMacSchedulerTdmaRR::GetTypeId ());

  //nrHelper->Initialize();
  nrHelper->LteChannelModelInitialization ();
  
  // position the base stations
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  LoadEnbLocations (enbPositionAlloc);
  
  NodeContainer gnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;

  gnbNodes.Create (enbPositionAlloc->GetSize());
  enbPositionAlloc->Add (Vector (300,300,6));
  lteEnbNodes.Create (1);
  allEnbNodes.Add(gnbNodes);
  allEnbNodes.Add(lteEnbNodes);

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  // UE nodes
  NodeContainer ueNodes;
  ueNodes.Create (numOfUes);

  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);
  for(uint32_t i=0; i<ueNodes.GetN(); i++)
  {
    ueNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (ueX[i], ueY[i], 1.5));
  }
  
  /*
   * CC band configuration n257F (NR Release 15): one contiguous CC of 400MHz. 
   * In this example, the CC contains a single BWP occupying the whole CC bandwidth.
   *
   * -------------- Band --------------
   * -------------- CC0  --------------
   * -------------- BWP0 --------------
  */
  double centralFrequencyBand = centerFreq;
  const uint8_t numCcPerBand = 1;

  BandwidthPartInfo::Scenario scenario = BandwidthPartInfo::UMi_StreetCanyon;
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;

  CcBwpCreator::SimpleOperationBandConf bandConf (centralFrequencyBand,
                                                  bandwidth,
                                                  numCcPerBand,
                                                  scenario);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  nrHelper->InitializeOperationBand(&band);
  allBwps = CcBwpCreator::GetAllBwps({band});

  // Set true to use cell scanning method, false to use the default power method.
  bool enableCellScan = true;
  if (enableCellScan)
  {
    idealBeamformingHelper->SetAttribute ("IdealBeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
  }

  // Core latency
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

  // Antennas for all the UEs
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (ueNumRows));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (ueNumColumns));

  // Antennas for all the gNbs
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (gNBNumRows));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (gNBNumColumns));

	// install Nr net devices
  NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
  NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice (ueNodes, allBwps);
  NetDeviceContainer lteNetDev = nrHelper->InstallLteEnbDevice (lteEnbNodes);

  // create PGW
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  
  // Create the internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (5)));

  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  // Setup routing
  Ptr<OutputStreamWrapper> stream =Create<OutputStreamWrapper> (&std::clog);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<Ipv4StaticRouting> remoteHostStaticRouting;
  remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  remoteHostStaticRouting->PrintRoutingTable(stream);

  Ptr<Ipv4StaticRouting> pgwStaticRouting;
  pgwStaticRouting = ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  pgwStaticRouting->PrintRoutingTable(stream);

  // Install IP stack on UEs
  internet.Install (ueNodes);
  
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (ueNetDev);

  // Assign IP address to UEs
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<Node> ueNode = ueNodes.Get(u);
    
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting;
    ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    ueStaticRouting->PrintRoutingTable(stream);
  }

  // CBR 400 Mbps Application
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  uint16_t udpCbrPort = 10000;
  
  Time packetInterval (Seconds (packetSize * 8 / static_cast<double> (dataRate.GetBitRate ())));

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    UdpServerHelper udpServer(udpCbrPort);
    serverApps.Add(udpServer.Install(ueNodes.Get(u)));

    UdpClientHelper udpClient(ueIpIface.GetAddress(u), udpCbrPort);
    udpClient.SetAttribute("PacketSize", UintegerValue(packetSize));
    udpClient.SetAttribute("Interval", TimeValue(packetInterval));
    udpClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    clientApps.Add(udpClient.Install(remoteHost));
  }

  // start server and client apps
  serverApps.Start(Seconds(0.));
  clientApps.Start(Seconds(0.));
  serverApps.Stop(Seconds(simTime));
  clientApps.Stop(Seconds(simTime - 0.2));

  for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

	//Connecting all the X2 Interfaces
	for(size_t i=0;i<gnbNodes.GetN()-1;++i){
    for(size_t j=i+1;j<gnbNodes.GetN();++j){
        epcHelper->AddX2Interface(gnbNodes.Get(i), gnbNodes.Get(j));
    }
  }

  Ptr<NetDevice> lteEnbNetDevice = lteEnbNodes.Get (0)->GetDevice (0);
  uint16_t lteCellId = lteEnbNodes.Get(0)->GetDevice (0)->GetObject <LteEnbNetDevice> ()->GetCellId();

  for(size_t i=0; i<gnbNodes.GetN(); i++)
  {
    epcHelper->AddX2Interface(gnbNodes.Get(i), lteEnbNodes.Get(0));
    gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetRrc ()->SetClosestLteCellId (lteCellId);

    if (gnbNodes.Get(i)->GetDevice (0)->GetObject <NrGnbNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
        NS_LOG_UNCOND ("Beam Manager is not initialized for gnb Node with Id " << gnbNodes.Get(i)->GetId());
    }
  }

  for (size_t i=0; i<ueNodes.GetN (); i++)
  {
    if (ueNodes.Get(i)->GetDevice (0)->GetObject <NrUeNetDevice> ()->GetPhy (0)->GetBeamManager () == nullptr)
    {
      NS_LOG_UNCOND ("Beam Manager is not initialized for UE node with Id " << ueNodes.Get(i)->GetId());
    }
  }
  
  // load ray-tracing data
  nrHelper->RayTraceModelLoadData (ueNetDev, &band, walkId, input_raytracing_folder);

  // attach UEs to the closest eNB
  nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);
  for(uint32_t i=0; i<ueNetDev.GetN(); i++)
  {
    nrHelper->AttachToLteCoordinator (ueNetDev.Get(i)->GetObject <NetDevice> (), lteEnbNetDevice);
  }

  if (realisticIA)
  {
    for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      for (uint32_t i = 0; i < DynamicCast<NrGnbNetDevice> (*it)->GetCcMapSize (); ++i)
        {
            for(uint32_t j=0; j<ueNetDev.GetN(); j++)
            {
              ueNetDev.Get(j)->GetObject <NrUeNetDevice> ()->GetPhy (i)->SetNumerology (uint16_t(num));
            }
        }
    }
  }

  // change size of global variables to match # of UEs
  std::vector<double> vector1 (ueNodes.GetN() , 0.0);
  std::vector<int> vector2 (ueNodes.GetN() , 0);
  totalUdpPacketsReceived = vector1;
  totalTimElapsedUdp = vector1;
  totalBytesReceived = vector1;
  totalTimeElapsed = vector1;
  numPacketsReceived = vector2;
  totalDelay = vector1;

  // enable the traces provided by the mmWave module
  nrHelper->EnableTraces();
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  
  // Configure and schedule IP throughput and IP latency logging
  std::vector<Ptr<OutputStreamWrapper>> streamVector_ip;
  for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> ipStream = asciiTraceHelper.CreateFileStream (path + "DlIpStats" + std::to_string (imsi) + ".txt");
    streamVector_ip.emplace_back(ipStream);
  }
  Simulator::Schedule (Seconds (0.3), &GetThroughput, &flowmon, monitor, streamVector_ip);

  if (ipLatencyLogging)
  {
    // IP latency logging
    for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
    {
      Ptr<OutputStreamWrapper> ipLatencyStream = asciiTraceHelper.CreateFileStream(path + "DlIpLatency" + std::to_string(imsi) + ".txt");

      Ptr<Ipv4> ipv4 = ueNodes.Get(imsi - 1)->GetObject<Ipv4>();
      Ptr<Ipv4L3Protocol> ipv4L3Protocol = ipv4->GetObject<Ipv4L3Protocol>();
      ipv4L3Protocol->TraceConnectWithoutContext("Rx", MakeBoundCallback(&GetIpLatency, ipLatencyStream));
    }
  }

  if (udpLatencyLogging)
  {
    // UDP latency logging
    for (uint64_t imsi = 1; imsi <= ueNetDev.GetN(); imsi++)
    {
      Ptr<OutputStreamWrapper> udpLatencyStream = asciiTraceHelper.CreateFileStream(path + "DlUdpLatency" + std::to_string(imsi) + ".txt");

      Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(serverApps.Get(imsi - 1));
      udpServer->TraceConnectWithoutContext("Rx", MakeBoundCallback(&GetUdpLatency, udpLatencyStream));
    }
  }

  // Configure and schedule UDP throughput logging
  std::vector<Ptr<OutputStreamWrapper>> streamVector;
  for (uint64_t imsi = 1; imsi <= ueNodes.GetN(); imsi++)
  {
    Ptr<OutputStreamWrapper> udpStream = asciiTraceHelper.CreateFileStream (path + "DlUdpStats" + std::to_string (imsi) + ".txt");
    streamVector.emplace_back (udpStream);
  }
  Simulator::Schedule (Seconds(0.5), &GetUdpThroughput, serverApps, streamVector);


  // connect custom trace sinks for RRC connection establishment and handover notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}

void SetParameters()
{
	//This parameter controls the beamforming periodicity. If 0 then no periodic beamforming.
	int BFperiodicity;
	if(adaptiveBF)
	{
		BFperiodicity = 0;
	}
	else
	{
		BFperiodicity = 250; //if adaptive BF is turned off, trigger BF every 250 ms (on each position update)
	}
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::NrGnbPhy::UpdateSinrEstimatePeriod", DoubleValue (1600.0));
	Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity", TimeValue(MilliSeconds(BFperiodicity)));
	Config::SetDefault ("ns3::NrGnbPhy::IADelay", DoubleValue(5250));
	Config::SetDefault ("ns3::NrGnbPhy::BeamTrainingDelay", DoubleValue(5));
	Config::SetDefault ("ns3::NrGnbPhy::OmniNrFallback", BooleanValue(omniFallback));
  Config::SetDefault ("ns3::NrGnbMac::NumberOfBeamsTbRLM", UintegerValue (8));

  //In the following, set RaySourceType to "Inventory" if Inventory.txt is going to be used. Else, set it to "EnbTraceData"
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::RaySourceType", StringValue("Inventory"));

  if (antennaConfig == "AntennaConfigDefault")
  {
    Config::SetDefault ("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue (gNBVerticalBeamStep));
    Config::SetDefault ("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue (180.0 / (double)gNBNumRows));

    Config::SetDefault ("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
    Config::SetDefault ("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (180.0 / (double)ueNumRows));
  }
  else
  {
    Config::SetDefault("ns3::NrGnbPhy::GnbElevationAngleStep", DoubleValue(gNBVerticalBeamStep));
    Config::SetDefault("ns3::NrGnbPhy::GnbHorizontalAngleStep", DoubleValue(gNBHorizontalBeamStep));

    Config::SetDefault("ns3::NrUePhy::UeHorizontalAngleStep", DoubleValue (ueHorizontalBeamStep));
    Config::SetDefault("ns3::NrUePhy::UeElevationAngleStep", DoubleValue (ueVerticalBeamStep));
  }

  Config::SetDefault("ns3::NrUePhy::GnbSectionNumber", UintegerValue (63));
  Config::SetDefault("ns3::NrUePhy::CellSelectionCriterion", StringValue ("PeakSnr"));
  Config::SetDefault("ns3::NrUePhy::BeamSweepThreshold", DoubleValue (10.0));
  Config::SetDefault("ns3::NrUePhy::MaxRateThreshold", DoubleValue (MRThreshold));
  Config::SetDefault("ns3::NrUePhy::TXSSBScanDirections", UintegerValue (ssbRLMTXDirections));
  Config::SetDefault("ns3::NrPhy::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrHelper::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrGnbMac::MinCSIRSFromServingGnb", UintegerValue(minCSIRSFromServingGnb));
  Config::SetDefault("ns3::NrHelper::LoadBalancing", BooleanValue (loadBalancing));
  Config::SetDefault("ns3::NrPhy::AntennaConfiguration", StringValue(antennaConfig));

  Config::SetDefault("ns3::NrHelper::AdaptiveBeamforming", BooleanValue (adaptiveBF));
  Config::SetDefault("ns3::NrHelper::RealisticIA", BooleanValue (realisticIA));
  Config::SetDefault("ns3::ThreeGppChannelModel::RealisticBeamSweep", BooleanValue (realisticIA));
  Config::SetDefault("ns3::NrHelper::RadioLinkMonitoring", BooleanValue (rlmOn));
  Config::SetDefault("ns3::NrHelper::NumberOfRLMDirections", UintegerValue (noOfBeamsTbRLM));
  Config::SetDefault("ns3::NrGnbMac::NumberOfBeamsTbRLM", UintegerValue (noOfBeamsTbRLM));
  Config::SetDefault("ns3::NrPhy::SSBRLMOn", BooleanValue (ssbRlmOn));
  Config::SetDefault ("ns3::NrUePhy::CompleteTxSweepDuration", TimeValue(completeSSBDuration));
  Config::SetDefault ("ns3::NrUePhy::NoOfCSIRSResourcesPerSSBurst", UintegerValue(maxCSIRSResourcesPerFrame * 2));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSPeriodicity", UintegerValue (csiRSPeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::CSIRSOffset", UintegerValue (csiRSOffset));
  Config::SetDefault ("ns3::LteEnbRrc::MaxNumOfCSIRSResource", UintegerValue (maxCSIRSResourcesPerFrame));


	Config::SetDefault("ns3::NrGnbPhy::ApplyBeamformingDelay", BooleanValue(BFdelay));
  Config::SetDefault ("ns3::NrGnbPhy::TxPower", DoubleValue (txPower));
  Config::SetDefault ("ns3::NrUePhy::TxPower", DoubleValue (txPower));
	Config::SetDefault ("ns3::NrGnbPhy::Numerology", UintegerValue(num));

	Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::ChannelTypeRaytracing", BooleanValue(true));
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ChannelTypeRaytracing", BooleanValue (true));

  Config::SetDefault ("ns3::ThreeGppChannelModel::Scenario", StringValue("UMi-StreetCanyon"));
  // important to set frequency into the 3gpp model
  Config::SetDefault ("ns3::ThreeGppChannelModel::Frequency", DoubleValue(28e9));
  Config::SetDefault ("ns3::ThreeGppSpectrumPropagationLossModel::Frequency", StringValue ("28GHz"));

  Config::SetDefault ("ns3::NrGnbPhy::UpdateUeSinrEstimatePeriod", DoubleValue (0));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (200*1024));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MicroSeconds(100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity",TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (RLFThreshold));
  Config::SetDefault("ns3::LteEnbRrc::MaxRateThreshold", DoubleValue (MRThreshold));
  Config::SetDefault("ns3::LteUeRrc::MaxRateThreshold", DoubleValue (MRThreshold));
  
	//Realisitc X2 and S1-U links
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay",
                      TimeValue (MicroSeconds (5)));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDataRate",
                      DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkMtu", UintegerValue (10000));
  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay",
                      TimeValue (MicroSeconds (5)));

	Config::SetDefault ("ns3::NrPhyRxTrace::OutputFileName",
                      StringValue (path + "RxPacketTrace.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::PhyRxThruFileName",
                      StringValue (path + "PhyRxThruData.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::MacRxThruFileName",
                      StringValue (path + "MacRxThruData.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpOutputFilename",
                      StringValue (path + "DlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::UlPdcpOutputFilename",
                      StringValue (path + "UlPdcpStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlPdcpThruFilename",
                      StringValue (path + "DlPdcpThruStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcOutputFilename",
                      StringValue (path + "DlRlcStats.txt"));
  Config::SetDefault ("ns3::NrBearerStatsCalculator::DlRlcThruFilename",
                      StringValue (path + "DlRlcThruStats.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepFileName",
                      StringValue (path + "BeamSweepTrace.txt"));
  Config::SetDefault ("ns3::NrPhyRxTrace::RadioLinkMonitoringFileName",
                      StringValue (path + "RadioLinkMonitoringTrace.txt"));
  if (!realisticIA)
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Ideal"));
  }
  else
  {
    Config::SetDefault ("ns3::NrPhyRxTrace::BeamSweepType",
                      StringValue ("Real"));
  }
}
