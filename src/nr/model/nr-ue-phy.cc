/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015 NYU WIRELESS, Tandon School of Engineering, New York University
*   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/

#define NS_LOG_APPEND_CONTEXT                                            \
  do                                                                     \
    {                                                                    \
      std::clog << " [ CellId " << GetCellId() << ", bwpId "             \
                << GetBwpId () << "] ";                                  \
    }                                                                    \
  while (false);

#include "nr-ue-phy.h"
#include "nr-ue-net-device.h"
#include "nr-spectrum-value-helper.h"
#include "nr-ch-access-manager.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/node.h>
#include <ns3/double.h>
#include <ns3/lte-radio-bearer-tag.h>
#include <algorithm>
#include <cfloat>
#include <ns3/boolean.h>
#include <ns3/pointer.h>
#include "beam-manager.h"
#include <ns3/beamforming-vector.h>
#include <ns3/nr-gnb-phy.h>
#include <ns3/nr-gnb-net-device.h>
#include <ns3/nr-control-messages.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrUePhy");
NS_OBJECT_ENSURE_REGISTERED (NrUePhy);

NrUePhy::NrUePhy ()
{
  NS_LOG_FUNCTION (this);
  m_wbCqiLast = Simulator::Now ();
  m_cellSinrMap.clear ();
  m_ueCphySapProvider = new MemberLteUeCphySapProvider<NrUePhy> (this);
  m_resetIdealBeamforming = true;
  m_IAalreadyTriggered = false;
  m_txSSBCounterPerRx = 0;
  m_beamTbSwept = BeamId (0, 70.0);
  m_ssbRLMProcessorMap.clear ();
  m_ssbRMCounter = 0;
  m_conveyPacketsToMac = true;

  if (m_realisticIA)
  {
    m_IAperformed = true;
  }
  else
  {
    m_IAperformed = false;
  }  
  m_cellIdCounter = 1;

  Simulator::Schedule(m_ueMeasurementsFilterPeriod, &NrUePhy::ReportUeMeasurements, this);
}

NrUePhy::~NrUePhy ()
{
  NS_LOG_FUNCTION (this);
}

void
NrUePhy::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_ueCphySapProvider;
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  m_registeredEnb.clear ();
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  NrPhy::DoDispose ();
}

TypeId
NrUePhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrUePhy")
    .SetParent<NrPhy> ()
    .AddConstructor<NrUePhy> ()
    .AddAttribute ("TxPower",
                   "Transmission power in dBm",
                   DoubleValue (2.0),
                   MakeDoubleAccessor (&NrUePhy::m_txPower),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("NoiseFigure",
                   "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                   " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                   "\"the difference in decibels (dB) between"
                   " the noise output of the actual receiver to the noise output of an "
                   " ideal receiver with the same overall gain and bandwidth when the receivers "
                   " are connected to sources at the standard noise temperature T0.\" "
                  "In this model, we consider T0 = 290K.",
                   DoubleValue (5.0), // nr code from NYU and UniPd assumed in the code the value of 5dB, thats why we configure the default value to that
                   MakeDoubleAccessor (&NrUePhy::m_noiseFigure),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SpectrumPhy",
                   "The SpectrumPhy associated to this NrPhy",
                   TypeId::ATTR_GET,
                   PointerValue (),
                   MakePointerAccessor (&NrPhy::GetSpectrumPhy),
                   MakePointerChecker <NrSpectrumPhy> ())
    .AddAttribute ("LBTThresholdForCtrl",
                   "After a DL/UL transmission, if we have less than this value to send the UL CTRL, we consider the channel as granted",
                   TimeValue (MicroSeconds (25)),
                   MakeTimeAccessor (&NrUePhy::m_lbtThresholdForCtrl),
                   MakeTimeChecker ())
    .AddAttribute ("TbDecodeLatency",
                   "Transport block decode latency",
                   TimeValue (MicroSeconds (100)),
                   MakeTimeAccessor (&NrPhy::SetTbDecodeLatency,
                   &NrPhy::GetTbDecodeLatency),
                   MakeTimeChecker ())
    .AddAttribute ("NoOfCSIRSResourcesPerSSBurst",
                   "Number of CSI-RS Resources within an SS Burst",
                    UintegerValue (16),
                    MakeUintegerAccessor (&NrUePhy::m_noOfCSIRSResourcesPerSSBurst),
                    MakeUintegerChecker<uint16_t> ())
    .AddTraceSource ("ReportCurrentCellRsrpSinr",
                     "RSRP and SINR statistics.",
                     MakeTraceSourceAccessor (&NrUePhy::m_reportCurrentCellRsrpSinrTrace),
                     "ns3::CurrentCellRsrpSinr::TracedCallback")
    .AddTraceSource ("ReportUplinkTbSize",
                     "Report allocated uplink TB size for trace.",
                     MakeTraceSourceAccessor (&NrUePhy::m_reportUlTbSize),
                     "ns3::UlTbSize::TracedCallback")
    .AddTraceSource ("ReportDownlinkTbSize",
                     "Report allocated downlink TB size for trace.",
                     MakeTraceSourceAccessor (&NrUePhy::m_reportDlTbSize),
                     "ns3::DlTbSize::TracedCallback")
    .AddAttribute("UeMeasurementsFilterPeriod",
                    "Time period for reporting UE measurements, i.e., the"
                    "length of layer-1 filtering.",
                    TimeValue(MilliSeconds(200)),
                    MakeTimeAccessor(&NrUePhy::m_ueMeasurementsFilterPeriod),
                    MakeTimeChecker())
    .AddTraceSource("ReportRsrp",
                    "RSRP statistics.",
                    MakeTraceSourceAccessor(&NrUePhy::m_reportRsrpTrace),
                    "ns3::CurrentRsrp::TracedCallback")
    .AddTraceSource("DlCtrlSinr",
                    "Report the SINR computed for DL CTRL",
                    MakeTraceSourceAccessor(&NrUePhy::m_dlCtrlSinrTrace),
                    "ns3::NrUePhy::DlCtrlSinrTracedCallback")
    .AddTraceSource ("UePhyRxedCtrlMsgsTrace",
                     "Ue PHY Control Messages Traces.",
                     MakeTraceSourceAccessor (&NrUePhy::m_phyRxedCtrlMsgsTrace),
                     "ns3::NrPhyRxTrace::RxedUePhyCtrlMsgsTracedCallback")
    .AddTraceSource ("UePhyTxedCtrlMsgsTrace",
                     "Ue PHY Control Messages Traces.",
                     MakeTraceSourceAccessor (&NrUePhy::m_phyTxedCtrlMsgsTrace),
                     "ns3::NrPhyRxTrace::TxedUePhyCtrlMsgsTracedCallback")
    .AddTraceSource ("UePhyRxedDlDciTrace",
                     "Ue PHY DL DCI Traces.",
                     MakeTraceSourceAccessor (&NrUePhy::m_phyUeRxedDlDciTrace),
                     "ns3::NrPhyRxTrace::RxedUePhyDlDciTracedCallback")
    .AddTraceSource ("UePhyTxedHarqFeedbackTrace",
                     "Ue PHY DL HARQ Feedback Traces.",
                     MakeTraceSourceAccessor (&NrUePhy::m_phyUeTxedHarqFeedbackTrace),
                     "ns3::NrPhyRxTrace::TxedUePhyHarqFeedbackTracedCallback")
    .AddTraceSource ("ReportPowerSpectralDensity",
                     "Power Spectral Density data.",
                     MakeTraceSourceAccessor (&NrUePhy::m_reportPowerSpectralDensity),
                     "ns3::NrUePhy::PowerSpectralDensityTracedCallback")
    .AddTraceSource ("BeamSweepTrace",
                     "trace fired when a beam sweep is completed at UE PHY",
                     MakeTraceSourceAccessor (&NrUePhy::m_beamSweepTrace),
                     "ns3::BeamSweepTraceParams::TracedCallback")
    .AddTraceSource ("RadioLinkMonitoringTrace",
                     "trace fired when radio link monitoring is adjusted at UE PHY",
                     MakeTraceSourceAccessor (&NrUePhy::m_radioLinkMonitoringTrace),
                     "ns3::RadioLinkMonitoringTrace::TracedCallback")
    .AddAttribute ("BeamSweepThreshold",
                    "SNR threshold below which beam sweep will be done [dB]",
                    DoubleValue (0.0),
                    MakeDoubleAccessor (&NrUePhy::m_beamSweepThreshold),
                    MakeDoubleChecker<double> (-10000.0, 20.0))
    .AddAttribute ("MaxRateThreshold",
                    "SNR Threshold for max. rate",
                    DoubleValue (22.7),
                    MakeDoubleAccessor (&NrUePhy::m_maxRateThreshold),
                    MakeDoubleChecker <double> (-1000.0, 1000.0))
    .AddAttribute ("OutageThreshold",
                    "SNR threshold for outage events [dB]",
                    DoubleValue (-5.0),
                    MakeDoubleAccessor (&NrUePhy::m_outageThreshold),
                    MakeDoubleChecker <long double> (-10000.0, 10.0))   
    .AddAttribute ("n310",
                   "Counter for SINR below threshold events",
                   UintegerValue (2),
                   MakeUintegerAccessor (&NrUePhy::m_n310),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("IdealBFTimer",
                   "Timer for UE to perform ideal beamforming",
                   TimeValue (MilliSeconds(500)),
                   MakeTimeAccessor (&NrUePhy::m_idealBFTimer),
                   MakeTimeChecker ())
    .AddAttribute ("UeElevationAngleStep",
                   "Angle step that will be used for sweep at elevation",
                   DoubleValue (20),
                   MakeDoubleAccessor (&NrUePhy::SetUeVerticalAngleStep),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UeHorizontalAngleStep",
                   "Angle Step that will be used for sweep at azimuth",
                   DoubleValue (40),
                   MakeDoubleAccessor (&NrUePhy::SetUeHorizontalAngleStep),
                   MakeDoubleChecker<double> (1.0, 90.0))
    .AddAttribute ("GnbSectionNumber",
                   "Total number of sections that gNB can sweep (known by UE",
                   UintegerValue (63),
                   MakeUintegerAccessor (&NrUePhy::m_gnbSectorNumber),
                   MakeUintegerChecker<uint16_t> (0, 1000))
    .AddAttribute ("CellSelectionCriterion",
                   "Criterion by which the cell selection will be done",
                   EnumValue (CellSelectionCriterion::PeakSNR),
                   MakeEnumAccessor (&NrUePhy::m_cellSelectionCriterion),
                   MakeEnumChecker (PeakSNR, "PeakSnr",
                                    MaxAvgSNR, "MaxAvgSnr"))
    .AddAttribute ("CompleteTxSweepDuration",
                   "Time for the gNB to sweep all its sectors",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&NrUePhy::m_completeSSBurstDuration),
                   MakeTimeChecker ())
    .AddAttribute ("TXSSBScanDirections",
                   "Number of TX directions that will be scanned for SSB RLM (not for IA)",
                   UintegerValue (10),
                   MakeUintegerAccessor (&NrUePhy::m_noOfTxSSBScanDirections),
                   MakeUintegerChecker<uint16_t> (0, 40))
    .AddAttribute ("GnbHorizSectorNumber",
                   "Number of horizontal sectors at the gNB",
                   UintegerValue (21),
                   MakeUintegerAccessor (&NrUePhy::m_gnbHorizSectorNumber),
                   MakeUintegerChecker<uint16_t> ());
      ;
  return tid;
}

void
NrUePhy::ChannelAccessGranted (const Time &time)
{
  NS_LOG_FUNCTION (this);
  NS_UNUSED (time);
  // That will be granted only till the end of the slot
  m_channelStatus = GRANTED;
}

void
NrUePhy::ChannelAccessDenied ()
{
  NS_LOG_FUNCTION (this);
  m_channelStatus = NONE;
}

void
NrUePhy::SetUeCphySapUser (LteUeCphySapUser* s)
{
  NS_LOG_FUNCTION (this);
  m_ueCphySapUser = s;
}

LteUeCphySapProvider*
NrUePhy::GetUeCphySapProvider ()
{
  NS_LOG_FUNCTION (this);
  return (m_ueCphySapProvider);
}

void
NrUePhy::SetTxPower (double pow)
{
  m_txPower = pow;
}
double
NrUePhy::GetTxPower () const
{
  return m_txPower;
}

void
NrUePhy::SetDlAmc(const Ptr<const NrAmc> &amc)
{
  m_amc = amc;
}

void
NrUePhy::SetSubChannelsForTransmission (const std::vector <int> &mask, uint32_t numSym)
{
  Ptr<SpectrumValue> txPsd = GetTxPowerSpectralDensity (mask);
  NS_ASSERT (txPsd);

  m_reportPowerSpectralDensity (m_currentSlot, txPsd, numSym * GetSymbolPeriod (), m_rnti, m_imsi, GetBwpId (), GetCellId ());
  m_spectrumPhy->SetTxPowerSpectralDensity (txPsd);
}

void
NrUePhy::DoSendControlMessage (Ptr<NrControlMessage> msg)
{
  NS_LOG_FUNCTION (this << msg);
  EnqueueCtrlMessage (msg);
}

void
NrUePhy::DoSendControlMessageNow (Ptr<NrControlMessage> msg)
{
  NS_LOG_FUNCTION (this << msg);
  EnqueueCtrlMsgNow (msg);
}

void
NrUePhy::RegisterToEnb (uint16_t bwpId)
{
  NS_LOG_FUNCTION (this);

  InitializeMessageList ();

  Ptr<SpectrumValue> noisePsd = GetNoisePowerSpectralDensity ();
  m_spectrumPhy->SetNoisePowerSpectralDensity (noisePsd);

  DoSetCellId (bwpId);

  Ptr<NrGnbNetDevice> gnbNetDevice = m_registeredEnb.find (bwpId)->second;
  DynamicCast<NrUeNetDevice> (m_netDevice)->SetTargetEnb (gnbNetDevice);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void
NrUePhy::RegisterOtherEnb (uint16_t cellId, Ptr<NrGnbNetDevice> gnbNetDevice)
{
  NS_ASSERT_MSG (m_registeredEnb.find (cellId) == m_registeredEnb.end (), "Gnb already registered");
  m_registeredEnb[cellId] = gnbNetDevice;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void
NrUePhy::SetUlCtrlSyms(uint8_t ulCtrlSyms)
{
  m_ulCtrlSyms = ulCtrlSyms;
}

void
NrUePhy::SetDlCtrlSyms(uint8_t dlCtrlSyms)
{
  m_dlCtrlSyms = dlCtrlSyms;
}

void
NrUePhy::SetNumRbPerRbg (uint32_t numRbPerRbg)
{
  m_numRbPerRbg = numRbPerRbg;
}

void
NrUePhy::SetPattern (const std::string &pattern)
{
  NS_LOG_FUNCTION (this);

  static std::unordered_map<std::string, LteNrTddSlotType> lookupTable =
  {
    { "DL", LteNrTddSlotType::DL },
    { "UL", LteNrTddSlotType::UL },
    { "S",  LteNrTddSlotType::S },
    { "F",  LteNrTddSlotType::F },
  };

  std::vector<LteNrTddSlotType> vector;
  std::stringstream ss (pattern);
  std::string token;
  std::vector<std::string> extracted;

   while (std::getline(ss, token, '|'))
     {
       extracted.push_back(token);
     }

   for (const auto & v : extracted)
     {
       vector.push_back (lookupTable[v]);
     }

   m_tddPattern = vector;
}

uint32_t
NrUePhy::GetNumRbPerRbg () const
{
  return m_numRbPerRbg;
}

uint32_t
NrUePhy::GetChannelBandwidth() const
{
  // m_channelBandwidth is in kHz * 100
  return m_channelBandwidth * 1000 * 100;
}

double
NrUePhy::ComputeAvgSinr (const SpectrumValue &sinr)
{
  // averaged SINR among RBs
  double sum = 0.0;
  uint8_t rbNum = 0;
  Values::const_iterator it;

  for (it = sinr.ConstValuesBegin (); it != sinr.ConstValuesEnd (); it++)
    {
      sum += (*it);
      rbNum++;
    }

  double avrgSinr = (rbNum > 0) ? (sum / rbNum) : DBL_MAX;

  return avrgSinr;
}

void
NrUePhy::InsertAllocation (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  VarTtiAllocInfo varTtiInfo (dci);
  m_currSlotAllocInfo.m_varTtiAllocInfo.push_back (varTtiInfo);
  std::sort (m_currSlotAllocInfo.m_varTtiAllocInfo.begin (), m_currSlotAllocInfo.m_varTtiAllocInfo.end ());
}

void
NrUePhy::InsertFutureAllocation (const SfnSf &sfnSf,
                                     const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  VarTtiAllocInfo varTtiInfo (dci);
  if (SlotAllocInfoExists (sfnSf))
    {
      auto & ulSlot = PeekSlotAllocInfo (sfnSf);
      ulSlot.m_varTtiAllocInfo.push_back (varTtiInfo);
      std::sort (ulSlot.m_varTtiAllocInfo.begin (), ulSlot.m_varTtiAllocInfo.end ());
    }
  else
    {
      SlotAllocInfo slotAllocInfo = SlotAllocInfo (sfnSf);
      slotAllocInfo.m_varTtiAllocInfo.push_back (varTtiInfo);
      PushBackSlotAllocInfo (slotAllocInfo);
    }
}

void
NrUePhy::PhyCtrlMessagesReceived (const Ptr<NrControlMessage> &msg)
{
  NS_LOG_FUNCTION (this);

  if (msg->GetMessageType () == NrControlMessage::DL_DCI)
    {
      auto dciMsg = DynamicCast<NrDlDciMessage> (msg);
      auto dciInfoElem = dciMsg->GetDciInfoElement ();

      m_phyRxedCtrlMsgsTrace (m_currentSlot, GetCellId (), m_rnti, GetBwpId (), msg);

      if (dciInfoElem->m_rnti != 0 && dciInfoElem->m_rnti != m_rnti)
        {
          return;   // DCI not for me
        }


      SfnSf dciSfn = m_currentSlot;
      uint32_t k0Delay = dciMsg->GetKDelay ();
      dciSfn.Add (k0Delay);

      NS_LOG_DEBUG ("UE" << m_rnti << " DL-DCI received for slot " << dciSfn <<
                    " symStart " << static_cast<uint32_t> (dciInfoElem->m_symStart) <<
                    " numSym " << static_cast<uint32_t> (dciInfoElem->m_numSym) <<
                    " tbs " << dciInfoElem->m_tbSize <<
                    " harqId " << static_cast<uint32_t> (dciInfoElem->m_harqProcess));

      /* BIG ASSUMPTION: We assume that K0 is always 0 */

      auto it = m_harqIdToK1Map.find (dciInfoElem->m_harqProcess);
      if (it!=m_harqIdToK1Map.end ())
        {
          m_harqIdToK1Map.erase (m_harqIdToK1Map.find (dciInfoElem->m_harqProcess));
        }

      m_harqIdToK1Map.insert (std::make_pair (dciInfoElem->m_harqProcess, dciMsg->GetK1Delay ()));

      m_phyUeRxedDlDciTrace (m_currentSlot, GetCellId (), m_rnti, GetBwpId (), dciInfoElem->m_harqProcess, dciMsg->GetK1Delay ());

      InsertAllocation (dciInfoElem);

      m_phySapUser->ReceiveControlMessage (msg);
    }
  else if (msg->GetMessageType () == NrControlMessage::UL_DCI)
    {
      auto dciMsg = DynamicCast<NrUlDciMessage> (msg);
      auto dciInfoElem = dciMsg->GetDciInfoElement ();

      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), m_rnti, GetBwpId (), msg);

      if (dciInfoElem->m_rnti != 0 && dciInfoElem->m_rnti != m_rnti)
        {
          return;   // DCI not for me
        }

      SfnSf ulSfnSf = m_currentSlot;
      uint32_t k2Delay = dciMsg->GetKDelay ();
      ulSfnSf.Add (k2Delay);

      NS_LOG_DEBUG ("UE" << m_rnti <<
                    " UL-DCI received for slot " << ulSfnSf <<
                    " symStart " << static_cast<uint32_t> (dciInfoElem->m_symStart) <<
                    " numSym " << static_cast<uint32_t> (dciInfoElem->m_numSym) <<
                    " tbs " << dciInfoElem->m_tbSize <<
                    " harqId " << static_cast<uint32_t> (dciInfoElem->m_harqProcess));

      if (ulSfnSf == m_currentSlot)
        {
          InsertAllocation (dciInfoElem);
        }
      else
        {
          InsertFutureAllocation (ulSfnSf, dciInfoElem);
        }

      m_phySapUser->ReceiveControlMessage (msg);
    }
  else if (msg->GetMessageType () == NrControlMessage::MIB)
    {
      NS_LOG_INFO ("received MIB");
      Ptr<NrMibMessage> msg2 = DynamicCast<NrMibMessage> (msg);
      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), m_rnti, GetBwpId (), msg);
      m_ueCphySapUser->RecvMasterInformationBlock (GetCellId (), msg2->GetMib ());
    }
  else if (msg->GetMessageType () == NrControlMessage::SIB1)
    {
      Ptr<NrSib1Message> msg2 = DynamicCast<NrSib1Message> (msg);
      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), m_rnti, GetBwpId (), msg);
      m_ueCphySapUser->RecvSystemInformationBlockType1 (GetCellId (), msg2->GetSib1 ());
    }
  else if (msg->GetMessageType () == NrControlMessage::RAR)
    {
      Ptr<NrRarMessage> rarMsg = DynamicCast<NrRarMessage> (msg);
      Simulator::Schedule ((GetSlotPeriod () * (GetL1L2CtrlLatency ()/2)), &NrUePhy::DoReceiveRar, this, rarMsg);
    }
  else if (msg->GetMessageType () == NrControlMessage::PSS && m_realisticIA)
    {
      Ptr<NrPssMessage> pssMsg = DynamicCast<NrPssMessage> (msg);
      ProcessSSBs (pssMsg);
    }
  else if (msg->GetMessageType () == NrControlMessage::CSI_RS && m_realisticIA && m_rlmOn)
    {
      Ptr<NrCSIRSMessage> csiRSMsg = DynamicCast<NrCSIRSMessage> (msg);
      if (csiRSMsg->GetDestinationImsi()!=m_imsi)
      {
        return;   // CSIRS not for me. Most likely for another UE within the same beam.
      }
      ProcessCSIRSs (csiRSMsg);
    }  
  else
    {
      NS_LOG_INFO ("Message type not recognized " << msg->GetMessageType ());
      m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), m_rnti, GetBwpId (), msg);
      m_phySapUser->ReceiveControlMessage (msg);
    }
}

void
NrUePhy::TryToPerformLbt ()
{
  NS_LOG_FUNCTION (this);
  uint8_t ulCtrlSymStart = 0;
  uint8_t ulCtrlNumSym = 0;

  for (const auto & alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
      if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL && alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
          ulCtrlSymStart = alloc.m_dci->m_symStart;
          ulCtrlNumSym = alloc.m_dci->m_numSym;
          break;
        }
    }

  if (ulCtrlNumSym != 0)
    {
      // We have an UL CTRL symbol scheduled and we have to transmit CTRLs..
      // .. so we check that we have at least 25 us between the latest DCI,
      // or we have to schedule an LBT event.

      Time limit = m_lastSlotStart + GetSlotPeriod () -
          ((GetSymbolsPerSlot () - ulCtrlSymStart) * GetSymbolPeriod ()) -
          m_lbtThresholdForCtrl;

      for (const auto & alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
        {
          int64_t symbolPeriod = GetSymbolPeriod ().GetMicroSeconds ();
          int64_t dciEndsAt = m_lastSlotStart.GetMicroSeconds () +
              ((alloc.m_dci->m_numSym + alloc.m_dci->m_symStart) * symbolPeriod);

          if (alloc.m_dci->m_type != DciInfoElementTdma::DATA)
            {
              continue;
            }

          if (limit.GetMicroSeconds () < dciEndsAt)
            {
              NS_LOG_INFO ("This data DCI ends at " << MicroSeconds (dciEndsAt) <<
                           " which is inside the LBT shared COT (the limit is " <<
                           limit << "). No need for LBT");
              m_lbtEvent.Cancel (); // Forget any LBT we previously set, because of the new
                                    // DCI information
              m_channelStatus = GRANTED;
            }
          else
            {
              NS_LOG_INFO ("This data DCI starts at " << +alloc.m_dci->m_symStart << " for " <<
                           +alloc.m_dci->m_numSym << " ends at " << MicroSeconds (dciEndsAt) <<
                           " which is outside the LBT shared COT (the limit is " <<
                           limit << ").");
            }
        }
      if (m_channelStatus != GRANTED)
        {
          Time sched = m_lastSlotStart - Simulator::Now () +
              (GetSymbolPeriod () * ulCtrlSymStart) - MicroSeconds (25);
          NS_LOG_INFO ("Scheduling an LBT for sending the UL CTRL at " <<
                       Simulator::Now () + sched);
          m_lbtEvent.Cancel ();
          m_lbtEvent = Simulator::Schedule (sched, &NrUePhy::RequestAccess, this);
        }
      else
        {
          NS_LOG_INFO ("Not scheduling LBT: the UE has a channel status that is GRANTED");
        }
    }
  else
    {
      NS_LOG_INFO ("Not scheduling LBT; the UE has no UL CTRL symbols available");
    }
}

void
NrUePhy::RequestAccess ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("Request access at " << Simulator::Now () << " because we have to transmit UL CTRL");
  m_cam->RequestAccess (); // This will put the m_channelStatus to granted when
                           // the channel will be granted.
}

void
NrUePhy::DoReceiveRar (Ptr<NrRarMessage> rarMsg)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_INFO ("Received RAR in slot " << m_currentSlot);
  m_phyRxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), m_rnti, GetBwpId (), rarMsg);

  for (auto it = rarMsg->RarListBegin (); it != rarMsg->RarListEnd (); ++it)
    {
      if (it->rapId == m_raPreambleId)
        {
          m_phySapUser->ReceiveControlMessage (rarMsg);
          if (m_recvCSIMap.size() != 0)
          {
            m_recvCSIMap.clear ();  
          }
          if (m_subOptimalCSIBeamMap.size () != 0)
          {
            m_subOptimalCSIBeamMap.clear ();
          }
        }
    }
}

void
NrUePhy::PushCtrlAllocations (const SfnSf currentSfnSf)
{
  NS_LOG_FUNCTION (this);

  // The UE does not know anything from the GNB yet, so listen on the default
  // bandwidth.
  std::vector<uint8_t> rbgBitmask (GetRbNum (), 1);

  // The UE still doesn't know the TDD pattern, so just add a DL CTRL
  if (m_tddPattern.size () == 0)
    {
      NS_LOG_INFO ("TDD Pattern unknown, insert DL CTRL at the beginning of the slot");
      VarTtiAllocInfo dlCtrlSlot (std::make_shared<DciInfoElementTdma> (0, m_dlCtrlSyms,
                                                                        DciInfoElementTdma::DL,
                                                                        DciInfoElementTdma::CTRL, rbgBitmask));
      m_currSlotAllocInfo.m_varTtiAllocInfo.push_front (dlCtrlSlot);
      return;
    }

  uint64_t currentSlotN = currentSfnSf.Normalize () % m_tddPattern.size ();

  if (m_tddPattern[currentSlotN] < LteNrTddSlotType::UL)
    {
      NS_LOG_INFO ("The current TDD pattern indicates that we are in a " <<
                   m_tddPattern[currentSlotN] <<
                   " slot, so insert DL CTRL at the beginning of the slot");
      VarTtiAllocInfo dlCtrlSlot (std::make_shared<DciInfoElementTdma> (0, m_dlCtrlSyms,
                                                                        DciInfoElementTdma::DL,
                                                                        DciInfoElementTdma::CTRL, rbgBitmask));
      m_currSlotAllocInfo.m_varTtiAllocInfo.push_front (dlCtrlSlot);
    }
  if (m_tddPattern[currentSlotN] > LteNrTddSlotType::DL)
    {
      NS_LOG_INFO ("The current TDD pattern indicates that we are in a " <<
                   m_tddPattern[currentSlotN] <<
                   " slot, so insert UL CTRL at the end of the slot");
      VarTtiAllocInfo ulCtrlSlot (std::make_shared<DciInfoElementTdma> (GetSymbolsPerSlot () - m_ulCtrlSyms,
                                                                        m_ulCtrlSyms,
                                                                        DciInfoElementTdma::UL,
                                                                        DciInfoElementTdma::CTRL, rbgBitmask));
      m_currSlotAllocInfo.m_varTtiAllocInfo.push_back (ulCtrlSlot);
    }
}

void
NrUePhy::StartSlot (const SfnSf &s)
{
  NS_LOG_FUNCTION (this);
  m_currentSlot = s;
  m_lastSlotStart = Simulator::Now ();

  // Call MAC before doing anything in PHY
  m_phySapUser->SlotIndication (m_currentSlot);   // trigger mac

  // update the current slot object, and insert DL/UL CTRL allocations depending on the TDD pattern
  if (SlotAllocInfoExists (m_currentSlot))
    {
      m_currSlotAllocInfo = RetrieveSlotAllocInfo (m_currentSlot);
    }
  else
    {
      m_currSlotAllocInfo = SlotAllocInfo (m_currentSlot);
    }

  PushCtrlAllocations (m_currentSlot);

  NS_ASSERT (m_currSlotAllocInfo.m_sfnSf == m_currentSlot);

  NS_LOG_INFO ("UE " << m_rnti << " start slot " << m_currSlotAllocInfo.m_sfnSf <<
               " composed by the following allocations, total " << m_currSlotAllocInfo.m_varTtiAllocInfo.size ());
  for (const auto & alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
      std::string direction, type;
      if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL)
        {
          type = "CTRL";
        }
      else if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL_DATA)
        {
          type = "CTRL_DATA";
        }
      else
        {
          type = "DATA";
        }

      if (alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
          direction = "UL";
        }
      else
        {
          direction = "DL";
        }
      NS_LOG_INFO ("Allocation from sym " << static_cast<uint32_t> (alloc.m_dci->m_symStart) <<
                   " to sym " << static_cast<uint32_t> (alloc.m_dci->m_numSym + alloc.m_dci->m_symStart) <<
                   " direction " << direction << " type " << type);
    }

  TryToPerformLbt ();

  VarTtiAllocInfo allocation = m_currSlotAllocInfo.m_varTtiAllocInfo.front ();
  m_currSlotAllocInfo.m_varTtiAllocInfo.pop_front ();

  auto nextVarTtiStart = GetSymbolPeriod () * allocation.m_dci->m_symStart;


  auto ctrlMsgs = PopCurrentSlotCtrlMsgs ();
  if (m_netDevice)
    {
      DynamicCast<NrUeNetDevice> (m_netDevice)->RouteOutgoingCtrlMsgs (ctrlMsgs, GetBwpId ());
    }
  else
    {
      // No netDevice (that could happen in tests) so just redirect them to us
      for (const auto & msg : ctrlMsgs)
        {
          EncodeCtrlMsg (msg);
        }

    }


  Simulator::Schedule (nextVarTtiStart, &NrUePhy::StartVarTti, this, allocation.m_dci);
}


Time
NrUePhy::DlCtrl(const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  NS_LOG_DEBUG ("UE" << m_rnti <<
                " RXing DL CTRL frame for"
                " symbols "  << +dci->m_symStart <<
                "-" << +(dci->m_symStart + dci->m_numSym - 1) <<
                "\t start " << Simulator::Now () <<
                " end " << (Simulator::Now () + varTtiPeriod));

  m_tryToPerformLbt = true;

  return varTtiPeriod;
}

Time
NrUePhy::UlCtrl (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  if (m_ctrlMsgs.size () == 0)
    {
      NS_LOG_INFO   ("UE" << m_rnti << " reserved space for UL CTRL frame for symbols " <<
                    +dci->m_symStart << "-" <<
                    +(dci->m_symStart + dci->m_numSym - 1) <<
                    "\t start " << Simulator::Now () << " end " <<
                    (Simulator::Now () + varTtiPeriod - NanoSeconds (1.0)) <<
                    " but no data to transmit");
      m_cam->Cancel ();
      return varTtiPeriod;
    }
  else if (m_channelStatus != GRANTED)
    {
      NS_LOG_INFO ("UE" << m_rnti << " has to transmit CTRL but channel not granted");
      m_cam->Cancel ();
      return varTtiPeriod;
    }

  for (const auto & msg : m_ctrlMsgs)
    {
      m_phyTxedCtrlMsgsTrace (m_currentSlot,  GetCellId (), dci->m_rnti, GetBwpId (), msg);

      if (msg->GetMessageType () == NrControlMessage::DL_HARQ)
        {
          Ptr<NrDlHarqFeedbackMessage> harqMsg = DynamicCast<NrDlHarqFeedbackMessage> (msg);
          uint8_t harqId = harqMsg->GetDlHarqFeedback ().m_harqProcessId;

          auto it = m_harqIdToK1Map.find (harqId);
          if (it!=m_harqIdToK1Map.end ())
            {
              m_phyUeTxedHarqFeedbackTrace (m_currentSlot, GetCellId (), m_rnti, GetBwpId (),
                                            static_cast<uint32_t> (harqId), it->second);
            }
        }
    }

  std::vector<int> channelRbs;
  for (uint32_t i = 0; i < GetRbNum (); i++)
    {
      channelRbs.push_back (static_cast<int> (i));
    }

  SetSubChannelsForTransmission (channelRbs, dci->m_numSym);

  NS_LOG_DEBUG ("UE" << m_rnti << " TXing UL CTRL frame for symbols " <<
                +dci->m_symStart << "-" <<
                +(dci->m_symStart + dci->m_numSym - 1) <<
                "\t start " << Simulator::Now () << " end " <<
                (Simulator::Now () + varTtiPeriod - NanoSeconds (1.0)));

  SendCtrlChannels (varTtiPeriod - NanoSeconds (1.0));

  ChannelAccessDenied (); // Reset the channel status
  return varTtiPeriod;
}

Time
NrUePhy::DlData (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);

  m_receptionEnabled = true;
  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;

  m_spectrumPhy->AddExpectedTb (dci->m_rnti, dci->m_ndi, dci->m_tbSize, dci->m_mcs,
                                        FromRBGBitmaskToRBAssignment (dci->m_rbgBitmask),
                                        dci->m_harqProcess, dci->m_rv, true,
                                        dci->m_symStart, dci->m_numSym, m_currentSlot);
  if (!m_IAperformed)
  {
    GetBeamManager()->ChangeBeamformingVector (m_netDevice->GetObject<NrUeNetDevice> ()->GetTargetEnb ());
  }  

  m_reportDlTbSize (m_netDevice->GetObject <NrUeNetDevice> ()->GetImsi (), dci->m_tbSize);
  NS_LOG_DEBUG ("UE" << m_rnti <<
                " RXing DL DATA frame for"
                " symbols "  << +dci->m_symStart <<
                "-" << +(dci->m_symStart + dci->m_numSym - 1) <<
                " num of rbg assigned: " << FromRBGBitmaskToRBAssignment (dci->m_rbgBitmask).size () <<
                "\t start " << Simulator::Now () <<
                " end " << (Simulator::Now () + varTtiPeriod));

  return varTtiPeriod;
}

Time
NrUePhy::UlData(const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);
  SetSubChannelsForTransmission (FromRBGBitmaskToRBAssignment (dci->m_rbgBitmask), dci->m_numSym);
  Time varTtiPeriod = GetSymbolPeriod () * dci->m_numSym;
  std::list<Ptr<NrControlMessage> > ctrlMsg;
  Ptr<PacketBurst> pktBurst = GetPacketBurst (m_currentSlot, dci->m_symStart);
  if (pktBurst && pktBurst->GetNPackets () > 0)
    {
      std::list< Ptr<Packet> > pkts = pktBurst->GetPackets ();
      LteRadioBearerTag bearerTag;
      if (!pkts.front ()->PeekPacketTag (bearerTag))
        {
          NS_FATAL_ERROR ("No radio bearer tag");
        }
    }
  else
    {
      // put an error, as something is wrong. The UE should not be scheduled
      // if there is no data for him...
      NS_FATAL_ERROR ("The UE " << dci->m_rnti << " has been scheduled without data");
    }
  m_reportUlTbSize (m_netDevice->GetObject <NrUeNetDevice> ()->GetImsi (), dci->m_tbSize);

  NS_LOG_DEBUG ("UE" << m_rnti <<
                " TXing UL DATA frame for" <<
                " symbols "  << +dci->m_symStart <<
                "-" << +(dci->m_symStart + dci->m_numSym - 1)
                     << "\t start " << Simulator::Now () <<
                " end " << (Simulator::Now () + varTtiPeriod));

  Simulator::Schedule (NanoSeconds (1.0), &NrUePhy::SendDataChannels, this,
                       pktBurst, ctrlMsg, varTtiPeriod - NanoSeconds (2.0), dci->m_symStart);
  return varTtiPeriod;
}

void
NrUePhy::StartVarTti (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);
  Time varTtiPeriod;

  m_currTbs = dci->m_tbSize;
  m_receptionEnabled = false;

  if (dci->m_type == DciInfoElementTdma::CTRL && dci->m_format == DciInfoElementTdma::DL)
    {
      varTtiPeriod = DlCtrl (dci);
    }
  else if (dci->m_type == DciInfoElementTdma::CTRL && dci->m_format == DciInfoElementTdma::UL)
    {
      varTtiPeriod = UlCtrl (dci);
    }
  else if (dci->m_type == DciInfoElementTdma::DATA && dci->m_format == DciInfoElementTdma::DL)
    {
      varTtiPeriod = DlData (dci);
    }
  else if (dci->m_type == DciInfoElementTdma::DATA && dci->m_format == DciInfoElementTdma::UL)
    {
      varTtiPeriod = UlData (dci);
    }

  Simulator::Schedule (varTtiPeriod, &NrUePhy::EndVarTti, this, dci);
}


void
NrUePhy::EndVarTti (const std::shared_ptr<DciInfoElementTdma> &dci)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("DCI started at symbol " << static_cast<uint32_t> (dci->m_symStart) <<
               " which lasted for " << static_cast<uint32_t> (dci->m_numSym) <<
               " symbols finished");

  if (m_tryToPerformLbt)
    {
      TryToPerformLbt ();
      m_tryToPerformLbt = false;
    }

  if (m_currSlotAllocInfo.m_varTtiAllocInfo.size () == 0)
    {
      // end of slot
      m_currentSlot.Add (1);

      Simulator::Schedule (m_lastSlotStart + GetSlotPeriod () - Simulator::Now (),
                           &NrUePhy::StartSlot, this, m_currentSlot);
    }
  else
    {
      VarTtiAllocInfo allocation = m_currSlotAllocInfo.m_varTtiAllocInfo.front ();
      m_currSlotAllocInfo.m_varTtiAllocInfo.pop_front ();

      Time nextVarTtiStart = GetSymbolPeriod () * allocation.m_dci->m_symStart;

      Simulator::Schedule (nextVarTtiStart + m_lastSlotStart - Simulator::Now (),
                           &NrUePhy::StartVarTti, this, allocation.m_dci);
    }

  m_receptionEnabled = false;
}

void
NrUePhy::PhyDataPacketReceived (const Ptr<Packet> &p)
{
  if (m_conveyPacketsToMac)
  {
    m_recvPhyPduEvent = Simulator::Schedule (GetTbDecodeLatency (),
                                &NrUePhySapUser::ReceivePhyPdu,
                                m_phySapUser,
                                p);
  }  
}

void
NrUePhy::SendDataChannels (const Ptr<PacketBurst> &pb,
                               const std::list<Ptr<NrControlMessage> > &ctrlMsg,
                               const Time &duration, uint8_t slotInd)
{
  if (pb->GetNPackets () > 0)
    {
      LteRadioBearerTag tag;
      if (!pb->GetPackets ().front ()->PeekPacketTag (tag))
        {
          NS_FATAL_ERROR ("No radio bearer tag");
        }
    }
  Ptr<NrUeNetDevice> ueRx = DynamicCast<NrUeNetDevice> (m_netDevice);

  if (!m_IAperformed)
  {
    GetBeamManager ()->ChangeBeamformingVector (ueRx->GetTargetEnb ());
  }  
  
  m_spectrumPhy->StartTxDataFrames (pb, ctrlMsg, duration, slotInd, m_imsi);
}

void
NrUePhy::SendCtrlChannels (Time prd)
{
  m_spectrumPhy->StartTxUlControlFrames (m_ctrlMsgs, prd);
  m_ctrlMsgs.clear ();
}

Ptr<NrDlCqiMessage>
NrUePhy::CreateDlCqiFeedbackMessage (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this);
  SpectrumValue newSinr = sinr;
  // CREATE DlCqiLteControlMessage
  Ptr<NrDlCqiMessage> msg = Create<NrDlCqiMessage> ();
  msg->SetSourceBwp (GetBwpId ());
  DlCqiInfo dlcqi;

  dlcqi.m_rnti = m_rnti;
  dlcqi.m_cqiType = DlCqiInfo::WB;

  std::vector<int> cqi;

  uint8_t mcs;
  dlcqi.m_wbCqi = m_amc->CreateCqiFeedbackWbTdma (newSinr, mcs);

  msg->SetDlCqi (dlcqi);
  return msg;
}

void
NrUePhy::GenerateDlCqiReport (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this);
  // Not totally sure what this is about. We have to check.
  if (m_ulConfigured && (m_rnti > 0) && m_receptionEnabled)
    {
      if (Simulator::Now () > m_wbCqiLast)
        {
          SpectrumValue newSinr = sinr;
          Ptr<NrDlCqiMessage> msg = CreateDlCqiFeedbackMessage (newSinr);

          if (msg)
            {
              DoSendControlMessage (msg);
            }
          m_reportCurrentCellRsrpSinrTrace (GetCellId (), m_rnti, 0.0, ComputeAvgSinr (sinr), GetBwpId ());
        }
    }
}

void
NrUePhy::EnqueueDlHarqFeedback (const DlHarqInfo &m)
{
  NS_LOG_FUNCTION (this);
  // get the feedback from NrSpectrumPhy and send it through ideal PUCCH to gNB
  Ptr<NrDlHarqFeedbackMessage> msg = Create<NrDlHarqFeedbackMessage> ();
  msg->SetSourceBwp (GetBwpId ());
  msg->SetDlHarqFeedback (m);

  auto k1It = m_harqIdToK1Map.find (m.m_harqProcessId);

  NS_LOG_DEBUG ("ReceiveLteDlHarqFeedback" << " Harq Process " <<
                static_cast<uint32_t> (k1It->first) <<
                " K1: " << k1It->second << " Frame " << m_currentSlot);

  Time event = m_lastSlotStart + (GetSlotPeriod () * k1It->second);
  if (event <= Simulator::Now ())
    {
      Simulator::ScheduleNow (&NrUePhy::DoSendControlMessageNow, this, msg);
    }
  else
    {
      Simulator::Schedule (event - Simulator::Now (), &NrUePhy::DoSendControlMessageNow, this, msg);
    }
}

void
NrUePhy::SetCam(const Ptr<NrChAccessManager> &cam)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (cam != nullptr);
  m_cam = cam;
  m_cam->SetAccessGrantedCallback (std::bind (&NrUePhy::ChannelAccessGranted, this,
                                              std::placeholders::_1));
  m_cam->SetAccessDeniedCallback (std::bind (&NrUePhy::ChannelAccessDenied, this));
}

const SfnSf &
NrUePhy::GetCurrentSfnSf () const
{
  return m_currentSlot;
}

uint16_t
NrUePhy::GetRnti ()
{
  return m_rnti;
}

void
NrUePhy::DoReset ()
{
  NS_LOG_FUNCTION (this);
}

void
NrUePhy::DoStartCellSearch (uint16_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << dlEarfcn);
}

void
NrUePhy::DoSynchronizeWithEnb (uint16_t cellId, uint16_t dlEarfcn)
{
  NS_LOG_FUNCTION (this << cellId << dlEarfcn);
  DoSynchronizeWithEnb (cellId);
}

void
NrUePhy::DoSetPa (double pa)
{
  NS_LOG_FUNCTION (this << pa);
}

void
NrUePhy::DoSetRsrpFilterCoefficient (uint8_t rsrpFilterCoefficient)
{
  NS_LOG_FUNCTION (this << +rsrpFilterCoefficient);
}

void
NrUePhy::DoSynchronizeWithEnb (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  DoSetCellId (cellId);
  m_spectrumPhy->SetNoisePowerSpectralDensity (GetNoisePowerSpectralDensity ());

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if (m_registeredEnb.find (cellId) != m_registeredEnb.end())
  {
    RegisterToEnb (m_registeredEnb.find (cellId)->first);
  }
  else
  {
    NS_FATAL_ERROR ("Unknown gNB");
  }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

BeamId
NrUePhy::GetBeamId (uint16_t rnti) const
{
  NS_LOG_FUNCTION (this);
  // That's a bad specification: the UE PHY doesn't know anything about its beam id.
  NS_UNUSED (rnti);
  NS_FATAL_ERROR ("ERROR");
}

void
NrUePhy::ScheduleStartEventLoop (uint32_t nodeId, uint16_t frame, uint8_t subframe, uint16_t slot)
{
  NS_LOG_FUNCTION (this);
  Simulator::ScheduleWithContext (nodeId, MilliSeconds (0), &NrUePhy::StartEventLoop,
                                  this, frame, subframe, slot);
}

void
NrUePhy::ReportUeMeasurements()
{
    NS_LOG_FUNCTION(this);

    // LteUeCphySapUser::UeMeasurementsParameters ret;

    std::map<uint16_t, UeMeasurementsElement>::iterator it;
    for (it = m_ueMeasurementsMap.begin(); it != m_ueMeasurementsMap.end(); it++)
    {
        double avg_rsrp;
        // double avg_rsrq = 0;
        if ((*it).second.rsrpNum != 0)
        {
            avg_rsrp = (*it).second.rsrpSum / static_cast<double>((*it).second.rsrpNum);
        }
        else
        {
            NS_LOG_WARN(" RSRP nSamples is zero!");
            avg_rsrp = 0;
        }

        NS_LOG_DEBUG(" Report UE Measurements for CellId "
                     << (*it).first << " Reporting UE " << m_rnti << " Av. RSRP " << avg_rsrp
                     << " (nSamples " << +((*it).second.rsrpNum) << ")"
                     << " BwpID " << GetBwpId());

        m_reportRsrpTrace(GetCellId(), m_imsi, m_rnti, avg_rsrp, GetBwpId());

        //N Block hier war original auch auskommentiert
        /* LteUeCphySapUser::UeMeasurementsElement newEl;
        newEl.m_cellId = (*it).first;
        newEl.m_rsrp = avg_rsrp;
        newEl.m_rsrq = avg_rsrq;  //LEAVE IT 0 FOR THE MOMENT
        ret.m_ueMeasurementsList.push_back (newEl);
        ret.m_componentCarrierId = GetBwpId ();*/
    }

    // report to RRC
    // m_ueCphySapUser->ReportUeMeasurements (ret);

    m_ueMeasurementsMap.clear();
    Simulator::Schedule(m_ueMeasurementsFilterPeriod, &NrUePhy::ReportUeMeasurements, this);
}

//N neu für DlCtrlSinr Tracing
void
NrUePhy::ReportDlCtrlSinr(const SpectrumValue& sinr, uint8_t streamId)
{
    NS_LOG_FUNCTION(this);
    uint32_t rbUsed = 0;
    double sinrSum = 0.0;

    for (uint32_t i = 0; i < sinr.GetValuesN(); i++)
    {
        double currentSinr = sinr.ValuesAt(i);
        if (currentSinr != 0)
        {
            rbUsed++;
            sinrSum += currentSinr;
        }
    }

    NS_ASSERT(rbUsed);
    m_dlCtrlSinrTrace(GetCellId(), m_rnti, m_imsi, sinrSum / rbUsed, GetBwpId(), streamId);
}

void
NrUePhy::ReceivePss(uint16_t cellId, const Ptr<SpectrumValue>& p)
{
    NS_LOG_FUNCTION(this);

    double sum = 0.0;
    uint16_t nRB = 0;

    uint32_t subcarrierSpacing;
    subcarrierSpacing = 15000 * static_cast<uint32_t>(std::pow(2, GetNumerology()));

    Values::const_iterator itPi;
    for (itPi = p->ConstValuesBegin(); itPi != p->ConstValuesEnd(); itPi++)
    {
        // convert PSD [W/Hz] to linear power [W] for the single RE
        double powerTxW = (*itPi) * subcarrierSpacing;
        sum += powerTxW;
        nRB++;
    }

    // measure instantaneous RSRP now (in dBm)
    double rsrp = 10 * log10(1000 * (sum / static_cast<double>(nRB)));

    NS_LOG_DEBUG("RSRP value updated: " << rsrp << " dBm"
                                        << " for Cell Id: " << cellId << " RNTI: " << m_rnti);

    // store RSRP measurements
    std::map<uint16_t, UeMeasurementsElement>::iterator itMeasMap =
        m_ueMeasurementsMap.find(cellId);
    if (itMeasMap == m_ueMeasurementsMap.end())
    {
        // insert new entry
        UeMeasurementsElement newEl;
        newEl.rsrpSum = rsrp;
        newEl.rsrpNum = 1;
        newEl.rsrqSum = 0;
        newEl.rsrqNum = 0;

        NS_LOG_DEBUG("New RSRP entry for Cell Id: " << cellId << " RNTI: " << m_rnti
                                                    << " RSRP: " << newEl.rsrpSum << " dBm"
                                                    << " number of entries: " << +newEl.rsrpNum);

        m_ueMeasurementsMap.insert(std::pair<uint16_t, UeMeasurementsElement>(cellId, newEl));
    }
    else
    {
        (*itMeasMap).second.rsrpSum += rsrp;
        (*itMeasMap).second.rsrpNum++;

        NS_LOG_DEBUG("Update RSRP entry for Cell Id: "
                     << cellId << " RNTI: " << m_rnti
                     << " RSRP Sum: " << (*itMeasMap).second.rsrpSum << " dBm"
                     << " number of entries: " << +((*itMeasMap).second.rsrpNum));
    }
}

void
NrUePhy::StartEventLoop (uint16_t frame, uint8_t subframe, uint16_t slot)
{
  NS_LOG_FUNCTION (this);
  if (m_realisticIA)
  {
    Simulator::Schedule (GetSlotPeriod () - NanoSeconds (1.0),
                        &NrUePhy::AdjustAntennaForBeamSweep,
                        this);
  }
  SfnSf startSlot (frame, subframe, slot, GetNumerology ());
  StartSlot (startSlot);
  
}

void
NrUePhy::DoSetDlBandwidth (uint16_t dlBandwidth)
{
  NS_LOG_FUNCTION (this << +dlBandwidth);
  if (m_channelBandwidth != dlBandwidth)
    {
      m_channelBandwidth = dlBandwidth;
      UpdateRbNum ();
    }
}


void
NrUePhy::DoConfigureUplink (uint16_t ulEarfcn, uint8_t ulBandwidth)
{
  NS_LOG_FUNCTION (this << ulEarfcn << +ulBandwidth);
  // Ignore this; should be equal to dlBandwidth
  m_ulConfigured = true;
}

void
NrUePhy::DoConfigureReferenceSignalPower (int8_t referenceSignalPower)
{
  NS_LOG_FUNCTION (this << referenceSignalPower);
}

void
NrUePhy::DoSetRnti (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  m_rnti = rnti;
}

void
NrUePhy::DoSetTransmissionMode (uint8_t txMode)
{
  NS_LOG_FUNCTION (this << +txMode);
}

void
NrUePhy::DoSetSrsConfigurationIndex (uint16_t srcCi)
{
  NS_LOG_FUNCTION (this << srcCi);
}

void
NrUePhy::SetPhySapUser (NrUePhySapUser* ptr)
{
  m_phySapUser = ptr;
}

void
NrUePhy::DoResetPhyAfterRlf ()
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("NrUePhy does not have RLF functionality yet");
}

void
NrUePhy::DoResetRlfParams ()
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("NrUePhy does not have RLF functionality yet");
}

void
NrUePhy::DoStartInSnycDetection ()
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("NrUePhy does not have RLF functionality yet");
}

void
NrUePhy::DoSetImsi (uint64_t imsi)
{
  NS_LOG_FUNCTION (this);
  m_imsi = imsi;
}

void
NrUePhy::UpdateSinrEstimate (uint16_t cellId, double sinr)
{
  NS_LOG_FUNCTION (this);
  if (m_cellSinrMap.find (cellId) != m_cellSinrMap.end ()) 
  {
    m_cellSinrMap.find (cellId)->second = sinr;
  }
  else
  {
    m_cellSinrMap.insert (std::pair<uint16_t, double> (cellId, sinr));
  }
  uint16_t currentCellId = GetCellId ();
  if (cellId == currentCellId)
  {
    long double currentCellSinr = 10 * std::log10 (m_cellSinrMap.find (currentCellId)->second);
    m_lastPerceivedSinr = currentCellSinr;

    if (currentCellSinr < m_beamSweepThreshold)
    {
      m_consecutiveSinrBelowThreshold++;
      if (m_consecutiveSinrBelowThreshold > m_n310)
      {
        // TODO raise a call to upper layers
        NS_LOG_DEBUG ("Phy layer detects SNR below threshold for " << m_n310 << " times");

        if (m_resetIdealBeamforming)
        {
          NS_LOG_UNCOND("SINR below more than defined threshold, performing ideal beamforming to all enbs, IMSI:" << m_imsi);
          if (m_adaptiveBF)
          {
            if (!m_IAalreadyTriggered)
            {
              m_IAalreadyTriggered = true;
              m_cellIDSSBMap.clear();
              m_recvCSIMap.clear();
              Simulator::Schedule(MicroSeconds(5), &NrUePhy::RaiseNotifyOutOfSyncNr, this);
              Simulator::Schedule(m_beamSweepTimer.Get(), &NrUePhy::DoSetPhyIAFlag, this, false);
            }
            else
            {
              NS_LOG_UNCOND("IA already triggered, wont trigger until it is completed, IMSI:" << m_imsi);
            }

            //}
          }
        }
      }
    }    
    else
    {
      m_consecutiveSinrBelowThreshold = 0;
    }
    NS_LOG_DEBUG ("Phy layers: update sinr value for cell " << currentCellId << " to " << currentCellSinr << " m_consecutiveSinrBelowThreshold " << (uint16_t)m_consecutiveSinrBelowThreshold << " at time " << Simulator::Now ());    
  }
  else if (currentCellId == 0)
  {
    if (!m_IAalreadyTriggered)
    {
      m_IAalreadyTriggered = true;
      m_cellIDSSBMap.clear ();
      m_recvCSIMap.clear ();
      Simulator::Schedule (MicroSeconds(5), &NrUePhy::RaiseNotifyOutOfSyncNr, this);
      Simulator::Schedule (m_beamSweepTimer.Get(), &NrUePhy::DoSetPhyIAFlag, this, false);
    }
    else
    {
      NS_LOG_UNCOND ("IA already triggered, wont trigger until it is completed");
    }
  }
  
}

void 
NrUePhy::SetPhyIdealBeamformingHelper (Ptr<IdealBeamformingHelper> idealBeamformingHelper)
{
  m_phyIdealBeamformingHelper = idealBeamformingHelper;
}

void 
NrUePhy::ResetIdealBFTimer ()
{
  if (!m_resetIdealBeamforming)
  {
    m_resetIdealBeamforming = true;
  }
}

std::vector<BeamId>
NrUePhy::DoGenerateBeamVectorMap() 
{
  std::vector<BeamId> beamVectorMapUe;
  std::vector<double> elevationDegrees = {90.0 - (m_ueElevationAngleStep / 2.0), 90 + (m_ueElevationAngleStep / 2.0)};
  std::pair<double, double> elevationBeginEnd;
  uint16_t rxNumRows;

  switch (m_antennaConfig)
  {
  case AntennaConfigDefault:
    rxNumRows = GetAntennaArray ()->GetNumOfRowElements ();
    for (double rxTheta = 60.0; rxTheta < 121.0; rxTheta += m_ueElevationAngleStep)
      {
        for (uint16_t rxSector = 0; rxSector <= rxNumRows; rxSector++)
        {
          NS_ASSERT (rxSector < UINT16_MAX);

          beamVectorMapUe.emplace_back (BeamId (rxSector, rxTheta));
        }
      }
      m_ueSectorNumber = (rxNumRows + 1) * (((120.0 - 60.0) / 20.0) + 1);
      break;
    case AntennaConfigInets:
      rxNumRows = (180.0 / m_ueHorizontalAngleStep);
      elevationBeginEnd = {30.0, 90.0};
      
      for (double rxTheta = elevationBeginEnd.first; rxTheta <= elevationBeginEnd.second; rxTheta += m_ueElevationAngleStep)
      {
        for (uint16_t rxSector  = 0; rxSector <= rxNumRows; rxSector++)
        {
          NS_ASSERT (rxSector < UINT16_MAX);

          beamVectorMapUe.emplace_back (BeamId (rxSector, rxTheta));
        }
      }
      m_ueSectorNumber = (rxNumRows + 1) * (((elevationBeginEnd.second - elevationBeginEnd.first) / m_ueElevationAngleStep) + 1);
      break;
  default:
    NS_ABORT_MSG ("Undefined Antenna Configuration");
    break;
  }  

  
  m_ueBeamVectorList = beamVectorMapUe;

  m_beamManager->SetSector (m_ueBeamVectorList.at (0).GetSector (), m_ueBeamVectorList.at(0).GetElevation());

  m_ssbRLMScanDirectionNumber = m_ueSectorNumber;
  m_noOfTxSSBScanDirections = m_gnbSectorNumber;

  return beamVectorMapUe;
}

void 
NrUePhy::ProcessSSBs (Ptr<NrPssMessage> pssMsg)
{
  uint16_t cellId = pssMsg->GetCellId ();
  SfnSf currentSfn = GetCurrentSfnSf ();
  uint64_t imsi = m_netDevice->GetObject<NrUeNetDevice> ()->GetImsi ();

  if (m_IAperformed && pssMsg->GetDestinationImsi () == 0)
  {
    if (m_cellIDSSBMap.find (cellId) == m_cellIDSSBMap.end ())
    {
      if (currentSfn.GetFrame () % (uint8_t)2 == 0 &&
          currentSfn.GetSubframe () == 0 &&
          currentSfn.GetSlot () == 0 &&
          pssMsg->GetSymbolOffset () == 0)
      {
        Ptr<SSBProcessor> ssbProcessor = Create<SSBProcessor> ();
        ssbProcessor->m_cellId = cellId;
        m_cellIDSSBMap.insert(std::pair<uint8_t, Ptr<SSBProcessor>> (cellId, ssbProcessor));
        m_cellIDSSBMap.at (cellId)->SetStartingSfn (GetCurrentSfnSf ());
        m_cellIDSSBMap.at (cellId)->txSectorNumber++;
        m_cellIDSSBMap.at (cellId)->InsertBeamIdSNRPair (1, {1, m_spectrumPhy->GetLastReceivedSNR (cellId)});
        m_txSSBCounterPerRx +=1;
      }  
    }
    else
    {
      const uint16_t txSectorNumber = ++m_cellIDSSBMap.at(cellId)->txSectorNumber;
      const uint8_t rxSectorNumber = m_cellIDSSBMap.at(cellId)->rxSectorNumber;
      m_txSSBCounterPerRx += 1;
      

      if (rxSectorNumber < m_ueSectorNumber)
      {
        if (txSectorNumber <= m_gnbSectorNumber)
          {
            m_cellIDSSBMap.at(cellId)->InsertBeamIdSNRPair (m_cellIDSSBMap.at(cellId)->rxSectorNumber + 1,
              std::pair<uint16_t, double> (txSectorNumber, m_spectrumPhy->GetLastReceivedSNR(cellId)));
            
            if (txSectorNumber == m_gnbSectorNumber)
              {
                BeamId nextRxBeamId;

                if (m_txSSBCounterPerRx >= 10 * m_gnbSectorNumber) // 10 is for the total number of cells
                {
                  NS_LOG_UNCOND (Simulator::Now().GetSeconds() << " IMSI: " << imsi << " Sweep for RX Sector " << m_cellIDSSBMap.at(cellId)->rxSectorNumber + 1 << " is finished");
                  if (rxSectorNumber < m_ueSectorNumber - 1)
                  {
                    nextRxBeamId = m_ueBeamVectorList.at (m_cellIDSSBMap.at (cellId)->rxSectorNumber + 1); 
                    m_beamTbSwept = nextRxBeamId;
                    m_beamManager->SetSector (nextRxBeamId.GetSector (), nextRxBeamId.GetElevation());
                  }
                  else
                  {
                    nextRxBeamId = m_ueBeamVectorList.at (0);
                  }                
                  m_txSSBCounterPerRx = 0;
                }
                m_cellIDSSBMap.at (cellId)->rxSectorNumber++; 
                m_cellIDSSBMap.at (cellId)->txSectorNumber = 0;
              }
          }
      }
      else
      {
        m_cellIDSSBMap.at(cellId)->rxSectorNumber = 0;
        m_cellIDSSBMap.at(cellId)->txSectorNumber = 0;
        m_cellIDSSBMap.at(cellId)->sweepComplete = true;

        CheckIfSweepIsComplete();
      }
    }
  }
  else if (!m_IAperformed && m_ssbRlmOn)
  {
   if (m_ssbRLMProcessorMap.find(cellId) == m_ssbRLMProcessorMap.end())
   {
     if (IsFirstSSBInBurst(currentSfn, pssMsg->GetSymbolOffset()))
     {
       Ptr<SSBProcessor> ssbProcessor = Create<SSBProcessor> ();
       ssbProcessor->m_cellId = cellId;
       m_ssbRLMProcessorMap.insert(std::pair<uint8_t, Ptr<SSBProcessor>> (cellId, ssbProcessor));
       m_ssbRLMProcessorMap.at(cellId)->SetStartingSfn(GetCurrentSfnSf());
       m_ssbRLMProcessorMap.at(cellId)->txSectorNumber++;
       m_ssbRLMProcessorMap.at(cellId)->InsertBeamIdSNRPair(1, {1, m_spectrumPhy->GetLastReceivedSNR(cellId)});
       m_ssbRLMProcessorMap.at(cellId)->m_startingSymbolOffset = pssMsg->GetSymbolOffset();
       m_ssbRLMProcessorMap.at(cellId)->m_rlmStartingRXIndex = m_ssbRMCounter;
     }
   }
   else
   {
     const uint16_t txSectorNumber = ++m_ssbRLMProcessorMap.at(cellId)->txSectorNumber;
     const uint8_t rxSectorNumber = m_ssbRLMProcessorMap.at(cellId)->rxSectorNumber;

     if (rxSectorNumber < m_ssbRLMScanDirectionNumber)
     {
       if (txSectorNumber <= m_noOfTxSSBScanDirections)
       {
         m_ssbRLMProcessorMap.at(cellId)->InsertBeamIdSNRPair(m_ssbRLMProcessorMap.at(cellId)->rxSectorNumber + 1,
                                                std::pair<uint16_t, double>(txSectorNumber, m_spectrumPhy->GetLastReceivedSNR(cellId)));

         if (txSectorNumber == m_noOfTxSSBScanDirections)
         {
           m_ssbRLMProcessorMap.at(cellId)->rxSectorNumber++;
           m_ssbRLMProcessorMap.at(cellId)->txSectorNumber = 0;
           m_ssbRMCounter = m_ssbRLMProcessorMap.at (cellId)->rxSectorNumber;

           if (rxSectorNumber == m_ssbRLMScanDirectionNumber - 1)
           {
             m_ssbRLMProcessorMap.at(cellId)->rxSectorNumber = m_ssbRLMProcessorMap.at(cellId)->txSectorNumber = 0;

             bool completeReportTable = true;

             if (m_ssbRLMProcessorMap.at(cellId)->rxBeamtxSectorSNRMap.size() == m_ssbRLMScanDirectionNumber)
             {
               for (std::map<uint16_t, Ptr<SSBProcessor>>::iterator ssbRLMProcessorIter = m_ssbRLMProcessorMap.begin();
                    ssbRLMProcessorIter != m_ssbRLMProcessorMap.end();
                    ++ssbRLMProcessorIter)
               {
                 for (std::map<uint16_t, std::map<uint16_t, double>>::iterator txBeamIter = ssbRLMProcessorIter->second->rxBeamtxSectorSNRMap.begin();
                      txBeamIter != ssbRLMProcessorIter->second->rxBeamtxSectorSNRMap.end();
                      ++txBeamIter)
                 {
                   if (txBeamIter->second.size() != m_noOfTxSSBScanDirections)
                   {
                     completeReportTable = false;
                   }
                   break;
                 }
               }
             }
             else
             {
               completeReportTable = false;
             }

             if (completeReportTable)
             {
               std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> gNBOptimalBeamMap;
               std::vector<BeamId> newRlmBeamVector;

               for (auto cellIndex = 1; cellIndex <= 10; cellIndex++)
               {
                 Ptr<SSBProcessor> ssbProcessor = m_ssbRLMProcessorMap.at(cellIndex);
                 std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> listOfRLMBeams;
                 for (auto n = 0; n < m_noOfBeamsTbRLM; n++)
                 {
                   FindMaximumSNR(ssbProcessor, m_ssbRLMScanDirectionNumber, m_gnbSectorNumber, cellId, false);
                   listOfRLMBeams.emplace_back(std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>(std::pair<SfnSf, uint16_t>(ssbProcessor->m_startingSfn, ssbProcessor->m_maxTxRxPairForCell.second.first),
                                                                                                                std::pair<double, BeamId>(ssbProcessor->m_maxSNRPerCell, ssbProcessor->m_maxTxRxPairForCell.second.second)));
                   newRlmBeamVector.emplace_back(ssbProcessor->m_maxTxRxPairForCell.second.second);
                 }

                 gNBOptimalBeamMap.insert(std::pair<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>>(
                     cellIndex, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>(
                                    listOfRLMBeams)));
               }

               std::vector<struct OptimalRLMBeamStruct> bestBeams = RetrieveOptimalRlmBeams(gNBOptimalBeamMap, GetCellId());

               std::vector<std::pair<uint8_t, uint16_t>> optimalBeamIndex;
               std::map<uint8_t, SfnSf> mapOfStartingSfn;
               std::vector<std::pair<uint8_t, BeamId>> tmp_beamsTbRLM;

               for (uint8_t n = 0; n < m_noOfBeamsTbRLM; n++)
               {
                 optimalBeamIndex.emplace_back(std::make_pair(bestBeams.at(n).cellId, bestBeams.at(n).optimalBeamIndex));
                 mapOfStartingSfn.insert({bestBeams.at(n).cellId, bestBeams.at(n).startingSfnSf});

                 tmp_beamsTbRLM.push_back(std::make_pair(bestBeams.at(n).cellId, bestBeams.at(n).beamId));
               }

               if (m_minCSIRSFromServingGnb < 4)
               {
                 // check if a previously monitored gNB (not the serving one) is no longer being monitored
                 // in that case, we need to signal that to that gNB
                 for (uint8_t i = 0; i < m_noOfBeamsTbRLM; i++)
                 {
                   const uint8_t oldCellId = m_beamsTbRLM.at(m_imsi).at(i).first;
                   auto it = std::find_if(optimalBeamIndex.begin(), optimalBeamIndex.end(), [&oldCellId](const std::pair<uint8_t, uint16_t> &p) { return p.first == oldCellId; });
                   // m_beamsTbRLM contains the old monitored gNBs, optimalBeamIndex only the new gNBs
                   if (it == optimalBeamIndex.end())
                   {
                     optimalBeamIndex.emplace_back(std::make_pair(oldCellId, 0));
                     mapOfStartingSfn.insert({oldCellId, SfnSf (0, 0, 0, 0)});
                   }
                 }
               }
               m_beamsTbRLM.erase(m_imsi);
               m_beamsTbRLM.insert({m_imsi, tmp_beamsTbRLM});
               m_recvCSIMap.clear();
               m_subOptimalCSIBeamMap.clear ();

               RadioLinkMonitoringTraceParams rlmParams;
               rlmParams.imsi = m_imsi;
               rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::UE_PERIODIC_SWEEP;
               rlmParams.beamsTbRLM = m_beamsTbRLM[m_imsi];
               m_radioLinkMonitoringTrace(rlmParams);

               NS_LOG_UNCOND("CSI-RS-RLM for IMSI: " << m_imsi << " , monitor beams from cell: " 
                              << std::to_string(optimalBeamIndex[0].first) << ", " 
                              << std::to_string(optimalBeamIndex[1].first) << ", " 
                              << std::to_string(optimalBeamIndex[2].first) << ", " 
                              << std::to_string(optimalBeamIndex[3].first));
               m_ueCphySapUser->SendSSBRSReport(optimalBeamIndex, mapOfStartingSfn, GetCellId(), m_noOfBeamsTbReported);
             }

             ResetSSBRLMProcessor();
             m_ssbRMCounter = 0;
           }
         }
       }
     }
     else
     {
       ResetSSBRLMProcessor();
       m_ssbRMCounter = 0;
     }
   }
  }
}

void
NrUePhy::RegisterToRxSpectrum ()
{
  Ptr<SpectrumValue> noisePsd = GetNoisePowerSpectralDensity ();
  m_spectrumPhy->SetNoisePowerSpectralDensity (noisePsd);
}

void 
SSBProcessor::InsertBeamIdSNRPair (uint16_t rxSectorNumber, std::pair<uint16_t, double> txSectorSNRPair)
{
  if (rxBeamtxSectorSNRMap.find(rxSectorNumber) == rxBeamtxSectorSNRMap.end()) 
  {
    std::map<uint16_t,double> rxSectorSpecificVector;
    rxSectorSpecificVector.insert({txSectorSNRPair.first, txSectorSNRPair.second});
    rxBeamtxSectorSNRMap.insert ({rxSectorNumber, rxSectorSpecificVector});
  }
  else
  {
    rxBeamtxSectorSNRMap.at(rxSectorNumber).insert({txSectorSNRPair.first, txSectorSNRPair.second});
  }  
}

void 
SSBProcessor::SetStartingSfn (SfnSf startingSfn)
{
  m_startingSfn = startingSfn;
}

void 
NrUePhy::CheckIfSweepIsComplete()
{
  for (std::map<uint8_t, Ptr<SSBProcessor>>::iterator cellIterator = m_cellIDSSBMap.begin(); cellIterator != m_cellIDSSBMap.end(); ++cellIterator)
  {
    if (!cellIterator->second->sweepComplete)
    {
      return;
    }
  }

  m_txSSBCounterPerRx = 0;

  NS_LOG_UNCOND ("IA Sweep is completed at UE (IMSI " << m_imsi << ") side, best beam will be selected");
  std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>>
                cellOptimalBeamMap = RetrieveCellOptimalMap ();

  m_IAperformed = false;
  m_conveyPacketsToMac = true;
  m_spectrumPhy->SetIAState (false);
  m_IAalreadyTriggered = false;
  m_ssbRMCounter = 0;
  
  m_ueCphySapUser->SendOptimalBeamMapToLteCoordinator (cellOptimalBeamMap);
  std::pair<uint8_t, BeamId> m_lastOptimalCellBeamPair = RetrieveOptimalGnbFromMap (cellOptimalBeamMap);

  if (m_minCSIRSFromServingGnb == 4)
  {
    // delete all entries which are not from the best SINR cell -> monitor only beams from the best gNB
    if (m_lastOptimalCellBeamPair.first == 1)
    {
      m_beamsTbRLM.at(m_imsi).erase(m_beamsTbRLM.at(m_imsi).begin() + m_noOfBeamsTbRLM,
                                    m_beamsTbRLM.at(m_imsi).begin() + m_beamsTbRLM.at(m_imsi).size());
    }
    else if (m_lastOptimalCellBeamPair.first == 10)
    {
      m_beamsTbRLM.at(m_imsi).erase(m_beamsTbRLM.at(m_imsi).begin(), m_beamsTbRLM.at(m_imsi).begin() + 9 * m_noOfBeamsTbRLM);
    }
    else
    {
      m_beamsTbRLM.at(m_imsi).erase(m_beamsTbRLM.at(m_imsi).begin() + m_lastOptimalCellBeamPair.first * m_noOfBeamsTbRLM,
                                    m_beamsTbRLM.at(m_imsi).begin() + m_beamsTbRLM.at(m_imsi).size());
      m_beamsTbRLM.at(m_imsi).erase(m_beamsTbRLM.at(m_imsi).begin(), m_beamsTbRLM.at(m_imsi).begin() +
                                                                         (m_lastOptimalCellBeamPair.first - 1) * m_noOfBeamsTbRLM);
    }
  }
  else
  {
    // montior beams from different cells
    std::vector<struct OptimalRLMBeamStruct> bestBeams = RetrieveOptimalRlmBeams(cellOptimalBeamMap, m_lastOptimalCellBeamPair.first);
    std::vector<std::pair<uint8_t, BeamId>> tmp_beamsTbRLM;
    for (uint8_t n = 0; n < m_noOfBeamsTbRLM; n++)
    {
      tmp_beamsTbRLM.push_back(std::make_pair(bestBeams.at(n).cellId, bestBeams.at(n).beamId));
    }

    m_beamsTbRLM.erase(m_imsi);
    m_beamsTbRLM.insert({m_imsi, tmp_beamsTbRLM});
  }

  RadioLinkMonitoringTraceParams rlmParams;
  rlmParams.imsi = m_imsi;
  rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::UE_EXPLICIT_SWEEP;
  rlmParams.beamsTbRLM = m_beamsTbRLM[m_imsi];
  m_radioLinkMonitoringTrace(rlmParams);

  m_ueSSBRLMVectorList = m_ueBeamVectorList;

  for (size_t n = 0; n < m_beamsTbRLM.at(m_imsi).size (); n++)
  {
    if (std::find (m_ueSSBRLMVectorList.begin (), 
                  m_ueSSBRLMVectorList.end (), 
                  m_beamsTbRLM.at(m_imsi).at(n).second) != m_ueSSBRLMVectorList.end ())
    {
      m_ueSSBRLMVectorList.erase (std::remove(m_ueSSBRLMVectorList.begin(),
                                              m_ueSSBRLMVectorList.end (),
                                              m_beamsTbRLM.at(m_imsi).at(n).second),
                                              m_ueSSBRLMVectorList.end());
    }
  }

  m_currBeamformingVector = BeamformingVector (
      CreateDirectionalBfv (m_beamManager->GetAntennaArray (), m_lastOptimalCellBeamPair.second.GetSector (),
      m_lastOptimalCellBeamPair.second.GetElevation ()), m_lastOptimalCellBeamPair.second);

  BeamSweepTraceParams params;
  params.imsi = m_imsi;
  params.m_beamSweepOrigin = BeamSweepTraceParams::UE_COMPLETED_BEAM_SWEEP;
  params.currentCell = GetCellId ();
  params.foundCell = m_lastOptimalCellBeamPair.first;
  params.foundSector = m_lastOptimalCellBeamPair.second.GetSector ();
  params.foundElevation = m_lastOptimalCellBeamPair.second.GetElevation ();
  m_beamSweepTrace(params);
  
  std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>>::iterator cellOptimalBeamMapIt;
  for (cellOptimalBeamMapIt = cellOptimalBeamMap.begin (); cellOptimalBeamMapIt != cellOptimalBeamMap.end (); ++cellOptimalBeamMapIt)
  {
    if (m_registeredEnb.find (cellOptimalBeamMapIt->first) != m_registeredEnb.end ())
    {
      BeamformingVector bfvToGnb = BeamformingVector (
          CreateDirectionalBfv (m_beamManager->GetAntennaArray (),
                                cellOptimalBeamMapIt->second.at(0).second.second.GetSector (),
                                cellOptimalBeamMapIt->second.at(0).second.second.GetElevation ()),
                                cellOptimalBeamMapIt->second.at(0).second.second); 
      
      m_beamManager->SaveBeamformingVector (bfvToGnb, m_registeredEnb.find(cellOptimalBeamMapIt->first)->second);
    }
  }

  if (m_ueCphySapUser->IsRrcIdleStart())
  {
    // same condition thats tested in LTE coordinator for handovers
    if (10 * log10(cellOptimalBeamMap.at(m_lastOptimalCellBeamPair.first).at(0).second.first) > 5)
    {
      m_beamManager->SetSector(m_lastOptimalCellBeamPair.second.GetSector(),
                              m_lastOptimalCellBeamPair.second.GetElevation());

      ReEstablishConnectionWithCell(m_lastOptimalCellBeamPair.first);

      RadioLinkMonitoringTraceParams rlmParams;
      rlmParams.imsi = m_imsi;
      rlmParams.m_radioLinkMonitoringOrigin = RadioLinkMonitoringTraceParams::HANDOVER_AFTER_UE_IA_SWEEP;
      rlmParams.targetCellId = m_lastOptimalCellBeamPair.first;
      m_radioLinkMonitoringTrace(rlmParams);
    }
    else
    {
      // new IA will be started
      Simulator::Schedule(MilliSeconds(10), &NrUePhy::SetIAStateOfAllGnbs, this, false);
    }
  }
  else if (m_lastOptimalCellBeamPair.first != GetCellId())
  {
    // A BeamTracking sweep has been conducted and the best cell is not the currently serving one.
    // Keep the gNBs in IA state. Handover is scheduled via LTE coordinator.
  }
  else
  {
    Simulator::Schedule(MilliSeconds(10), &NrUePhy::SetIAStateOfAllGnbs, this, false);
  }
}

std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>>
NrUePhy::RetrieveCellOptimalMap ()
{
  auto txSectorNumber = m_gnbSectorNumber;
  auto rxSectorNumber = m_ueBeamVectorList.size();

  std::map<uint8_t, Ptr<SSBProcessor>>::iterator cellIteratorFirst = m_cellIDSSBMap.begin();
  NS_ASSERT_MSG (cellIteratorFirst != m_cellIDSSBMap.end(), "SSB Error, Map empty");
  Ptr<SSBProcessor> ssbProcessorFirst = cellIteratorFirst->second;


  if (ssbProcessorFirst->rxBeamtxSectorSNRMap.size() != 0)
  {
    rxSectorNumber = ssbProcessorFirst->rxBeamtxSectorSNRMap.at(1).size ();
  }

  m_cellToSNRAvgMap.clear ();

  std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> gNBOptimalBeamMap;

  m_beamsTbRLM.erase(m_imsi);
  m_beamsTbRLM.insert ({m_imsi, {}});
  
  for (std::map<uint8_t, Ptr<SSBProcessor>>::iterator cellIterator = m_cellIDSSBMap.begin(); cellIterator != m_cellIDSSBMap.end(); ++cellIterator)
  {
    Ptr<SSBProcessor> ssbProcessor = cellIterator->second;
    std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> listOfRLMBeams;
    //std::cout << "m_noOfBeamsTbRLM 3: " << std::to_string(m_noOfBeamsTbRLM) << std::endl;
    for (auto rlmIndex = 0; rlmIndex < m_noOfBeamsTbRLM; rlmIndex++)
    {
      FindMaximumSNR (ssbProcessor, rxSectorNumber, txSectorNumber, cellIterator->first, true);
      listOfRLMBeams.emplace_back (std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>(std::pair<SfnSf, uint16_t> (ssbProcessor->m_startingSfn, ssbProcessor->m_maxTxRxPairForCell.second.first),
                                    std::pair<double, BeamId> (ssbProcessor->m_maxSNRPerCell, ssbProcessor->m_maxTxRxPairForCell.second.second)));
      m_beamsTbRLM.at(m_imsi).emplace_back (std::make_pair(cellIterator->first, ssbProcessor->m_maxTxRxPairForCell.second.second));
    }

    gNBOptimalBeamMap.insert (std::pair<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> (
                              cellIterator->first, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> (
                                listOfRLMBeams
                              )));
  }
  return gNBOptimalBeamMap;
}

std::pair<uint8_t, BeamId>
NrUePhy::RetrieveOptimalGnbFromMap (std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> cellOptimalGnbMap)
{
  uint8_t maxSnrCell;
  auto currSNR = 0.0;
  auto maxSNR = 0.0;
  
  for (auto const& iter : cellOptimalGnbMap)
  {
    currSNR = iter.second.at(0).second.first;
    if (currSNR > maxSNR)
    {
      maxSNR = currSNR;
      maxSnrCell = iter.first;
    }
  }

  return std::pair<uint8_t, BeamId> (maxSnrCell, cellOptimalGnbMap[maxSnrCell].at(0).second.second);
}

std::vector<NrUePhy::OptimalRLMBeamStruct>
NrUePhy::RetrieveOptimalRlmBeams (std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> cellOptimalGnbMap, uint8_t cellId)
{
  // identify best beams for CSI-RS RLM and delete remaining ones from m_beamsTbRLM
  // cellId:  in the case of a beam sweep, this specifies the best available gNB. Otherwise it it the currently serving gNB
  std::vector<struct OptimalRLMBeamStruct> bestServingBeams;
  std::vector<struct OptimalRLMBeamStruct> bestBeams;

  NS_ASSERT(cellId != 0);

  for (auto const &iter : cellOptimalGnbMap)
    {
      if (iter.first != cellId)
      {
        double SNR = iter.second.at(0).second.first;
        if (10 * std::log10(SNR) > m_maxRateThreshold) // only enable monitoring if SNR from that cell is good enough
        {
          // only monitor the best beam from each potential cell
          struct OptimalRLMBeamStruct thisRLMBeam;
          thisRLMBeam.snr = iter.second.at(0).second.first;
          thisRLMBeam.cellId = iter.first;
          thisRLMBeam.beamId = iter.second.at(0).second.second;
          thisRLMBeam.startingSfnSf = iter.second.at(0).first.first;
          thisRLMBeam.optimalBeamIndex = iter.second.at(0).first.second;

          bestBeams.push_back(thisRLMBeam);
        }
      }
    }

    std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> it = cellOptimalGnbMap.at(cellId);
    for (uint8_t i = 0; i < m_noOfBeamsTbRLM; i++)
    {
      struct OptimalRLMBeamStruct thisRLMBeam;
      thisRLMBeam.snr = it.at(i).second.first;
      thisRLMBeam.cellId = cellId;
      thisRLMBeam.beamId = it.at(i).second.second;
      thisRLMBeam.startingSfnSf = it.at(i).first.first;
      thisRLMBeam.optimalBeamIndex = it.at(i).first.second;

      bestServingBeams.push_back(thisRLMBeam);
      if (m_minCSIRSFromServingGnb == 0 && i == 0 && ((uint8_t)bestBeams.size() >= m_noOfBeamsTbRLM))
      {
        // None of the serving gNB beams is required to be monitored. Insert the best one from this cell into the bestBeams struct.
        // Third if statement:  do not insert best beam from serving cell if bestBeams.size() >= m_noOfBeamsTbRLM.
        //                      This prevents double insertion of that beam as remaining free slots in bestBeams are filled with beams from bestServingBeams.
        bestBeams.push_back(thisRLMBeam);
      }
    }

  sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b) {return a.snr > b.snr;});
  
  if (m_minCSIRSFromServingGnb == 0)
  {
    // no beam monitoring required for the serving gNB
    if ((uint8_t) bestBeams.size() < m_noOfBeamsTbRLM)
    {
      // fill in remaining slots if some are unused
      bestBeams.insert(bestBeams.end(), bestServingBeams.begin(), bestServingBeams.end() - bestBeams.size());
      sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b) {return a.snr > b.snr;});
    }
    else if ((uint8_t) bestBeams.size() > m_noOfBeamsTbRLM)
    {
      bestBeams.erase(bestBeams.begin() + m_noOfBeamsTbRLM, bestBeams.end());
    }
    return bestBeams;
  }
  else if (m_minCSIRSFromServingGnb == m_noOfBeamsTbRLM)
  {
    // all beams are from the serving gNB
    return bestServingBeams;
  }
  else
  {
    if ((uint8_t) bestBeams.size() > m_noOfBeamsTbRLM - m_minCSIRSFromServingGnb)
    {
      bestBeams.erase(bestBeams.begin() + (m_noOfBeamsTbRLM - m_minCSIRSFromServingGnb), bestBeams.end());
    }
    bestBeams.insert(bestBeams.end(), bestServingBeams.begin(), bestServingBeams.end() - bestBeams.size());
    sort(bestBeams.begin(), bestBeams.end(), [](struct OptimalRLMBeamStruct &a, struct OptimalRLMBeamStruct &b) {return a.snr > b.snr;});

    return bestBeams;
  }
}

BeamId
NrUePhy::RetrieveNextRxBeamId (BeamId prevRxBeamId)
{
  std::vector<BeamId>::iterator cIndexatUeBVList = std::find(m_ueBeamVectorList.begin(), 
                                            m_ueBeamVectorList.end(),
                                            prevRxBeamId);
          std::advance(cIndexatUeBVList, 1);
  return *cIndexatUeBVList;
}


SSBProcessor::SSBProcessor ()
{
  m_pssReceived = m_firstPBCHReceived = m_sssReceived = false;
}

SSBProcessor::~SSBProcessor ()
{

}

const Ptr<NetDevice>
NrUePhy::DoGetDevice ()
{
  return DynamicCast<NetDevice> (m_netDevice);
}

void 
NrUePhy::SetPHYEpcHelper (Ptr<EpcHelper> epcHelper)
{
  m_phyEpcHelper = epcHelper;
}

void
NrUePhy::DoStartBeamSweep (NrPhy::BeamSweepType beamSweepType)
{
  if (!m_IAperformed)
  {
    if (m_realisticIA)
    {
      m_beamManager->SetSector (0, 70);
      m_beamTbSwept = BeamId (0, 70);
      SetIAStateOfAllGnbs (true);
      switch (beamSweepType)
      {
      case BeamSweepType::IASweep:
        // RLF should have occured, do not forward packets to MAC layer
        m_conveyPacketsToMac = false;
        m_IAperformed = true;
        m_spectrumPhy->SetIAState (true);
        m_cellIDSSBMap.clear ();  
        m_recvCSIMap.clear ();
        m_ssbRLMProcessorMap.clear();
        m_txSSBCounterPerRx = 0;
        //m_ueCphySapUser->SendUeDeregisterToGnb ();
        //DoSetCellId (0);
        break;
        
      case BeamSweepType::BeamTracking:
        // Called from LteUeRrc::DoRecvUeDeRegistrationUpdate when sweep is necessary but no RLF occurs
        // forward packets to MAC layer
        m_conveyPacketsToMac = true;
        m_IAperformed = true;
        m_spectrumPhy->SetIAState (true);
        m_cellIDSSBMap.clear ();  
        m_recvCSIMap.clear ();
        m_ssbRLMProcessorMap.clear();
        m_txSSBCounterPerRx = 0;
        //m_ueCphySapUser->SendUeDeregisterToGnb ();
        //DoSetCellId (0);
        
        break;
      default:
        break;
      }
    }
    else
    {
      m_conveyPacketsToMac = false;
      m_IAperformed = true;
      SetIAStateOfAllGnbs (true);
      BeamId prevBeamId, newBeamId, prevBeamIdEnb, newBeamIdEnb;
      m_tempBeamIdStorage.clear ();
      m_cellIdealSNRMap.clear ();
      
      Ptr<NrUeNetDevice> ueNetDev = m_netDevice->GetObject<NrUeNetDevice> ();

      for (std::map<uint16_t, Ptr<NrGnbNetDevice>>::iterator enbIt = m_registeredEnb.begin (); enbIt != m_registeredEnb.end(); ++enbIt)
      {
        prevBeamId = m_beamManager->GetBeamId (enbIt->second);
        prevBeamIdEnb = enbIt->second->GetPhy(0)->GetBeamManager ()->GetBeamId (ueNetDev);
        m_phyIdealBeamformingHelper->AddBeamformingTask (enbIt->second, ueNetDev);
        newBeamId = m_beamManager->GetBeamId (enbIt->second);
        newBeamIdEnb = enbIt->second->GetPhy(0)->GetBeamManager ()->GetBeamId (ueNetDev);
        auto tempBfv = BeamformingVector (CreateDirectionalBfv (m_beamManager->GetAntennaArray (),
              prevBeamId.GetSector (), prevBeamId.GetElevation ()), prevBeamId);
        auto tempBfvEnb = BeamformingVector (CreateDirectionalBfv (enbIt->second->GetPhy (0)->GetBeamManager ()->GetAntennaArray (),
              prevBeamIdEnb.GetSector (), prevBeamIdEnb.GetElevation ()), prevBeamIdEnb);
        m_beamManager->SaveBeamformingVector (tempBfv, enbIt->second);
        m_beamManager->ChangeBeamformingVector (enbIt->second);

        enbIt->second->GetPhy(0)->GetBeamManager ()->SaveBeamformingVector (tempBfvEnb, ueNetDev);
        enbIt->second->GetPhy(0)->GetBeamManager ()->ChangeBeamformingVector (ueNetDev);

        m_tempBeamIdStorage.insert (std::pair <uint8_t, BeamId> (enbIt->first, newBeamId));
        m_tempBeamIdStorageEnb.insert (std::pair<uint8_t, BeamId> (enbIt->first, newBeamIdEnb));
      }
      if (m_completeSSBurstDuration.GetMilliSeconds () == 0)
      {
        Simulator::Schedule (MilliSeconds(20), &NrUePhy::FinishIdealBeamforming, this);
      }
      else
      {
        Simulator::Schedule (m_completeSSBurstDuration * m_ueSectorNumber, &NrUePhy::FinishIdealBeamforming, this);
      }
      
    }        
  }
  else
  {
    NS_LOG_UNCOND ("A beam sweep has already been started, wait for it to complete");
  }  
}

void 
NrUePhy::RaiseNotifyOutOfSyncNr ()
{
  m_ueCphySapUser->NotifyOutOfSyncNr ();
}

void 
NrUePhy::SetIAStateOfAllGnbs (bool iaState)
{
  for (std::map<uint16_t, Ptr<NrGnbNetDevice>>::iterator enbIt = m_registeredEnb.begin (); enbIt != m_registeredEnb.end(); ++enbIt)
    {
      enbIt->second->GetPhy(0)->SetGnbIAState (iaState);
    }
}

void 
NrUePhy::NotifyConnectionSuccessful ()
{
  NS_LOG_UNCOND("Set IA State FALSE -> from NotifyConnectionSuccesful from UE: " << m_imsi);
  SetIAStateOfAllGnbs (false);
  m_IAalreadyTriggered = false;
  m_ueCphySapUser->ClearHandoverEventsAtCoordinator ();
}

void
NrUePhy::DoSetPhyIAFlag (bool iaState)
{
  m_IAalreadyTriggered = iaState;
}

void
NrUePhy::DoCancelPendingReceivePduEvents ()
{
  if (m_recvPhyPduEvent.IsRunning ())
  {
    m_recvPhyPduEvent.Cancel ();
  }
}

void 
NrUePhy::AdjustAntennaForBeamSweep ()
{
  SfnSf currentSfn = GetCurrentSfnSf ();
  auto currFrameNum = currentSfn.GetFrame ();
  auto currSubFrameNUm = currentSfn.GetSubframe ();
  auto currSlotNum = currentSfn.GetSlot ();

  if (m_IAperformed)
  {
    if (currFrameNum == 0 || currFrameNum % (uint16_t)2 == 0)
    {
      if (currSubFrameNUm == 0 || currSubFrameNUm == 1 || currSubFrameNUm == 2 || currSubFrameNUm == 3)
      {
        m_beamManager->SetSector (m_beamTbSwept.GetSector (), m_beamTbSwept.GetElevation ());
      }
    }
  }
  else
  {
    if (currFrameNum % (uint16_t)2 == 0 && m_ssbRlmOn)
    {
      if (currSubFrameNUm == 0 || currSubFrameNUm == 1 || currSubFrameNUm == 2 || currSubFrameNUm == 3)
      {
        BeamId ssbRLMBeam;

        if (m_ssbRMCounter > m_ueBeamVectorList.size() - 1) // Something has gone wrong, delete the content of m_ssbRLMProcessorMap
        {
          ResetSSBRLMProcessor();
          m_ssbRMCounter = 0;
        }
        else
        {
          ssbRLMBeam = m_ueBeamVectorList.at(m_ssbRMCounter);
        }

        if (ssbRLMBeam != BeamId (0,0))
        {
          m_beamManager->SetSector (ssbRLMBeam.GetSector (), ssbRLMBeam.GetElevation ());
        }
      }
    }

    if (m_beamsTbRLM.size() != 0) // fallback for strategy 1a, skip following section
    {
      if (m_beamsTbRLM.at(m_imsi).size () != 0 && m_csiRSResourcesTbRLM.size () != 0 && m_rlmOn)
      {
        auto frameRemainder = currFrameNum % (uint16_t)2;
        if (m_csiRSResourcesTbRLM.find(frameRemainder) != m_csiRSResourcesTbRLM.end())
        {
          auto csiForSpecFrame = m_csiRSResourcesTbRLM.at(frameRemainder).size();
          for (size_t n = 0; n < csiForSpecFrame; n++)
          {
            if (currSubFrameNUm == m_csiRSResourcesTbRLM.at(frameRemainder).at(n).first &&
                currSlotNum == m_csiRSResourcesTbRLM.at(frameRemainder).at(n).second)
            {
              auto rlmBeamIndex = m_csiRSResourcesTbRLM.at(frameRemainder).size() * frameRemainder + n;
              rlmBeamIndex %= (uint8_t)m_noOfBeamsTbRLM;

              m_beamManager->SetSector(m_beamsTbRLM.at(m_imsi).at(rlmBeamIndex).second.GetSector(),
                                       m_beamsTbRLM.at(m_imsi).at(rlmBeamIndex).second.GetElevation());
              m_csiRXBeam = m_beamsTbRLM.at(m_imsi).at(rlmBeamIndex).second;
              Simulator::Schedule(GetSymbolPeriod() * 2 + NanoSeconds(1.0),
                                  &BeamManager::ChangeBeamformingVector,
                                  m_beamManager,
                                  m_registeredEnb.at(m_beamsTbRLM.at(m_imsi).at(rlmBeamIndex).first));
            }
          }
        }
      }
    }    
  }
}

double 
NrUePhy::DoGetLastPerceivedSinr ()
{
  return m_lastPerceivedSinr;
}

void 
NrUePhy::DoSetInitialIAState (bool initialIAState)
{
  m_IAperformed = initialIAState;
  SetIAStateOfAllGnbs (false);
}

void
NrUePhy::SetIdealSNRForGnb (uint8_t cellId, double snr, BeamId currBeamId)
{
  if (m_cellIdealSNRMap.find (cellId) != m_cellIdealSNRMap.end ())
  {
    m_cellIdealSNRMap.find (cellId)->second = std::pair<double, BeamId> (snr, currBeamId);
  }
  else
  {
    m_cellIdealSNRMap.insert (std::pair<uint8_t, std::pair<double, BeamId>> (cellId, {snr, currBeamId}));
  }  
}

void
NrUePhy::ReEstablishConnectionWithCell (uint8_t cellId)
{
  if (GetCellId () == 0 && m_phyEpcHelper != nullptr)
  {
    m_phyEpcHelper->ActivateEpsBearer (m_netDevice, m_imsi, EpcTft::Default (), EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
  }

  RegisterToEnb (cellId);
  m_ueCphySapUser->SetRrcTempCellID (cellId);
  m_phySapUser->NotifyMacForRA (GetSlotPeriod () * 2);
  m_ueCphySapUser->ClearHandoverEventsAtCoordinator ();
}

void 
NrUePhy::FinishIdealBeamforming ()
{  
  if (m_cellIdealSNRMap.size () == 10)
  {
    for (std::map<uint16_t, Ptr<NrGnbNetDevice>>::iterator enbIt = m_registeredEnb.begin (); enbIt != m_registeredEnb.end(); ++enbIt)
      {
        auto newBeamId = m_cellIdealSNRMap.at (enbIt->first).second;
        auto newBeamIdEnb = m_tempBeamIdStorageEnb.at (enbIt->first);

        auto tempBfv = BeamformingVector (CreateDirectionalBfv (m_beamManager->GetAntennaArray (),
            newBeamId.GetSector (), newBeamId.GetElevation ()), newBeamId);
        auto tempBfvEnb = BeamformingVector (CreateDirectionalBfv (enbIt->second->GetPhy(0)->GetBeamManager ()->GetAntennaArray (),
            newBeamIdEnb.GetSector (), newBeamIdEnb.GetElevation ()), newBeamIdEnb);

        m_beamManager->SaveBeamformingVector (tempBfv, enbIt->second);
        m_beamManager->ChangeBeamformingVector (enbIt->second);

        enbIt->second->GetPhy (0)->GetBeamManager ()->SaveBeamformingVector (tempBfvEnb, m_netDevice->GetObject<NrUeNetDevice> ());
        enbIt->second->GetPhy (0)->GetBeamManager ()->ChangeBeamformingVector (m_netDevice->GetObject<NrUeNetDevice> ());
      }

      m_IAperformed = false;
      m_conveyPacketsToMac = true;
      m_spectrumPhy->SetIAState (false);
      m_IAalreadyTriggered = false;

      std::map<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> 
                    cellOptimalBeamMap;
      double maxSNR = 0;
      double currSNR = 0;
      uint8_t cellId = 0, maxCellId = 0;

      for (auto iterator = m_cellIdealSNRMap.begin (); iterator != m_cellIdealSNRMap.end (); ++iterator)
      {
        cellId = iterator->first;
        currSNR = iterator->second.first;
        std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> listOfRLMBeams;

        for (auto it = 0; it < m_noOfBeamsTbRLM; it++)
        {
          listOfRLMBeams.emplace_back (std::pair<SfnSf, uint16_t> (GetCurrentSfnSf (), 0), 
                                      std::pair<double, BeamId> (currSNR, BeamId (0, 0)));
        }

        cellOptimalBeamMap.insert (std::pair<uint8_t, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>>> 
                                  (cellId, std::vector<std::pair<std::pair<SfnSf, uint16_t>, std::pair<double, BeamId>>> 
                                  (listOfRLMBeams)));
        
        if (maxSNR < currSNR)
        {
          maxSNR = currSNR;
          maxCellId = cellId;
        }
      }

      BeamSweepTraceParams paramsRXUpdate;
      paramsRXUpdate.imsi = m_imsi;
      paramsRXUpdate.m_beamSweepOrigin = BeamSweepTraceParams::UE_COMPLETED_BEAM_SWEEP;
      paramsRXUpdate.currentCell = GetCellId ();
      paramsRXUpdate.foundCell = maxCellId;
      paramsRXUpdate.foundSector = m_beamManager->GetBeamId (m_registeredEnb.at(maxCellId)).GetSector ();
      paramsRXUpdate.foundElevation = m_beamManager->GetBeamId (m_registeredEnb.at(maxCellId)).GetElevation ();
      m_beamSweepTrace(paramsRXUpdate);

      BeamSweepTraceParams paramsTXUpdate;
      paramsTXUpdate.imsi = m_imsi;
      paramsTXUpdate.m_beamSweepOrigin = BeamSweepTraceParams::GNB_RECVD_BEAM_REPORT;
      paramsTXUpdate.foundCell = maxCellId;
      paramsTXUpdate.foundSector = m_registeredEnb.at(maxCellId)->GetPhy(0)->GetBeamManager ()->GetBeamId (m_netDevice).GetSector ();
      paramsTXUpdate.foundElevation = m_registeredEnb.at (maxCellId)->GetPhy(0)->GetBeamManager ()->GetBeamId (m_netDevice).GetElevation ();
      m_beamSweepTrace (paramsTXUpdate);

      m_ueCphySapUser->SendOptimalBeamMapToLteCoordinator (cellOptimalBeamMap);

      if (m_ueCphySapUser->IsRrcIdleStart ())
      {
        ReEstablishConnectionWithCell (maxCellId);
      }
      else
      {
        Simulator::Schedule (MilliSeconds (10), &NrUePhy::SetIAStateOfAllGnbs, this, false);
      }
  }
  else
  {
    Simulator::Schedule (MilliSeconds (0), &NrUePhy::DoStartBeamSweep, this, BeamSweepType::IASweep);
  }
}

void 
NrUePhy::FindMaximumSNR (Ptr<SSBProcessor> ssbProcessor, uint16_t rxSectorNumber, uint16_t txSectorNumber, uint8_t cellIndex, bool isIAperformed)
{
  //double sumSNR = 0.0;
  double maxSNRforCell = 0.0;
  double currSNR = 0.0;
  std::pair<uint8_t, std::pair<uint16_t, BeamId>> currTxRxPair = 
        std::pair<uint8_t, std::pair<uint16_t, BeamId>> (0, {0, {0, 0}});
  std::pair<int, int> maxRxTxPair;
  auto rxMapSize = ssbProcessor->rxBeamtxSectorSNRMap.size ();

  for (size_t rxIt = 1; rxIt <= rxMapSize; rxIt++)
  {
    if (ssbProcessor->rxBeamtxSectorSNRMap.find (rxIt) != ssbProcessor->rxBeamtxSectorSNRMap.end())
    {
      auto txMapSize = ssbProcessor->rxBeamtxSectorSNRMap.at(rxIt).size ();

      for (size_t txIt = 1; txIt <= txMapSize; txIt++)
      {
        if (ssbProcessor->rxBeamtxSectorSNRMap.at(rxIt).find (txIt) != ssbProcessor->rxBeamtxSectorSNRMap.at(rxIt).end())
        {
          currSNR = ssbProcessor->rxBeamtxSectorSNRMap.at(rxIt).at(txIt);
          BeamId ueBeam;
          if (isIAperformed)
          {
            ueBeam = m_ueBeamVectorList.at (rxIt - 1);
          }
          else
          {
            ueBeam = m_ueBeamVectorList.at((rxIt - 1 + ssbProcessor->m_rlmStartingRXIndex) % m_ueBeamVectorList.size());
          }

          currTxRxPair = std::pair<uint8_t, std::pair<uint16_t, BeamId>>
                (cellIndex, {(txIt + ssbProcessor->m_startingSymbolOffset) % txSectorNumber, ueBeam});

          if (currSNR >= maxSNRforCell)
          {
            maxRxTxPair = std::pair<int, int> (rxIt, txIt);
            maxSNRforCell = currSNR;
            ssbProcessor->m_maxTxRxPairForCell = currTxRxPair;
          }
        }
        else
        {
          NS_LOG_UNCOND ("Could not find " << txIt << 
                "in ssbProcessor->rxBeamtxSectorSNRMap.at(" << rxIt << ")");    
        }
      }
    }
    else
    {
      NS_LOG_UNCOND ("Could not find " << rxIt << "in ssbProcessor->rxBeamtxSectorSNRMap");
    }
  }
  ssbProcessor->m_maxSNRPerCell = maxSNRforCell;
  ssbProcessor->rxBeamtxSectorSNRMap.at(maxRxTxPair.first).at (maxRxTxPair.second) = 1e-20;
}

void 
NrUePhy::DoConfigureCSIRSResources (std::pair<uint8_t, uint8_t> startingSubframeSlotPair,
                  uint8_t csiRSOffset,
                  uint8_t csiRSPeriodicity,
                  uint8_t noOfCSIRSResourcesPerSSBurst)
{
  m_noOfCSIRSResourcesPerSSBurst = noOfCSIRSResourcesPerSSBurst;
  SfnSf csiRSSfn;
  if (m_imsi + csiRSOffset > 48)
  {
    // CSI-RS slot exceeds available ressources in frame 0 -> request ressource slot in frame 1.
    csiRSSfn = SfnSf (1, startingSubframeSlotPair.first, startingSubframeSlotPair.second, GetNumerology ());
  }
  else
  {
    csiRSSfn = SfnSf (0, startingSubframeSlotPair.first, startingSubframeSlotPair.second, GetNumerology ());
  }
  SfnSfKey sfnSfKey;
  m_csiRSResourcesTbRLM.clear ();

  for (uint8_t n = 0; n < noOfCSIRSResourcesPerSSBurst; n++)
  {
    if (m_csiRSResourcesTbRLM.find (csiRSSfn.GetFrame ()) == m_csiRSResourcesTbRLM.end ())
    {
      std::vector<std::pair<uint8_t, uint8_t>> listOfCSIRSResources;
      listOfCSIRSResources.emplace_back (csiRSSfn.GetSubframe (), csiRSSfn.GetSlot ());
      m_csiRSResourcesTbRLM.insert ({csiRSSfn.GetFrame (), listOfCSIRSResources});
    }
    else
    {
      m_csiRSResourcesTbRLM.at(csiRSSfn.GetFrame()).push_back ({csiRSSfn.GetSubframe (), csiRSSfn.GetSlot ()});
    }
    csiRSSfn = csiRSSfn.GetFutureSfnSf (csiRSPeriodicity + 1);
  }
}

void
NrUePhy::ProcessCSIRSs (Ptr<NrCSIRSMessage> csiRSMsg)
{
  if (!m_IAperformed)
  {
    auto cellIdFromMsg = csiRSMsg->GetCellId ();

    if (cellIdFromMsg == GetCellId () || m_minCSIRSFromServingGnb < 4)
    {
      SfnSf currentSfnSf = GetCurrentSfnSf ();
      auto frameRemainder = currentSfnSf.GetFrame () % (uint8_t)2;
      auto currentSubframe = currentSfnSf.GetSubframe();
      auto currentSlot = currentSfnSf.GetSlot();
      SfnSfKey sfnSfKey = std::pair<uint8_t, std::pair<uint8_t, uint8_t>>(
          frameRemainder,
          std::pair<uint8_t, uint8_t>(currentSubframe,
                                      currentSlot));

      if (m_recvCSIMap.find(sfnSfKey) != m_recvCSIMap.end())
      {
        // Something already in the map at this position -> Synchronization error, clear map
        m_recvCSIMap.clear();
        m_subOptimalCSIBeamMap.clear();
      }

      m_recvCSIMap.insert({sfnSfKey, {{m_csiRXBeam, cellIdFromMsg}, {BeamId(csiRSMsg->GetTXBeamId().first, csiRSMsg->GetTXBeamId().second), csiRSMsg->GetSnrAvg()}}});

      if (m_recvCSIMap.size() == m_noOfBeamsTbRLM)
      {
        // clear map at m_imsi and fill with empty vector
        m_csiRSTbReported.erase(m_imsi);
        m_csiRSTbReported.insert({m_imsi, {}});

        Ptr<NRCSIReportMessage> csiReportMesssage = Create<NRCSIReportMessage>();
        csiReportMesssage->SetCellId(GetCellId());
        csiReportMesssage->SetSourceBwp(GetBwpId());
        csiReportMesssage->SetRnti(GetRnti());
        std::pair<std::pair<SfnSfKey, uint8_t>, std::pair<BeamId, double>> csiRSReportMap;

        for (uint8_t n = 0; n < m_noOfBeamsTbRLM; n++)
        {
          auto maxSNR = 0.0;
          auto currSNR = 0.0;
          SfnSfKey maxCSIRSIndex;
          std::map<SfnSfKey, std::pair<std::pair<BeamId, uint8_t>, std::pair<BeamId, double>>>::iterator recvCSIIterator;
          for (recvCSIIterator = m_recvCSIMap.begin(); recvCSIIterator != m_recvCSIMap.end(); ++recvCSIIterator)
          {
            currSNR = recvCSIIterator->second.second.second;

            if (currSNR > maxSNR)
            {
              maxSNR = currSNR;
              maxCSIRSIndex = recvCSIIterator->first;
            }
          }

          if (n < m_noOfBeamsTbReported)
          {
            m_csiRSTbReported.at(m_imsi).emplace_back(std::pair<std::pair<SfnSfKey, uint8_t>, std::pair<BeamId, double>>(
                {maxCSIRSIndex,
                 m_recvCSIMap.at(maxCSIRSIndex).first.second},
                {m_recvCSIMap.at(maxCSIRSIndex).first.first,
                 m_recvCSIMap.at(maxCSIRSIndex).second.second}));

            csiRSReportMap = {{maxCSIRSIndex, m_recvCSIMap.at(maxCSIRSIndex).first.second},
                              {m_recvCSIMap.at(maxCSIRSIndex).second.first, round(m_recvCSIMap.at(maxCSIRSIndex).second.second)}};

            csiReportMesssage->SetCSIRSComponent(n, csiRSReportMap);
          }
          else
          {
            m_subOptimalCSIBeamMap.emplace_back(std::pair<std::pair<SfnSfKey, uint8_t>, std::pair<BeamId, double>>(
                {maxCSIRSIndex,
                 m_recvCSIMap.at(maxCSIRSIndex).first.second},
                {m_recvCSIMap.at(maxCSIRSIndex).first.first,
                 m_recvCSIMap.at(maxCSIRSIndex).second.second}));
          }
          m_recvCSIMap.erase(maxCSIRSIndex);
        }

        std::map<uint8_t, double> maxSNRCellMap;

        for (uint8_t csiIter = 0; csiIter < m_csiRSTbReported.at(m_imsi).size(); csiIter++)
        {
          uint8_t cellIdOfReport = m_csiRSTbReported.at(m_imsi).at(csiIter).first.second;
          double snrOfReport = m_csiRSTbReported.at(m_imsi).at(csiIter).second.second;

          if (maxSNRCellMap.find(cellIdOfReport) == maxSNRCellMap.end())
          {
            // map ensures that only the best beam of every gNB is handled here!
            maxSNRCellMap.insert({cellIdOfReport, snrOfReport});
            BeamId maxSNRBeamId = m_csiRSTbReported.at(m_imsi).at(csiIter).second.first;
            BeamId oldBeamId = m_beamManager->GetBeamId(m_registeredEnb.at(GetCellId()));

            BeamformingVector maxSNRBfv = BeamformingVector(
                CreateDirectionalBfv(m_beamManager->GetAntennaArray(), maxSNRBeamId.GetSector(),
                                     maxSNRBeamId.GetElevation()),
                maxSNRBeamId);
            m_beamManager->SaveBeamformingVector(maxSNRBfv, m_registeredEnb.at(cellIdOfReport));

            if (cellIdOfReport == GetCellId())
            {

              if (maxSNRBeamId != oldBeamId)
              {
                m_beamManager->ChangeBeamformingVector(m_registeredEnb.at(GetCellId()));

                BeamSweepTraceParams params;
                params.imsi = m_imsi;
                params.m_beamSweepOrigin = BeamSweepTraceParams::UE_BEAM_ADJUSTMENT;
                params.foundCell = GetCellId();
                params.foundSector = maxSNRBeamId.GetSector();
                params.foundElevation = maxSNRBeamId.GetElevation();

                m_beamSweepTrace(params);
              }
            }
            UpdateSinrEstimate(cellIdOfReport, m_csiRSTbReported.at(m_imsi).at(0).second.second);
          }
        }
        DoSendControlMessage(csiReportMesssage);
      }
    }
  }
}

void 
NrUePhy::ResetSSBRLMProcessor ()
{
  m_ssbRLMProcessorMap.clear();
}

bool
NrUePhy::SSBTbProcessed (SfnSf currSfn, uint8_t symbolOffset)
{
  return true;
}

bool 
NrUePhy::IsFirstSSBInBurst (SfnSf currSfn, uint8_t symbolOffset)
{
  if (currSfn.GetFrame() % (uint8_t)2 == 0 &&
      currSfn.GetSubframe() == 0 &&
      currSfn.GetSlot() == 0 &&
      symbolOffset == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void 
NrUePhy::DoNotifyHandoverSuccessful ()
{
  m_ssbRMCounter = 0;
  ResetSSBRLMProcessor ();
  m_recvCSIMap.clear ();
  m_subOptimalCSIBeamMap.clear ();
}

void 
NrUePhy::SetUeHorizontalAngleStep (double horizontalAngleStep)
{
  m_ueHorizontalAngleStep = horizontalAngleStep;
}

double 
NrUePhy::GetUeHorizontalAngleStep () const
{
  return m_ueHorizontalAngleStep;
}

void 
NrUePhy::SetUeVerticalAngleStep (double verticalAngleStep)
{
  m_ueElevationAngleStep = verticalAngleStep;
}

double
NrUePhy::GetUeVerticalAngleStep () const
{
  return m_ueElevationAngleStep;
}

}

