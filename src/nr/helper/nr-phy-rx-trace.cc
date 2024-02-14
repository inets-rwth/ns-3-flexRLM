/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
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
 *   Author: Marco Miozzo <marco.miozzo@cttc.es>
 *           Nicola Baldo  <nbaldo@cttc.es>
 *
 *   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
 *                        Sourjya Dutta <sdutta@nyu.edu>
 *                        Russell Ford <russell.ford@nyu.edu>
 *                        Menglei Zhang <menglei@nyu.edu>
 */



#include <ns3/log.h>
#include "nr-phy-rx-trace.h"
#include <ns3/simulator.h>
#include <stdio.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrPhyRxTrace");

NS_OBJECT_ENSURE_REGISTERED (NrPhyRxTrace);

std::ofstream NrPhyRxTrace::m_rsrpSinrFile;
std::string NrPhyRxTrace::m_rsrpSinrFileName;

std::ofstream NrPhyRxTrace::m_rxPacketTraceFile;
std::string NrPhyRxTrace::m_rxPacketTraceFilename;

std::ofstream NrPhyRxTrace::m_rsrpFile;
std::string NrPhyRxTrace::m_rsrpFilename;

std::ofstream m_dlCtrlSinrFile;
std::string m_dlCtrlSinrFileName;

std::ofstream m_sinrEstimateFile;
std::string m_sinrEstimateFileName;

std::ofstream NrPhyRxTrace::m_beamSweepTraceFile;
std::string NrPhyRxTrace::m_beamSweepTraceFilename;

std::ofstream NrPhyRxTrace::m_radioLinkMonitoringTraceFile;
std::string NrPhyRxTrace::m_radioLinkMonitoringTraceFilename;

std::ofstream NrPhyRxTrace::m_rxedGnbPhyCtrlMsgsFile;
std::string NrPhyRxTrace::m_rxedGnbPhyCtrlMsgsFileName;
std::ofstream NrPhyRxTrace::m_txedGnbPhyCtrlMsgsFile;
std::string NrPhyRxTrace::m_txedGnbPhyCtrlMsgsFileName;

std::ofstream NrPhyRxTrace::m_rxedUePhyCtrlMsgsFile;
std::string NrPhyRxTrace::m_rxedUePhyCtrlMsgsFileName;
std::ofstream NrPhyRxTrace::m_txedUePhyCtrlMsgsFile;
std::string NrPhyRxTrace::m_txedUePhyCtrlMsgsFileName;
std::ofstream NrPhyRxTrace::m_rxedUePhyDlDciFile;
std::string NrPhyRxTrace::m_rxedUePhyDlDciFileName;

std::ofstream NrPhyRxTrace::m_rxMacThruFile;
std::string NrPhyRxTrace::m_rxMacThruFilename;
std::map<uint64_t,double> NrPhyRxTrace::timeLastMacRx = {};
std::map<uint64_t, uint32_t> NrPhyRxTrace::totalBytesMacRx = {};

std::ofstream NrPhyRxTrace::m_rxThruFile;
std::string NrPhyRxTrace::m_rxThruFilename;
std::map<uint64_t, double> NrPhyRxTrace::timeLastOutput = {};
std::map<uint64_t, uint32_t> NrPhyRxTrace::totalBytesReceived = {};

std::map<uint64_t, uint32_t> NrPhyRxTrace::lastFrameNum; // = 1
std::map<uint64_t, uint32_t> NrPhyRxTrace::lastFrameNumMacRx; // = 1

std::string NrPhyRxTrace::m_beamSweepType;
bool NrPhyRxTrace::m_realisticIA = false;

NrPhyRxTrace::NrPhyRxTrace ()
{
}

NrPhyRxTrace::~NrPhyRxTrace ()
{
  if (m_rsrpSinrFile.is_open ())
    {
      m_rsrpSinrFile.close ();
    }

  if (m_rxPacketTraceFile.is_open ())
    {
      m_rxPacketTraceFile.close ();
    }

  if (m_rsrpFile.is_open ())
    {
      m_rsrpFile.close ();
    }

  if (m_dlCtrlSinrFile.is_open())
    {
        m_dlCtrlSinrFile.close();
    }

  if (m_sinrEstimateFile.is_open())
    {
        m_sinrEstimateFile.close();
    }

  if  (m_beamSweepTraceFile.is_open ())
    {
      m_beamSweepTraceFile.close (); 
    }

  if  (m_radioLinkMonitoringTraceFile.is_open ())
    {
      m_radioLinkMonitoringTraceFile.close (); 
    }

  if (m_rxedGnbPhyCtrlMsgsFile.is_open ())
    {
      m_rxedGnbPhyCtrlMsgsFile.close ();
    }

  if (m_txedGnbPhyCtrlMsgsFile.is_open ())
    {
      m_txedGnbPhyCtrlMsgsFile.close ();
    }

  if (m_rxedUePhyCtrlMsgsFile.is_open ())
    {
      m_rxedUePhyCtrlMsgsFile.close ();
    }

  if (m_txedUePhyCtrlMsgsFile.is_open ())
    {
      m_txedUePhyCtrlMsgsFile.close ();
    }

  if (m_rxedUePhyDlDciFile.is_open ())
    {
      m_rxedUePhyDlDciFile.close ();
    }

  if (m_rxMacThruFile.is_open ())
  {
    m_rxMacThruFile.close ();
  }

  if (m_rxThruFile.is_open ())
  {
    m_rxThruFile.close ();
  }
}

TypeId
NrPhyRxTrace::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrPhyRxTrace")
    .SetParent<Object> ()
    .AddConstructor<NrPhyRxTrace> ()
    .AddAttribute ("MacRxThruFileName",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("MacRxThruData.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetMacrxThruFilename),
                   MakeStringChecker ())
    .AddAttribute ("OutputFileName",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("RxPacketTrace.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetOutputFilename),
                   MakeStringChecker ())
    .AddAttribute ("PhyRxThruFileName",
                   "Name of the file where the uplink results will be saved.",
                   StringValue ("PhyRxThruData.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetrxThruFilename),
                   MakeStringChecker ())
    .AddAttribute ("RsrpFileName",
                   "Name of the file where the RSRP results will be saved.",
                   StringValue ("RsrpData.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetRsrpFilename),
                   MakeStringChecker ())
    .AddAttribute ("DlCtrlSinrFileName",
                   "Name of the file where the DlCtrlSinr results will be saved.",
                   StringValue ("DlCtrlSinrData.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetDlCtrlSinrFilename),
                   MakeStringChecker ())
    .AddAttribute ("SinrEstimateFileName",
                   "Name of the file where the Sinr estimate results will be saved.",
                   StringValue ("SinrEstimateData.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetSinrEstimateFilename),
                   MakeStringChecker ())
    .AddAttribute ("BeamSweepFileName",
                   "Name of the file where the beam sweep traces will be saved",
                   StringValue ("BeamSweepTrace.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetBeamSweepFileName),
                   MakeStringChecker ())
    .AddAttribute ("RadioLinkMonitoringFileName",
                   "Name of the file where the radio link monitoring traces will be saved",
                   StringValue ("RadioLinkMonitoringTrace.txt"),
                   MakeStringAccessor (&NrPhyRxTrace::SetRadioLinkMonitoringFileName),
                   MakeStringChecker ())
    .AddAttribute ("BeamSweepType",
                   "Variable that sets whether realistic or idealistic beam sweep will be used",
                   StringValue ("Ideal"),
                   MakeStringAccessor (&NrPhyRxTrace::SetBeamSweepType),
                   MakeStringChecker ())
  ;
  return tid;
}

void
NrPhyRxTrace::RsrpCallback([[maybe_unused]] Ptr<NrPhyRxTrace> phyStats,
                                 [[maybe_unused]] std::string path,
                                 uint16_t cellId,
                                 uint16_t imsi,
                                 uint16_t rnti,
                                 double avgRsrp,
                                 uint8_t bwpId)
{
    NS_LOG_INFO("UE" << rnti << "with IMSI " << imsi << " over bwp ID " << bwpId
                     << "->Generate RsrpTrace");
    if (!m_rsrpFile.is_open())
    {
        /*std::ostringstream oss;
        oss << m_resultsFolder << "DlDataSinr" << m_simTag.c_str() << ".txt";
        m_rsrpFileName = oss.str();
        m_rsrpFile.open(m_rsrpFileName.c_str());*/

        //Better keep iNets format:
        m_rsrpFile.open (m_rsrpFilename.c_str ());

        m_rsrpFile << "Time"
                         << "\t"
                         << "CellId"
                         << "\t"
                         << "IMSI"
                         << "\t"
                         << "RNTI"
                         << "\t"
                         << "RSRP(dB)"
                         << "\t" << std::endl;

        if (!m_rsrpFile.is_open())
        {
            NS_FATAL_ERROR("Could not open tracefile");
        }
    }

    m_rsrpFile << Simulator::Now().GetSeconds() << "\t" << cellId << "\t" << imsi << "\t"
                     << rnti << "\t" << avgRsrp << "\t" << std::endl; //N avgRsrp bereits in dB gegeben
}

void NrPhyRxTrace::DlCtrlSinrCallback([[maybe_unused]] Ptr<NrPhyRxTrace> phyStats,
                                      [[maybe_unused]] std::string path,
                                      uint16_t cellId,
                                      uint16_t rnti,
                                      uint64_t imsi,
                                      double avgSinr,
                                      uint16_t bwpId,
                                      uint8_t streamId)
{
    NS_LOG_INFO("UE" << rnti << "of " << cellId << " over bwp ID " << bwpId
                     << "->Generate DlCtrlSinrTrace");

    if (!m_dlCtrlSinrFile.is_open())
    {
        /* std::ostringstream oss;
        oss << m_resultsFolder << "DlCtrlSinr" << m_simTag.c_str() << ".txt";
        m_dlCtrlSinrFileName = oss.str();
        m_dlCtrlSinrFile.open(m_dlCtrlSinrFileName.c_str());*/

        //Better keep iNets format:
        m_dlCtrlSinrFile.open (m_dlCtrlSinrFileName.c_str ());

        m_dlCtrlSinrFile << "Time"
                         << "\t"
                         << "CellId"
                         << "\t"
                         << "RNTI"
                         << "\t"
                         << "IMSI"
                         << "\t"
                         << "BWPId"
                         << "\t"
                         << "StreamId"
                         << "\t"
                         << "SINR(dB)" << std::endl;

        if (!m_dlCtrlSinrFile.is_open())
        {
            NS_FATAL_ERROR("Could not open tracefile");
        }
    }

    m_dlCtrlSinrFile << Simulator::Now().GetSeconds() << "\t" << cellId << "\t" << rnti << "\t" << imsi << "\t"
                     << bwpId << "\t" << +streamId << "\t" << 10 * log10(avgSinr) << std::endl;
}

void NrPhyRxTrace::SinrEstimateCallback([[maybe_unused]] Ptr<NrPhyRxTrace> phyStats,
                                      [[maybe_unused]] std::string path,
                                      uint16_t cellId,
                                      uint16_t rnti,
                                      uint64_t imsi,
                                      double avgSinr,
                                      bool csirs)
{
    NS_LOG_INFO("UE" << rnti << "of " << cellId
                     << "->Generate DlCtrlSinrTrace");

    if (!m_sinrEstimateFile.is_open())
    {

        //Better keep iNets format:
        m_sinrEstimateFile.open(m_sinrEstimateFileName.c_str());

        m_sinrEstimateFile << "Time"
                           << "\t"
                           << "CellId"
                           << "\t"
                           << "RNTI"
                           << "\t"
                           << "IMSI"
                           << "\t"
                           << "SINR(dB)"
                           << "\t"
                           << "csirs?" << std::endl;

        if (!m_sinrEstimateFile.is_open())
        {
            NS_FATAL_ERROR("Could not open tracefile");
        }
    }

    m_sinrEstimateFile << Simulator::Now().GetSeconds() << "\t" << cellId << "\t" << rnti << "\t" << imsi << "\t"
                       << "\t" << 10 * log10(avgSinr) << "\t" << (int)csirs << std::endl;
}

void
NrPhyRxTrace::ReportCurrentCellRsrpSinrCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                                     uint16_t cellId, uint16_t rnti, double power, double avgSinr, uint16_t bwpId)
{
  NS_LOG_INFO ("UE" << rnti << "of " << cellId << " over bwp ID " << bwpId << "->Generate RsrpSinrTrace");
  NS_UNUSED (phyStats);
  NS_UNUSED (path);
  //phyStats->ReportInterferenceTrace (imsi, sinr);
  //phyStats->ReportPowerTrace (imsi, power);

  if (!m_rsrpSinrFile.is_open ())
      {
        m_rsrpSinrFileName = "SinrTrace.txt";
        m_rsrpSinrFile.open (m_rsrpSinrFileName.c_str ());
        m_rsrpSinrFile << "Time" << "\t" << "IMSI" <<
                                    "\t" << "SINR (dB)" << std::endl;

        if (!m_rsrpSinrFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_rsrpSinrFile << Simulator::Now ().GetSeconds ()
                 << "\t" << cellId << "\t" << rnti << "\t" << bwpId
                 << "\t" << avgSinr << "\t" << power << std::endl;
}

void
NrPhyRxTrace::UlSinrTraceCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                       uint64_t imsi, SpectrumValue& sinr, SpectrumValue& power)
{
  NS_LOG_INFO ("UE" << imsi << "->Generate UlSinrTrace");
  uint64_t tti_count = Now ().GetMicroSeconds () / 125;
  uint32_t rb_count = 1;
  FILE* log_file;
  char fname[255];
  sprintf (fname, "UE_%llu_UL_SINR_dB.txt", (long long unsigned ) imsi);
  log_file = fopen (fname, "a");
  Values::iterator it = sinr.ValuesBegin ();
  while (it != sinr.ValuesEnd ())
    {
      //fprintf(log_file, "%d\t%d\t%f\t \n", tti_count/2, rb_count, 10*log10(*it));
      fprintf (log_file, "%llu\t%llu\t%d\t%f\t \n",(long long unsigned )tti_count / 8 + 1, (long long unsigned )tti_count % 8 + 1, rb_count, 10 * log10 (*it));
      rb_count++;
      it++;
    }
  fflush (log_file);
  fclose (log_file);
  //phyStats->ReportInterferenceTrace (imsi, sinr);
  //phyStats->ReportPowerTrace (imsi, power);
}

void
NrPhyRxTrace::RxedGnbPhyCtrlMsgsCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                              SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                              uint8_t bwpId, Ptr<const NrControlMessage> msg)
{
  if (!m_rxedGnbPhyCtrlMsgsFile.is_open ())
      {
        m_rxedGnbPhyCtrlMsgsFileName = "RxedGnbPhyCtrlMsgsTrace.txt";
        m_rxedGnbPhyCtrlMsgsFile.open (m_rxedGnbPhyCtrlMsgsFileName.c_str ());
        m_rxedGnbPhyCtrlMsgsFile << "Time" << "\t" << "Entity"  << "\t" <<
                                    "Frame" << "\t" << "SF" << "\t" << "Slot" <<
                                    "\t" << "VarTTI" << "\t" << "nodeId" <<
                                    "\t" << "RNTI" << "\t" << "bwpId" <<
                                    "\t" << "MsgType" << std::endl;

        if (!m_rxedGnbPhyCtrlMsgsFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_rxedGnbPhyCtrlMsgsFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                              "\t" << "ENB PHY Rxed" << "\t" << sfn.GetFrame () <<
                              "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                              "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                              "\t" << nodeId << "\t" << rnti <<
                              "\t" << static_cast<uint32_t> (bwpId) << "\t";


  if (msg->GetMessageType () == NrControlMessage::DL_CQI)
    {
      m_rxedGnbPhyCtrlMsgsFile << "DL_CQI";
    }
  else if (msg->GetMessageType () == NrControlMessage::SR)
    {
      m_rxedGnbPhyCtrlMsgsFile << "SR";
    }
  else if (msg->GetMessageType () == NrControlMessage::BSR)
    {
      m_rxedGnbPhyCtrlMsgsFile << "BSR";
    }
  else if (msg->GetMessageType () == NrControlMessage::RACH_PREAMBLE)
    {
      m_rxedGnbPhyCtrlMsgsFile << "RACH_PREAMBLE";
    }
  else if (msg->GetMessageType () == NrControlMessage::DL_HARQ)
    {
      m_rxedGnbPhyCtrlMsgsFile << "DL_HARQ";
    }
  else
    {
      m_rxedGnbPhyCtrlMsgsFile << "Other";
    }
  m_rxedGnbPhyCtrlMsgsFile << std::endl;
}

void
NrPhyRxTrace::TxedGnbPhyCtrlMsgsCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                              SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                              uint8_t bwpId, Ptr<const NrControlMessage> msg)
{
  if (!m_txedGnbPhyCtrlMsgsFile.is_open ())
      {
        m_txedGnbPhyCtrlMsgsFileName = "TxedGnbPhyCtrlMsgsTrace.txt";
        m_txedGnbPhyCtrlMsgsFile.open (m_txedGnbPhyCtrlMsgsFileName.c_str ());
        m_txedGnbPhyCtrlMsgsFile << "Time" << "\t" << "Entity" << "\t" <<
                                    "Frame" << "\t" << "SF" << "\t" << "Slot" <<
                                    "\t" << "VarTTI" << "\t" << "nodeId" <<
                                    "\t" << "RNTI" << "\t" << "bwpId" <<
                                    "\t" << "MsgType" << std::endl;

        if (!m_txedGnbPhyCtrlMsgsFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_txedGnbPhyCtrlMsgsFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                              "\t" << "ENB PHY Txed" << "\t" << sfn.GetFrame () <<
                              "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                              "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                              "\t" << nodeId << "\t" << rnti <<
                              "\t" << static_cast<uint32_t> (bwpId) << "\t";

  if (msg->GetMessageType () == NrControlMessage::MIB)
    {
      m_txedGnbPhyCtrlMsgsFile << "MIB";
    }
  else if (msg->GetMessageType () == NrControlMessage::SIB1)
    {
      m_txedGnbPhyCtrlMsgsFile << "SIB1";
    }
  else if (msg->GetMessageType () == NrControlMessage::RAR)
    {
      m_txedGnbPhyCtrlMsgsFile << "RAR";
    }
  else if (msg->GetMessageType () == NrControlMessage::DL_DCI)
    {
      m_txedGnbPhyCtrlMsgsFile << "DL_DCI";
    }
  else if (msg->GetMessageType () == NrControlMessage::UL_DCI)
    {
      m_txedGnbPhyCtrlMsgsFile << "UL_UCI";
    }
  else
    {
      m_txedGnbPhyCtrlMsgsFile << "Other";
    }
  m_txedGnbPhyCtrlMsgsFile << std::endl;
}

void
NrPhyRxTrace::RxedUePhyCtrlMsgsCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                             SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                             uint8_t bwpId, Ptr<const NrControlMessage> msg)
{
  if (!m_rxedUePhyCtrlMsgsFile.is_open ())
      {
        m_rxedUePhyCtrlMsgsFileName = "RxedUePhyCtrlMsgsTrace.txt";
        m_rxedUePhyCtrlMsgsFile.open (m_rxedUePhyCtrlMsgsFileName.c_str ());
        m_rxedUePhyCtrlMsgsFile << "Time" << "\t" << "Entity" << "\t" <<
                                   "Frame" << "\t" << "SF" << "\t" << "Slot" <<
                                   "\t" << "VarTTI" << "\t" << "nodeId" <<
                                   "\t" << "RNTI" << "\t" << "bwpId" << "\t" <<
                                   "MsgType" << std::endl;

        if (!m_rxedUePhyCtrlMsgsFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_rxedUePhyCtrlMsgsFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                             "\t" << "UE  PHY Rxed" << "\t" << sfn.GetFrame ()<<
                             "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                             "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                             "\t" << nodeId << "\t" << rnti <<
                             "\t" << static_cast<uint32_t> (bwpId) << "\t";

  if (msg->GetMessageType () == NrControlMessage::UL_DCI)
    {
      m_rxedUePhyCtrlMsgsFile << "UL_DCI";
    }
  else if (msg->GetMessageType () == NrControlMessage::DL_DCI)
    {
      m_rxedUePhyCtrlMsgsFile << "DL_DCI";
    }
  else if (msg->GetMessageType () == NrControlMessage::MIB)
    {
      m_rxedUePhyCtrlMsgsFile << "MIB";
    }
  else if (msg->GetMessageType () == NrControlMessage::SIB1)
    {
      m_rxedUePhyCtrlMsgsFile << "SIB1";
    }
  else if (msg->GetMessageType () == NrControlMessage::RAR)
    {
      m_rxedUePhyCtrlMsgsFile << "RAR";
    }
  else
    {
      m_rxedUePhyCtrlMsgsFile << "Other";
    }
  m_rxedUePhyCtrlMsgsFile << std::endl;
}

void
NrPhyRxTrace::TxedUePhyCtrlMsgsCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                             SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                             uint8_t bwpId, Ptr<const NrControlMessage> msg)
{
  if (!m_txedUePhyCtrlMsgsFile.is_open ())
      {
        m_txedUePhyCtrlMsgsFileName = "TxedUePhyCtrlMsgsTrace.txt";
        m_txedUePhyCtrlMsgsFile.open (m_txedUePhyCtrlMsgsFileName.c_str ());
        m_txedUePhyCtrlMsgsFile << "Time" << "\t" << "Entity" << "\t" <<
                                   "Frame" << "\t" << "SF" << "\t" << "Slot" <<
                                   "\t" << "VarTTI" << "\t" << "nodeId" <<
                                   "\t" << "RNTI" << "\t" << "bwpId" <<
                                   "\t" << "MsgType" << std::endl;

        if (!m_txedUePhyCtrlMsgsFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_txedUePhyCtrlMsgsFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                             "\t" << "UE  PHY Txed" << "\t" << sfn.GetFrame () <<
                             "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                             "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                             "\t" << nodeId << "\t" << rnti <<
                             "\t" << static_cast<uint32_t> (bwpId) << "\t";

  if (msg->GetMessageType () == NrControlMessage::RACH_PREAMBLE)
    {
      m_txedUePhyCtrlMsgsFile << "RACH_PREAMBLE";
    }
  else if (msg->GetMessageType () == NrControlMessage::SR)
    {
      m_txedUePhyCtrlMsgsFile << "SR";
    }
  else if (msg->GetMessageType () == NrControlMessage::BSR)
    {
      m_txedUePhyCtrlMsgsFile << "BSR";
    }
  else if (msg->GetMessageType () == NrControlMessage::DL_CQI)
    {
      m_txedUePhyCtrlMsgsFile << "DL_CQI";
    }
  else if (msg->GetMessageType () == NrControlMessage::DL_HARQ)
    {
      m_txedUePhyCtrlMsgsFile << "DL_HARQ";
    }
  else
    {
      m_txedUePhyCtrlMsgsFile << "Other";
    }
  m_txedUePhyCtrlMsgsFile << std::endl;
}

void
NrPhyRxTrace::RxedUePhyDlDciCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                          SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                          uint8_t bwpId, uint8_t harqId, uint32_t k1Delay)
{
  if (!m_rxedUePhyDlDciFile.is_open ())
      {
        m_rxedUePhyDlDciFileName = "RxedUePhyDlDciTrace.txt";
        m_rxedUePhyDlDciFile.open (m_rxedUePhyDlDciFileName.c_str ());
        m_rxedUePhyDlDciFile << "Time" << "\t" << "Entity"  << "\t" << "Frame" <<
                                "\t" << "SF" << "\t" << "Slot" << "\t" <<
                                "nodeId" << "\t" << "RNTI" << "\t" << "bwpId" <<
                                "\t" << "Harq ID" << "\t" << "K1 Delay" << std::endl;

        if (!m_rxedUePhyDlDciFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_rxedUePhyDlDciFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                          "\t" << "DL DCI Rxed" << "\t" << sfn.GetFrame () <<
                          "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                          "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                          "\t" << nodeId << "\t" << rnti << "\t" <<
                          static_cast<uint32_t> (bwpId) << "\t" <<
                          static_cast<uint32_t> (harqId) << "\t" <<
                          k1Delay << std::endl;
}

void
NrPhyRxTrace::TxedUePhyHarqFeedbackCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                                 SfnSf sfn, uint16_t nodeId, uint16_t rnti,
                                                 uint8_t bwpId, uint8_t harqId, uint32_t k1Delay)
{
  if (!m_rxedUePhyDlDciFile.is_open ())
      {
        m_rxedUePhyDlDciFileName = "RxedUePhyDlDciTrace.txt";
        m_rxedUePhyDlDciFile.open (m_rxedUePhyDlDciFileName.c_str ());
        m_rxedUePhyDlDciFile << "Time" << "\t"<< "Entity"  << "\t" << "Frame" <<
                                "\t" << "SF" << "\t" << "Slot" << "\t" <<
                                "nodeId" << "\t" << "RNTI" << "\t" << "bwpId" <<
                                "\t" << "Harq ID" << "\t" << "K1 Delay" << std::endl;

        if (!m_rxedUePhyDlDciFile.is_open ())
          {
            NS_FATAL_ERROR ("Could not open tracefile");
          }
      }

  m_rxedUePhyDlDciFile << Simulator::Now ().GetNanoSeconds () / (double) 1e9 <<
                          "\t" << "HARQ FD Txed" << "\t" << sfn.GetFrame () <<
                          "\t" << static_cast<uint32_t> (sfn.GetSubframe ()) <<
                          "\t" << static_cast<uint32_t> (sfn.GetSlot ()) <<
                          "\t" << nodeId << "\t" << rnti << "\t" <<
                          static_cast<uint32_t> (bwpId) << "\t" <<
                          static_cast<uint32_t> (harqId) << "\t" <<
                          k1Delay << std::endl;
}

void
NrPhyRxTrace::ReportInterferenceTrace (uint64_t imsi, SpectrumValue& sinr)
{
  uint64_t tti_count = Now ().GetMicroSeconds () / 125;
  uint32_t rb_count = 1;
  FILE* log_file;
  char fname[255];
  sprintf (fname, "UE_%llu_SINR_dB.txt", (long long unsigned ) imsi);
  log_file = fopen (fname, "a");
  Values::iterator it = sinr.ValuesBegin ();
  while (it != sinr.ValuesEnd ())
    {
      //fprintf(log_file, "%d\t%d\t%f\t \n", tti_count/2, rb_count, 10*log10(*it));
      fprintf (log_file, "%llu\t%llu\t%d\t%f\t \n",(long long unsigned) tti_count / 8 + 1, (long long unsigned) tti_count % 8 + 1, rb_count, 10 * log10 (*it));
      rb_count++;
      it++;
    }
  fflush (log_file);
  fclose (log_file);
}

void
NrPhyRxTrace::ReportPowerTrace (uint64_t imsi, SpectrumValue& power)
{

  uint32_t tti_count = Now ().GetMicroSeconds () / 125;
  uint32_t rb_count = 1;
  FILE* log_file;
  char fname[255];
  printf (fname, "UE_%llu_ReceivedPower_dB.txt", (long long unsigned) imsi);
  log_file = fopen (fname, "a");
  Values::iterator it = power.ValuesBegin ();
  while (it != power.ValuesEnd ())
    {
      fprintf (log_file, "%llu\t%llu\t%d\t%f\t \n",(long long unsigned) tti_count / 8 + 1,(long long unsigned) tti_count % 8 + 1, rb_count, 10 * log10 (*it));
      rb_count++;
      it++;
    }
  fflush (log_file);
  fclose (log_file);
}

void
NrPhyRxTrace::ReportPacketCountUeCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                               UePhyPacketCountParameter param)
{
  phyStats->ReportPacketCountUe (param);
}
void
NrPhyRxTrace::ReportPacketCountEnbCallback (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                                GnbPhyPacketCountParameter param)
{
  phyStats->ReportPacketCountEnb (param);
}

void
NrPhyRxTrace::ReportDownLinkTBSize (Ptr<NrPhyRxTrace> phyStats, std::string path,
                                        uint64_t imsi, uint64_t tbSize)
{
  phyStats->ReportDLTbSize (imsi, tbSize);
}



void
NrPhyRxTrace::ReportPacketCountUe (UePhyPacketCountParameter param)
{
  FILE* log_file;
  char fname[255];
  sprintf (fname,"UE_%llu_Packet_Trace.txt", (long long unsigned) param.m_imsi);
  log_file = fopen (fname, "a");
  if (param.m_isTx)
    {
      fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, param.m_noBytes, 0);
    }
  else
    {
      fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, 0, param.m_noBytes);
    }

  fflush (log_file);
  fclose (log_file);

}

void
NrPhyRxTrace::ReportPacketCountEnb (GnbPhyPacketCountParameter param)
{
  FILE* log_file;
  char fname[255];
  sprintf (fname,"BS_%llu_Packet_Trace.txt",(long long unsigned) param.m_cellId);
  log_file = fopen (fname, "a");
  if (param.m_isTx)
    {
      fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, param.m_noBytes, 0);
    }
  else
    {
      fprintf (log_file, "%d\t%d\t%d\n", param.m_subframeno, 0, param.m_noBytes);
    }

  fflush (log_file);
  fclose (log_file);
}

void
NrPhyRxTrace::ReportDLTbSize (uint64_t imsi, uint64_t tbSize)
{
  FILE* log_file;
  char fname[255];
  sprintf (fname,"UE_%llu_Tb_Size.txt", (long long unsigned) imsi);
  log_file = fopen (fname, "a");

  fprintf (log_file, "%llu \t %llu\n", (long long unsigned )Now ().GetMicroSeconds (), (long long unsigned )tbSize);
  fprintf (log_file, "%lld \t %llu \n",(long long int) Now ().GetMicroSeconds (), (long long unsigned) tbSize);
  fflush (log_file);
  fclose (log_file);
}

void
NrPhyRxTrace::RxPacketTraceUeCallback (Ptr<NrPhyRxTrace> phyStats, std::string path, RxPacketTraceParams params)
{
  if (!m_rxPacketTraceFile.is_open ())
    {
      //m_rxPacketTraceFilename = "RxPacketTrace.txt";
      m_rxPacketTraceFile.open (m_rxPacketTraceFilename.c_str ());
      m_rxPacketTraceFile << "\ttime\tframe\tsubF\tslot\t1stSym\tsymbol#\tcellId\trnti\timsi\ttbSize\tmcs\trv\tSINR(dB)\tSNR(dB)\tcorrupt\tTBler" << std::endl;
      if (!m_rxPacketTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
    }

  m_rxPacketTraceFile << "DL"
                      << "\t" << Simulator::Now().GetSeconds() 
                      << "\t" << params.m_frameNum
                      << "\t" << (unsigned)params.m_subframeNum
                      << "\t" << (unsigned)params.m_slotNum
                      << "\t" << (unsigned)params.m_symStart
                      << "\t" << (unsigned)params.m_numSym
                      << "\t" << params.m_cellId
                      << "\t" << params.m_rnti
                      << "\t" << params.m_imsi
                      << "\t" << params.m_tbSize
                      << "\t" << (unsigned)params.m_mcs
                      << "\t" << (unsigned)params.m_rv
                      << "\t" << 10 * log10 (params.m_sinr)
                      << "\t" << 10 * log10 (params.m_snr)
                      << "\t" << params.m_corrupt
                      << "\t" << params.m_tbler
                      //<< "\t" << (unsigned)params.m_bwpId 
                      << std::endl;

  if (params.m_corrupt)
    {
      NS_LOG_DEBUG ("DL TB error\t" << params.m_frameNum
                                    << "\t" << (unsigned)params.m_subframeNum
                                    << "\t" << (unsigned)params.m_slotNum
                                    << "\t" << (unsigned)params.m_symStart
                                    << "\t" << (unsigned)params.m_numSym
                                    << "\t" << params.m_rnti
                                    << "\t" << params.m_tbSize <<
                    "\t" << (unsigned)params.m_mcs <<
                    "\t" << (unsigned)params.m_rv <<
                    "\t" << params.m_sinr <<
                    "\t" << params.m_snr <<
                    "\t" << params.m_tbler <<
                    "\t" << params.m_corrupt <<
                    "\t" << (unsigned)params.m_bwpId);
    }
}
void
NrPhyRxTrace::RxPacketTraceEnbCallback (Ptr<NrPhyRxTrace> phyStats, std::string path, RxPacketTraceParams params)
{
  if (!m_rxPacketTraceFile.is_open ())
    {
      //m_rxPacketTraceFilename = "RxPacketTraceUL.txt";
      m_rxPacketTraceFile.open (m_rxPacketTraceFilename.c_str ());
      m_rxPacketTraceFile << "\ttime\tframe\tsubF\tslot\t1stSym\tsymbol#\tcellId\trnti\timsi\ttbSize\tmcs\trv\tSINR(dB)\tSNR(dB)\tcorrupt\tTBler" << std::endl;

      if (!m_rxPacketTraceFile.is_open ())
        {
          NS_FATAL_ERROR ("Could not open tracefile");
        }
    }
  m_rxPacketTraceFile << "UL"
                      << "\t" << Simulator::Now().GetSeconds()
                      << "\t" << params.m_frameNum 
                      << "\t" << (unsigned)params.m_subframeNum
                      << "\t" << (unsigned)params.m_slotNum
                      << "\t" << (unsigned)params.m_symStart
                      << "\t" << (unsigned)params.m_numSym 
                      << "\t" << params.m_cellId
                      << "\t" << params.m_rnti 
                      << "\t" << params.m_tbSize 
                      << "\t" << (unsigned)params.m_mcs 
                      << "\t" << (unsigned)params.m_rv 
                      << "\t" << 10 * log10 (params.m_sinr) 
                      << "\t" << 10 * log10 (params.m_snr)
                      << "\t" << params.m_corrupt 
                      << "\t" << params.m_tbler 
                      //<< "\t" << params.m_ccId 
                      << std::endl;

  if (params.m_corrupt)
    {
      NS_LOG_DEBUG ("UL TB error\t" << params.m_frameNum << "\t" << (unsigned)params.m_subframeNum
                                    << "\t" << (unsigned)params.m_slotNum
                                    << "\t" << (unsigned)params.m_symStart
                                    << "\t" << (unsigned)params.m_numSym
                                    << "\t" << params.m_rnti << "\t" << params.m_tbSize << "\t" << (unsigned)params.m_mcs << "\t" << (unsigned)params.m_rv << "\t"
                                    << params.m_sinr << "\t" 
                                    << params.m_snr << "\t"<< params.m_tbler << "\t" << params.m_corrupt << "\t" << params.m_sinrMin << " \t" << params.m_bwpId);
    }
}

void
NrPhyRxTrace::MacRxUe (Ptr<NrPhyRxTrace> phyStats,  std::string path, uint16_t a, uint8_t b, uint32_t pktSize, SfnSf currentSlot, uint64_t imsi)
{
  if (lastFrameNumMacRx.find(imsi) == lastFrameNumMacRx.end()) {
    //this is the first time calling this function for params.m_imsi
    //initialise some data structures
    lastFrameNumMacRx.insert(std::pair<uint64_t,uint32_t>(imsi, 1));
    totalBytesMacRx.insert(std::pair<uint64_t,uint32_t>(imsi, 0));
    timeLastMacRx.insert(std::pair<uint64_t,double>(imsi, 0));
  }

  totalBytesMacRx.at(imsi) += pktSize;
  bool canWrite;
  if (m_realisticIA)
  {
    canWrite = ((Simulator::Now ().GetSeconds () - timeLastMacRx.at(imsi)) > 0.04) &&
       (currentSlot.GetFrame () % 2 == 1 &&
        currentSlot.GetSubframe () == 5 &&
       ((currentSlot.GetFrame () - lastFrameNumMacRx.at(imsi)) % 2  == 0));
  }
  else
  {
    canWrite = (Simulator::Now ().GetSeconds () - timeLastMacRx.at(imsi)) > 0.05;
  }
  
  if (canWrite)
  {
    if (!m_rxMacThruFile.is_open ())
    {
      m_rxMacThruFile.open (m_rxMacThruFilename.c_str ());
    }

    m_rxMacThruFile << Simulator::Now ().GetSeconds () << "\t"
                          << imsi << "\t"
                          << totalBytesMacRx.at(imsi) * 8/ (Simulator::Now ().GetSeconds () - timeLastMacRx.at(imsi))
                          << std::endl;
    timeLastMacRx.at(imsi) = Simulator::Now ().GetSeconds ();
    lastFrameNumMacRx.at(imsi) = currentSlot.GetFrame ();
    totalBytesMacRx.at(imsi) = 0;
  }
}

void 
NrPhyRxTrace::BeamSweepTraceCallback (Ptr<NrPhyRxTrace> phyStats, std::string path, BeamSweepTraceParams params)
{
  if (!m_beamSweepTraceFile.is_open ())
    {
      m_beamSweepTraceFile.open (m_beamSweepTraceFilename.c_str ());
      if (!m_beamSweepTraceFile.is_open ())
      {
        NS_FATAL_ERROR ("Could not open tracefile");
      }
    }

  

  switch (params.m_beamSweepOrigin)
  {
  case BeamSweepTraceParams::COORDINATOR_INITIATED_HANDOVER:
    m_beamSweepTraceFile << Simulator::Now ().GetSeconds ()
                         << "\tIMSI: " << params.imsi
                         << "\tCO"
                         << "\tTRIGGERED BEAM SWEEP. CURRENT CELL: " << (unsigned)params.currentCell
                         << "\tCURRENT SNR: " << params.snrBeforeSweep
                         << "\tSNR Difference: " << params.snrDiffBeforeHO
                         << "\tMAX OBSERVED CELL: " << (unsigned)params.maxCellBeforeHandover
                         << std::endl;
    break;
  
  case BeamSweepTraceParams::UE_INITIATED_OUTAGE:
    m_beamSweepTraceFile << Simulator::Now().GetSeconds()
                         << "\tIMSI: " << params.imsi
                         << "\tUE"
                         << "\tSTARTED BEAM SWEEP. CURRENT CELL: " << (unsigned)params.currentCell
                         << "\tCURRENT SNR: " << params.snrBeforeSweep
                         << std::endl;
    break;
  case BeamSweepTraceParams::UE_INITIATED_BEAM_REFINEMENT:
    m_beamSweepTraceFile << Simulator::Now().GetSeconds() 
                         << "\tIMSI: " << params.imsi
                         << "\tGNB"
                         << "\tSTARTED BEAM REFINEMENT. CURRENT CELL: " << (unsigned)params.currentCell
                         << "\tCURRENT SNR: " << params.snrBeforeSweep
                         << std::endl;
    break;
  case BeamSweepTraceParams::UE_COMPLETED_BEAM_SWEEP:
    m_beamSweepTraceFile << Simulator::Now().GetSeconds()
                         << "\tIMSI: " << params.imsi
                         << "\tUE"
                         << "\tCOMPLETED BEAM SWEEP. PREV CELL: " << (unsigned)params.currentCell
                         << "\tFOUND CELL: " << (unsigned)params.foundCell
                         << "\tFOUND RX SECTOR: " << params.foundSector
                         << "\tFOUND RX ELEVATION: " << params.foundElevation
                         << std::endl;
    break;
  case BeamSweepTraceParams::GNB_CHECK_OMNI_SWEEP:
    m_beamSweepTraceFile << Simulator::Now ().GetSeconds() 
                         << "\tIMSI: " << params.imsi
                         << "\tGNB"
                         << "\tBEAM SWEEP DONE TO CHECK WHETHER FAULTY OMNI TX WAS PRESENT"
                         << std::endl;
    break;
  case BeamSweepTraceParams::GNB_RECVD_BEAM_REPORT:
    m_beamSweepTraceFile << Simulator::Now ().GetSeconds()
                         << "\tIMSI: " << params.imsi
                         << "\tGNB"
                         << "\tBEAM REPORT RECEIVED. CELL: " << (unsigned)params.foundCell
                         << "\tFOUND TX SECTOR: " << params.foundSector
                         << "\tFOUND TX ELEVATION: " << params.foundElevation
                         << std::endl;
    break;
  case BeamSweepTraceParams::UE_BEAM_ADJUSTMENT:
    m_beamSweepTraceFile << Simulator::Now ().GetSeconds ()
                         << "\tIMSI: " << params.imsi
                         << "\tUE"
                         << "\tADJUSTED BEAM: CELL: " <<(unsigned)params.foundCell
                         << "\tNEW RX SECTOR: " << params.foundSector
                         << "\tNEW RX ELEVATION" << params.foundElevation
                         << std::endl;
    break;
  case BeamSweepTraceParams::GNB_BEAM_ADJUSTMENT:
    m_beamSweepTraceFile << Simulator::Now ().GetSeconds ()
                         << "\tIMSI: " << params.imsi
                         << "\tGNB"
                         << "\tADJUSTED BEAM: CELL: " <<(unsigned)params.foundCell
                         << "\tNEW TX SECTOR: " << params.foundSector
                         << "\tNEW TX ELEVATION" << params.foundElevation
                         << std::endl;
  default:
    break;
  }
}

void 
NrPhyRxTrace::RadioLinkMonitoringTraceCallback (Ptr<NrPhyRxTrace> phyStats, std::string path, RadioLinkMonitoringTraceParams params)
{
  if (!m_radioLinkMonitoringTraceFile.is_open())
  {
    m_radioLinkMonitoringTraceFile.open(m_radioLinkMonitoringTraceFilename.c_str());
    if (!m_radioLinkMonitoringTraceFile.is_open())
    {
      NS_FATAL_ERROR("Could not open tracefile");
    }
  }

  switch (params.m_radioLinkMonitoringOrigin)
  {
  case RadioLinkMonitoringTraceParams::UE_PERIODIC_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tUE PERIODIC SWEEP."
                                   << "\tBEAMS FROM CELLS: "
                                   << (unsigned)params.beamsTbRLM[0].first << ", "
                                   << (unsigned)params.beamsTbRLM[1].first << ", "
                                   << (unsigned)params.beamsTbRLM[2].first << ", "
                                   << (unsigned)params.beamsTbRLM[3].first
                                   << " ARE NOW MONITORED."
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::UE_EXPLICIT_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tUE EXPLICIT SWEEP."
                                   << "\tBEAMS FROM CELLS: "
                                   << (unsigned)params.beamsTbRLM[0].first << ", "
                                   << (unsigned)params.beamsTbRLM[1].first << ", "
                                   << (unsigned)params.beamsTbRLM[2].first << ", "
                                   << (unsigned)params.beamsTbRLM[3].first
                                   << " ARE NOW MONITORED."
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_AFTER_UE_IA_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tAFTER UE IA BEAM SWEEP:"
                                   << "\tINITIATED CONNECTION ESTABLISHMENT TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_AFTER_RRC_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tAFTER RRC BEAM SWEEP DUE TO SNR BELOW THRESHOLD:"
                                   << "\tHANDOVER SCHEDULED FROM CELL: " << (unsigned)params.sourceCellId
                                   << " TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::NO_HANDOVER_AFTER_RRC_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tAFTER RRC BEAM SWEEP DUE TO SNR BELOW THRESHOLD:"
                                   << "\tNO HANDOVER WAS SCHEDULED: "
                                   << "\tSNR HAS RECOVERED OR SERVING CELL REMAINS TO PROVIDE THE BEST SNR."
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_AFTER_UE_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tAFTER UE BEAM TRACKING SWEEP:"
                                   << "\tHANDOVER SCHEDULED FROM CELL: " << (unsigned)params.sourceCellId
                                   << " TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::NO_HANDOVER_AFTER_UE_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tAFTER UE BEAM TRACKING SWEEP:"
                                   << "\tNO HANDOVER WAS SCHEDULED: "
                                   << "SERVING CELL REMAINS TO PROVIDE THE BEST SNR."
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_WITHOUT_SWEEP:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tHANDOVER SCHEDULED WITHOUT BEAM SWEEP FROM CELL: " << (unsigned)params.sourceCellId
                                   << " TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_ADAPTION:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tHANDOVER WAS ADAPTED: "
                                   << " NEW TARGET IS CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_CANCELLED:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tHANDOVER WAS CANCELLED."
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_EXECUTED:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tHANDOVER IS EXECUTED FROM CELL: " << (unsigned)params.sourceCellId
                                   << " TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  case RadioLinkMonitoringTraceParams::HANDOVER_LOAD_BALANCING:
    m_radioLinkMonitoringTraceFile << Simulator::Now().GetSeconds()
                                   << "\tIMSI: " << params.imsi
                                   << "\tLOAD BALANCING HANDOVER:"
                                   << "\tHANDOVER INITIATED FROM CELL: " << (unsigned)params.sourceCellId
                                   << " TO CELL: " << (unsigned)params.targetCellId
                                   << std::endl;
    break;
  default:
    break;
  }
}

void
NrPhyRxTrace::SetMacrxThruFilename (std::string fileName)
{
  m_rxMacThruFilename = fileName;
}

void
NrPhyRxTrace::SetOutputFilename ( std::string fileName)
{
  NS_LOG_INFO ("Filename: " << fileName);
  m_rxPacketTraceFilename = fileName;
}

void
NrPhyRxTrace::SetrxThruFilename ( std::string fileName)
{
  m_rxThruFilename = fileName;
}

void
NrPhyRxTrace::SetRsrpFilename ( std::string fileName)
{
  m_rsrpFilename = fileName;
}

void
NrPhyRxTrace::SetDlCtrlSinrFilename ( std::string fileName)
{
  m_dlCtrlSinrFileName = fileName;
}

void NrPhyRxTrace::SetSinrEstimateFilename(std::string fileName)
{
  m_sinrEstimateFileName = fileName;
}

void
NrPhyRxTrace::SetBeamSweepFileName ( std::string fileName)
{
  m_beamSweepTraceFilename = fileName;
}

void
NrPhyRxTrace::SetRadioLinkMonitoringFileName ( std::string fileName)
{
  m_radioLinkMonitoringTraceFilename = fileName;
}

void
NrPhyRxTrace::SetBeamSweepType (std::string beamSweepType)
{
  if (beamSweepType == "Ideal")
  {
    m_realisticIA = false;
  }
  else if (beamSweepType == "Real")
  {
    m_realisticIA = true;
  }
  else
  {
    NS_ABORT_MSG ("Unidentified beam sweep type");
  }
}

//New Trace! Should Trace Phy Throughput
void
NrPhyRxTrace::RxNewTraceUeCallback (Ptr<NrPhyRxTrace> phyStats, std::string path, RxPacketTraceParams params)
{
  if (lastFrameNum.find(params.m_imsi) == lastFrameNum.end()) {
    //this is the first time calling this function for params.m_imsi
    //initialise some data structures
    lastFrameNum.insert(std::pair<uint64_t,uint32_t>(params.m_imsi, 1));
    totalBytesReceived.insert(std::pair<uint64_t,uint32_t>(params.m_imsi, 0));
    timeLastOutput.insert(std::pair<uint64_t,double>(params.m_imsi, 0));
  }

  totalBytesReceived.at(params.m_imsi) += params.m_tbSize;
  bool canWrite;
  if (m_realisticIA)
  {
    canWrite = ((Simulator::Now ().GetSeconds () - timeLastOutput.at(params.m_imsi)) > 0.04) &&
       (params.m_frameNum % 2 == 1 &&
        params.m_subframeNum == 5 &&
       ((params.m_frameNum - lastFrameNum.at(params.m_imsi)) % 2  == 0));
  }
  else
  {
    canWrite = (Simulator::Now ().GetSeconds () - timeLastOutput.at(params.m_imsi)) > 0.05;
  }
  
  if (canWrite)
  {
    if (!m_rxThruFile.is_open ())
    {
      m_rxThruFile.open (m_rxThruFilename.c_str ());
    }

    m_rxThruFile <<  Simulator::Now ().GetSeconds () << "\t"
                          << params.m_imsi << "\t"
                          << totalBytesReceived.at(params.m_imsi) * 8/ (Simulator::Now ().GetSeconds () - timeLastOutput.at(params.m_imsi))
                          << std::endl;
    timeLastOutput.at(params.m_imsi) = Simulator::Now ().GetSeconds ();
    lastFrameNum.at(params.m_imsi) = params.m_frameNum;
    totalBytesReceived.at(params.m_imsi) = 0;
  }
}

} /* namespace ns3 */
