/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) Author: Sangmo Sung, 
*   2021 University of Hanyang, Dep. of Electronics Convergence Engineering, UN lab.
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
*/
/*
  In this examples, we considered two groups of vehicles traveling in
  the same direction, either in the same ([sameLane] = true) or different lanes
  ([sameLane] = false). Each group is composed of two vehicles, one behind the
  other, moving at a constant speed of [speed] m/s and keeping a safety distance of
  [intraGroupDistance] m. Within a group, the rear vehicle acts as a server and
  generates data packets which are sent to the front vehicle. We considered an
  ON-OFF traffic model, in which a UDP source keeps switching between the ON and
  the OFF states. During the ON state, the source generates packets at a
  constant rate for [onPeriod] ms, while in the OFF state it stays idle for a random
  amount of time, which follows an exponential distribution with mean
  [offPeriod] ms. All vehicles operate at 60 GHz with a bandwidth of 250 MHz,
  possibly interfering in case of concurrent transmissions, and are equipped
  with a Uniform Planar Array (UPA) of [numAntennaElements] antenna elements to
  establish directional communications.
  The simulation runs for [stopTime] ms, at outputs the overall Packet Reception
  Ratio.
*/

#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-propagation-loss-model.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/rain-snow-attenuation.h"
#include "ns3/system-wall-clock-ms.h"
#include "ns3/traci-applications-module.h"
#include "ns3/traci-module.h"
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"

NS_LOG_COMPONENT_DEFINE ("NRV2VmmWaveSimulator");

using namespace ns3;
using namespace millicar;

AsciiTraceHelper asciiTraceHelper;
Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("Test/Test_data_220214_article/slv_s60_i50.0_k0.8606_a0.7656_v_R_Urban.txt");

// 1. Variables
 
// VEHICLE PARAMETER - Platoon scenario
double interGroupDistance = 20;         // distance between the two groups in meters
bool sameLane = false;                  // if true the two groups lie on the same lane (as a platoon), if false they lie on adjacent lanes
double speed_kmh = 60.0;                // All car 
double speed = speed_kmh/3.6;           // convert km/h to m/s
std::string scenario = "V2V-Urban";     //'V2V-Highway', 'V2V-Urban', 'Extended-V2V-Highway','Extended-V2V-Urban'

// WEATHER PARAMETER
std::string weatCond = "2";             // Sunny: 1, Rainy: 2, Rainy and Snowy: 3
bool combined_rain_snow = false;        // rainy & snowy condition: true
uint32_t intensityOfRain = 50.0;        // rain intensity in mm/h, not worked "R"
double k = 0.8606;                         // 0.8606 regression coffiecient for computing specific rain attenuation // (horizontal/vertical polarization) "k"
double alpha = 0.7656;                     // 0.7656 regression coffiecient for computing specific rain // attenuation (horizontal/vertical polarization) "a"
std::string channel_condition = "v";    // l: los , n: nlos, v: nlosv, a: all
std::string full_channel_condition = "";

// 2. Constants(Changable)

// PROPAGATION PARAMETER
double h0 = 0.0;                        // mean annual 0C isotherm height above mean sea level
double altitude = 0.0;                  // altitude in meters above the sea level
bool orthogonalResources = true;        // if true, resouces are orthogonal among the two groups, if false resources are shared
double pathLossVal = 0.0;
uint64_t delayNs;
double bandwidth = 2.5e8;               // bandwidth in Hz 250[MHz]
bool UseAmc = false;  
double frequency = 60e9;                // frequency in Hz 60[GHz]
double dataRate = 100e6;                // data rate in bps 100[Mbps]
uint32_t mcs = 28;                      // modulation and coding scheme
uint32_t numAntennaElements = 4;        // number of antenna elements
bool shandowing = false;

// PACKET NUM COUNTER
uint32_t g_txPacketsGroup1 = 0; // tx packet counter for group 1
uint32_t g_txPacketsGroup2 = 0; // tx packet counter for group 2
uint32_t g_rxPacketsGroup1 = 0; // rx packet counter for group 1
uint32_t g_rxPacketsGroup2 = 0; // rx packet counter for group 2 

// 3. Simulation 

// TX, RX SETTING
static void Tx (Ptr<OutputStreamWrapper> stream, uint8_t group, Ptr<const Packet> p)
{
  *stream->GetStream () << "Tx\t" << Simulator::Now ().GetSeconds () << "\t" << p->GetSize () << std::endl;
  if (group == 1)
  {
    ++g_txPacketsGroup1;
  }
  else if (group == 2)
  {
    ++g_txPacketsGroup2;
  }
}
static void Rx (Ptr<OutputStreamWrapper> stream, uint8_t group, Ptr<const Packet> packet, const Address& from)
{
  Ptr<Packet> newPacket = packet->Copy ();
  SeqTsHeader seqTs;
  newPacket->RemoveHeader (seqTs);
  if (seqTs.GetTs ().GetNanoSeconds () != 0)
  {
    delayNs = Simulator::Now ().GetNanoSeconds () - seqTs.GetTs ().GetNanoSeconds ();
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize() << "\t" <<  delayNs << std::endl;
  }
  else
  {
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize() << std::endl;
  }
  if (group == 1)
  {
    ++g_rxPacketsGroup1;
  }
  else if (group == 2)
  {
    ++g_rxPacketsGroup2;
  }
}

// SUMO LINKING
Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();

// COMPUTE PATHLOSS
void computePathLoss(NetDeviceContainer, NetDeviceContainer, double);

// MAIN CODE
int main(int argc, char *argv[]) {
  if (channel_condition == "a")
  {
    full_channel_condition = "all";
  }
  else if(channel_condition =="l")
  {
    full_channel_condition = "los";
  }
  else if(channel_condition == "n")
  {
    full_channel_condition = "nlos";
  } 
  else if(channel_condition == "v")
  {
    full_channel_condition = "nlos-v";
  }
  uint32_t simulationRun = 2;   // number of simulation - to be applied
  uint32_t startTime = 10;      // application start time in milliseconds
  uint32_t stopTime = 500;      // application stop time in milliseconds
  uint32_t onPeriod = 100;      // on period duration in milliseconds
  uint32_t offPeriod = 100;     // mean duration of the off period in milliseconds

  ns3::Time simulationTime(ns3::Seconds(80)); // according to SUMO simulation time
  double stepTime = 0.1;
  CommandLine cmd;
 
  cmd.AddValue("startTime", "application stop time in milliseconds", startTime);
  cmd.AddValue("stopTime", "application stop time in milliseconds", stopTime);
  cmd.AddValue("onPeriod", "on period duration in milliseconds", onPeriod);
  cmd.AddValue("offPeriod", "mean duration of the off period in milliseconds", offPeriod);
  cmd.AddValue("dataRate", "data rate in bps", dataRate);
  cmd.AddValue("UseAmc", "AMC use", UseAmc);
  cmd.AddValue("mcs", "modulation and coding scheme", mcs);
  cmd.AddValue("interGroupDistance", "distance between the two groups in meters", interGroupDistance);
  cmd.AddValue("speed", "the speed of the vehicles in m/s", speed);
  cmd.AddValue("numAntennaElements", "number of antenna elements", numAntennaElements);
  cmd.AddValue("orthogonalResources", "if true, resouces are orthogonal among the two groups, if false resources are shared", orthogonalResources);
  cmd.AddValue("sameLane", "if true the two groups lie on the same lane, if false they lie on adjacent lanes", sameLane);
  cmd.AddValue("intensityOfRain", "Specifies the value of rain intensity", intensityOfRain);
  cmd.AddValue("combined_rain_snow", "Defines weather the attenuation from combined rain and wet snow should be computed", combined_rain_snow);
  cmd.AddValue("channel_condition", "Defines the channel condition", channel_condition);
  cmd.AddValue("scenario", "Defines the scenario where the communication takes place", scenario);
  cmd.AddValue("simulationRun", "Defines the number of simulations", simulationRun);
  cmd.AddValue("k", "Regression coefficient k", k);
  cmd.AddValue("alpha", "Regression coefficient alpha", alpha);
  cmd.AddValue("altitude", "Altitude in meters above the sea level", altitude);
  cmd.AddValue("h0", "Mean annual 0C isotherm height above mean sea level", h0);

  cmd.Parse(argc, argv);

  Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(frequency));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Scenario", StringValue(scenario));
  Config::SetDefault("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue(bandwidth));
  Config::SetDefault("ns3::RainAttenuation::RainRate", UintegerValue(intensityOfRain));
  Config::SetDefault("ns3::RainAttenuation::k", DoubleValue(k));
  Config::SetDefault("ns3::RainAttenuation::alpha", DoubleValue(alpha));
  Config::SetDefault("ns3::RainSnowAttenuation::altitude", DoubleValue(altitude));
  Config::SetDefault("ns3::RainSnowAttenuation::h0", DoubleValue(h0));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Shadowing", BooleanValue(shandowing));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::SnowEffect", BooleanValue(combined_rain_snow));
  Config::SetDefault("ns3::MmWaveVehicularSpectrumPropagationLossModel::UpdatePeriod", TimeValue(MilliSeconds(1)));
  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (UseAmc));
  Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (mcs));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue (channel_condition)); //'l' for LOS, 'n' for NLOS, 'v' for NLOSv, 'a' for all.
  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (500*1024));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElements", UintegerValue (numAntennaElements));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElementPattern", StringValue ("3GPP-V2V"));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::IsotropicAntennaElements", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::NumSectors", UintegerValue (2));

  // SUMO Configuration
  sumoClient->SetAttribute("SumoConfigPath", StringValue("scratch/nrv2v_mmwave_sim_cv/sumo_ns3_example.sumocfg"));
  // sumoClient->SetAttribute("SumoBinaryPath", StringValue("")); // use system installation of sumo
  sumoClient->SetAttribute("SynchInterval", TimeValue(Seconds(1.0)));
  sumoClient->SetAttribute("StartTime", TimeValue(Seconds(0.0)));
  sumoClient->SetAttribute("SumoPort", UintegerValue(3400));
  sumoClient->SetAttribute("PenetrationRate", DoubleValue(1.0)); // portion of vehicles equipped with 5G terminal
  sumoClient->SetAttribute("SumoLogFile", BooleanValue(true));
  sumoClient->SetAttribute("SumoStepLog", BooleanValue(false));
  sumoClient->SetAttribute("SumoSeed", IntegerValue(10));
  sumoClient->SetAttribute("SumoAdditionalCmdOptions", StringValue("--fcd-output sumoTrace.xml"));
  sumoClient->SetAttribute("SumoWaitForSocket", TimeValue(Seconds(1.0)));
  sumoClient->SetAttribute("SumoGUI", BooleanValue(true));

  // Create the nodes
  NodeContainer group1, group2;
  group1.Create (2);
  group2.Create (2);
  uint32_t nodeCounter(0);
  
  // Create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (group1);
  mobility.Install (group2);

  double intraGroupDistance = std::max (2.0, 2*speed); // distance between cars belonging to the same group in meters
  group1.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  group1.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  group1.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance,0,0));
  group1.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  if (sameLane)
  {
    group2.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance+interGroupDistance,0,0));
    group2.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance*2+interGroupDistance,0,0));
  }
  else
  {
    group2.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance,interGroupDistance,0));
    group2.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance*2,interGroupDistance,0));
  }
  group2.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));
  group2.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  // Create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();
  helper->SetPropagationLossModelType("ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  helper->SetNumerology(3); //https://www.sharetechnote.com/html/5G/5G_FrameStructure.html
  // NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices(n);
  NetDeviceContainer devs1 = helper->InstallMmWaveVehicularNetDevices (group1);
  NetDeviceContainer devs2 = helper->InstallMmWaveVehicularNetDevices (group2);
  InternetStackHelper internet;
  // internet.Install(n);
  internet.Install (group1);
  internet.Install (group2);

  Ipv4AddressHelper ipv4;

  // helper->PairDevices(devs);
  // NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devs1);

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  i = ipv4.Assign (devs2);
  
  if (orthogonalResources)
  {
    // resources are orthogonally partitioned among the two groups
    helper->PairDevices (NetDeviceContainer (devs1, devs2));
  }
  else
  {
    // resources are othogally partitioned among devices belonging to the
    // same group, while shared among the two groups
    helper->PairDevices(devs1);
    helper->PairDevices(devs2);
  }

  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (group1.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (group1.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  NS_LOG_DEBUG("IPv4 Address node 0 group 1: " << group1.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1 group 1: " << group1.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());

  staticRouting = ipv4RoutingHelper.GetStaticRouting (group2.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (group2.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  // create the random variables used to setup the applications
  Ptr<ConstantRandomVariable> onPeriodRv = CreateObjectWithAttributes<ConstantRandomVariable> ("Constant", DoubleValue (onPeriod / 1000.0));
  Ptr<ExponentialRandomVariable> offPeriodRv = CreateObjectWithAttributes<ExponentialRandomVariable> ("Mean", DoubleValue (offPeriod / 1000.0));

  // create the appplications for group 1
  uint32_t port = 1234;
  OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress (group1.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port)));
  onoff.SetConstantRate (DataRate (std::to_string (dataRate)+"b/s"));
  onoff.SetAttribute ("OnTime", PointerValue (onPeriodRv));
  onoff.SetAttribute ("OffTime", PointerValue (offPeriodRv));
  ApplicationContainer onOffApps = onoff.Install (group1.Get (0)); 

  PacketSinkHelper sink ("ns3::UdpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
  ApplicationContainer packetSinkApps = sink.Install (group1.Get (1)); //1,0 -> 1,1
  // ApplicationContainer packetSinkApps = sink.Install (group2.Get (0)); //1,0 -> 1,1

  // create the applications for group 2
  onoff.SetAttribute ("Remote", AddressValue (InetSocketAddress (group2.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port)));
  onOffApps.Add (onoff.Install (group2.Get (0)));

  sink.SetAttribute ("Local", AddressValue (InetSocketAddress (Ipv4Address::GetAny (), port)));
  packetSinkApps.Add (sink.Install (group2.Get (1))); //2,0 -> 2,1
  // packetSinkApps.Add (sink.Install (group1.Get (0))); //2,0 -> 2,1

  onOffApps.Start (MilliSeconds (startTime));
  onOffApps.Stop (MilliSeconds (stopTime));

  packetSinkApps.Start (MilliSeconds (0.0));

  // connect the trace sources to the sinks
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("group-1.txt");
  onOffApps.Get (0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 1));
  packetSinkApps.Get (0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream, 1));

  stream = asciiTraceHelper.CreateFileStream ("group-2.txt");
  onOffApps.Get(1)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 2));
  packetSinkApps.Get (1)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream, 2));


  VehicleSpeedControlHelper vehicleSpeedControlHelper(9);
  vehicleSpeedControlHelper.SetAttribute("Client", (PointerValue)sumoClient);

  // a callback function to setup the nodes
  std::function<Ptr<Node>()> setupNew5GNode = [&]() -> Ptr<Node> {
    if (nodeCounter >= group1.GetN())
      NS_FATAL_ERROR("Node Container empty!: " << nodeCounter
                                               << " nodes created.");

    // don't create and install the protocol stack of the node at simulation
    // time -> take from "node pool"
    Ptr<Node> includedNode = group1.Get(nodeCounter);
    ++nodeCounter; // increment counter for next node
    if (nodeCounter == 2)
    {
      nodeCounter = 0;
    }
    // Install Application
    ApplicationContainer vehicleSpeedControlApps =
    vehicleSpeedControlHelper.Install(includedNode);
    vehicleSpeedControlApps.Start(Seconds(0.0));
    vehicleSpeedControlApps.Stop(simulationTime);

    return includedNode;
  };
  // a callback function for node shutdown
  std::function<void(Ptr<Node>)> shutdown5GNode = [](Ptr<Node> exNode) {
    // stop all applications
    Ptr<VehicleSpeedControl> vehicleSpeedControl =
        exNode->GetApplication(0)->GetObject<VehicleSpeedControl>();
    if (vehicleSpeedControl)
      vehicleSpeedControl->StopApplicationNow();

    // set position outside communication range in SUMO
    Ptr<ConstantPositionMobilityModel> mob =
        exNode->GetObject<ConstantPositionMobilityModel>();
    mob->SetPosition(
        Vector(-100.0 + (rand() % 25), 320.0 + (rand() % 25), 250.0));
  };

  // start traci client
  sumoClient->SumoSetup(setupNew5GNode, shutdown5GNode);
  computePathLoss(devs1, devs2, stepTime);
  
  for(int i=0; i<5; i++){
  Simulator::Stop(simulationTime);
  // AnimationInterface anim ("animation.xml");
  Simulator::Run();
  Simulator::Destroy();
  }
  
}

void computePathLoss(NetDeviceContainer devs1, NetDeviceContainer devs2, double stepTime) {

  auto channel = DynamicCast<MmWaveVehicularNetDevice>(devs1.Get(0))
                     ->GetPhy()
                     ->GetSpectrumPhy()
                     ->GetSpectrumChannel();
  PointerValue plm; //path loss model
  channel->GetAttribute("PropagationLossModel", plm);
  Ptr<MmWaveVehicularPropagationLossModel> pathloss = DynamicCast<MmWaveVehicularPropagationLossModel>(plm.Get<PropagationLossModel>());

  Ptr<MobilityModel> mobileNode1 = devs1.Get(0)->GetNode()->GetObject<MobilityModel>();
  Ptr<MobilityModel> mobileNode2 = devs2.Get(0)->GetNode()->GetObject<MobilityModel>();
  Vector3D pos = mobileNode1->GetPosition();
  Vector3D pos1 = mobileNode2->GetPosition();
  std::cout << "\n The Distance between Group1 0th and Groups1 1th is: " << pos-pos1 << std::endl;
  
  pathLossVal = pathloss->GetLoss(mobileNode1, mobileNode2);
  std::cout << "\n The value of the path loss is: " << pathLossVal << std::endl;
  
  double distance3D = mobileNode2->GetDistanceFrom(mobileNode1);                              // Distance ??????
  double weatherAtten = pathloss->GetWeatherAttenuation(distance3D, pos.z, pos1.z);           // Weather Atten. ??? ??????

  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << speed_kmh << "\t" << intensityOfRain << "\t" << pathLossVal << "\t" 
                        << distance3D << "\t" << weatherAtten << "\t" << weatCond << "\t" << scenario << "\t" << intensityOfRain << "\t" << k << "\t" << alpha << "\t" << full_channel_condition << std::endl;

  std::cout << "\n The additional value of the weather attenuation is: "
            << weatherAtten << std::endl;
  Simulator::Schedule(Seconds(stepTime), &computePathLoss, devs1, devs2, stepTime);
}
