/* Opis: UAANET routing simulacija sa izborom ns-3 mobility modela
 * 10 UAV cvorova, mjerenje PDR, delay, overhead, throughput
 *
 * NAPOMENA o saobracaju:
 * Koristi se OnOffApplication (izvor) + PacketSink (odredište) umjesto
 * UdpEchoClient/Server. Razlog: UdpEcho generise saobracaj u OBA smjera
 * (request klijent->server na portu 9/10, i reply server->klijent na
 * ephemeral portu), pa FlowMonitor klasifikator vidi dva razlicita toka
 * za isti "logicki" prijenos i komplikuje filtriranje po portu.
 * OnOff/PacketSink salje samo u JEDNOM smjeru (izvor->sink), sa fiksnim
 * destinationPort = 9 (data) ili 10 (C2), pa je filtriranje jednoznacno.
 */
 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/aodvetx-module.h"
#include "ns3/qspu-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/group-mobility-helper.h"
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>
#include <limits>
#include <algorithm>
#include <cctype>
#include <string>
 
using namespace ns3;
 
NS_LOG_COMPONENT_DEFINE("UAANETRouting");

// ===============================================
// Remote-ID broadcast statistika 
// ===============================================

static uint64_t g_remoteIdTxPackets = 0;
static uint64_t g_remoteIdTxBytes = 0;

static uint64_t g_remoteIdRxPackets = 0;
static uint64_t g_remoteIdRxBytes = 0;

// NodeId -> broj primljenih Remote-ID paketa
static std::map<uint32_t, uint64_t> g_remoteIdRxPerNode;

// NodeId -> IP adresa, koristi se da ne brojimo self-reception ako se desi 
static std::map<uint32_t, Ipv4Address> g_nodeIpAddress;


void ReceiveRemoteIdPacket(Ptr<Socket> socket){
 Address from;
 Ptr<Packet>packet;
 
 while((packet=socket->RecvFrom(from))){
  uint32_t receiverNodeId = socket -> GetNode()->GetId();
  
  // Ako socket vrati source Ip, pokusaj ignorisati paket
  // ako ga je primio isti node koji ga je poslao 
  
  if(InetSocketAddress:: IsMatchingType(from)){
   Ipv4Address senderIp= InetSocketAddress::ConvertFrom(from).GetIpv4();
   auto it = g_nodeIpAddress.find(receiverNodeId);
   if(it != g_nodeIpAddress.end() && it->second == senderIp){
    continue;
   }
  }
  g_remoteIdRxPackets++;
  g_remoteIdRxBytes+=packet->GetSize();
  g_remoteIdRxPerNode[receiverNodeId]++;
 }
}
void
SendRemoteIdBroadcast(Ptr<Socket> socket,
                      uint32_t packetSize,
                      uint16_t port,
                      Time interval,
                      Time stopTime)
{
    if (Simulator::Now() >= stopTime)
    {
        return;
    }

    Ptr<Packet> packet = Create<Packet>(packetSize);

    socket->SendTo(
        packet,
        0,
        InetSocketAddress(Ipv4Address("255.255.255.255"), port)
    );

    g_remoteIdTxPackets++;
    g_remoteIdTxBytes += packetSize;

    if (Simulator::Now() + interval < stopTime)
    {
        Simulator::Schedule(
            interval,
            &SendRemoteIdBroadcast,
            socket,
            packetSize,
            port,
            interval,
            stopTime
        );
    }
}


// ===============================================
// Izbor UAV mobility modela
// Podrzani modeli:
//   RWP          -> ns3::RandomWaypointMobilityModel
//   RWALK2D      -> ns3::RandomWalk2dMobilityModel
//   GAUSS_MARKOV -> ns3::GaussMarkovMobilityModel
//   GROUP        -> ns3::GroupMobilityHelper
//                   parent/reference: ns3::RandomWaypointMobilityModel
//                   member/child:     ns3::RandomWalk2dMobilityModel
// ===============================================

static std::string
NormalizeMobilityName(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    if (value == "RANDOMWAYPOINT" || value == "RWP")
    {
        return "RWP";
    }
    if (value == "RANDOMWALK" || value == "RANDOMWALK2D" || value == "RWALK" || value == "RWALK2D")
    {
        return "RWALK2D";
    }
    if (value == "GAUSSMARKOV" || value == "GAUSS_MARKOV" || value == "GM")
    {
        return "GAUSS_MARKOV";
    }
    if (value == "GROUP" || value == "GROUPMOBILITY" || value == "RPGM")
    {
        return "GROUP";
    }

    NS_FATAL_ERROR("Unknown mobilityModel: " << value
                   << ". Use RWP, RWALK2D, GAUSS_MARKOV or GROUP.");
    return "RWP";
}

static Ptr<RandomBoxPositionAllocator>
CreateUavBoxAllocator(double areaSize,
                      double uavMinAltitude,
                      double uavMaxAltitude)
{
    Ptr<RandomBoxPositionAllocator> allocator = CreateObject<RandomBoxPositionAllocator>();

    Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable>();
    xVal->SetAttribute("Min", DoubleValue(0.0));
    xVal->SetAttribute("Max", DoubleValue(areaSize));

    Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable>();
    yVal->SetAttribute("Min", DoubleValue(0.0));
    yVal->SetAttribute("Max", DoubleValue(areaSize));

    Ptr<UniformRandomVariable> zVal = CreateObject<UniformRandomVariable>();
    zVal->SetAttribute("Min", DoubleValue(uavMinAltitude));
    zVal->SetAttribute("Max", DoubleValue(uavMaxAltitude));

    allocator->SetAttribute("X", PointerValue(xVal));
    allocator->SetAttribute("Y", PointerValue(yVal));
    allocator->SetAttribute("Z", PointerValue(zVal));

    return allocator;
}

static Ptr<RandomBoxPositionAllocator>
CreateGroupMemberOffsetAllocator(double groupRadius)
{
    Ptr<RandomBoxPositionAllocator> allocator = CreateObject<RandomBoxPositionAllocator>();

    Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable>();
    xVal->SetAttribute("Min", DoubleValue(-groupRadius));
    xVal->SetAttribute("Max", DoubleValue(groupRadius));

    Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable>();
    yVal->SetAttribute("Min", DoubleValue(-groupRadius));
    yVal->SetAttribute("Max", DoubleValue(groupRadius));

    // RandomWalk2d mijenja samo x/y komponentu child modela.
    // Z se ovdje koristi kao fiksni pocetni vertikalni offset u odnosu na parent.
    Ptr<UniformRandomVariable> zVal = CreateObject<UniformRandomVariable>();
    zVal->SetAttribute("Min", DoubleValue(-10.0));
    zVal->SetAttribute("Max", DoubleValue(10.0));

    allocator->SetAttribute("X", PointerValue(xVal));
    allocator->SetAttribute("Y", PointerValue(yVal));
    allocator->SetAttribute("Z", PointerValue(zVal));

    return allocator;
}

static std::string
BuildSpeedRandomVariable(double nodeMinSpeed,
                         double nodeSpeed,
                         bool fixedNodeSpeed)
{
    if (fixedNodeSpeed)
    {
        return "ns3::ConstantRandomVariable[Constant=" + std::to_string(nodeSpeed) + "]";
    }

    return "ns3::UniformRandomVariable[Min=" + std::to_string(nodeMinSpeed) +
           "|Max=" + std::to_string(nodeSpeed) + "]";
}

static void
InstallUavMobility(NodeContainer uavNodes,
                   const std::string& mobilityModel,
                   double areaSize,
                   double nodeMinSpeed,
                   double nodeSpeed,
                   bool fixedNodeSpeed,
                   double nodePause,
                   double uavMinAltitude,
                   double uavMaxAltitude,
                   double randomWalkTime,
                   double gaussMarkovAlpha,
                   double gaussMarkovTimeStep,
                   double groupRadius)
{
    std::string model = NormalizeMobilityName(mobilityModel);
    std::string speedRv = BuildSpeedRandomVariable(nodeMinSpeed, nodeSpeed, fixedNodeSpeed);

    Ptr<RandomBoxPositionAllocator> uavPositionAlloc =
        CreateUavBoxAllocator(areaSize, uavMinAltitude, uavMaxAltitude);

    Box boxBounds(0.0, areaSize,
                  0.0, areaSize,
                  uavMinAltitude, uavMaxAltitude);

    Rectangle rectangleBounds(0.0, areaSize,
                              0.0, areaSize);

    MobilityHelper uavMobility;
    uavMobility.SetPositionAllocator(uavPositionAlloc);

    if (model == "RWP")
    {
        uavMobility.SetMobilityModel(
            "ns3::RandomWaypointMobilityModel",
            "Speed", StringValue(speedRv),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(nodePause) + "]"),
            "PositionAllocator", PointerValue(uavPositionAlloc));
        uavMobility.Install(uavNodes);
    }
    else if (model == "RWALK2D")
    {
        uavMobility.SetMobilityModel(
            "ns3::RandomWalk2dMobilityModel",
            "Mode", StringValue("Time"),
            "Time", TimeValue(Seconds(randomWalkTime)),
            "Speed", StringValue(speedRv),
            "Bounds", RectangleValue(rectangleBounds));
        uavMobility.Install(uavNodes);
    }
    else if (model == "GAUSS_MARKOV")
    {
        uavMobility.SetMobilityModel(
            "ns3::GaussMarkovMobilityModel",
            "Bounds", BoxValue(boxBounds),
            "TimeStep", TimeValue(Seconds(gaussMarkovTimeStep)),
            "Alpha", DoubleValue(gaussMarkovAlpha),
            "MeanVelocity", StringValue(speedRv),
            "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283185307179586]"),
            "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.20|Max=0.20]"),
            "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=2.0]"),
            "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.8]"),
            "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.05|Bound=0.3]"));
        uavMobility.Install(uavNodes);
    }
    else if (model == "GROUP")
    {
        Ptr<RandomBoxPositionAllocator> referencePositionAlloc =
            CreateUavBoxAllocator(areaSize, uavMinAltitude, uavMaxAltitude);

        Ptr<RandomBoxPositionAllocator> memberOffsetAlloc =
            CreateGroupMemberOffsetAllocator(groupRadius);

        Rectangle memberBounds(-groupRadius, groupRadius,
                               -groupRadius, groupRadius);

        GroupMobilityHelper groupMobility;
        groupMobility.SetReferencePositionAllocator(referencePositionAlloc);
        groupMobility.SetReferenceMobilityModel(
            "ns3::RandomWaypointMobilityModel",
            "Speed", StringValue(speedRv),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(nodePause) + "]"),
            "PositionAllocator", PointerValue(referencePositionAlloc));

        groupMobility.SetMemberPositionAllocator(memberOffsetAlloc);
        groupMobility.SetMemberMobilityModel(
            "ns3::RandomWalk2dMobilityModel",
            "Mode", StringValue("Time"),
            "Time", TimeValue(Seconds(randomWalkTime)),
            "Speed", StringValue(speedRv),
            "Bounds", RectangleValue(memberBounds));

        groupMobility.Install(uavNodes);
    }
}

int main(int argc, char *argv[])
{
 // Parametri simulacije
 uint32_t nGcs = 1;
 uint32_t nUavs= 10;
 uint32_t nNodes = nGcs + nUavs;
 uint32_t run = 1;
 double simTime = 100.0; // sekundi
 double areaSize = 1000.0; // dimenzija kvadratnog podrucja (m)
 double nodeSpeed = 20.0; // m/s (tipicna UAV brzina)
 double nodePause = 2.0; // pauza na waypoint-u
 double nodeMinSpeed = 10.0; // minimalna brzina za random speed RV
 bool fixedNodeSpeed = false; // ako je true, koristi tacno nodeSpeed
 std::string mobilityModel = "RWP"; // RWP, RWALK2D, GAUSS_MARKOV, GROUP
 double randomWalkTime = 1.0; // s, period promjene smjera za RandomWalk2d
 double gaussMarkovAlpha = 0.85;
 double gaussMarkovTimeStep = 1.0; // s
 double groupRadius = 50.0; // m, relativni prostor kretanja clanova grupe
 double txRange = 300.0; // domet radio modula (m)
 double uavMinAltitude = 20.0;
 double uavMaxAltitude = 120.0;
 double txPowerDbm = 20.0;
 double rxSensitivityDbm = -85.0;
 bool enableRemoteId = false;
 bool enableNetAnim = false;
 bool enableFlowMonitorXml = false;
 std::string routingProtocol = "AODV";
// std::string txtFileName = "aodv_results.txt";
 std::string csvFileName = "aodv_results.csv";
 std::string flowmonXmlFileName = "aodv-flowmon.xml";
 std::string netanimFileName = "aodv-anim.xml";
 
 CommandLine cmd;
 cmd.AddValue("nGcs", "Broj GCS cvorova", nGcs);
 cmd.AddValue("nUavs", "Broj UAV cvorova", nUavs);
 cmd.AddValue("run", "RNG run broj za nezavisna ponavljanja", run);
 cmd.AddValue("simTime", "Trajanje simulacije (s)", simTime);
 cmd.AddValue("areaSize", "Dimenzija kvadratnog podrucja (m)", areaSize);
 cmd.AddValue("nodeSpeed", "Maksimalna brzina UAV (m/s)", nodeSpeed);
 cmd.AddValue("nodeMinSpeed", "Minimalna brzina UAV za random speed RV (m/s)", nodeMinSpeed);
 cmd.AddValue("fixedNodeSpeed", "Ako je true, koristi tacno nodeSpeed umjesto Uniform[min,max]", fixedNodeSpeed);
 cmd.AddValue("nodePause", "Pauza UAV na waypoint-u (s)", nodePause);
 cmd.AddValue("mobilityModel", "UAV mobility model: RWP, RWALK2D, GAUSS_MARKOV, GROUP", mobilityModel);
 cmd.AddValue("randomWalkTime", "RandomWalk2d Time mode period promjene smjera (s)", randomWalkTime);
 cmd.AddValue("gaussMarkovAlpha", "Gauss-Markov alpha", gaussMarkovAlpha);
 cmd.AddValue("gaussMarkovTimeStep", "Gauss-Markov timestep (s)", gaussMarkovTimeStep);
 cmd.AddValue("groupRadius", "Radius/offset zone za clanove GroupMobilityHelper modela (m)", groupRadius);
 cmd.AddValue("txRange", "Maksimalni radio domet (m)", txRange);
 cmd.AddValue("uavMinAltitude", "Minimalna visina UAV-a", uavMinAltitude);
 cmd.AddValue("uavMaxAltitude", "Maksimalna visina UAV-a", uavMaxAltitude);
 cmd.AddValue("txPowerDbm", "WiFi Tx snaga (dBm)", txPowerDbm);
 cmd.AddValue("rxSensitivityDbm", "WiFi Rx sensitivity (dBm)", rxSensitivityDbm);
 cmd.AddValue("routingProtocol", "Routing protocol: AODV, OLSR, DSDV, DSR, AODV_ETX, QSPU", routingProtocol);
 cmd.AddValue("enableRemoteId", "Ukljuci Remote-ID broadcast", enableRemoteId);
 cmd.AddValue("enableNetAnim", "Generisi NetAnim XML", enableNetAnim);
 cmd.AddValue("enableFlowMonitorXml", "Generisi FlowMonitor XML", enableFlowMonitorXml);
 //cmd.AddValue("txtFile", "TXT izlazni fajl", txtFileName);
 cmd.AddValue("csvFile", "CSV izlazni fajl", csvFileName);
 cmd.Parse(argc, argv);
 mobilityModel = NormalizeMobilityName(mobilityModel);
 if (nodeSpeed < nodeMinSpeed)
 {
     NS_FATAL_ERROR("nodeSpeed mora biti >= nodeMinSpeed");
 }
 if (groupRadius <= 0.0)
 {
     NS_FATAL_ERROR("groupRadius mora biti > 0");
 }
 
 // Postavimo seed za reproducibilnost. Isti seed + razlicit run daju nezavisna ponavljanja.
 RngSeedManager::SetSeed(12345);
 RngSeedManager::SetRun(run);
 
 NS_LOG_UNCOND("========================================");
 NS_LOG_UNCOND("        UAANET ROUTING SIMULACIJA");
 NS_LOG_UNCOND("========================================");
 NS_LOG_UNCOND("Trajanje simulacije: " << simTime << " s");
 NS_LOG_UNCOND("Velicina podrucja: " << areaSize << " x " << areaSize << " m");
 NS_LOG_UNCOND("Radio domet: " << txRange << " m");
 NS_LOG_UNCOND("Mobility model: " << mobilityModel);
 NS_LOG_UNCOND("Brzina UAV: " << (fixedNodeSpeed ? nodeSpeed : nodeMinSpeed) << " - " << nodeSpeed << " m/s");
 NS_LOG_UNCOND("Pauza UAV: " << nodePause << " s");
 NS_LOG_UNCOND("RandomWalk2d timestep: " << randomWalkTime << " s");
 NS_LOG_UNCOND("Gauss-Markov alpha/timestep: " << gaussMarkovAlpha << " / " << gaussMarkovTimeStep << " s");
 NS_LOG_UNCOND("Group radius: " << groupRadius << " m");
 NS_LOG_UNCOND("Visina UAV: " << uavMinAltitude << " - " << uavMaxAltitude << " m");
 NS_LOG_UNCOND("RNG run: " << run);
 NS_LOG_UNCOND("Tx power: " << txPowerDbm << " dBm");
 NS_LOG_UNCOND("Rx sensitivity: " << rxSensitivityDbm << " dBm");
 NS_LOG_UNCOND("Remote-ID: " << (enableRemoteId ? "enabled" : "disabled"));
 NS_LOG_UNCOND("NetAnim XML: " << (enableNetAnim ? "enabled" : "disabled"));
 NS_LOG_UNCOND("FlowMonitor XML: " << (enableFlowMonitorXml ? "enabled" : "disabled"));
 
 // 1. Kreiraj cvorove
NodeContainer gcsNodes;
gcsNodes.Create(nGcs);

NodeContainer uavNodes;
uavNodes.Create(nUavs);

NodeContainer allNodes;
allNodes.Add(gcsNodes);
allNodes.Add(uavNodes);

nNodes = allNodes.GetN();

NS_LOG_UNCOND("GCS cvorovi: " << nGcs);
NS_LOG_UNCOND("UAV cvorovi: " << nUavs);
NS_LOG_UNCOND("Ukupno cvorova: " << nNodes);

 // 2. Konfiguriraj WiFi (802.11a - pogodno za UAV)
WifiHelper wifi;
wifi.SetStandard(WIFI_STANDARD_80211a);

wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
    "DataMode", StringValue("OfdmRate24Mbps"),
    "ControlMode", StringValue("OfdmRate24Mbps"));

YansWifiPhyHelper wifiPhy;
wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
YansWifiChannelHelper wifiChannel;

wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
    "Frequency", DoubleValue(5.18e9));


wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(txRange));


wifiPhy.SetChannel(wifiChannel.Create());

// Radio parametri
wifiPhy.Set("TxPowerStart", DoubleValue(txPowerDbm));     // dBm
wifiPhy.Set("TxPowerEnd", DoubleValue(txPowerDbm));       // dBm
wifiPhy.Set("RxSensitivity", DoubleValue(rxSensitivityDbm));   // dBm

WifiMacHelper wifiMac;
wifiMac.SetType("ns3::AdhocWifiMac");

Config::SetDefault("ns3::WifiMacQueue::MaxDelay",TimeValue(MilliSeconds(50)));

NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, allNodes);
 
 // 3. Mobility
 // 3.1 GCS mobility - fixed
 Ptr<ListPositionAllocator>gcsPositionAlloc = CreateObject<ListPositionAllocator>();
 
 for (uint32_t i = 0 ; i< nGcs; i++){
 	double x = (i+1)*areaSize/(nGcs+1);
 	double y = (i+1)*areaSize/(nGcs+1);
 	double z = 0.0;
 		
 	gcsPositionAlloc -> Add(Vector(x,y,z));
 	
 	NS_LOG_UNCOND("GCS-"<<i<<" pozicija: x="<<x<<", y="<<y<<", z="<<z);
 }
 MobilityHelper gcsMobility;
 gcsMobility.SetPositionAllocator(gcsPositionAlloc);
 gcsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
 gcsMobility.Install(gcsNodes);
 
 // 3.2 UAV mobility - izbor preko --mobilityModel
 // RWP          = ns3::RandomWaypointMobilityModel
 // RWALK2D      = ns3::RandomWalk2dMobilityModel
 // GAUSS_MARKOV = ns3::GaussMarkovMobilityModel
 // GROUP        = ns3::GroupMobilityHelper, parent RandomWaypoint, member RandomWalk2d
 InstallUavMobility(uavNodes,
                    mobilityModel,
                    areaSize,
                    nodeMinSpeed,
                    nodeSpeed,
                    fixedNodeSpeed,
                    nodePause,
                    uavMinAltitude,
                    uavMaxAltitude,
                    randomWalkTime,
                    gaussMarkovAlpha,
                    gaussMarkovTimeStep,
                    groupRadius);
 
 // 4. Internet stack 
 //AodvHelper aodv;
 //aodv.Set("NetDiameter",UintegerValue(35));
 //aodv.Set("RreqRetries",UintegerValue(2));
 //aodv.Set("RreqRateLimit",UintegerValue(10));
 //aodv.Set("MaxQueueLen",UintegerValue(256));
 //aodv.Set("MaxQueueTime",TimeValue(Seconds(50)));
 
 
 
 InternetStackHelper internet;
 
 if(routingProtocol == "AODV"){
 	AodvHelper aodv;
 	internet.SetRoutingHelper(aodv);
 	internet.Install(allNodes);
 }
 else if(routingProtocol == "OLSR"){
 	OlsrHelper olsr;
 	internet.SetRoutingHelper(olsr);
 	internet.Install(allNodes);
 }
 else if(routingProtocol == "DSDV"){
 	DsdvHelper dsdv;
 	internet.SetRoutingHelper(dsdv);
 	internet.Install(allNodes);
 }
 else if (routingProtocol == "DSR"){
 	DsrHelper dsr;
 	DsrMainHelper dsrMain;
 	
 	internet.Install(allNodes);
 	dsrMain.Install(dsr, allNodes);
 }
 else if (routingProtocol == "AODV_ETX"){
 	AodvEtxHelper aodvEtx;
 	internet.SetRoutingHelper(aodvEtx);
 	internet.Install(allNodes);
 }
else if (routingProtocol == "QSPU")
{
  QspuHelper qspu;

  qspu.Set("GatewayNodeId", UintegerValue(gcsNodes.Get(0)->GetId()));
  qspu.Set("TxRange", DoubleValue(txRange));
  qspu.Set("DataRateBps", DoubleValue(24000000.0));

  qspu.Set("IgadInterval", TimeValue(Seconds(4.0)));
  qspu.Set("UavadInterval", TimeValue(Seconds(4.0)));

  qspu.Set("Alpha", DoubleValue(0.33));
  qspu.Set("Beta", DoubleValue(0.33));
  qspu.Set("Gamma", DoubleValue(0.34));

  internet.SetRoutingHelper(qspu);
  internet.Install(allNodes);
}
 else{
 	NS_FATAL_ERROR("Unknown routingProtocol: "<< routingProtocol<<". Use AODV, OLSR, DSDV, DSR, AODV_ETX or QSPU!");
 }
 //internet.SetRoutingHelper(aodv);
 //internet.Install(allNodes);
 
 // 5. Dodijeli IP adrese
 Ipv4AddressHelper ipv4;
 ipv4.SetBase("10.1.1.0", "255.255.255.0");
 Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
 
 NS_LOG_UNCOND("\n--- IP adresiranje ---");

for (uint32_t i = 0; i < nGcs; i++)
{
    NS_LOG_UNCOND("GCS-" << i
                  << " | NodeId=" << gcsNodes.Get(i)->GetId()
                  << " | IP=" << interfaces.GetAddress(i));
}

for (uint32_t i = 0; i < nUavs; i++)
{
    uint32_t interfaceIndex = nGcs + i;

    NS_LOG_UNCOND("UAV-" << i
                  << " | NodeId=" << uavNodes.Get(i)->GetId()
                  << " | IP=" << interfaces.GetAddress(interfaceIndex));
}
for (uint32_t i = 0; i < allNodes.GetN(); i++)
{
    uint32_t nodeId = allNodes.Get(i)->GetId();
    g_nodeIpAddress[nodeId] = interfaces.GetAddress(i);
}
 
// ============================================================
// 6. Aplikacije - realističniji UAANET/FANET saobraćaj
//
// C2 control:   GCS -> UAV, stalni mali kontrolni tok
// C2 command:   GCS -> UAV, povremeni komandni burst
// Telemetry:    UAV -> GCS, stalni statusni tok
// Remote-ID:    UAV -> broadcast, stalni identifikacijski tok
// Video/Image:  UAV -> GCS, povremeni bursty data tok
// ============================================================

uint16_t telemetryPort = 9;
uint16_t c2ControlPort = 10;
uint16_t videoPort     = 11;
uint16_t remoteIdPort  = 12;
uint16_t c2CommandPort = 13;

double appStart = 20.0;
double appStop  = simTime -10.0;

uint32_t mainGcsIndex = 0;
Ipv4Address gcsAddress = interfaces.GetAddress(mainGcsIndex);
NS_LOG_UNCOND("\n--- Saobracajni model ---");
NS_LOG_UNCOND("Glavni GCS: GCS-0, IP=" << gcsAddress);
NS_LOG_UNCOND("App start: " << appStart << " s");
NS_LOG_UNCOND("App stop: " << appStop << " s");
NS_LOG_UNCOND("Telemetry: svi UAV -> GCS-0, port " << telemetryPort);
NS_LOG_UNCOND("C2 control: GCS-0 -> svaki UAV, port " << c2ControlPort);
NS_LOG_UNCOND("C2 command: GCS-0 -> svaki UAV, port " << c2CommandPort);
NS_LOG_UNCOND("Video/Image: svi UAV -> GCS-0, port " << videoPort);
if(enableRemoteId){
NS_LOG_UNCOND("Remote-ID: UAV one-hop broadcast, port " << remoteIdPort);
}

// ------------------------------------------------------------
// 6.1 Sinkovi
// ------------------------------------------------------------

// Telemetry sink na GCS-u
PacketSinkHelper telemetrySinkHelper(
    "ns3::UdpSocketFactory",
    InetSocketAddress(Ipv4Address::GetAny(), telemetryPort)
);

ApplicationContainer telemetrySinkApps =
    telemetrySinkHelper.Install(gcsNodes.Get(0));

telemetrySinkApps.Start(Seconds(0.0));
telemetrySinkApps.Stop(Seconds(simTime));


// Video/Image sink na GCS-u
PacketSinkHelper videoSinkHelper(
    "ns3::UdpSocketFactory",
    InetSocketAddress(Ipv4Address::GetAny(), videoPort)
);

ApplicationContainer videoSinkApps =
    videoSinkHelper.Install(gcsNodes.Get(0));

videoSinkApps.Start(Seconds(0.0));
videoSinkApps.Stop(Seconds(simTime));


// C2 control sinkovi na svim UAV-ovima
PacketSinkHelper c2ControlSinkHelper(
    "ns3::UdpSocketFactory",
    InetSocketAddress(Ipv4Address::GetAny(), c2ControlPort)
);

ApplicationContainer c2ControlSinkApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    c2ControlSinkApps.Add(c2ControlSinkHelper.Install(uavNodes.Get(i)));
}

c2ControlSinkApps.Start(Seconds(0.0));
c2ControlSinkApps.Stop(Seconds(simTime));


// C2 command sinkovi na svim UAV-ovima
PacketSinkHelper c2CommandSinkHelper(
    "ns3::UdpSocketFactory",
    InetSocketAddress(Ipv4Address::GetAny(), c2CommandPort)
);

ApplicationContainer c2CommandSinkApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    c2CommandSinkApps.Add(c2CommandSinkHelper.Install(uavNodes.Get(i)));
}

c2CommandSinkApps.Start(Seconds(0.0));
c2CommandSinkApps.Stop(Seconds(simTime));

// ------------------------------------------------------------
// Remote-ID receiver sockets
//
// Svi čvorovi slušaju Remote-ID broadcast na remoteIdPort.
// To znači da GCS i UAV-ovi mogu primiti Remote-ID poruke.
// ------------------------------------------------------------

std::vector<Ptr<Socket>> remoteIdRxSockets;

if (enableRemoteId)
{
    for (uint32_t i = 0; i < allNodes.GetN(); i++)
    {
        Ptr<Socket> recvSocket = Socket::CreateSocket(
            allNodes.Get(i),
            UdpSocketFactory::GetTypeId()
        );

        InetSocketAddress localAddress =
            InetSocketAddress(Ipv4Address::GetAny(), remoteIdPort);

        recvSocket->Bind(localAddress);
        recvSocket->SetRecvCallback(MakeCallback(&ReceiveRemoteIdPacket));

        remoteIdRxSockets.push_back(recvSocket);
    }
}

// ------------------------------------------------------------
// 6.2 Telemetry: UAV -> GCS
//
// Modeluje: UAV ID, poziciju, brzinu, visinu, bateriju,
// status misije, timestamp.
//
// 128 B, 5 paketa/s:
// 128 B * 8 * 5 = 5120 bps
// ------------------------------------------------------------
ApplicationContainer telemetryApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    OnOffHelper telemetry(
        "ns3::UdpSocketFactory",
        InetSocketAddress(gcsAddress, telemetryPort)
    );

    telemetry.SetAttribute("DataRate", StringValue("5120bps"));
    telemetry.SetAttribute("PacketSize", UintegerValue(128));

    telemetry.SetAttribute("OnTime",
        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    telemetry.SetAttribute("OffTime",
        StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer app = telemetry.Install(uavNodes.Get(i));

    app.Start(Seconds(appStart + i * 0.05));
    app.Stop(Seconds(appStop));

    telemetryApps.Add(app);
}


// ------------------------------------------------------------
// 6.3 C2 control / heartbeat: GCS -> UAV
//
// Modeluje stalni kontrolni/keep-alive tok.
//
// 64 B, 20 paketa/s:
// 64 B * 8 * 20 = 10240 bps
//
// To je jedan paket otprilike svakih 50 ms.
// ------------------------------------------------------------
ApplicationContainer c2ControlApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    Ipv4Address uavAddress = interfaces.GetAddress(nGcs + i);

    OnOffHelper c2Control(
        "ns3::UdpSocketFactory",
        InetSocketAddress(uavAddress, c2ControlPort)
    );

    c2Control.SetAttribute("DataRate", StringValue("10240bps"));
    c2Control.SetAttribute("PacketSize", UintegerValue(64));

    c2Control.SetAttribute("OnTime",
        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    c2Control.SetAttribute("OffTime",
        StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer app = c2Control.Install(gcsNodes.Get(0));

    app.Start(Seconds(appStart + 1.0 + i * 0.05));
    app.Stop(Seconds(appStop));

    c2ControlApps.Add(app);
}


// ------------------------------------------------------------
// 6.4 C2 command: GCS -> UAV
//
// Modeluje povremene naredbe:
// novi waypoint, promjena visine, promjena brzine,
// aktiviranje senzora, return-to-home, emergency command.
//
// Bursty:
// - OnTime = 0.3 s
// - OffTime = Exponential Mean 8 s
// - PacketSize = 128 B
// - DataRate = 20480 bps
//
// Dok je ON:
// 128 B * 8 = 1024 bita
// 20480 bps / 1024 bita = 20 paketa/s
// Za 0.3 s izađe oko 6 paketa po command burstu.
// ------------------------------------------------------------
ApplicationContainer c2CommandApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    Ipv4Address uavAddress = interfaces.GetAddress(nGcs + i);

    OnOffHelper c2Command(
        "ns3::UdpSocketFactory",
        InetSocketAddress(uavAddress, c2CommandPort)
    );

    c2Command.SetAttribute("DataRate", StringValue("20480bps"));
    c2Command.SetAttribute("PacketSize", UintegerValue(128));

    c2Command.SetAttribute("OnTime",
        StringValue("ns3::ConstantRandomVariable[Constant=0.3]"));
    c2Command.SetAttribute("OffTime",
        StringValue("ns3::ExponentialRandomVariable[Mean=8.0]"));

    ApplicationContainer app = c2Command.Install(gcsNodes.Get(0));

    app.Start(Seconds(appStart + 3.0 + i * 0.1));
    app.Stop(Seconds(appStop));

    c2CommandApps.Add(app);
}

// ------------------------------------------------------------
// 6.5 Video/Image: UAV -> GCS
//
// Modeluje povremeni prijenos slike/videa/senzorskih podataka.
//
// Svi UAV-ovi mogu slati, ali bursty:
// - DataRate dok je ON: 50 kbps
// - PacketSize: 1024 B
// - OnTime: Exponential Mean 3 s
// - OffTime: Exponential Mean 12 s
//
// Prosječni duty cycle:
// 3 / (3 + 12) = 20%
//
// XXXXXXXAko imaš 20 UAV:
// XXXXXXpeak load tokom ON perioda može biti 20 * 50 kbps = 5 Mbps,
// XXXXXXXali prosječno oko 1 Mbps zbog pauza.
// ------------------------------------------------------------
ApplicationContainer videoApps;

for (uint32_t i = 0; i < nUavs; i++)
{
    OnOffHelper video(
        "ns3::UdpSocketFactory",
        InetSocketAddress(gcsAddress, videoPort)
    );

    video.SetAttribute("DataRate", StringValue("50kbps"));
    video.SetAttribute("PacketSize", UintegerValue(1024));

    video.SetAttribute("OnTime",
        StringValue("ns3::ExponentialRandomVariable[Mean=3.0]"));
    video.SetAttribute("OffTime",
        StringValue("ns3::ExponentialRandomVariable[Mean=12.0]"));

    ApplicationContainer app = video.Install(uavNodes.Get(i));

    app.Start(Seconds(appStart + 5.0 + i * 0.2));
    app.Stop(Seconds(appStop));

    videoApps.Add(app);
}

// ------------------------------------------------------------
// 6.6 Remote-ID sender sockets
//
// Svaki UAV šalje jedan Remote-ID broadcast paket svake sekunde.
// Paket je one-hop broadcast: čuju ga samo čvorovi u radio dometu.
// ------------------------------------------------------------

uint32_t remoteIdPacketSize = 200;
Time remoteIdInterval = Seconds(1.0);

std::vector<Ptr<Socket>> remoteIdTxSockets;

if (enableRemoteId)
{
    for (uint32_t i = 0; i < nUavs; i++)
    {
        Ptr<Socket> sendSocket = Socket::CreateSocket(
            uavNodes.Get(i),
            UdpSocketFactory::GetTypeId()
        );

        sendSocket->SetAllowBroadcast(true);
        sendSocket->Bind();

        remoteIdTxSockets.push_back(sendSocket);

        Simulator::Schedule(
            Seconds(appStart + 2.0 + i * 0.03),
            &SendRemoteIdBroadcast,
            sendSocket,
            remoteIdPacketSize,
            remoteIdPort,
            remoteIdInterval,
            Seconds(appStop)
        );
    }
}
 // 7. FlowMonitor za mjerenje metrika
 FlowMonitorHelper flowmon;
 Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
 // 8. NetAnim (opcionalno; za serije eksperimenata je bolje drzati disabled)
AnimationInterface* anim = nullptr;

if (enableNetAnim)
{
    anim = new AnimationInterface(netanimFileName);
    anim->SetMaxPktsPerTraceFile(5000000);
    anim->EnablePacketMetadata(true);
    anim->SetMobilityPollInterval(Seconds(0.05));
    anim->EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(simTime));
    anim->EnableWifiMacCounters(Seconds(0), Seconds(simTime));

    // GCS cvorovi - crveni
    for (uint32_t i = 0; i < gcsNodes.GetN(); i++)
    {
        anim->UpdateNodeDescription(gcsNodes.Get(i), "GCS-" + std::to_string(i));
        anim->UpdateNodeColor(gcsNodes.Get(i), 255, 0, 0);
    }

    // UAV cvorovi - plavi
    for (uint32_t i = 0; i < uavNodes.GetN(); i++)
    {
        anim->UpdateNodeDescription(uavNodes.Get(i), "UAV-" + std::to_string(i));
        anim->UpdateNodeColor(uavNodes.Get(i), 0, 0, 255);
    }
}

 // 9. Pokretanje simulacije
 NS_LOG_UNCOND("Pokrecem simulaciju...");
 Simulator::Stop(Seconds(simTime));
 Simulator::Run();
 
 
 
 
// ============================================================
// 10. FlowMonitor analiza rezultata
//     Aplikacijski saobracaj se odvaja od AODV routing overhead-a.
//     AODV koristi UDP port 654 za svoje kontrolne/routing poruke.
// ============================================================

monitor->CheckForLostPackets();

Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

std::map<FlowId, FlowMonitor::FlowStats> flowStats =
    monitor->GetFlowStats();

struct TrafficStats
{
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    uint64_t txBytes = 0;
    uint64_t rxBytes = 0;
    double delaySumSeconds = 0.0;

    uint32_t flowCount = 0;
    uint32_t successfulFlowCount = 0;
};

TrafficStats telemetryStats;
TrafficStats c2ControlStats;
TrafficStats c2CommandStats;
TrafficStats videoStats;
TrafficStats routingOverheadStats;

const uint16_t aodvRoutingPort = 654;
const uint16_t olsrRoutingPort = 698;
const uint16_t dsdvRoutingPort = 269;
const uint8_t dsrIpProtocol = 48;
const uint8_t udpIpProtocol = 17; 


auto AddFlowToStats = [](TrafficStats& s, const FlowMonitor::FlowStats& fs)
{
    s.txPackets += fs.txPackets;
    s.rxPackets += fs.rxPackets;
    s.txBytes += fs.txBytes;
    s.rxBytes += fs.rxBytes;
    s.delaySumSeconds += fs.delaySum.GetSeconds();

    s.flowCount++;

    if (fs.rxPackets > 0)
    {
        s.successfulFlowCount++;
    }
};

auto IsApplicationPort = [&](uint16_t port) -> bool
{
    return port == telemetryPort ||
           port == c2ControlPort ||
           port == c2CommandPort ||
           port == videoPort;
};

auto IsRoutingControlFlow = [&](const Ipv4FlowClassifier::FiveTuple& t)->bool 
{
	if (routingProtocol == "AODV"){
		return t.protocol == udpIpProtocol && (t.sourcePort == aodvRoutingPort || t.destinationPort == aodvRoutingPort);
	}
	if (routingProtocol == "AODV_ETX"){
		return t.protocol == udpIpProtocol && (t.sourcePort == aodvRoutingPort || t.destinationPort == aodvRoutingPort);
	}
	if (routingProtocol == "QSPU"){
		return t.protocol == udpIpProtocol && (t.sourcePort == aodvRoutingPort || t.destinationPort == aodvRoutingPort);
	}
	if (routingProtocol == "OLSR"){
		return t.protocol == udpIpProtocol && (t.sourcePort == olsrRoutingPort || t.destinationPort == olsrRoutingPort);
	}
	if (routingProtocol == "DSDV"){
		return t.protocol == udpIpProtocol && (t.sourcePort == dsdvRoutingPort || t.destinationPort == dsdvRoutingPort);
	}
	if (routingProtocol == "DSR"){
		return t.protocol == dsrIpProtocol;
	}
	return false;
};

for (const auto& flow : flowStats)
{
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
    const FlowMonitor::FlowStats& fs = flow.second;

    // AODV routing/control overhead. Ne mijesati sa aplikacijskim saobracajem.
    if (IsRoutingControlFlow(t))
    {
        AddFlowToStats(routingOverheadStats, fs);
        continue;
    }
    // Ignorisi sve sto nije jedan od definisanih aplikacijskih portova.
    // Remote-ID broadcast se ne klasifikuje standardno kroz FlowMonitor,
    // pa ga ne ubrajamo u ove unicast aplikacijske metrike.
    if (!IsApplicationPort(t.destinationPort))
    {
        continue;
    }

    if (t.destinationPort == telemetryPort)
    {
        AddFlowToStats(telemetryStats, fs);
    }
    else if (t.destinationPort == c2ControlPort)
    {
        AddFlowToStats(c2ControlStats, fs);
    }
    else if (t.destinationPort == c2CommandPort)
    {
        AddFlowToStats(c2CommandStats, fs);
    }
    else if (t.destinationPort == videoPort)
    {
        AddFlowToStats(videoStats, fs);
    }
}

// Aktivno vrijeme aplikacija
double activeTime = appStop - appStart;

auto GetPdr = [](const TrafficStats& s) -> double
{
    if (s.txPackets == 0)
    {
        return 0.0;
    }

    return static_cast<double>(s.rxPackets) / s.txPackets * 100.0;
};

auto GetLoss = [](const TrafficStats& s) -> double
{
    if (s.txPackets == 0)
    {
        return 0.0;
    }

    return static_cast<double>(s.txPackets - s.rxPackets) / s.txPackets * 100.0;
};

auto GetAvgDelayMs = [](const TrafficStats& s) -> double
{
    if (s.rxPackets == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return s.delaySumSeconds / s.rxPackets * 1000.0;
};

auto GetThroughputKbps = [activeTime](const TrafficStats& s) -> double
{
    if (activeTime <= 0.0)
    {
        return 0.0;
    }

    return static_cast<double>(s.rxBytes * 8) / (activeTime * 1000.0);
};

auto GetOfferedLoadKbps = [activeTime](const TrafficStats& s) -> double
{
    if (activeTime <= 0.0)
    {
        return 0.0;
    }

    return static_cast<double>(s.txBytes * 8) / (activeTime * 1000.0);
};

auto CombineStats = [](const TrafficStats& a, const TrafficStats& b) -> TrafficStats
{
    TrafficStats c;

    c.txPackets = a.txPackets + b.txPackets;
    c.rxPackets = a.rxPackets + b.rxPackets;
    c.txBytes = a.txBytes + b.txBytes;
    c.rxBytes = a.rxBytes + b.rxBytes;
    c.delaySumSeconds = a.delaySumSeconds + b.delaySumSeconds;
    c.flowCount = a.flowCount + b.flowCount;
    c.successfulFlowCount = a.successfulFlowCount + b.successfulFlowCount;

    return c;
};

TrafficStats totalC2Stats = CombineStats(c2ControlStats, c2CommandStats);

auto PrintStats = [&](const std::string& name, const TrafficStats& s)
{
    NS_LOG_UNCOND("\n--- " << name << " ---");
    NS_LOG_UNCOND("Flowovi ukupno: " << s.flowCount);
    NS_LOG_UNCOND("Uspjesni flowovi: " << s.successfulFlowCount);
    NS_LOG_UNCOND("Tx paketi: " << s.txPackets);
    NS_LOG_UNCOND("Rx paketi: " << s.rxPackets);
    NS_LOG_UNCOND("PDR: " << GetPdr(s) << " %");
    NS_LOG_UNCOND("Packet loss: " << GetLoss(s) << " %");
    NS_LOG_UNCOND("Avg delay: " << GetAvgDelayMs(s) << " ms");
    NS_LOG_UNCOND("Throughput: " << GetThroughputKbps(s) << " kbps");
    NS_LOG_UNCOND("Offered load: " << GetOfferedLoadKbps(s) << " kbps");
};

auto PrintRoutingOverheadStats = [&](const TrafficStats& s)
{
    NS_LOG_UNCOND("\n--- AODV routing overhead, port 654 ---");
    NS_LOG_UNCOND("Routing flow entries: " << s.flowCount);
    NS_LOG_UNCOND("Routing active entries: " << s.successfulFlowCount);
    NS_LOG_UNCOND("Routing Tx paketi: " << s.txPackets);
    NS_LOG_UNCOND("Routing Rx paketi: " << s.rxPackets);
    NS_LOG_UNCOND("Routing Tx bytes: " << s.txBytes);
    NS_LOG_UNCOND("Routing Rx bytes: " << s.rxBytes);
    NS_LOG_UNCOND("Routing offered load: " << GetOfferedLoadKbps(s) << " kbps");
};

NS_LOG_UNCOND("\n========================================");
NS_LOG_UNCOND("          REZULTATI SIMULACIJE");
NS_LOG_UNCOND("========================================");

PrintStats("Telemetry UAV -> GCS", telemetryStats);
PrintStats("C2 Control GCS -> UAV", c2ControlStats);
PrintStats("C2 Command GCS -> UAV", c2CommandStats);
PrintStats("Ukupni C2 saobracaj", totalC2Stats);
PrintStats("Video/Image UAV -> GCS", videoStats);

PrintRoutingOverheadStats(routingOverheadStats);

uint64_t appRxBytes = telemetryStats.rxBytes + totalC2Stats.rxBytes + videoStats.rxBytes;
uint64_t appTxBytes = telemetryStats.txBytes + totalC2Stats.txBytes + videoStats.txBytes;
uint64_t appRxPackets = telemetryStats.rxPackets + totalC2Stats.rxPackets + videoStats.rxPackets;
uint64_t appTxPackets = telemetryStats.txPackets + totalC2Stats.txPackets + videoStats.txPackets;

double routingBytesPerDeliveredAppByte = 0.0;
double routingPacketsPerDeliveredAppPacket = 0.0;

if (appRxBytes > 0)
{
    routingBytesPerDeliveredAppByte =
        static_cast<double>(routingOverheadStats.txBytes) / appRxBytes;
}

if (appRxPackets > 0)
{
    routingPacketsPerDeliveredAppPacket =
        static_cast<double>(routingOverheadStats.txPackets) / appRxPackets;
}

NS_LOG_UNCOND("\n--- Normalizovani routing overhead ---");
NS_LOG_UNCOND("Aplikacijski Rx bytes: " << appRxBytes);
NS_LOG_UNCOND("Aplikacijski Rx paketi: " << appRxPackets);
NS_LOG_UNCOND("Routing Tx bytes / delivered app byte: "
              << routingBytesPerDeliveredAppByte);
NS_LOG_UNCOND("Routing Tx packets / delivered app packet: "
              << routingPacketsPerDeliveredAppPacket);

// 11. Remote-Id metrike 

double remoteIdActiveTime = enableRemoteId ? (appStop - (appStart + 2.0)) : 0.0;

double remoteIdTxLoadKbps = 0.0;
double remoteIdRxLoadKbps = 0.0;
double avgRemoteIdReceivers = 0.0;
double remoteIdReceptionRatio = 0.0;

if (remoteIdActiveTime > 0.0)
{
    remoteIdTxLoadKbps =
        static_cast<double>(g_remoteIdTxBytes * 8) /
        (remoteIdActiveTime * 1000.0);

    remoteIdRxLoadKbps =
        static_cast<double>(g_remoteIdRxBytes * 8) /
        (remoteIdActiveTime * 1000.0);
}

if (g_remoteIdTxPackets > 0)
{
    avgRemoteIdReceivers =
        static_cast<double>(g_remoteIdRxPackets) /
        g_remoteIdTxPackets;

    uint64_t maxPossibleRxCopies =
        g_remoteIdTxPackets * (allNodes.GetN() - 1);

    if (maxPossibleRxCopies > 0)
    {
        remoteIdReceptionRatio =
            static_cast<double>(g_remoteIdRxPackets) /
            maxPossibleRxCopies * 100.0;
    }

NS_LOG_UNCOND("\n--- Remote-ID broadcast statistika ---");
NS_LOG_UNCOND("Tx broadcast paketi: " << g_remoteIdTxPackets);
NS_LOG_UNCOND("Rx broadcast kopije: " << g_remoteIdRxPackets);
NS_LOG_UNCOND("Tx bytes: " << g_remoteIdTxBytes);
NS_LOG_UNCOND("Rx bytes: " << g_remoteIdRxBytes);
NS_LOG_UNCOND("Prosjecan broj prijema po broadcast paketu: "
              << avgRemoteIdReceivers);
NS_LOG_UNCOND("Broadcast reception ratio: "
              << remoteIdReceptionRatio << " %");
NS_LOG_UNCOND("Remote-ID offered load: "
              << remoteIdTxLoadKbps << " kbps");
NS_LOG_UNCOND("Remote-ID received-copy load: "
              << remoteIdRxLoadKbps << " kbps");
}


// ============================================================
// 12. Snimanje rezultata u TXT fajl
// ============================================================
/*
std::ofstream outFile(txtFileName);

auto WriteStats = [&](std::ofstream& out, const std::string& name,
                      const TrafficStats& s, bool printPdr = true)
{
    out << "\n--- " << name << " ---\n";
    out << "Flowovi ukupno: " << s.flowCount << "\n";
    out << "Uspjesni flowovi: " << s.successfulFlowCount << "\n";
    out << "Tx paketi: " << s.txPackets << "\n";
    out << "Rx paketi: " << s.rxPackets << "\n";
    out << "Tx bytes: " << s.txBytes << "\n";
    out << "Rx bytes: " << s.rxBytes << "\n";

    if (printPdr)
    {
        out << "PDR (%): " << GetPdr(s) << "\n";
        out << "Packet loss (%): " << GetLoss(s) << "\n";
        out << "Avg delay (ms): " << GetAvgDelayMs(s) << "\n";
        out << "Throughput (kbps): " << GetThroughputKbps(s) << "\n";
    }
    else
    {
        out << "PDR (%): N/A za routing overhead\n";
        out << "Packet loss (%): N/A za routing overhead\n";
        out << "Avg delay (ms): N/A za routing overhead\n";
        out << "Throughput (kbps): N/A za routing overhead\n";
    }

    out << "Offered load (kbps): " << GetOfferedLoadKbps(s) << "\n";
};

outFile << "========================================\n";
outFile << "UAANET AODV rezultati\n";
outFile << "========================================\n";
outFile << "Protocol: " << protocol << "\n";
outFile << "RNG run: " << run << "\n";
outFile << "Broj GCS cvorova: " << nGcs << "\n";
outFile << "Broj UAV cvorova: " << nUavs << "\n";
outFile << "Ukupno cvorova: " << allNodes.GetN() << "\n";
outFile << "Sim time: " << simTime << " s\n";
outFile << "App start: " << appStart << " s\n";
outFile << "App stop: " << appStop << " s\n";
outFile << "Active time: " << activeTime << " s\n";
outFile << "Area size: " << areaSize << " x " << areaSize << " m\n";
outFile << "Node speed: " << nodeSpeed << " m/s\n";
outFile << "Node pause: " << nodePause << " s\n";
outFile << "Tx range: " << txRange << " m\n";
outFile << "Tx power: " << txPowerDbm << " dBm\n";
outFile << "Rx sensitivity: " << rxSensitivityDbm << " dBm\n";
outFile << "UAV altitude range: " << uavMinAltitude << " - " << uavMaxAltitude << " m\n";
outFile << "Remote-ID enabled: " << (enableRemoteId ? "true" : "false") << "\n";

WriteStats(outFile, "Telemetry UAV -> GCS", telemetryStats);
WriteStats(outFile, "C2 Control GCS -> UAV", c2ControlStats);
WriteStats(outFile, "C2 Command GCS -> UAV", c2CommandStats);
WriteStats(outFile, "Ukupni C2 saobracaj", totalC2Stats);
WriteStats(outFile, "Video/Image UAV -> GCS", videoStats);
WriteStats(outFile, "AODV routing overhead, port 654", routingOverheadStats, false);

outFile << "\n--- Normalizovani routing overhead ---\n";
outFile << "Aplikacijski Tx bytes: " << appTxBytes << "\n";
outFile << "Aplikacijski Rx bytes: " << appRxBytes << "\n";
outFile << "Aplikacijski Tx paketi: " << appTxPackets << "\n";
outFile << "Aplikacijski Rx paketi: " << appRxPackets << "\n";
outFile << "Routing Tx bytes / delivered app byte: "
        << routingBytesPerDeliveredAppByte << "\n";
outFile << "Routing Tx packets / delivered app packet: "
        << routingPacketsPerDeliveredAppPacket << "\n";
if(enableRemoteId){
outFile << "\n--- Remote-ID broadcast statistika ---\n";
outFile << "Tx broadcast paketi: " << g_remoteIdTxPackets << "\n";
outFile << "Rx broadcast kopije: " << g_remoteIdRxPackets << "\n";
outFile << "Tx bytes: " << g_remoteIdTxBytes << "\n";
outFile << "Rx bytes: " << g_remoteIdRxBytes << "\n";
outFile << "Prosjecan broj prijema po broadcast paketu: "
        << avgRemoteIdReceivers << "\n";
outFile << "Broadcast reception ratio (%): "
        << remoteIdReceptionRatio << "\n";
outFile << "Remote-ID offered load (kbps): "
        << remoteIdTxLoadKbps << "\n";
outFile << "Remote-ID received-copy load (kbps): "
        << remoteIdRxLoadKbps << "\n";

outFile << "\nRemote-ID prijem po cvorovima:\n";
for (const auto& entry : g_remoteIdRxPerNode)
{
    outFile << "Node " << entry.first
            << " primio Remote-ID paketa: "
            << entry.second << "\n";
}
}
outFile.close();
*/
// ============================================================
// 13. CSV izlaz: jedan red = jedan eksperiment/run
// ============================================================
std::ifstream csvCheck(csvFileName);
bool csvHasContent = csvCheck.good() &&
    (csvCheck.peek() != std::ifstream::traits_type::eof());
csvCheck.close();

std::ofstream csvFile(csvFileName, std::ios::app);
csvFile << std::fixed << std::setprecision(6);

if (!csvHasContent)
{
    csvFile
        << "protocol,run,nGcs,nUavs,nNodes,simTime,areaSize,txRange,nodeSpeed,nodeMinSpeed,fixedNodeSpeed,nodePause,mobilityModel,randomWalkTime,gaussMarkovAlpha,gaussMarkovTimeStep,groupRadius,"
        << "uavMinAltitude,uavMaxAltitude,txPowerDbm,rxSensitivityDbm,appStart,appStop,remoteIdEnabled,"
        << "telemetry_flowCount,telemetry_successfulFlows,telemetry_txPackets,telemetry_rxPackets,telemetry_pdr,telemetry_loss,telemetry_delayMs,telemetry_throughputKbps,telemetry_offeredLoadKbps,"
        << "c2Control_flowCount,c2Control_successfulFlows,c2Control_txPackets,c2Control_rxPackets,c2Control_pdr,c2Control_loss,c2Control_delayMs,c2Control_throughputKbps,c2Control_offeredLoadKbps,"
        << "c2Command_flowCount,c2Command_successfulFlows,c2Command_txPackets,c2Command_rxPackets,c2Command_pdr,c2Command_loss,c2Command_delayMs,c2Command_throughputKbps,c2Command_offeredLoadKbps,"
        << "totalC2_flowCount,totalC2_successfulFlows,totalC2_txPackets,totalC2_rxPackets,totalC2_pdr,totalC2_loss,totalC2_delayMs,totalC2_throughputKbps,totalC2_offeredLoadKbps,"
        << "video_flowCount,video_successfulFlows,video_txPackets,video_rxPackets,video_pdr,video_loss,video_delayMs,video_throughputKbps,video_offeredLoadKbps,"
        << "routing_flowEntries,routing_activeEntries,routing_txPackets,routing_rxPackets,routing_txBytes,routing_rxBytes,routing_offeredLoadKbps,routingBytesPerDeliveredAppByte,routingPacketsPerDeliveredAppPacket,"
        << "remoteId_txPackets,remoteId_rxCopies,remoteId_txBytes,remoteId_rxBytes,remoteId_avgReceivers,remoteId_receptionRatio,remoteId_offeredLoadKbps,remoteId_receivedCopyLoadKbps"
        << "\n";
}

auto WriteCsvStats = [&](const TrafficStats& s)
{
    csvFile << s.flowCount << ","
            << s.successfulFlowCount << ","
            << s.txPackets << ","
            << s.rxPackets << ","
            << GetPdr(s) << ","
            << GetLoss(s) << ","
            << GetAvgDelayMs(s) << ","
            << GetThroughputKbps(s) << ","
            << GetOfferedLoadKbps(s);
};

csvFile << routingProtocol << ","
        << run << ","
        << nGcs << ","
        << nUavs << ","
        << nNodes << ","
        << simTime << ","
        << areaSize << ","
        << txRange << ","
        << nodeSpeed << ","
        << nodeMinSpeed << ","
        << (fixedNodeSpeed ? 1 : 0) << ","
        << nodePause << ","
        << mobilityModel << ","
        << randomWalkTime << ","
        << gaussMarkovAlpha << ","
        << gaussMarkovTimeStep << ","
        << groupRadius << ","
        << uavMinAltitude << ","
        << uavMaxAltitude << ","
        << txPowerDbm << ","
        << rxSensitivityDbm << ","
        << appStart << ","
        << appStop << ","
        << (enableRemoteId ? 1 : 0) << ",";

WriteCsvStats(telemetryStats);
csvFile << ",";
WriteCsvStats(c2ControlStats);
csvFile << ",";
WriteCsvStats(c2CommandStats);
csvFile << ",";
WriteCsvStats(totalC2Stats);
csvFile << ",";
WriteCsvStats(videoStats);
csvFile << ","
        << routingOverheadStats.flowCount << ","
        << routingOverheadStats.successfulFlowCount << ","
        << routingOverheadStats.txPackets << ","
        << routingOverheadStats.rxPackets << ","
        << routingOverheadStats.txBytes << ","
        << routingOverheadStats.rxBytes << ","
        << GetOfferedLoadKbps(routingOverheadStats) << ","
        << routingBytesPerDeliveredAppByte << ","
        << routingPacketsPerDeliveredAppPacket << ","
        << g_remoteIdTxPackets << ","
        << g_remoteIdRxPackets << ","
        << g_remoteIdTxBytes << ","
        << g_remoteIdRxBytes << ","
        << avgRemoteIdReceivers << ","
        << remoteIdReceptionRatio << ","
        << remoteIdTxLoadKbps << ","
        << remoteIdRxLoadKbps
        << "\n";

csvFile.close();

if (enableFlowMonitorXml)
{
    monitor->SerializeToXmlFile(flowmonXmlFileName, true, true);
}

//NS_LOG_UNCOND("\nRezultati sacuvani u " << txtFileName);
NS_LOG_UNCOND("CSV red dodat u " << csvFileName);
if (enableFlowMonitorXml)
{
    NS_LOG_UNCOND("FlowMonitor XML sacuvan u " << flowmonXmlFileName);
}
if (enableNetAnim)
{
    NS_LOG_UNCOND("NetAnim XML sacuvan u " << netanimFileName);
}

 Simulator::Destroy();
 return 0;
}


 

