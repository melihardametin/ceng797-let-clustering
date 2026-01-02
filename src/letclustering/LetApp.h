#ifndef __LETCLUSTERING_LETAPP_H_
#define __LETCLUSTERING_LETAPP_H_

#include <omnetpp.h>
#include <map>
#include <set>
#include "inet/common/geometry/common/Coord.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "LetMessage_m.h"
#include "inet/power/storage/SimpleEpEnergyStorage.h"

using namespace omnetpp;
using namespace inet;
using namespace src::letclustering;

struct NeighborData {
    double letValue;
    double theirWeight;
    int theirLeaderId;
    int role;
    simtime_t lastSeen;
};

class LetApp : public cSimpleModule, public UdpSocket::ICallback
{
  public:
    int myRole = 0; // 0:Undef, 1:CH, 2:Member, 3:GW

  protected:
    IMobility *mobility;
    UdpSocket socket;

    // Mesajlar
    cMessage *selfMsg;     // Hello Timer
    cMessage *cleanupMsg;  // Maintenance Timer
    cMessage *dataMsg;     // Data Generation Timer
    cMessage *unmarkMsg;   // Visual Reset Timer

    L3Address destAddress;
    int localPort, destPort;
    int myId; // Array Index

    // LET Değişkenleri
    double myTotalWeight = 0.0;
    std::map<int, NeighborData> neighborTable;
    int currentLeaderId = -1;

    cLineFigure *connectionLine = nullptr;

    // Routing (Deduplication)
    std::set<std::string> seenPackets;
    unsigned long mySeqNum = 0;

    // İstatistik Değişkenleri
    long pktsSent = 0;
    long pktsReceived = 0;
    simtime_t roleStartTime;

    // Sinyaller
    simsignal_t pdrSignal;
    simsignal_t delaySignal;
    simsignal_t hopCountSignal;
    simsignal_t clusterHeadLifetimeSignal;
    simsignal_t controlOverheadSignal;
    simsignal_t isClusterHeadSignal;

    inet::power::SimpleEpEnergyStorage *energyStorage = nullptr;

    simsignal_t clusterSizeSignal;
    simsignal_t residualEnergySignal;
    simsignal_t throughputSignal;

    double totalBitsReceived = 0;
    simsignal_t clusterChangeRateSignal;

    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void refreshDisplay() const override;
    virtual ~LetApp();
    double calculateLET(double xi, double yi, double vi, double thetai,
                        double xj, double yj, double vj, double thetaj, double R);
    void runClusteringLogic();
    void removeOldNeighbors();

    void sendHelloPacket();
    void sendDataPacket();
    void processDataPacket(Packet *packet);

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override { delete indication; }
    virtual void socketClosed(UdpSocket *socket) override {}

    void updateVisuals();
    void resetNodeColors();
};

#endif
