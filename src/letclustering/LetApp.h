#ifndef __LETCLUSTERING_LETAPP_H_
#define __LETCLUSTERING_LETAPP_H_

#include <omnetpp.h>
#include <map>
#include "inet/common/geometry/common/Coord.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "LetMessage_m.h"

using namespace omnetpp;
using namespace inet;
using namespace src::letclustering;

struct NeighborData {
    double letValue;
    double theirWeight;
    int theirLeaderId;
    simtime_t lastSeen;
};

class LetApp : public cSimpleModule, public UdpSocket::ICallback
{
  protected:
    IMobility *mobility;
    UdpSocket socket;
    cMessage *selfMsg;
    cMessage *cleanupMsg;

    L3Address destAddress;
    int localPort, destPort;

    int myId;
    int myRole = 0; // 0: Undefined, 1: CH, 2: Member
    double myTotalWeight = 0.0;

    std::map<int, NeighborData> neighborTable;

    // --- GÖRSELLEŞTİRME DEĞİŞKENLERİ ---
    int currentLeaderId = -1;       // Bağlı olduğum liderin ID'si
    cLineFigure *connectionLine = nullptr; // Çizgi objesi

    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }

    virtual void refreshDisplay() const override;

    virtual ~LetApp();

    double calculateLET(double xi, double yi, double vi, double thetai,
                        double xj, double yj, double vj, double thetaj, double R);
    void sendHelloPacket();
    void runClusteringLogic();
    void removeOldNeighbors();
    void updateVisuals();

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override { delete indication; }
    virtual void socketClosed(UdpSocket *socket) override {}
};

#endif
