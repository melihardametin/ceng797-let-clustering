#ifndef __LETCLUSTERING_LETAPP_H_
#define __LETCLUSTERING_LETAPP_H_

#include <omnetpp.h>
#include "inet/common/geometry/common/Coord.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "LetMessage_m.h"
using namespace omnetpp;
using namespace inet;

class LetApp : public cSimpleModule, public UdpSocket::ICallback
{
  protected:
    IMobility *mobility;
    UdpSocket socket;
    cMessage *selfMsg;

    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }

    double calculateLET(double xi, double yi, double vi, double thetai,
                        double xj, double yj, double vj, double thetaj, double R);

    void sendHelloPacket();

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override { delete indication; }
    virtual void socketClosed(UdpSocket *socket) override {}
};

#endif
