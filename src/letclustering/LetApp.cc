#include "LetApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include <algorithm>
#include <cmath>
#include <string>

Define_Module(LetApp);
using namespace inet::units::values;

LetApp::~LetApp() {
    cancelAndDelete(selfMsg);
    cancelAndDelete(cleanupMsg);
    cancelAndDelete(dataMsg);
    cancelAndDelete(unmarkMsg);
    if (connectionLine) {
        cCanvas *canvas = getParentModule()->getParentModule()->getCanvas();
        if (canvas) canvas->removeFigure(connectionLine);
        delete connectionLine;
    }
}

void LetApp::initialize(int stage)
{
    if (stage == inet::INITSTAGE_APPLICATION_LAYER) {
        localPort = par("localPort");
        destPort = par("destPort");
        destAddress = L3AddressResolver().resolve("224.0.0.1");

        myId = getParentModule()->getIndex();

        mobility = check_and_cast<IMobility *>(getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.joinMulticastGroup(destAddress);
        socket.setMulticastLoop(false);
        socket.setCallback(this);

        selfMsg = new cMessage("sendHello");
        cleanupMsg = new cMessage("cleanup");
        dataMsg = new cMessage("generateData");
        unmarkMsg = new cMessage("unmarkColors");

        scheduleAt(simTime() + par("messageInterval") + uniform(0, 0.1), selfMsg);
        scheduleAt(simTime() + 0.5, cleanupMsg);

        scheduleAt(simTime() + 10.0 + uniform(0, 2), dataMsg);

        connectionLine = new cLineFigure(("link_" + std::to_string(myId)).c_str());
        connectionLine->setLineStyle(cFigure::LINE_DOTTED);
        connectionLine->setLineColor(cFigure::RED);
        connectionLine->setVisible(false);
        getParentModule()->getParentModule()->getCanvas()->addFigure(connectionLine);

        pdrSignal = registerSignal("pdr");
        delaySignal = registerSignal("endToEndDelay");
        hopCountSignal = registerSignal("hopCount");
        clusterHeadLifetimeSignal = registerSignal("clusterHeadLifetime");
        controlOverheadSignal = registerSignal("controlOverhead");
        isClusterHeadSignal = registerSignal("isClusterHead");
        clusterChangeRateSignal = registerSignal("clusterChangeRate");
        clusterSizeSignal = registerSignal("clusterSize");
        residualEnergySignal = registerSignal("residualEnergy");
        throughputSignal = registerSignal("throughput");

        roleStartTime = simTime();

        cModule *host = getParentModule();
        if (host->getSubmodule("energyStorage")) {
            energyStorage = check_and_cast<inet::power::SimpleEpEnergyStorage*>(host->getSubmodule("energyStorage"));
        }
    }
}

void LetApp::handleMessage(cMessage *msg)
{
    if (msg == selfMsg) {
        sendHelloPacket();
        scheduleAt(simTime() + par("messageInterval"), selfMsg);
    }
    else if (msg == cleanupMsg) {
        removeOldNeighbors();
        runClusteringLogic();
        scheduleAt(simTime() + 0.5, cleanupMsg);
    }
    else if (msg == dataMsg) {
        sendDataPacket();
        scheduleAt(simTime() + uniform(0.5, 1.5), dataMsg);
    }
    else if (msg == unmarkMsg) {
        resetNodeColors();
    }
    else if (strcmp(msg->getName(), "floodingTimer") == 0) {
        Packet *pkt = (Packet *)msg->getContextPointer();
        if (pkt) socket.sendTo(pkt, destAddress, destPort);
        delete msg;
    }
    else {
        socket.processMessage(msg);
    }
}

// ------------------------------------------------------------------
// CLUSTERING MANTIĞI (LET)
// ------------------------------------------------------------------
void LetApp::sendHelloPacket()
{
    Coord pos = mobility->getCurrentPosition();
    Coord vel = mobility->getCurrentVelocity();

    auto packet = new Packet("LetHello");
    auto helloPayload = makeShared<LetHello>();

    helloPayload->setSenderId(myId);
    helloPayload->setRole(myRole);
    helloPayload->setCurrentWeight(myTotalWeight);
    helloPayload->setCurrentLeaderId(currentLeaderId);

    helloPayload->setPositionX(pos.x);
    helloPayload->setPositionY(pos.y);
    helloPayload->setSpeed(vel.length());
    helloPayload->setAngle(atan2(vel.y, vel.x));

    helloPayload->setChunkLength(B(64));
    packet->insertAtBack(helloPayload);

    emit(controlOverheadSignal, (long)helloPayload->getChunkLength().get());

    socket.sendTo(packet, destAddress, destPort);
}

void LetApp::runClusteringLogic()
{
    // 1. Ağırlık Hesabı
    myTotalWeight = 0.0;
    for (auto const& [id, data] : neighborTable) {
        double val = (data.letValue > 1000) ? 1000 : data.letValue;
        myTotalWeight += val;
    }

    // 2. CH Seçimi
    bool iamStrongest = true;
    int bestNeighborId = -1;
    double maxScore = -1.0;
    double myScore = myTotalWeight + (myId * 0.0001);

    for (auto const& [id, data] : neighborTable) {
        double neighborScore = data.theirWeight + (id * 0.0001);
        if (neighborScore > maxScore) {
            maxScore = neighborScore;
            bestNeighborId = id;
        }
        if (neighborScore > myScore) iamStrongest = false;
    }

    // 3. Rol Atama
    int oldRole = myRole;
    int oldLeader = currentLeaderId;

    if (iamStrongest) {
        myRole = ROLE_CH;
        currentLeaderId = myId;
    } else {
        myRole = ROLE_MEMBER;
        currentLeaderId = bestNeighborId;

        bool isGw = false;
        for (auto const& [id, data] : neighborTable) {
            if (id == currentLeaderId) continue;
            if (data.role == ROLE_CH || (data.theirLeaderId != -1 && data.theirLeaderId != currentLeaderId)) {
                isGw = true;
                break;
            }
        }
        if (isGw) myRole = ROLE_GATEWAY;
    }

    if (myRole != oldRole || currentLeaderId != oldLeader) {
        if (oldRole == ROLE_CH) {
            emit(clusterHeadLifetimeSignal, simTime() - roleStartTime);
        }
        emit(clusterChangeRateSignal, 1);
        roleStartTime = simTime();
    }

    emit(isClusterHeadSignal, (myRole == ROLE_CH) ? 1 : 0);


    if (myRole == ROLE_CH) {
        int myClusterSize = 0;
        for (auto const& [id, data] : neighborTable) {
            if (data.theirLeaderId == myId) {
                myClusterSize++;
            }
        }

        emit(clusterSizeSignal, myClusterSize);
    } else {
        emit(clusterSizeSignal, 0); // CH değilsem boyutum 0
    }


    if (energyStorage) {
        double residual = energyStorage->getResidualEnergyCapacity().get();
        emit(residualEnergySignal, residual);

        if (residual <= 0) {

        }
    }

    updateVisuals();
}

double LetApp::calculateLET(double xi, double yi, double vi, double thetai,
                            double xj, double yj, double vj, double thetaj, double R)
{
    double a = (vi * cos(thetai)) - (vj * cos(thetaj));
    double b = xi - xj;
    double c = (vi * sin(thetai)) - (vj * sin(thetaj));
    double d = yi - yj;
    double denominator = (a * a) + (c * c);

    if (fabs(denominator) < 0.00001) return 1000.0;
    double delta = (denominator * R * R) - pow((a * d - b * c), 2);
    if (delta < 0) return 0.0;
    double numerator = -(a * b + c * d) + sqrt(delta);
    return std::max(0.0, numerator / denominator);
}

// ------------------------------------------------------------------
// BASİTLEŞTİRİLMİŞ ROUTING (BACKBONE FLOODING)
// ------------------------------------------------------------------

void LetApp::sendDataPacket()
{
    if (currentLeaderId == -1 && myRole != ROLE_CH) return;

    int destId = intuniform(0, getParentModule()->getParentModule()->par("numHosts").intValue() - 1);
    if (destId == myId) return;

    auto packet = new Packet("LetData");
    auto dataPayload = makeShared<LetData>();

    dataPayload->setSrcId(myId);
    dataPayload->setDestId(destId);
    dataPayload->setSeqId(mySeqNum++);
    dataPayload->setHopCount(0);
    dataPayload->setChunkLength(B(1000));
    dataPayload->setCreationTime(simTime());

    packet->insertAtBack(dataPayload);

    std::string pktId = std::to_string(myId) + "_" + std::to_string(mySeqNum - 1);
    seenPackets.insert(pktId);

    socket.sendTo(packet, destAddress, destPort);
    pktsSent++;
}

void LetApp::processDataPacket(Packet *packet)
{
    auto data = packet->peekAtFront<LetData>();
    int destId = data->getDestId();
    int srcId = data->getSrcId();
    std::string pktId = std::to_string(srcId) + "_" + std::to_string(data->getSeqId());

    if (seenPackets.find(pktId) != seenPackets.end()) {
        delete packet;
        return;
    }
    seenPackets.insert(pktId);

    if (destId == myId) {
        pktsReceived++;

        long bits = packet->getBitLength();
        totalBitsReceived += bits;

        double currentThroughput = totalBitsReceived / simTime().dbl();
        emit(throughputSignal, currentThroughput);


        emit(pdrSignal, 1);
        emit(hopCountSignal, data->getHopCount());
        emit(delaySignal, simTime() - data->getCreationTime());

        getParentModule()->bubble("ARRIVED!");
        getParentModule()->getDisplayString().setTagArg("i", 1, "blue");

        if (unmarkMsg->isScheduled()) {
            cancelEvent(unmarkMsg);
        }
        scheduleAt(simTime() + 0.2, unmarkMsg);

        delete packet;
        return;
    }

    if (data->getHopCount() > 15) { delete packet; return; }

    if (myRole == ROLE_MEMBER) {
        delete packet;
        return;
    }


    auto fwdPacket = packet->dup();
    auto fwdData = fwdPacket->removeAtFront<LetData>();
    fwdData->setHopCount(fwdData->getHopCount() + 1);
    fwdPacket->insertAtFront(fwdData);

    cMessage *timer = new cMessage("floodingTimer");
    timer->setContextPointer(fwdPacket);
    scheduleAt(simTime() + uniform(0.001, 0.010), timer);

    delete packet;
}


void LetApp::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    if (packet->getBitLength() == 0) { delete packet; return; }
    auto chunk = packet->peekAtFront<Chunk>();

    if (auto helloMsg = dynamicPtrCast<const LetHello>(chunk)) {
        int nId = helloMsg->getSenderId();
        if (nId == myId) { delete packet; return; }

        Coord pos = mobility->getCurrentPosition();
        Coord vel = mobility->getCurrentVelocity();
        double mySpeed = vel.length();
        double myAngle = (mySpeed < 0.001) ? 0.0 : atan2(vel.y, vel.x);

        double let = calculateLET(pos.x, pos.y, mySpeed, myAngle,
                                  helloMsg->getPositionX(), helloMsg->getPositionY(),
                                  helloMsg->getSpeed(), helloMsg->getAngle(), 250.0);

        neighborTable[nId].letValue = let;
        neighborTable[nId].theirWeight = helloMsg->getCurrentWeight();
        neighborTable[nId].theirLeaderId = helloMsg->getCurrentLeaderId();
        neighborTable[nId].role = helloMsg->getRole();
        neighborTable[nId].lastSeen = simTime();
        delete packet;
    }
    else if (auto dataPkt = dynamicPtrCast<const LetData>(chunk)) {
        processDataPacket(packet);
    }
    else {
        delete packet;
    }
}

void LetApp::removeOldNeighbors()
{
    simtime_t timeout = 2.0;
    for (auto it = neighborTable.begin(); it != neighborTable.end(); ) {
        if (simTime() - it->second.lastSeen > timeout) {
             it = neighborTable.erase(it);
        } else {
            ++it;
        }
    }
}

void LetApp::updateVisuals()
{
    std::string color = "green";
    std::string roleTxt = "";
    if (myRole == ROLE_CH) { color = "red"; roleTxt = "CH"; }
    else if (myRole == ROLE_GATEWAY) { color = "yellow"; roleTxt = "GW"; }

    getParentModule()->getDisplayString().setTagArg("i", 1, color.c_str());
    getParentModule()->getDisplayString().setTagArg("t", 0, roleTxt.c_str());
}

void LetApp::resetNodeColors() { updateVisuals(); }

void LetApp::refreshDisplay() const {
    if (!connectionLine) return;
    if ((myRole == ROLE_MEMBER || myRole == ROLE_GATEWAY) && currentLeaderId != -1) {
        cModule *leaderMod = getParentModule()->getParentModule()->getSubmodule("host", currentLeaderId);
        if (leaderMod) {
            auto myMobility = check_and_cast<IMobility*>(getParentModule()->getSubmodule("mobility"));
            auto leaderMobility = check_and_cast<IMobility*>(leaderMod->getSubmodule("mobility"));
            connectionLine->setStart(cFigure::Point(myMobility->getCurrentPosition().x, myMobility->getCurrentPosition().y));
            connectionLine->setEnd(cFigure::Point(leaderMobility->getCurrentPosition().x, leaderMobility->getCurrentPosition().y));
            connectionLine->setVisible(true);
        } else { connectionLine->setVisible(false); }
    } else { connectionLine->setVisible(false); }
}

void LetApp::finish()
{
    recordScalar("PacketsSent", pktsSent);
    recordScalar("PacketsReceived", pktsReceived);
    if (pktsSent > 0) {
        double ratio = (double)pktsReceived / pktsSent;
        recordScalar("PDR", ratio);
        emit(pdrSignal, (long)(ratio * 100));
    }
}
