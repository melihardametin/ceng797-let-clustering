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
        myId = getParentModule()->getId();

        mobility = check_and_cast<IMobility *>(getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.joinMulticastGroup(destAddress);
        socket.setMulticastLoop(false);
        socket.setCallback(this);

        selfMsg = new cMessage("sendHello");
        cleanupMsg = new cMessage("cleanup");

        scheduleAt(simTime() + par("messageInterval") + uniform(0, 0.1), selfMsg);
        scheduleAt(simTime() + 0.5, cleanupMsg);

        connectionLine = new cLineFigure(("link_" + std::to_string(myId)).c_str());
        connectionLine->setLineStyle(cFigure::LINE_DOTTED);
        connectionLine->setLineColor(cFigure::RED);
        connectionLine->setLineWidth(2);
        connectionLine->setEndArrowhead(cFigure::ARROW_TRIANGLE);
        connectionLine->setVisible(false);
        connectionLine->setZIndex(-1);
        getParentModule()->getParentModule()->getCanvas()->addFigure(connectionLine);
    }
}

void LetApp::handleMessage(cMessage *msg)
{
    if (msg == selfMsg) {
        sendHelloPacket();
        double interval = par("messageInterval").doubleValue();
        scheduleAt(simTime() + interval + uniform(-0.01, 0.01), selfMsg);
    }
    else if (msg == cleanupMsg) {
        removeOldNeighbors();
        runClusteringLogic();
        scheduleAt(simTime() + 0.5, cleanupMsg);
    }
    else {
        socket.processMessage(msg);
    }
}

void LetApp::refreshDisplay() const {
    if (!connectionLine || !connectionLine->isVisible()) return;

    Coord myPos = mobility->getCurrentPosition();
    cModule *leaderMod = getSimulation()->getModule(currentLeaderId);

    if (leaderMod) {
        auto leaderMobility = check_and_cast<IMobility*>(leaderMod->getSubmodule("mobility"));
        Coord leaderPos = leaderMobility->getCurrentPosition();
        connectionLine->setStart(cFigure::Point(myPos.x, myPos.y));
        connectionLine->setEnd(cFigure::Point(leaderPos.x, leaderPos.y));
    } else {
        const_cast<cLineFigure*>(connectionLine)->setVisible(false);
    }
}

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
    socket.sendTo(packet, destAddress, destPort);
}

void LetApp::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    auto helloMsg = packet->peekAtFront<LetHello>();
    if (helloMsg) {
        int nId = helloMsg->getSenderId();

        Coord pos = mobility->getCurrentPosition();
        Coord vel = mobility->getCurrentVelocity();
        double let = calculateLET(pos.x, pos.y, vel.length(), atan2(vel.y, vel.x),
                                  helloMsg->getPositionX(), helloMsg->getPositionY(),
                                  helloMsg->getSpeed(), helloMsg->getAngle(), 250.0);

        neighborTable[nId].letValue = let;
        neighborTable[nId].theirWeight = helloMsg->getCurrentWeight();

        neighborTable[nId].theirLeaderId = helloMsg->getCurrentLeaderId();

        neighborTable[nId].lastSeen = simTime();
    }
    delete packet;
}

void LetApp::runClusteringLogic()
{
    // A. Kendi Ağırlığımı Hesapla
    myTotalWeight = 0.0;
    for (auto const& [id, data] : neighborTable) {
        double val = (data.letValue > 1000) ? 1000 : data.letValue;
        myTotalWeight += val;
    }

    // B. En İyi Lider Adayını Bul
    bool iamStrongest = true;
    int bestNeighborId = -1;
    double maxScore = -1.0;

    // Skor = Weight + (ID * epsilon)
    // Bu sayede puanlar yakınsa bile bir sıralama oluşur.
    double myScore = myTotalWeight + (myId * 0.0001);

    for (auto const& [id, data] : neighborTable) {
        double neighborScore = data.theirWeight + (id * 0.0001);

        if (neighborScore > maxScore) {
            maxScore = neighborScore;
            bestNeighborId = id;
        }

        if (neighborScore > myScore) {
            iamStrongest = false;
        }
    }

    // --- C. DÖNGÜ (LOOP) KONTROLÜ - KRİTİK BÖLÜM ---
    // Eğer ben birini (X) lider seçmeye niyetlendiysem (Member olacaksam),
    // ama X de beni lider seçtiyse (Loop), bu durumu kırmalıyız.

    if (!iamStrongest && bestNeighborId != -1) {
        NeighborData& leaderData = neighborTable[bestNeighborId];

        // KONTROL: Liderim (bestNeighborId) beni mi takip ediyor?
        if (leaderData.theirLeaderId == myId) {

            // DÖNGÜ VAR! (A->B ve B->A)
            // ÇÖZÜM: ID'si büyük olan sorumluluk alıp Lider (CH) olsun.
            // Diğeri (ID'si küçük olan) ona uymak zorunda kalacak.

            if (myId > bestNeighborId) {
                iamStrongest = true; // Kararımı değiştiriyorum, Lider benim!
            }
        }
    }

    // D. Rolü Ata
    if (iamStrongest) {
        myRole = 1; // CH
        currentLeaderId = -1;
    } else {
        myRole = 2; // Member
        currentLeaderId = bestNeighborId;
    }

    updateVisuals();
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
    auto displayString = getParentModule()->getDisplayString();

    if (myRole == 1) { // CH
        getParentModule()->getDisplayString().setTagArg("i", 1, "red");
        getParentModule()->getDisplayString().setTagArg("r", 0, "150");
        getParentModule()->getDisplayString().setTagArg("r", 4, "red");
        connectionLine->setVisible(false);
    } else { // Member
        getParentModule()->getDisplayString().setTagArg("i", 1, "green");
        getParentModule()->getDisplayString().setTagArg("r", 0, "");
        if (currentLeaderId != -1) {
            connectionLine->setVisible(true);
        } else {
            connectionLine->setVisible(false);
        }
    }
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

void LetApp::finish() {
    // Simülasyon bittiğinde yapılacak işlemler
}
