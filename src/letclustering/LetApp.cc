#include "LetApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3AddressResolver.h"

Define_Module(LetApp);

void LetApp::initialize(int stage)
{
    if (stage == inet::INITSTAGE_APPLICATION_LAYER) {

        selfMsg = new cMessage("sendHello");

        mobility = check_and_cast<IMobility *>(getParentModule()->getSubmodule("mobility"));

        int localPort = par("localPort");
        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setCallback(this);

        scheduleAt(simTime() + par("messageInterval"), selfMsg);
    }
}

void LetApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        sendHelloPacket();
        scheduleAt(simTime() + par("messageInterval"), selfMsg);
    }
    else {
        socket.processMessage(msg);
    }
}

void LetApp::sendHelloPacket()
{
    Coord pos = mobility->getCurrentPosition();
    Coord vel = mobility->getCurrentVelocity();
    double speed = vel.length();
    double angle = atan2(vel.y, vel.x);

    auto packet = new Packet("LetHello");
    auto helloPayload = makeShared<LetHello>();

    helloPayload->setPositionX(pos.x);
    helloPayload->setPositionY(pos.y);
    helloPayload->setSpeed(speed);
    helloPayload->setAngle(angle);

    helloPayload->setChunkLength(B(64));

    packet->insertAtBack(helloPayload);

    L3Address destAddr = L3AddressResolver().resolve("255.255.255.255");
    int destPort = par("destPort");
    socket.sendTo(packet, destAddr, destPort);
}

void LetApp::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    auto helloMsg = packet->peekAtFront<LetHello>();

    if (helloMsg) {
        double nX = helloMsg->getPositionX();
        double nY = helloMsg->getPositionY();
        double nV = helloMsg->getSpeed();
        double nA = helloMsg->getAngle();

        Coord pos = mobility->getCurrentPosition();
        Coord vel = mobility->getCurrentVelocity();
        double myV = vel.length();
        double myA = atan2(vel.y, vel.x);

        double range = 250.0;

        double result = calculateLET(pos.x, pos.y, myV, myA, nX, nY, nV, nA, range);

        EV << "LET HESAPLANDI: " << result << " saniye (Komşu Node ile)" << endl;
    }

    delete packet;
}

double LetApp::calculateLET(double xi, double yi, double vi, double thetai,
                            double xj, double yj, double vj, double thetaj, double R)
{
    double a = (vi * cos(thetai)) - (vj * cos(thetaj));
    double b = xi - xj;
    double c = (vi * sin(thetai)) - (vj * sin(thetaj));
    double d = yi - yj;

    double denominator = (a * a) + (c * c);
    if (denominator < 0.00001) {
        return 99999.0;
    }

    double delta = (denominator * R * R) - pow((a * d - b * c), 2);

    if (delta < 0) {
        return 0.0;
    }

    double numerator = -(a * b + c * d) + sqrt(delta);
    return std::max(0.0, numerator / denominator);
}
