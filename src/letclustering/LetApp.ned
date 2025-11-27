package src.letclustering;

import inet.applications.contract.IApp;

simple LetApp like IApp
{
    parameters:
        int localPort = default(5000);
        int destPort = default(5000);
        volatile double messageInterval @unit(s) = default(1s);
        @display("i=block/app");

    gates:
        input socketIn;
        output socketOut;
}