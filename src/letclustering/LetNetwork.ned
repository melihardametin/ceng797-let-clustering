package src.letclustering;

import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.visualizer.integrated.IntegratedCanvasVisualizer;

network LetNetwork
{
    parameters:
        int numHosts;
        @display("bgb=600,400;bgg=100,1,grey95");

    submodules:
        configurator: Ipv4NetworkConfigurator {
            @display("p=50,150");
        }

        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=50,250");
        }

        host[numHosts]: AdhocHost {
            @display("p=250,200");
        }
}