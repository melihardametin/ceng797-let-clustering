package src.letclustering;

import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.common.contract. packetlevel.IRadioMedium;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.visualizer.common.IntegratedVisualizer;

network LetNetwork
{
    parameters:
        int numHosts;
        @display("bgb=1000,1000;bgg=100,1,grey95");

    submodules:
		visualizer: IntegratedVisualizer {
            @display("p=100,50");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=50,150");
        }

		radioMedium: <default("Ieee80211ScalarRadioMedium")> like IRadioMedium {
            @display("p=100,200");
        }

        host[numHosts]: AdhocHost {
            @display("p=250,200");
        }
}