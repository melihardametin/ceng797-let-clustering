package src.letclustering;

import inet.applications.contract.IApp;

simple LetApp like IApp
{
    parameters:
        int localPort = default(5000);
        int destPort = default(5000);
        volatile double messageInterval @unit(s) = default(1s);
        @display("i=block/app");

        // --- Sinyaller ---
        @signal[clusterHeadLifetime](type="simtime_t");
        @signal[clusterChangeRate](type="long");
        @signal[controlOverhead](type="long");
        @signal[isClusterHead](type="long");
        @signal[pdr](type="long");
        @signal[endToEndDelay](type="simtime_t");
        @signal[hopCount](type="long");
        @signal[clusterSize](type="long");
        @signal[residualEnergy](type="double");
        @signal[throughput](type="double");

        // --- İstatistikler ---
        @statistic[clusterHeadLifetime](title="CH Lifetime"; source="clusterHeadLifetime"; record=vector,mean,max; unit=s);
        @statistic[controlOverhead](title="Control Overhead"; source="controlOverhead"; record=sum; unit=B);
        @statistic[pdr](title="Packet Delivery Ratio"; source="pdr"; record=mean,vector);
        @statistic[endToEndDelay](title="Delay"; source="endToEndDelay"; record=vector,mean; unit=s);
        @statistic[hopCount](title="Hop Count"; source="hopCount"; record=vector,mean);
        @statistic[clusterSize](title="Cluster Size"; source="clusterSize"; record=vector,mean,max);
        @statistic[residualEnergy](title="Residual Energy"; source="residualEnergy"; record=vector,mean,min; unit=J);
        @statistic[throughput](title="Throughput"; source="throughput"; record=vector,mean; unit=bps);

    gates:
        input socketIn;
        output socketOut;
}