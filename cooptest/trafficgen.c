#include <fstream>
#include <string>
#include <vector> 
using namespace std;
//g++ -Wall trafficgen.c -o trafficgen && ./trafficgen 
 
int main()
{
	for(int kk = 0; kk < 5; kk++){
		string fname;
		string cct = "false";
		string tlr = "false";
		string dra = "false";
		string dct = "false";
		string disroute = "false";
		vector<string> src = {"6"};
		if(kk == 0) {
			fname = "cct";
			cct = "true";
		}
		if(kk == 1) {
			fname = "dct";
			dct = "true";
			disroute = "true";	
		}
		if(kk == 2) {
			fname = "dra";
			dra = "true";
			disroute = "true";
		}
		if(kk == 3) {
			fname = "tlr";
			tlr = "true";
		}
		if(kk == 4) {
			fname = "base";
		}
		
		fname += "-iridium-";
		fname += to_string(src.size());
	
		string outfname = fname;
		outfname +=".tr";
		fname += ".tcl";
		string starttime = "60.0";
		string finishtime = "120.0";
		
		vector<string> bfrom = {"6"};			//background from
		vector<string> bto = {"7"};			//background to
		string brate = "1Mbps";
	       	ofstream out;
		out.open(fname);
		out <<"global ns"<<endl;
		out <<"set ns [new Simulator]"<<endl;
		out<<endl;
		out <<"HandoffManager/Term set elevation_mask_ 8.2"<<endl;
		out <<"HandoffManager/Term set term_handoff_int_ 10"<<endl;
		out <<"HandoffManager/Sat set sat_handoff_int_ 10"<<endl;
		out <<"HandoffManager/Sat set latitude_threshold_ 85 "<<endl;
		out <<"Agent/SatRoute set latitude_threshold_ 85"<<endl;
		out <<"HandoffManager/Sat set longitude_threshold_ 10 "<<endl;
		out <<"HandoffManager set handoff_randomization_ true"<<endl;
		out <<"SatRouteObject set metric_delay_ true"<<endl;
		out <<"Node/SatNode set dist_routing_ "+disroute<<endl;
		out <<"SatRouteObject set data_driven_computation_ false"<<endl;
		out <<"ns-random 1"<<endl;
		out <<"Agent set ttl_ 32"<<endl;
		out<<endl;
		out <<"global opt"<<endl;
		out <<"set opt(chan)           Channel/Sat"<<endl;
		out <<"set opt(bw_down)        1.5Mb; # Downlink bandwidth (satellite to ground)"<<endl;
		out <<"set opt(bw_up)          1.5Mb; # Uplink bandwidth"<<endl;
		out <<"set opt(bw_isl)         1Mb"<<endl;
		out <<"set opt(phy)            Phy/Sat"<<endl;
		out <<"set opt(mac)            Mac/Sat"<<endl;
		out <<"set opt(ifq)            Queue/DropTail"<<endl;
		out <<"set opt(qlim)           50"<<endl;
		out <<"set opt(ll)             LL/Sat"<<endl;
		out <<"set opt(wiredRouting)   OFF"<<endl;
		out<<endl;
		out <<"set opt(alt)            780; # Polar satellite altitude (Iridium)"<<endl;
		out <<"set opt(inc)            86.4;"<<endl;
		out<<endl;
		out <<"set outfile [open " + outfname + " w]"<<endl;
		out <<"$ns trace-all $outfile"<<endl;
		out<<endl;
		out <<"set rate 1Mbps"<<endl;
		out <<"set packetsize 210"<<endl;
		out <<"SatRouteObject set islbw $opt(bw_isl)"<<endl;
		out <<"SatRouteObject set frate $rate"<<endl;
		out <<"SatRouteObject set psize $packetsize"<<endl;
		out <<"SatRouteObject set tlr_enabled "+tlr<<endl;
		out <<"SatRouteObject set cct_enabled "+cct<<endl;
		out <<"SatRouteObject set dct_enabled "+dct<<endl;
		out <<"SatRouteObject set dra_enabled "+dra<<endl;
		out<<endl;
		out <<"$ns node-config -satNodeType polar \\"<<endl;
		out <<"                -llType $opt(ll) \\"<<endl;
		out <<"                -ifqType $opt(ifq) \\"<<endl;
		out <<"                -ifqLen $opt(qlim) \\"<<endl;
		out <<"		       -macType $opt(mac) \\"<<endl;
		out <<"                -phyType $opt(phy) \\"<<endl;
		out <<"                -channelType $opt(chan) \\"<<endl;
		out <<"                -downlinkBW $opt(bw_down) \\"<<endl;
		out <<"                -wiredRouting $opt(wiredRouting) "<<endl;
		out<<endl;
		out <<"set alt $opt(alt)"<<endl;
		out <<"set inc $opt(inc)"<<endl;
		out<<endl;
		out <<"source ../sat-iridium-nodes.tcl"<<endl;
		out<<endl;
		out <<"source ../sat-iridium-links.tcl"<<endl;
		out<<endl;
		out <<"$ns node-config -satNodeType terminal"<<endl;
		out <<"set n100 [$ns node]"<<endl;
		out <<"$n100 set-position 37.9 -122.3; # Berkeley"<<endl;
		out <<"set n101 [$ns node]"<<endl;
		out <<"$n101 set-position 27.32 101.46; # Xi chang "<<endl;
		out<<endl;
		out <<"$n100 add-gsl polar $opt(ll) $opt(ifq) $opt(qlim) $opt(mac) $opt(bw_up) \\"<<endl;
		out <<"  $opt(phy) [$n0 set downlink_] [$n0 set uplink_]"<<endl;
		out <<"$n101 add-gsl polar $opt(ll) $opt(ifq) $opt(qlim) $opt(mac) $opt(bw_up) \\"<<endl;
		out <<"  $opt(phy) [$n0 set downlink_] [$n0 set uplink_]"<<endl;
		out<<endl;
		out <<"$ns trace-all-satlinks $outfile"<<endl;
		out<<endl;
		for(uint32_t i = 0; i < src.size(); i++){
			out<<"set udp"+to_string(i)+" [new Agent/UDP]"<<endl;
			out<<"$ns attach-agent $n"+src[i]+" $udp"+to_string(i)<<endl;
			out<<"set cbr"+to_string(i)+" [new Application/Traffic/CBR]"<<endl;
			out<<"$cbr"+to_string(i)+" attach-agent $udp"+to_string(i)<<endl;
			out<<"$cbr"+to_string(i)+" set packet_size_ $packetsize"<<endl;
			out<<"$cbr"+to_string(i)+" set rate_ $rate"<<endl;
			out<<endl;
			out<<"set null"+to_string(i)+" [new Agent/Null]"<<endl;
			out<<"$ns attach-agent $n7 $null"+to_string(i)<<endl;		//n101 is xi chang
			out<<endl;
			out<<"$ns connect $udp"+to_string(i)+" $null"+to_string(i)<<endl;
			out<<"$ns at "+starttime+" \"$cbr"+to_string(i)+" start\""<<endl;
			out<<endl;
		}
		/*
		for(uint32_t i = 0; i < bfrom.size(); i++){
			out<<"set udp"+to_string(i+src.size())+" [new Agent/UDP]"<<endl;
			out<<"$ns attach-agent $n"+bfrom[i]+" $udp"+to_string(i+src.size())<<endl;
			out<<"set cbr"+to_string(i+src.size())+" [new Application/Traffic/CBR]"<<endl;
			out<<"$cbr"+to_string(i+src.size())+" attach-agent $udp"+to_string(i+src.size())<<endl;
			out<<"$cbr"+to_string(i+src.size())+" set packet_size_ $packetsize"<<endl;
			out<<"$cbr"+to_string(i+src.size())+" set rate_ "+brate<<endl;
			out<<endl;		
			out<<"set null"+to_string(i+src.size())+" [new Agent/Null]"<<endl;
			out<<"$ns attach-agent $n"+bto[i]+" $null"+to_string(i+src.size())<<endl;
			out<<endl;
			out<<"$ns connect $udp"+to_string(i+src.size())+" $null"+to_string(i+src.size())<<endl;
			out<<"$ns at "+starttime+" \"$cbr"+to_string(i+src.size())+" start\""<<endl;
			out<<endl;
		}*/
		
		out <<"set satrouteobject_ [new SatRouteObject]"<<endl;
		out <<"$satrouteobject_ compute_routes"<<endl;
		out<<endl;
		out <<"$ns at "+finishtime+" \"finish\" ; # one earth rotation "<<endl;
		out<<endl;
		out <<"proc finish {} {"<<endl;
		out <<"  global ns outfile "<<endl;
		out <<"  $ns flush-trace"<<endl;
		out <<"  close $outfile"<<endl;
		out<<endl;
		out <<"	 exit 0"<<endl;
		out <<"}"<<endl;
		out<<endl;
		out <<"$ns run"<<endl;
	}
	return 0;
}
