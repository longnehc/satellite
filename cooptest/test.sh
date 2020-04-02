ns base-iridium-1.tcl 
gawk -f throughput.awk base-iridium-1.tr
ns tlr-iridium-1.tcl
gawk -f throughput.awk tlr-iridium-1.tr
ns dra-iridium-1.tcl
gawk -f throughput.awk dra-iridium-1.tr
ns dct-iridium-1.tcl
gawk -f throughput.awk dct-iridium-1.tr
ns cct-iridium-1.tcl
gawk -f throughput.awk cct-iridium-1.tr
 

