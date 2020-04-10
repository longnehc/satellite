/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1999 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the MASH Research
 *      Group at the University of California Berkeley.
 * 4. Neither the name of the University nor of the Research Group may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Contributed by Tom Henderson, UCB Daedalus Research Group, June 1999
 */

#ifndef lint
static const char rcsid[] =
    "@(#) $Header: /cvsroot/nsnam/ns-2/satellite/satroute.cc,v 1.13 2005/05/19 03:19:02 tomh Exp $";
#endif

#include "satroute.h"
#include "sattrace.h"
#include "satnode.h"
#include "satlink.h"
#include "route.h"
#include <address.h>
#include <iostream>
#include <fstream>

 
using namespace std;
extern "C"{
#include <glpk.h>
}

void test(){

/* declare variables */
  glp_prob *lp;
  int ia[1+1000], ja[1+1000];
  double ar[1+1000], z, x1, x2;
  /* create problem */
  lp = glp_create_prob();
  glp_set_prob_name(lp, "short");
  glp_set_obj_dir(lp, GLP_MAX);
  /* fill problem */
  glp_add_rows(lp, 2);
 // glp_set_row_name(lp, 1, "p");
  glp_set_row_bnds(lp, 1, GLP_UP, 0.0, 1.0);
 // glp_set_row_name(lp, 2, "q");
  glp_set_row_bnds(lp, 2, GLP_UP, 0.0, 2.0);
  glp_add_cols(lp, 2);
 // glp_set_col_name(lp, 1, "x1");
  glp_set_col_bnds(lp, 1, GLP_LO, 0.0, 0.0);
  glp_set_obj_coef(lp, 1, 0.6);
 // glp_set_col_name(lp, 2, "x2");
  glp_set_col_bnds(lp, 2, GLP_LO, 0.0, 0.0);
  glp_set_obj_coef(lp, 2, 0.5);
  ia[1] = 1, ja[1] = 1, ar[1] = 1.0; /* a[1,1] = 1 */
  ia[2] = 1, ja[2] = 2, ar[2] = 2.0; /* a[1,2] = 2 */
  ia[3] = 2, ja[3] = 1, ar[3] = 3.0; /* a[2,1] = 3 */
  ia[4] = 2, ja[4] = 2, ar[4] = 1.0; /* a[2,2] = 1 */
  glp_load_matrix(lp, 4, ia, ja, ar);
  /* solve problem */
  glp_simplex(lp, NULL);
  /* recover and display results */
  z = glp_get_obj_val(lp);
  x1 = glp_get_col_prim(lp, 1);
  x2 = glp_get_col_prim(lp, 2);
  printf("z = %g; x1 = %g; x2 = %g\n", z, x1, x2);
  /* housekeeping */
  glp_delete_prob(lp);
  glp_free_env();
}

static class SatRouteClass:public TclClass
{
  public:
	SatRouteClass ():TclClass ("Agent/SatRoute") { }
	TclObject *create (int, const char *const *) {
    		return (new SatRouteAgent ());
	}
} class_satroute;

double SatRouteAgent::latitude_threshold_ = 0;

 

SatRouteAgent::SatRouteAgent (): Agent (PT_MESSAGE), maxslot_(0), nslot_(0), slot_(0)
{
	bind ("myaddr_", &myaddr_);
	bind("latitude_threshold_", &latitude_threshold_);
}

SatRouteAgent* SatRouteAgent::instance_;

SatRouteAgent::~SatRouteAgent()
{
	if (slot_)
	    delete [] slot_;
}

void SatRouteAgent::alloc(int slot)
{
	slot_entry *old = slot_;
	int n = nslot_;
	if (old == 0)
		nslot_ = 32;
	while (nslot_ <= slot)
		nslot_ <<= 1;
	slot_ = new slot_entry[nslot_];
	memset(slot_, 0, nslot_ * sizeof(slot_entry));
	for (int i = 0; i < n; ++i) {
		slot_[i].next_hop = old[i].next_hop;
		slot_[i].entry = old[i].entry;
	}
	delete [] old;
}

void SatRouteAgent::install(int slot, int nh, NsObject* p)
{
	if (slot >= nslot_)
		alloc(slot);
	slot_[slot].next_hop = nh;
	slot_[slot].entry = p;
	if (slot >= maxslot_)
		maxslot_ = slot;
}

void SatRouteAgent::clear_slots()
{
	if (slot_)
		delete [] slot_;
	slot_ = 0;
	nslot_ = 0;
	maxslot_ = -1;
}

int SatRouteAgent::command (int argc, const char *const *argv)
{
        Tcl& tcl = Tcl::instance();
        if (argc == 2) {
        }
        if (argc == 3) {
               if (strcmp(argv[1], "set_node") == 0) {
                        node_ = (SatNode *) TclObject::lookup(argv[2]);
                        if (node_ == 0) {
                                tcl.resultf("no such object %s", argv[2]);
                                return (TCL_ERROR);
                        }
                        return (TCL_OK);
		}
	}
	return (Agent::command (argc, argv));
}

//dst ranges from >= 66; return value ranges from 0-65
int SatRouteAgent::dct_coop_selection(int dst){
	map<int, map<int, vector<double> > > coopprofile = SatRouteObject::instance().get_coopprofile();
	int t_dst = dst + 1;
	if(coopprofile.find(t_dst) == coopprofile.end()) {cout<<"coop to "<<t_dst<<" does not exists at "<<NOW<<endl;}
	map<int, vector<double> > tm = coopprofile[t_dst];
	double cur = NOW;
	int res=0;
	double maxdur = -1;      //record variable
	for(int i = 1; i <=66; i++){
		if(tm.find(i) != tm.end()){
			vector<double>tv = tm[i];
	//if(t_dst==68 && i == 35) cout<<"find "<<i<<" for dst= "<<t_dst<<endl;
                        for(int j = 0; j < tv.size(); j = j + 2){
				//if(j%2 == 0) cout<<"start "<<tv[j]<<",";
				//else cout<<"end "<<tv[j]<<",";
				if(int(tv[j]) <= int(cur) && j + 1 <tv.size()){
				     if(int(tv[j+1]) <= int(cur)) continue;
					//cout<<"find "<<i<<" for dst= "<<t_dst<<" dur = "<<tv[j+1]-cur<<endl;
				     if(tv[j+1]-cur > maxdur){
						res = i;
						maxdur = tv[j+1]-cur;
						//cout <<"tv[j]="<<tv[j]<<",res="<<res<<" NOW="<<cur<<" maxdur = "<<maxdur<<endl;
					}
					//cout<<"break!!!!!!!!!!: "<<i<<endl;
					break;
				}
			        if(int(tv[j]) <= int(cur) && j == tv.size() - 1){
				      if(2000-cur > maxdur){
					    res = i;
					    maxdur = 2000-cur;
				      }
				      break;
				}
			}
			
		}
        }
/*	if(res == 0) {
	cout<<"t_dst= "<<t_dst<<" NOW: "<<NOW<<endl;
		for(int i = 1; i <=66; i++){
			if(tm.find(i) != tm.end()){
				vector<double>tv = tm[i];
cout<<"i="<<i<<" "; 
		                for(int j = 0; j < tv.size(); j++){
					cout<<tv[j]<<",";
				}
cout<<endl;
			}
		}
	}*/
	if(res == 0){cout<<"coop to "<<t_dst<<" does not find at "<<NOW<<endl; exit(1);}
	//cout<<"find final: "<<res-1<<endl;
	return res-1;
}

//myaddr,dst ranges from 0-65
bool SatRouteAgent::isconnected(int myaddr, int dst){
	bool res = false;
	
	return res;
}

//from and to range from 1- 66
bool SatRouteAgent::droppacket(int from, int to){
	//return false; 		//drop packet switch
	srand((int)time(0));
	double x = (double)rand()/RAND_MAX;
	double y = SatRouteObject::instance().get_plr(from, to);
	if(x < y){
		//cout<<"The packet is droped from "<<from-1<<" to "<<to-1<<" plr = "<<y<<endl;
		return true;
	}
	else
		return false;  
}



/*
 *  Find a target for the received packet
 */

 
bool dropenabled = false;
void SatRouteAgent::forwardPacket(Packet * p)
{
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
	hdr_ip *iph = hdr_ip::access(p);
  	hdr_cmn *hdrc = HDR_CMN (p);
	NsObject *link_entry_;
	int lasthop = hdrc->last_hop_;
	hdrc->direction() = hdr_cmn::DOWN; // send it down the stack
	int dst = Address::instance().get_nodeaddr(iph->daddr());
	// Here we need to have an accurate encoding of the next hop routing
	// information
	if (myaddr_ == iph->daddr()) {
		printf("Error:  trying to forward a packet destined to self: %d\n", myaddr_); 
		Packet::free(p);
	}
	hdrc->addr_type_ = NS_AF_INET;
	hdrc->last_hop_ = myaddr_; // for tracing purposes 
	if (SatRouteObject::instance().data_driven_computation())
		SatRouteObject::instance().recompute_node(myaddr_);
	if (SatNode::dist_routing_ == 0) {
		if(NOW < 10)
			SatRouteObject::instance().compute_topology();
		adj_entry* pubadj_ = SatRouteObject::instance().getAdj();
		int size = 128;
		#define ADJ_ENTRY2(i, j) pubadj_[INDEX(i, j, size)].entry
		#define ADJ2(i, j) pubadj_[INDEX(i, j, size)].cost
		//cout<<"myaddr="<<myaddr_<<",dst="<<dst<<endl;
		if(ADJ2(myaddr_+1, dst+1) !=SAT_ROUTE_INFINITY){		//route for the background flow
			//cout<<"myaddr="<<myaddr_<<",dst="<<dst<<endl;
			link_entry_ = (NsObject*)ADJ_ENTRY2(myaddr_+1, dst+1);	
			//set next hop & call recv
			hdrc->next_hop_  = dst;
                	link_entry_->recv(p, (Handler *)0);
			return;
		}		
		else{
			if (slot_ == 0) { // No routes to anywhere
				if (node_->trace())
					node_->trace()->traceonly(p);
				Packet::free(p);
				return;
			}
			link_entry_ = slot_[dst].entry;
			if (link_entry_ == 0) {
				if (node_->trace())
					node_->trace()->traceonly(p);
				Packet::free(p);
				return;
			} 
			if(droppacket(myaddr_+1, slot_[dst].next_hop+1) && dropenabled){ 
				return;
			}
			hdrc->next_hop_ = slot_[dst].next_hop;
			link_entry_->recv(p, (Handler *)0);
			return;
		}
	} else {
		// DISTRIBUTED ROUTING LOOKUP COULD GO HERE
		//cout<<latitude_threshold_<<endl;
		SatRouteObject::instance().compute_topology();
		adj_entry* pubadj_ = SatRouteObject::instance().getAdj();
		int size = 128;
		#define ADJ_ENTRY2(i, j) pubadj_[INDEX(i, j, size)].entry
		#define ADJ2(i, j) pubadj_[INDEX(i, j, size)].cost
		int nxhop;

		if(ADJ2(myaddr_+1, dst+1) !=SAT_ROUTE_INFINITY) {
			nxhop = dst; 
			//cout<<"reach destination: "<<dst<<endl;
		}
		else{
			//cout<<"never"<<endl;
			int coop_index = dct_coop_selection(dst);			//coop_index ranges from 0-65
			//calculate nxthop by distributed algorithm
			if(SatRouteObject::instance().get_dra() == 1)
				nxhop = dra_routes(myaddr_, coop_index, lasthop);		//nxthop ranges from 0-65
			else if (SatRouteObject::instance().get_dct() == 1)
				nxhop = dct_routes(myaddr_, coop_index, lasthop);
			//cout<<"myaddr_: "<<myaddr_<<" coop_index: "<<coop_index<<" nx_hop: "<<nxhop<<endl;
		}		
		//get link entry from nodehead_
		if(droppacket(myaddr_+1, nxhop+1) && dropenabled){ 
			//cout<<"The packet is droped from "<<myaddr_<<" to "<<nxhop<<endl;;
			return;
		}
		
		link_entry_ = (NsObject*)ADJ_ENTRY2(myaddr_+1, nxhop+1);	
		//dump(pubadj_);
		//set next hop & call recv
		hdrc->next_hop_  = nxhop;
		//cout<<"link_entry: "<<link_entry_<<endl;
                link_entry_->recv(p, (Handler *)0); 
		
		//printf("Error:  distributed routing not available\n");
		//exit(1);
	}
}

void SatRouteAgent::dump(adj_entry* pubadj_)
{
	int i, src, dst;
	int size = 128; 
	for (i = 0; i < (size * size); i++) { 
		if (pubadj_[i].cost != SAT_ROUTE_INFINITY) {
			src = i / size - 1;
			dst = i % size - 1;
			printf("Found a link from %d to %d with cost %f at %f\n", src, dst, pubadj_[i].cost, NOW);
		/*	if(src ==6){
				#define ADJ2(i, j) pubadj_[INDEX(i, j, size_)].cost
				#define ADJ3(i, j) pubadj_[INDEX(i, j, size)].cost
				cout<<"cost= "<<ADJ2(src,dst)<< " src= "<< src<<" dst= "<< dst <<endl;
				cout<<"cost= "<<ADJ3(src,dst)<< " src= "<< src<<" dst= "<< dst <<endl;
			}*/
		}
        }
}

void SatRouteAgent::recv (Packet * p, Handler *)
{
	hdr_ip *iph = hdr_ip::access(p);
	hdr_cmn *cmh = hdr_cmn::access(p);

	if (iph->saddr() == myaddr_ && cmh->num_forwards() == 0) {
	 	// Must be a packet I'm originating... add the IP header
		iph->ttl_ = IP_DEF_TTL;
	} else if (iph->saddr() == myaddr_) {
		// I received a packet that I sent.  Probably a routing loop.
		Packet::free(p);
		return;
	} else {
		// Packet I'm forwarding...
		// Check the TTL.  If it is zero, then discard.
		if(--iph->ttl_ == 0) {
			Packet::free(p);
			return;
		}
	}
	if ((iph->saddr() != myaddr_) && (iph->dport() == ROUTER_PORT)) {
		// DISTRIBUTED ROUTING PROTOCOL COULD GO HERE
		printf("Error:  distributed routing not available\n");
		exit(1);
	} else {
		forwardPacket(p);
	}
}


int SatRouteAgent::next_plane(int sp, int dp)
{
   if (dp == sp)
	return 0;		//stay in plane
   else if(dp > sp)
	return 1;
   else
	return -1;
}

int SatRouteAgent::next_num(int sn, int dn)
{
  if (dn == sn)
	return 0;	//same index
  else if ((dn > sn && dn - sn <= 5) || (sn > dn && sn - dn > 5))
	return 1;	// num ++
  else
	return -1;
}

//myaddr,dst, nxhop ranges from 0-65
int SatRouteAgent::dct_routes(int myaddr, int dst, int lasthop){
	adj_entry* pubadj_ = SatRouteObject::instance().getAdj();
	int size = 128;		
        #define ADJ_ENTRY2(i, j) pubadj_[INDEX(i, j, size)].entry
	#define ADJ2(i, j) pubadj_[INDEX(i, j, size)].cost
	int sp = myaddr/11, sn = myaddr % 11;
	int dp = dst/11, dn = dst % 11;
	int np = next_plane(sp, dp);
	int nn = next_num(sn, dn);
	int nplane = sp, nnum = sn, nnum3 = sn;
	bool sameindex = false;
	int nh1, nh2, nh3;
	int res;
        if(nn == 1) nnum = (sn + 1 > 10) ? 0 : sn + 1;
	else if (nn == -1) nnum = (sn - 1 < 0) ? 10 : sn -1;
	if(np == 1) nplane = (sp + 1 > 5) ? 0 : sp + 1;
	else if (np == -1) nplane = (sp - 1 > 0) ? 5 : sp - 1;
	if(np == 0){
	    if(nn == 1 || nn == -1){
		//cout<<"Find link with same plane from "<< myaddr <<" to "<<11 * sp + nnum<<endl;	
		return 11 * sp + nnum;
	     } 
	    else {cout<<"myaddr and dst is the same node "<<dst<<","<<dp<<","<<dn<<","<<myaddr<<","<<sp<<","<<sn<<" nn="<<nn<<endl; exit(1);}	
	}
        if(nn == 0) { nh1 = nplane * 11 + sn; }
	//if(myaddr == 0 && dst == 24) cout<<"ddddd: "<<dp<<" dn="<<dn<<","<<myaddr<<","<<sp<<" sn="<<sn<<"nn="<<nn<<"np="<<np<<"nplane="<<nplane<<" nh1="<<nh1<<endl;
        else {
		nh1 = nplane * 11 + sn; 
		nh2 = sp * 11 + nnum;
		if(nn ==1) nnum3 = (sn - 1 < 0) ? 10 : sn -1;
		else nnum3 = (sn + 1 > 10) ? 0 : sn + 1;
		nh3 = sp * 11 + nnum3;
	}
	bool find = false;
	//dump(pubadj_); 
        if(nn == 0){			
		if(ADJ2(myaddr+1, nh1+1)!=SAT_ROUTE_INFINITY){
			//cout<<"Find link with same index from "<< myaddr <<" to "<<nh1<<" cost= "<<ADJ2(myaddr+1, nh1+1)<<" NOW="<<NOW<<endl;	
			return nh1;
		} else {
			//cout<<"The interplane isl does not exists. from "<<myaddr<<" to "<<dst<<endl ; 
			return (myaddr + 1 > 10) ? 0 : myaddr+1;
			dump(pubadj_);exit(1);
		}
	}
	else {		
		double plr1 = SatRouteObject::instance().get_plr(myaddr+1, nh1+1);
		double plr2 = SatRouteObject::instance().get_plr(myaddr+1, nh2+1);
		double qdelay1 = SatRouteObject::instance().node_load(myaddr+1, nh1+1);
		double qdelay2 = SatRouteObject::instance().node_load(myaddr+1, nh2+1);
		double delay1 = (ADJ2(myaddr+1, nh1+1) + qdelay1)/(1-plr1);
		double delay2 = (ADJ2(myaddr+1, nh2+1) + qdelay2)/(1-plr2);		
		if(nh2 == lasthop && delay1 != SAT_ROUTE_INFINITY) {
			res = nh1;
			//cout<<"Exist loop. Find link from "<< myaddr <<" to "<<nh1<<" cost= "<<delay1<<" NOW="<<NOW<<endl;
		}
		else if (nh2 == lasthop && delay1 == SAT_ROUTE_INFINITY)	{
			res = nh3;
			//cout<<"Exist loop. Detour from "<< myaddr <<" to "<<nh3<<" NOW="<<NOW<<endl;
		} else {
			if(delay1 > delay2) {
			res = nh2; 
			//cout<<"Find link from "<< myaddr <<" to "<<nh2<<" cost= "<<delay2<<"<"<<delay1<<" of "<<nh1<<" NOW="<<NOW<<" last "<<lasthop<<endl;
			}
			else {
			res = nh1; 
			//cout<<"Find link from "<< myaddr <<" to "<<nh1<<" cost= "<<delay1<<"<"<<delay2<<" of "<<nh2<<" NOW="<<NOW<<" last "<<lasthop<<endl;
			}
		}
	}
	return res;
}

//myaddr,dst, nxhop ranges from 0-65
int SatRouteAgent::dra_routes(int myaddr, int dst, int lasthop){
	adj_entry* pubadj_ = SatRouteObject::instance().getAdj();
	int size = 128;		
        #define ADJ_ENTRY2(i, j) pubadj_[INDEX(i, j, size)].entry
	#define ADJ2(i, j) pubadj_[INDEX(i, j, size)].cost
	int sp = myaddr/11, sn = myaddr % 11;
	int dp = dst/11, dn = dst % 11;
	int np = next_plane(sp, dp);
	int nn = next_num(sn, dn);
	int nplane = sp, nnum = sn, nnum3 = sn;
	bool sameindex = false;
	int nh1, nh2, nh3;
	int res;
        if(nn == 1) nnum = (sn + 1 > 10) ? 0 : sn + 1;
	else if (nn == -1) nnum = (sn - 1 < 0) ? 10 : sn -1;
	if(np == 1) nplane = (sp + 1 > 5) ? 0 : sp + 1;
	else if (np == -1) nplane = (sp - 1 > 0) ? 5 : sp - 1;
	if(np == 0){
	    if(nn == 1 || nn == -1){
		//cout<<"Find link with same plane from "<< myaddr <<" to "<<11 * sp + nnum<<endl;	
		return 11 * sp + nnum;
	     } 
	    else {cout<<"myaddr and dst is the same node "<<dst<<","<<dp<<","<<dn<<","<<myaddr<<","<<sp<<","<<sn<<" nn="<<nn<<endl; exit(1);}	
	}
        if(nn == 0) { nh1 = nplane * 11 + sn; }
	//if(myaddr == 0 && dst == 24) cout<<"ddddd: "<<dp<<" dn="<<dn<<","<<myaddr<<","<<sp<<" sn="<<sn<<"nn="<<nn<<"np="<<np<<"nplane="<<nplane<<" nh1="<<nh1<<endl;
        else {
		nh1 = nplane * 11 + sn; 
		nh2 = sp * 11 + nnum;
		if(nn ==1) nnum3 = (sn - 1 < 0) ? 10 : sn -1;
		else nnum3 = (sn + 1 > 10) ? 0 : sn + 1;
		nh3 = sp * 11 + nnum3;
	}
	bool find = false;
	//dump(pubadj_); 
        if(nn == 0){			
		if(ADJ2(myaddr+1, nh1+1)!=SAT_ROUTE_INFINITY){
			//cout<<"Find link with same index from "<< myaddr <<" to "<<nh1<<" cost= "<<ADJ2(myaddr+1, nh1+1)<<" NOW="<<NOW<<endl;	
			return nh1;
		} else {
			//cout<<"The interplane isl does not exists. from "<<myaddr<<" to "<<dst<<endl ; 
			return (myaddr + 1 > 10) ? 0 : myaddr+1;
			dump(pubadj_);exit(1);
		}
	}
	else {		
		double delay1 = ADJ2(myaddr+1, nh1+1);
		double delay2 = ADJ2(myaddr+1, nh2+1);
		if(nh2 == lasthop && delay1 != SAT_ROUTE_INFINITY) {
			res = nh1;
			//cout<<"Exist loop. Find link from "<< myaddr <<" to "<<nh1<<" cost= "<<delay1<<" NOW="<<NOW<<endl;
		}
		else if (nh2 == lasthop && delay1 == SAT_ROUTE_INFINITY)	{
			res = nh3;
			//cout<<"Exist loop. Detour from "<< myaddr <<" to "<<nh3<<" NOW="<<NOW<<endl;
		} else {
			if(delay1 > delay2) {
			res = nh2; 
			//cout<<"Find link from "<< myaddr <<" to "<<nh2<<" cost= "<<delay2<<"<"<<delay1<<" of "<<nh1<<" NOW="<<NOW<<" last "<<lasthop<<endl;
			}
			else {
			res = nh1; 
			//cout<<"Find link from "<< myaddr <<" to "<<nh1<<" cost= "<<delay1<<"<"<<delay2<<" of "<<nh2<<" NOW="<<NOW<<" last "<<lasthop<<endl;
			}
		}
	}
	return res;
}

 

//###########################################################################

void SatRouteTimer::expire(Event*)
{
        a_->route_timer();
}

static class SatRouteObjectClass:public TclClass
{
  public:
        SatRouteObjectClass ():TclClass ("SatRouteObject") { }
        TclObject *create (int, const char *const *) {
                return (new SatRouteObject ());
        }
} class_satrouteobject;

SatRouteObject* SatRouteObject::instance_;

void SatRouteObject::profile_test(){
 /*
	vector<double> tv = coopprofile[68][35]; 
	for(int i = 0; i < tv.size(); i++){	
		cout<<"2:"<<tv[i]<<endl;
	} 
*/	 
	
	map<int, vector<double> > tm = coopprofile[68];
	for(int i = 1; i <=66; i++){
		if(tm.find(i) != tm.end()){
			vector<double>tv = tm[i]; 
			for(int j = 0; j < tv.size() - 1; j = j + 2){ 
				if(tv[j]<540 && tv[j+1]>540){
					cout<<"Find:"<<i<<" from "<<tv[j]<<" to "<<tv[j+1]<<endl;
					break;
				}
			}
		}
	}
/*
	for(int k = 1; k < 86390; k++){
		vector<int> node;
		vector<int> start;
		vector<int> end;
		for(int i = 1; i <=66; i++){
			if(tm.find(i) != tm.end()){
				vector<double>tv = tm[i]; 
				for(int j = 0; j < tv.size(); j = j + 2){ 
					if(tv[j] <= k && j + 1 <tv.size()){
					     if(tv[j+1] < k) continue;
					     else {
						node.push_back(i);
 						start.push_back(tv[j]); 
						end.push_back(tv[j+1]); 
						break;
						}					     
					}
				}
			}
		}
		if(node.size() > 1) {
			for(int kk = 0; kk < node.size(); kk++)
				cout<<"ES connect to node "<<node[kk]<<" at "<<k<<" from "<<start[kk]<<" to "<<end[kk]<<endl;
		}
		//if(node.size() == 1) cout <<"Find "<<node[0]<<" to "<<68<<endl;
	}
	*/
}

bool dct=true;

int SatRouteObject::get_dct(){
	return dct_enabled;
}


int SatRouteObject::get_dra(){
	return dra_enabled;
}

SatRouteObject::SatRouteObject() : suppress_initial_computation_(0),route_timer_(this)
{
	//cout<<"Construction.";
	bind_bool("wiredRouting_", &wiredRouting_);
	bind_bool("metric_delay_", &metric_delay_);
	bind_bool("data_driven_computation_", &data_driven_computation_);
	bind_bool("cct_enabled", &cct_enabled);
	bind_bool("tlr_enabled", &tlr_enabled);
	bind_bool("dct_enabled", &dct_enabled);
	bind_bool("dra_enabled", &dra_enabled);
	bind("islbw", &islbw);
	bind("frate", &frate);
	bind_bool("psize", &psize);
	//memset((double **)plr, 0, 128 * 128 * sizeof(plr[0][0]));
	for(int i = 0; i < 128; i++){
		for(int j = 0; j < 128; j++){
			 plr[i][j] = 0;		
		}		
	}
	route_timer_.sched(1);
	load_coopprofile();
	load_plr();
        //src = {6}; 		//src from 0-65
	src = {6,7};	//{"6","7","21","22","36","37","51","52","66","67"};
	//src = {6,7,17,18};	//{"6","7","21","22","36","37","51","52","66","67"};
	//src = {6,7,17,18,28,29};	//{"6","7","21","22","36","37","51","52","66","67"};
	//src = {6,7,17,18,28,29,39,40};	//{"6","7","21","22","36","37","51","52","66","67"};
	//src = {6,7,17,18,28,29,39,40,50,51};	//{"6","7","21","22","36","37","51","52","66","67"};
	//ratemap[6] = frate;
	ratemap[6] = frate;ratemap[7] = frate;ratemap[17] = frate;ratemap[18] = frate;ratemap[28] = frate;
	ratemap[29] = frate;ratemap[39] = frate;ratemap[40] = frate;ratemap[50] = frate;ratemap[51] = frate;
	plrthr = 0.1;
	//profile_test();
	if(cct_enabled == 1 && tlr_enabled == 1) { 
		cout<<"tlr and cct cannot be activated at the same time"<<endl; exit(1);
		cout<<"1";	
	}
	if(dct_enabled==1 && dra_enabled==1){cout<<"tlr and cct cannot be activated at the same time"<<endl; exit(1);}
	if (SatNode::dist_routing_ == 1 && dct_enabled == 1) {cout<<"dct routing model."<<endl;}
	else if(SatNode::dist_routing_ == 1 && dra_enabled == 1) {cout<<"dra routing model."<<endl;}
	else if (SatNode::dist_routing_ == 0 && cct_enabled == 1) {cout<<"cct routing model."<<endl;bminit();}
	else if (SatNode::dist_routing_ == 0 && tlr_enabled == 1) {cout<<"tlr routing model"<<endl;}
	else if (SatNode::dist_routing_ == 0 && tlr_enabled == 0 && cct_enabled == 0) {cout<<"baseline routing model"<<endl;}
	//cout<<"ROUTEOBJECT INIT: "<<islbw<<","<<frate<<","<<psize<<endl;
}

 

void SatRouteObject::bminit(){
	islbwm = new double*[66];
	for(int i = 0; i < 66; i++) {
		islbwm[i] = new double[66];
		memset(islbwm[i], 0, sizeof(double) * 66);
	}
	//intra-plane isl
	for(int i = 0; i < 56; i += 11){
		for(int j = i; j < i + 11; j++){
			if(j + 1 != i + 11) {
				//cout<<"(Intraplane) The bandwidth from "<< j <<" to "<<j+1<<" is "<<islbw<<endl;
				islbwm[j][j+1] = islbw;
				islbwm[j+1][j] = islbw;	
			}
			else {
				islbwm[j][j-10] = islbw;
				islbwm[j-10][j] = islbw;
				//cout<<"(Intraplane) The bandwidth from "<< j <<" to "<<j-10<<" is "<<islbw<<endl;
			}
		}
		
	}

	//inter-plane isl
	for(int i = 0; i < 11; i++){
		for(int j = i; j < 45 + i; j += 11){
			islbwm[j][j+11] = islbw;
			islbwm[j+11][j] = islbw;
			//cout<<"(Interplane) The bandwidth from "<< j <<" to "<<j+11<<" is "<<islbw<<endl;
		}
	}
	cout<<"bandwidth matric initialization completed"<<endl;
}

void SatRouteObject::load_plr(){
	ifstream in;
	in.open("plr.txt");
    	if(!in){
        	cout << "open file failed" << endl;
        	return;
    	}
	for(int i = 1; i <= 66; i++){
		for(int j = i + 1; j <= 66; j++){
			double x;
			in >> x;
			plr[i][j] = x;
			plr[j][i] = x;
			//cout<<"Plr from "<< i << " to "<< j <<" is " <<x<<endl;
		}
	}
}
//from and to ranges from 1-66
double SatRouteObject::get_plr(int from, int to){
	if(from < 0 || from > 127 || to < 0 || to > 127) {
		cout<<"Invalid index. from = "<<from<<" to= "<<to<<endl;
		exit(1);
	}
        return plr[from][to];
}

map<int, map<int, vector<double> > > SatRouteObject::get_coopprofile(){
	return coopprofile;
}
adj_entry* SatRouteObject::getAdj(){
	return adj_;
}
void SatRouteObject::load_coopprofile(){
	ifstream in;
	in.open("coop.txt");
    	if(!in){
        	cout << "open file failed" << endl;
        	return;
    	}
	int termnum = 0;
	in >> termnum;
	for(int i = 1; i <= termnum; i++){
		int term_index = 66 + i;
		for(int j = 1; j <= 66; j++){
			int sat_index = j;
			int numOftime = 0;
			in >> numOftime;
			if(numOftime == 0) continue;
			else{
				vector<double> tv;
				for(int k = 0; k < numOftime; k++){
					double time;
					in >> time;
					tv.push_back(time);
				}
				coopprofile[term_index][sat_index]=tv;
			}
		}

	}
}

void SatRouteObject::route_timer(){
	route_timer_.resched(1); 
	
//	if(node_load(7,8) != 0) cout<<"dasdad: "<<node_load(7,8)<<endl;
//	dump();
}

int SatRouteObject::command (int argc, const char *const *argv)
{
        if (instance_ == 0)
                instance_ = this;
	if (argc == 2) {
		// While suppress_initial_computation_ may seem more 
		// appropriate as a bound variable, it is useful to 
		// implement the setting of this variable this way so that 
		// the ``instance_ = this'' assignment is made at the
		// start of simulation.
		if (strcmp(argv[1], "suppress_initial_computation") == 0) {
			suppress_initial_computation_ = 1;
			return (TCL_OK);
		}
		if (strcmp(argv[1], "compute_routes") == 0) {
			recompute();
			return (TCL_OK);
		}
		if (strcmp(argv[1], "dump") == 0) {
			printf("Dumping\n");
			dump();
			return (TCL_OK);
		}
	}
	return (RouteLogic::command(argc, argv));
}                       

// Wrapper to catch whether OTcl-based (wired-satellite) routing is enabled
void SatRouteObject::insert_link(int src, int dst, double cost)
{
	if (wiredRouting_) {
		Tcl::instance().evalf("[Simulator instance] sat_link_up %d %d %f", (src - 1), (dst - 1), cost);
	} else
		insert(src, dst, cost);
}

// Wrapper to catch whether OTcl-based (wired) routing is enabled
void SatRouteObject::insert_link(int src, int dst, double cost, void* entry)
{
	SatLinkHead* slhp = (SatLinkHead*) entry;
	if (wiredRouting_) {
		// Here we do an upcall to an instproc in ns-sat.tcl
		// that populates the link_(:) array
		Tcl::instance().evalf("[Simulator instance] sat_link_up %d %d %f %s %s", (src - 1), (dst - 1), cost, slhp->name(), slhp->queue()->name());
	} else
		insert(src, dst, cost, entry); // base class insert()
}

void SatRouteObject::recompute_node(int node)
{
	compute_topology();
	node_compute_routes(node);
	populate_routing_tables(node);
}
void SatRouteObject::recompute()
{

	// For very large topologies (e.g., Teledesic), we don't want to
	// waste a lot of time computing routes at the beginning of the
	// simulation.  This first if() clause suppresses route computations.

	if (data_driven_computation_ || suppress_initial_computation_)
	//    (NOW < 0.001 && suppress_initial_computation_) ) 
		return;
	else if (NOW < 10) return;			//to ensure that the handoff is invoked
	//else {
	else if(SatNode::dist_routing_ == 0){
		compute_topology();
		if (wiredRouting_) {
			Tcl::instance().evalf("[Simulator instance] compute-flat-routes");
		} else { 
			if(cct_enabled == 0 && tlr_enabled == 0){
				compute_routes(); // base class function
			}
			else if (cct_enabled == 1)
				cct_routes();
			else if (tlr_enabled == 1)
				tlr_routes();
		}
		if(cct_enabled == 0 && tlr_enabled == 0)
			populate_routing_tables();
	}
}


//node1,node2 ranges from 1-66
double SatRouteObject::node_load(int node1, int node2) {
	double res = 0;
	Phy *phytxp, *phyrxp;
	Channel *channelp;
	SatNode *recvnode;
	SatNode *nodep = (SatNode*) Node::nodehead_.lh_first;
	SatLinkHead *slhp;
	int total_length = 0, actual_length = 0;
	bool find = false;
		for (; nodep; nodep = (SatNode*) nodep->nextnode()) {
			if(nodep->address() + 1 == node1){		//compute_route's parameter starts from 0
				for (slhp = (SatLinkHead*) nodep->linklisthead().lh_first; slhp;
						  slhp = (SatLinkHead*) slhp->nextlinkhead()) {
					if(slhp->type() == LINK_ISL_INTERPLANE
							|| slhp->type() == LINK_ISL_INTRAPLANE){
					phytxp = (Phy*) slhp->phy_tx();
					assert(phytxp);
					channelp = phytxp->channel();
					phyrxp = channelp->ifhead_.lh_first;
					recvnode = (SatNode*) phyrxp->node();
						if(recvnode->address() + 1 == node2){
						//	std::cout<<"find: "<<node1 <<" to: "<< recvnode->address() + 1 <<" node2: "<< node2 << std::endl;
							if(slhp->queue()->limit() == 0) return 1;
						//	res = (double)slhp->queue()->length()/(double)slhp->queue()->limit();
							res = (double)slhp->queue()->length()*psize*8/(islbw*1000); //KB*8/(Mb*1000)			
							find = true;
							break;
						}
					}

				}
			}
			if(find)
				break;
		}
	//if(res!=0) std::cout<<"find: "<<1 + res<<std::endl;
	return res;
}

// Derives link adjacency information from the nodes and gives the current
// topology information to the RouteLogic.
void SatRouteObject::compute_topology()
{
	//cout<<"compute_topology called"<<endl;
	Node *nodep;
	Phy *phytxp, *phyrxp, *phytxp2, *phyrxp2;
	SatLinkHead *slhp;
	Channel *channelp, *channelp2;
	int src, dst; 
	double delay;

	// wired-satellite integration
	if (wiredRouting_) {
		// There are two route objects being used
		// a SatRouteObject and a RouteLogic (for wired)
		// We need to also reset the RouteLogic one
		Tcl::instance().evalf("[[Simulator instance] get-routelogic] reset");
	}
	reset_all();
	// Compute adjacencies.  Traverse linked list of nodes 
        for (nodep=Node::nodehead_.lh_first; nodep; nodep = nodep->nextnode()) {
	    // Cycle through the linked list of linkheads
	    if (!SatNode::IsASatNode(nodep->address()))
	        continue;
	    for (slhp = (SatLinkHead*) nodep->linklisthead().lh_first; slhp; 
	      slhp = (SatLinkHead*) slhp->nextlinkhead()) {
		if (slhp->type() == LINK_GSL_REPEATER)
		    continue;
		if (!slhp->linkup_)
		    continue;
		phytxp = (Phy *) slhp->phy_tx();
		assert(phytxp);
		channelp = phytxp->channel();
		if (!channelp) 
	 	    continue; // Not currently connected to channel
		// Next, look for receive interfaces on this channel
		phyrxp = channelp->ifhead_.lh_first;
		for (; phyrxp; phyrxp = phyrxp->nextchnl()) {
		    if (phyrxp == phytxp) {
			printf("Configuration error:  a transmit interface \
			  is a channel target\n");
			exit(1);
		    } 
		    if (phyrxp->head()->type() == LINK_GSL_REPEATER) {
			double delay_firsthop = ((SatChannel*)
				    channelp)->get_pdelay(phytxp->node(), 
				    phyrxp->node());
			if (!((SatLinkHead*)phyrxp->head())->linkup_)
		    	    continue;
			phytxp2 = ((SatLinkHead*)phyrxp->head())->phy_tx();
			channelp2 = phytxp2->channel();
			if (!channelp2) 
	 	    	    continue; // Not currently connected to channel
			phyrxp2 = channelp2->ifhead_.lh_first;
			for (; phyrxp2; phyrxp2 = phyrxp2->nextchnl()) {
		    	    if (phyrxp2 == phytxp2) {
			        printf("Config error: a transmit interface \
			          is a channel target\n");
			        exit(1);
			    }
		            // Found an adjacency relationship.
		            // Add this link to the RouteLogic
		            src = phytxp->node()->address() + 1;
		            dst = phyrxp2->node()->address() + 1;
			    if (src == dst)
				continue;
			    if (metric_delay_)
		                delay = ((SatChannel*) 
			          channelp2)->get_pdelay(phytxp2->node(), 
			          phyrxp2->node());
			    else {
				delay = 1;
				delay_firsthop = 0;
			    }
			    //cout<<"Add link from "<<src <<" to "<<dst<<",cost="<<delay+delay_firsthop<<endl;
			    insert_link(src, dst, delay+delay_firsthop, (void*)slhp);
			}
		    } else {
		        // Found an adjacency relationship.
		        // Add this link to the RouteLogic
		        src = phytxp->node()->address() + 1;
		        dst = phyrxp->node()->address() + 1;
			if (metric_delay_)
		            delay = ((SatChannel*) 
		              channelp)->get_pdelay(phytxp->node(), 
			      phyrxp->node());
			else
			    delay = 1;
			//if(src == 68 || dst == 68) cout<<"Add link from "<<src <<" to "<<dst<<",cost="<<delay<<endl;
			insert_link(src, dst, delay, (void*)slhp);
		    }
		}
	    }
	}
	//dump();
}

void SatRouteObject::populate_routing_tables(int node)
{
	//dump();
	//exit(1);	
	SatNode *snodep = (SatNode*) Node::nodehead_.lh_first;
	SatNode *snodep2;
	int next_hop, src, dst;
	NsObject *target;

	if (wiredRouting_) {
		Tcl::instance().evalf("[Simulator instance] populate-flat-classifiers [Node set nn_]");
		return;
	}
        for (; snodep; snodep = (SatNode*) snodep->nextnode()) {
		if (!SatNode::IsASatNode(snodep->address()))
			continue;   
		// First, clear slots of the current routing table
		if (snodep->ragent())
			snodep->ragent()->clear_slots();
		src = snodep->address();
		if (node != -1 && node != src)
			continue;
		snodep2 = (SatNode*) Node::nodehead_.lh_first;
		for (; snodep2; snodep2 = (SatNode*) snodep2->nextnode()) {
                        if (!SatNode::IsASatNode(snodep->address()))
                                continue;
			dst = snodep2->address();
			if(src == 66 || dst ==66) continue;
 			next_hop = lookup(src, dst);
			if (next_hop != -1 && src != dst) {
				// Here need to insert target into slot table
				target = (NsObject*) lookup_entry(src, dst);
				//if(dst == 67) {
				//cout<<"target="<<target<<endl;
				//cout<<"The next hop from "<<src<<" to "<<dst<<" is "<<next_hop<<endl;
				//}
				if (target == 0) {
					printf("Error, routelogic target ");
					printf("not populated from %d to %d at %f\n", src, dst, NOW); 
					dump();
					exit(1);
				}
				((SatNode*)snodep)->ragent()->install(dst, 
				    next_hop, target); 
			}
		}
	}
		
}
//look up: add 1 because route table range from 1-66. 
//However, the slot_ ranges from 0-65.
int SatRouteObject::lookup(int s, int d)
{                                       
	int src = s + 1;        
	int dst = d + 1;
	if (src >= size_ || dst >= size_) {
		return (-1); // Next hop = -1
	}
	return (route_[INDEX(src, dst, size_)].next_hop - 1);
}

void* SatRouteObject::lookup_entry(int s, int d)
{                       
	int src = s + 1;
	int dst = d + 1;
	if (src >= size_ || dst >= size_) {
		return (0); // Null pointer
	}
	return (route_[INDEX(src, dst, size_)].entry);
}

// This method is used for debugging only
void SatRouteObject::dump()
{
	int i, src, dst;
	for (i = 0; i < (size_ * size_); i++) {
		if (adj_[i].cost != SAT_ROUTE_INFINITY) {
			src = i / size_ - 1;
			dst = i % size_ - 1;
			if(src == 67 || dst== 67)
			printf("Found a link from %d to %d with cost %f at %f.\n", src, dst, adj_[i].cost, NOW);
			//cout<<ADJ(7,18)<<endl;
		}
        }
}


void SatRouteObject::tlrpathcal(int sp, int sn, int dp, int dn, int np, int nn, double delay, double mdelay, vector<int> path, vector<int>&tpath) {
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
	int nx_plane, nx_num;	
	if (np == 1)  nx_plane = sp + 1;
	else if (np == -1) nx_plane = sp - 1;
	else nx_plane == sp;
	if (nn == 1) nx_num = sn + 1 > 10 ? 0 : sn + 1;
	else if (nn == -1) nx_num = sn - 1 < 0 ? 10 : sn - 1;
	else nx_num = sn;
	//cout<<"sp="<<sp<<",sn="<<sn<<",dp="<<dp<<",dn="<<dn<<endl;
	if(sp == dp && sn == dn){
		/*
		cout<<"Find a path with delay="<<delay<<endl;
		for(int i = 0; i < path.size(); i++){
			cout<<path[i]<<"->";		
		}
		cout<<endl;*/
		if(delay < mdelay){
			tpath=path;
			mdelay = delay;		
		}
		return;
	}	
	if (sp != dp){
		if(ADJ(sp*11+sn+1, nx_plane*11+sn+1) == SAT_ROUTE_INFINITY){ 
		//	cout<<"Link1 from "<<sp*11+sn<<" to "<<nx_plane*11+sn<<" doesn't exists."<<endl;
		//	dump();
		} else {
			path.push_back(nx_plane*11+sn); 
			delay += ADJ(sp*11+sn+1, nx_plane*11+sn+1);
		//	cout<<"Find link1 from "<<sp*11+sn<<" to "<<nx_plane*11+sn<<endl;
			tlrpathcal(nx_plane, sn, dp, dn, np, nn, delay, mdelay, path, tpath);
			delay -= ADJ(sp*11+sn+1, nx_plane*11+sn+1);
			path.pop_back();
		}
	}
	if (sn != dn){
		if(ADJ(sp*11+sn+1, sp*11+nx_num+1) == SAT_ROUTE_INFINITY) {
		//	cout<<"Link2 from "<<sp*11+sn<<" to "<<sp*11+nx_num<<" doesn't exists."<<endl;
			//dump();
		} else {
			path.push_back(sp*11+nx_num); 
			delay += ADJ(sp*11+sn+1, sp*11+nx_num+1);
		//	cout<<"Find link2 from "<<sp*11+sn<<" to "<<sp*11+nx_num<<endl;
			tlrpathcal(sp, nx_num, dp, dn, np, nn, delay, mdelay, path, tpath); 
			delay -= ADJ(sp*11+sn+1, sp*11+nx_num+1);
			path.pop_back();
		}
	}

}

void SatRouteObject::cctpathcal(int sp, int sn, int dp, int dn, int np, int nn, double pplr, double delay, vector<int>&path, vector<double>& pplrs, vector<double>& delays, vector<vector<int> >& paths){
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
	int nx_plane, nx_num;	
	if (np == 1)  nx_plane = sp + 1;
	else if (np == -1) nx_plane = sp - 1;
	else nx_plane == sp;
	if (nn == 1) nx_num = sn + 1 > 10 ? 0 : sn + 1;
	else if (nn == -1) nx_num = sn - 1 < 0 ? 10 : sn - 1;
	else nx_num = sn;
	if(sp == dp && sn == dn){
		
		//cout<<"Find a path with delay="<<delay<<" pplr= "<<1-pplr<<endl;
		//for(int i = 0; i < path.size(); i++){	cout<<path[i]<<"->";}
		//cout<<endl;
		delays.push_back(delay);
		pplrs.push_back(1-pplr);
		paths.push_back(path);
		return;
	}	
	if (sp != dp){
		if(ADJ(sp*11+sn+1, nx_plane*11+sn+1) == SAT_ROUTE_INFINITY){ 
			//cout<<"Link1 from "<<sp*11+sn<<" to "<<nx_plane*11+sn<<" doesn't exists."<<endl;
			//dump();
		} else {
			path.push_back(nx_plane*11+sn);
			pplr *= (1-get_plr(sp*11+sn+1, nx_plane*11+sn+1));
			delay += ADJ(sp*11+sn+1, nx_plane*11+sn+1);
			//cout<<"Find link1 from "<<sp*11+sn<<" to "<<nx_plane*11+sn<<endl;
			cctpathcal(nx_plane, sn, dp, dn, np, nn, pplr, delay, path, pplrs, delays, paths);
			pplr /= (1-get_plr(sp*11+sn+1, nx_plane*11+sn+1));
			delay -= ADJ(sp*11+sn+1, nx_plane*11+sn+1);
			path.pop_back();
		}
	}
	if (sn != dn){
		if(ADJ(sp*11+sn+1, sp*11+nx_num+1) == SAT_ROUTE_INFINITY) {
			//cout<<"Link2 from "<<sp*11+sn<<" to "<<sp*11+nx_num<<" doesn't exists."<<endl;
			//dump();
		} else {
			path.push_back(sp*11+nx_num);
			pplr *= (1-get_plr(sp*11+sn+1, sp*11+nx_num+1));
			delay += ADJ(sp*11+sn+1, sp*11+nx_num+1);
			//cout<<"Find link2 from "<<sp*11+sn<<" to "<<sp*11+nx_num<<endl;
			cctpathcal(sp, nx_num, dp, dn, np, nn, pplr, delay, path, pplrs, delays, paths);
			pplr /= (1-get_plr(sp*11+sn+1, sp*11+nx_num+1));
			delay -= ADJ(sp*11+sn+1, sp*11+nx_num+1);
			path.pop_back();
		}
	}
}


void SatRouteObject::tlr_routes(){
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
	for(int i = 0; i < src.size(); i++){
		int source = src[i];
		int dest = 67;		// the earth station in xi chang
		int coop_index = tlr_coop_selection(dest);  //coop_index ranges from 0-65;
		//compute the routes from source to coop_index
		int sp = source/11, sn = source % 11;
		int dp = coop_index/11, dn = coop_index % 11;
		int np = SatRouteAgent::instance().next_plane(sp, dp);
		int nn = SatRouteAgent::instance().next_num(sn, dn);
		//cout<<"coop= "<<coop_index<<",dp="<<dp<<",dn="<<dn<<"// source="<<source<<",sp="<<sp<<" sn="<<sn<<",np="<<np<<",nn="<<nn<<endl;
		//path calculation
		vector<double> delays;
		vector<int> path;
		vector<int> tpath;
		path.push_back(source); 
		tlrpathcal(sp, sn, dp, dn, np, nn, 0, 9999, path, tpath);
		if(tpath.size() == 0) {cout<<"path from "<<src[i]<<" to "<<coop_index<<"not found"<<endl; exit(1);}
		if(src[i] == 7) {cout<<"path from "<<src[i]<<" to "<<coop_index<<" at "<<NOW<<endl;}
		map<int, int> mpath;
		//cout<<"Find a path with minimal delay: "<<tpath.size()<<endl;	
		for(int i = 0; i < tpath.size() - 1; i++){
		//	cout<<tpath[i]<<"->";
			mpath[tpath[i]] = tpath[i+1];	
		}
		mpath[tpath[tpath.size()-1]] = dest;	
		//cout<<tpath[tpath.size()-1]<<endl;
		//populate route tables
		NsObject *target;
		SatNode *snodep = (SatNode*) Node::nodehead_.lh_first;
		for (; snodep; snodep = (SatNode*) snodep->nextnode()) {
			if(mpath.find(snodep->address()) != mpath.end()){
				if (snodep->ragent()) snodep->ragent()->clear_slots();
				target = (NsObject*)ADJ_ENTRY(snodep->address()+1, mpath[snodep->address()]+1);
				if (target == 0) {
					printf("Error, routelogic target ");
					printf("not populated %f %d->%d,%d\n", NOW,snodep->address(),dest,mpath[snodep->address()]);
					profile_test();
					dump();					
					exit(1);
				}
				((SatNode*)snodep)->ragent()->install(dest, mpath[snodep->address()], target);
				//cout<<"The next hop from "<<snodep->address()<<" to "<<dest<<" is "<<mpath[snodep->address()]<<endl;
			}			
		}
	}
}

void SatRouteObject::init_plinks(){
	for (int i = 0; i < src.size(); i++){
		if(plinks.find(src[i]) != plinks.end()){
			vector<double**> plkv = plinks[src[i]];
			for(int j = 0; j < plkv.size(); j++){
				double** plk = plkv[j];
				for(int k = 0; k < 66; k++)
					delete[] plk[k];
				delete[] plk;
				//cout<<"The "<<j<<"th path of src "<<src[i]<<" is deleted."<<endl;
			}
		}
	}
	cout<<"init_plinks finished()"<<endl;
}

void SatRouteObject::build_plinks(map<int, vector<vector<int> > > candidate_paths){
	for (int i = 0; i < src.size(); i++){
		vector<double**> srcpaths;
		if(candidate_paths.find(src[i]) != candidate_paths.end()){
			vector<vector<int> > tv = candidate_paths[src[i]];
			for(int j = 0; j < tv.size(); j++){
				vector<int> pv = tv[j];
				double** plk = new double*[66];
				for(int i = 0; i < 66; i++) {
					plk[i] = new double[66];
					memset(plk[i], 0, sizeof(double) * 66);
				}
				//cout<<"Find a path with index="<<j<<endl;
				for(int k = 0; k < pv.size() - 1; k++){
					plk[pv[k]][pv[k+1]] = 1;		//TODO: set as flow rate
				//	cout<<pv[k]<<" -> ";
				}
				//cout<<pv[pv.size() - 1]<<endl;
				srcpaths.push_back(plk);
			}
			//cout<<"The number of paths for src="<<src[i]<<" is "<<tv.size()<<endl;
		}
		plinks[src[i]] = srcpaths;
	}
	cout<<"build_plinks finished()"<<endl;
}

void SatRouteObject::cct_routes(){
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
	int dest = 67;		// the earth station in xi chang
	map<int, int> coopmap;	//first:source index, second: coop index
	map<int, vector<vector<int> > > candidate_paths;
	map<int, vector<double> > candidate_pathdelays;
	map<int, vector<double> > candidate_pathplrs;
	map<int, vector<int> > final_paths;
	//init plinkmatric
	init_plinks();
	//calculate the cooperation node for all sources first
	coopmap = cct_coop_selection(src, dest);           //coop_index ranges from 0-65;
	//cout<<"calculate the candidate paths for all sources"<<endl;
	for(int i = 0; i < src.size(); i++){
		int source = src[i];
		if(coopmap.find(source) == coopmap.end()){cout<<"coop not found for source="<<source<<endl; exit(1);}
		int coop_index = coopmap[source];
		int sp = source/11, sn = source % 11;
		int dp = coop_index/11, dn = coop_index % 11;
		int np = SatRouteAgent::instance().next_plane(sp, dp);
		int nn = SatRouteAgent::instance().next_num(sn, dn);
		//cout<<"coop= "<<coop_index<<",dp="<<dp<<",dn="<<dn<<"// source="<<source<<",sp="<<sp<<" sn="<<sn<<",np="<<np<<",nn="<<nn<<endl;
		//candidate path calculation
		vector<vector<int> > paths;
		vector<double> delays;
		vector<double> pplrs;
		vector<int> cand_path;
		vector<int> f_path;
		cand_path.push_back(source);
		cctpathcal(sp, sn, dp, dn, np, nn, 1, 0, cand_path, pplrs, delays, paths);
		if(paths.size() == 0) {cout<<"path from "<<src[i]<<" to "<<coop_index<<"not found"<<endl; exit(1);}
		candidate_paths[source] = paths;
		candidate_pathplrs[source] = pplrs;
		candidate_pathdelays[source] = delays;
	}
	//cout<<"randomized rounding based path calculation for all sources"<<endl;
	build_plinks(candidate_paths);
	final_paths = rr_selection(candidate_pathplrs, candidate_pathdelays, candidate_paths);
	//cout<<"populate route tables"<<endl;
	for(int j = 0; j < src.size(); j++){
		if (final_paths.find(src[j]) == final_paths.end()) {cout<<"paths not found for source="<<src[j]<<endl; exit(1);}
		vector<int> f_path = final_paths[src[j]]; 
		map<int, int> mpath;
		//cout<<"Find a path with minimal delay: "<<endl;	
		for(int i = 0; i < f_path.size() - 1; i++){
		//	cout<<f_path[i]<<"->";
			mpath[f_path[i]] = f_path[i+1];	
		}
		mpath[f_path[f_path.size()-1]] = dest;	
		//cout<<f_path[f_path.size()-1]<<endl;
		NsObject *target;
		SatNode *snodep = (SatNode*) Node::nodehead_.lh_first;
		for (; snodep; snodep = (SatNode*) snodep->nextnode()) {
			if(mpath.find(snodep->address()) != mpath.end()){
				if (snodep->ragent()) snodep->ragent()->clear_slots();
				target = (NsObject*)ADJ_ENTRY(snodep->address()+1, mpath[snodep->address()]+1);
				if (target == 0) {
					printf("Error, routelogic target ");
					printf("not populated %f %d->%d,%d\n", NOW,snodep->address(),dest,mpath[snodep->address()]);
					dump();					
					exit(1);
				}
				((SatNode*)snodep)->ragent()->install(dest, mpath[snodep->address()], target);
				//cout<<"The next hop from "<<snodep->address()<<" to "<<dest<<" is "<<mpath[snodep->address()]<<endl;
			}			
		}
	}
}

int SatRouteObject::tlr_coop_selection(int dst){
	map<int, map<int, vector<double> > > coopprofile = SatRouteObject::instance().get_coopprofile();
	int t_dst = dst + 1;
	if(coopprofile.find(t_dst) == coopprofile.end()) {cout<<"coop to "<<t_dst<<" does not exists at "<<NOW<<endl;}
	map<int, vector<double> > tm = coopprofile[t_dst];
	double cur = NOW;
	int res=0;
	double maxcost = 999999999;      //record variable
	for(int i = 1; i <=66; i++){
		if(tm.find(i) != tm.end()){
			vector<double>tv = tm[i]; 
                        for(int j = 0; j < tv.size(); j = j + 2){ 
				if(int(tv[j]) <= int(cur) && j + 1 <tv.size()){
				     if(int(tv[j+1]) <= int(cur)) continue;
					//cout<<"find "<<i<<" for dst= "<<t_dst<<" dur = "<<tv[j+1]-cur<<endl;
				     else{
					//cout <<"Find "<<i-1<<" to "<<t_dst-1<<" with cost= "<<ADJ(i, t_dst)<<" from "<<tv[j]<<" to "<<tv[j+1]<<" at "<<cur<<endl;
					if(ADJ(i, t_dst) < maxcost){
						maxcost = ADJ(i, t_dst);
						res = i;
					}
					break;
				     }
				}
				if(int(tv[j]) <= int(cur) && j == tv.size() - 1){
					if(ADJ(i, t_dst) < maxcost){
						maxcost = ADJ(i, t_dst);
						res = i;
					}
					break;
				}
			}
			
		}
        }
	if(res == 0){cout<<"coop to "<<t_dst<<" does not find at "<<NOW<<endl; exit(1);}
	//cout<<"find final: "<<res<<endl;
	return res-1;
}

map<int, int> SatRouteObject::cct_coop_selection(vector<int> src, int dest){
	map<int, map<int, vector<double> > > coopprofile = SatRouteObject::instance().get_coopprofile();
	int t_dst = dest + 1;
	if(coopprofile.find(t_dst) == coopprofile.end()) {cout<<"coop to "<<t_dst<<" does not exists at "<<NOW<<endl;}
	double cur = NOW;
	map<int, int> mres;
	map<int, vector<double> > tm = coopprofile[t_dst];
	vector<int> avaicoop;
	for(int i = 1; i <=66; i++){
		if(tm.find(i) != tm.end()){
			vector<double>tv = tm[i]; 
		               for(int j = 0; j < tv.size(); j = j + 2){
				  if(int(tv[j]) <= int(cur) && j + 1 <tv.size()){
				     if(int(tv[j+1]) <= int(cur)) continue;
					//cout<<"find "<<i<<" for dst= "<<t_dst<<" dur = "<<tv[j+1]-cur<<endl;
				     else{avaicoop.push_back(i); break;}
				   }
				  if(int(tv[j]) <= int(cur) && j == tv.size() - 1){
				     avaicoop.push_back(i);
				     break;
				  }
				}
				
		}
	}
	if(avaicoop.size() == 0){cout<<"coop to "<<t_dst<<" does not find at "<<NOW<<endl; exit(1);}
	if(avaicoop.size() > 1){cout<<"coop to "<<t_dst-1<<" 1: "<<avaicoop[0]-1<<",2:"<<avaicoop[1]-1<<" at "<<NOW<<endl;}
	int cindex = 0;		 
	for(int i = 0; i < src.size(); i++){
		mres[src[i]] = avaicoop[cindex] - 1;	//coop ranges from 0 to 65
		if(cindex + 1 == avaicoop.size()) cindex = 0;
		else cindex++;
	}
	return mres;
}

/*
map<int, vector<int> > SatRouteObject::r_bake_selection(map<int, vector<double> > candidate_pathplrs, map<int, vector<double> > candidate_pathdelays, 
map<int, vector<vector<int> > > candidate_paths)
{ 
	map<int, vector<int> > mres;
	for(int j = 0; j < src.size(); j++){
		vector<int> res;
		vector<vector<int> > cpaths;
		int index;
		double minplr = 9999;
		double mdelay = 9999;
		if(candidate_paths.find(src[j]) == candidate_paths.end()){cout<<"paths doesn't exists source = "<<src[j]<<endl;;exit(1);}
		if(candidate_pathplrs.find(src[j]) == candidate_pathplrs.end()){cout<<"plrs doesn't exists source = "<<src[j]<<endl;;exit(1);}
		if(candidate_pathdelays.find(src[j]) == candidate_pathdelays.end()){cout<<"delays doesn't exists source = "<<src[j]<<endl;;exit(1);}
		vector<vector<int> > paths = candidate_paths[src[j]];
		vector<double> pplrs = candidate_pathplrs[src[j]];
		vector<double> delays = candidate_pathdelays[src[j]];
		int index_delay = 0;
		//cout<<"1"<<endl;
		for(int i = 0; i < paths.size(); i++){
			if(pplrs[i] < minplr){
				index = i;
				minplr = pplrs[i];
			}
			if(pplrs[i] < plrthr){
				cpaths.push_back(paths[i]);
			}
		}
		if(minplr > plrthr) {  // if the path satisfy plr does not exists.
			//cout<<" Select the path with minimal plr="<<minplr<<endl;
			mres[src[j]] = paths[index];
		} else { //select a path from cpaths	 	
			for(int i = 0; i < cpaths.size(); i++){
				if(delays[i] < mdelay){
					index_delay = i;
					mdelay = delays[i];
				}
			} 
			res = cpaths[index_delay];
			mres[src[j]] = res;	
		}
			
	}
	return mres;
}
*/

map<int, vector<int> > SatRouteObject::rr_selection(map<int, vector<double> > candidate_pathplrs, map<int, vector<double> > candidate_pathdelays, 
map<int, vector<vector<int> > > candidate_paths)
{ 
	map<int, vector<int> > mres;
	map<int, vector<double> > roundres;
	map<int, int> srcmap;	//key:ncols, value: src
	map<int, int> indexmap;	//key:ncols, value: index
	map<int, int> frommap;	//key:nrows, value: from index
	map<int, int> tomap;	//key:nrows, value: to index
	map<int, int> row2src;  //key:nrows, value: src
	//calculate the dimensions of the matric
	int nrows = 0, ncols = 0;
	for(int i = 0; i < src.size(); i++){
		nrows++;
		row2src[nrows] = src[i]; 
	}
	for(int i = 0; i < src.size(); i++){
		if(candidate_paths.find(src[i]) != candidate_paths.end()){
			for(int j = 0; j < candidate_paths[src[i]].size(); j++){
				ncols++;
				srcmap[ncols] = src[i];
				indexmap[ncols] = j;
			}	
			//cout<<"The number of paths to "<<src[i]<<" is "<<candidate_paths[src[i]].size()<<endl;			
		}
	}
	for(int i = 0; i < 66; i++)
		for(int j = 0; j < 66; j++)
			if(islbwm[i][j] != 0) {
				nrows++;
				frommap[nrows] = i;
				tomap[nrows] = j;
			}
	cout<<"nrows= "<<nrows<<",ncols="<<ncols<<",frate="<<frate<<",islbw="<<islbw<<endl;
	//initialize
	glp_prob *lp;
    	lp = glp_create_prob();
    	glp_set_obj_dir(lp, GLP_MIN);
	//auxiliary_variables_rows
	glp_add_rows(lp, nrows);
	for(int i = 1; i <= nrows; i++){
		if(i <= src.size()) glp_set_row_bnds(lp, i, GLP_FX, 1.0, 1.0);	        //sum = 1
		else glp_set_row_bnds(lp, i, GLP_DB, 0.0, islbw);			//bandwidth constraint
	} 
	//variables_columns
	glp_add_cols(lp, ncols);
	for(int i = 1; i <= ncols; i++)
		glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);		// x ranges from 0 to 1
	//objective function
	for(int i = 1; i <= ncols; i++){
		int sid = srcmap[i];
		int pid = indexmap[i];
		if(candidate_pathdelays.find(sid) == candidate_pathdelays.end()) {cout<<"path delay for src="<<sid<<" does not exists"<<endl; exit(1);}
		glp_set_obj_coef(lp, i, candidate_pathdelays[sid][pid]);
		//cout<<"The "<<pid<<"th path from src="<<sid<<"'s delay is "<<candidate_pathdelays[sid][pid]<<endl;
	}
	//constraint matrix
	int* ia = new int[1+nrows*ncols];
	int* ja = new int[1+nrows*ncols];
	double* ar = new double[1+nrows*ncols];
	int row = 1, col = 1;
	double value = 0;
	for(int i = 1; i <= nrows*ncols; i++){
		if(col == ncols + 1) { col = 1; row++;}
		if(row <= src.size()){
			if(row2src[row] == srcmap[col]) {
				value = 1;
				//cout<<"row="<<row<<",col="<<col<<" is set to "<<value<<endl;
			}
			else { 
				value = 0;
				//cout<<"row="<<row<<",col="<<col<<" is set to "<<value<<endl;			
			}
		}
		else {
			int from = frommap[row];
			int to = tomap[row];
			int sid = srcmap[col];
			int pid = indexmap[col];
			if(plinks.find(sid) == plinks.end()){cout<<"plinks src="<<sid<<" does not exists"<<endl; exit(1);}
			if(plinks[sid][pid][from][to] == 0) value = 0;
			else value = plinks[sid][pid][from][to];
			/*			
			if(value!=0) {
			cout<<"The "<<pid<<"-th path of src="<<sid<<" has a link from "<<from<<" to "<<to<<endl;
			cout<<"row="<<row<<",col="<<col<<",value="<<value<<endl;
			}*/
		}	
		ia[i] = row, ja[i] = col, ar[i] = value;		
		col++;
	}
	glp_load_matrix(lp, nrows*ncols, ia, ja, ar);
	//calculate
	glp_simplex(lp, NULL);
	//output
	double z;
	int sid;
    	z = glp_get_obj_val(lp); 
	for(int i = 1; i <= ncols; i++){
		double x = glp_get_col_prim(lp, i);
		sid = srcmap[i];
		if(roundres.find(sid) == roundres.end()){
			vector<double> sv;
			sv.push_back(x);
			roundres[sid] = sv;
		} else {
			roundres[sid].push_back(x);
		}
	}
	for(int i = 0; i < src.size(); i++){
		for(int pid = 0; pid < roundres[src[i]].size(); pid++){
			cout<<"The "<<pid<<"-th path to "<<src[i]<<" round results: "<<roundres[src[i]][pid]<<endl;
		}
	}
	//round
    	for(int i = 0; i < src.size(); i++){
		bool find =false;
		int pid = 0;
		while(!find){
			if(roundres.find(src[i]) == roundres.end()) {cout<<"round results of src="<<src[i]<<" does not exists"<<endl; exit(1);}
			if(pid == roundres[sid].size()) pid = 0;
			srand((int)time(0));
			assert(pid < candidate_paths[src[i]].size());
			//cout<<"1"<<endl;
			if((double)rand()/RAND_MAX < roundres[sid][pid]){
				find = true; 
				mres[src[i]] = candidate_paths[src[i]][pid];
				cout<<"Select the "<<pid<<"-th path for src="<<src[i]<<endl;
			}
			pid++;
		}
	} 
	//cleanup
	delete ia, ja, ar;
	glp_delete_prob(lp);
	glp_free_env();
	return mres;
}

void SatRouteObject::compute_routes()
{
	//cout<<"I am invoked at "<<NOW<<endl;  
	int n = size_;  	//size_ is 128
	int* parent = new int[n];
	double* hopcnt = new double[n];
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
#define ROUTE(i, j) route_[INDEX(i, j, size_)].next_hop
#define ROUTE_ENTRY(i, j) route_[INDEX(i, j, size_)].entry

 	 
//	cout<<"ADJ_ENTRY1:"<<ADJ_ENTRY(7, 18)<<endl;
	adj_entry* pubadj_ = SatRouteObject::instance().getAdj();
	#define ADJ_ENTRY2(i, j) pubadj_[INDEX(i, j, size_)].entry
//	cout<<"ADJ_ENTRY2:"<< ADJ_ENTRY2(7, 18)<<" NOW "<<NOW<<endl;

	 

	delete[] route_;
	route_ = new route_entry[n * n];
	memset((char *)route_, 0, n * n * sizeof(route_[0]));
	/* do for all the sources */
	int k;
	for (k = 1; k < n; ++k) {
		int v;
		for (v = 0; v < n; v++)
			parent[v] = v;

		/* set the route for all neighbours first */
		for (v = 1; v < n; ++v) {
			if (parent[v] != k) {
				//std::cout<<"naoguila:"<<k<<" 11 "<<v<<ADJ(1, 106)<<std::endl;
				hopcnt[v] = ADJ(k, v);
				//hopcnt[v] = SatRouteObject::hybridcost_[k][v];
				if (hopcnt[v] != INFINITY) {
					ROUTE(k, v) = v;
					ROUTE_ENTRY(k, v) = ADJ_ENTRY(k, v);
				}
			}
		}
		for (v = 1; v < n; ++v) {
			/*
			 * w is the node that is the nearest to the subtree
			 * that has been routed
			 */
			int o = 0;
			/* XXX */
			hopcnt[0] = INFINITY;
			int w;
			for (w = 1; w < n; w++)
				if (parent[w] != k && hopcnt[w] < hopcnt[o])
					o = w;
			parent[o] = k;
			/*
			 * update distance counts for the nodes that are
			 * adjacent to o
			 */
			if (o == 0)
				continue;
			for (w = 1; w < n; w++) {
				if (parent[w] != k &&
				    hopcnt[o] + ADJ(o, w) < hopcnt[w]) {
					ROUTE(k, w) = ROUTE(k, o);
					ROUTE_ENTRY(k, w) =
					    ROUTE_ENTRY(k, o);
					hopcnt[w] = hopcnt[o] + ADJ(o, w);
				}
			}
		}
	}
	/*
	 * The route to yourself is yourself.
	 */
	for (k = 1; k < n; ++k) {
		ROUTE(k, k) = k;
		ROUTE_ENTRY(k, k) = 0; // This should not matter
	}

	delete[] hopcnt;
	delete[] parent;
}
//node ranges from 0-65; adj & route table 1-66
void SatRouteObject::node_compute_routes(int node)
{
	//test();
        int n = size_;
        int* parent = new int[n];
        double* hopcnt = new double[n];
#define ADJ(i, j) adj_[INDEX(i, j, size_)].cost
#define ADJ_ENTRY(i, j) adj_[INDEX(i, j, size_)].entry
#define ROUTE(i, j) route_[INDEX(i, j, size_)].next_hop
#define ROUTE_ENTRY(i, j) route_[INDEX(i, j, size_)].entry
        delete[] route_;
        route_ = new route_entry[n * n];
        memset((char *)route_, 0, n * n * sizeof(route_[0]));
        /* compute routes only for node "node" */
        int k = node + 1; // must add one to get the right offset in tables  
        int v;
        for (v = 0; v < n; v++) 
                parent[v] = v;

        /* set the route for all neighbours first */
        for (v = 1; v < n; ++v) {
                if (parent[v] != k) {
                        hopcnt[v] = ADJ(k, v);
                        if (hopcnt[v] != SAT_ROUTE_INFINITY) {
                                ROUTE(k, v) = v;
                                ROUTE_ENTRY(k, v) = ADJ_ENTRY(k, v);
                        }
                }
        }
        for (v = 1; v < n; ++v) {
                /*
                 * w is the node that is the nearest to the subtree
                 * that has been routed
                 */
                int o = 0;
                /* XXX */
                hopcnt[0] = SAT_ROUTE_INFINITY;
                int w;
                for (w = 1; w < n; w++)
                        if (parent[w] != k && hopcnt[w] < hopcnt[o])
                                o = w;
                parent[o] = k;
                /*
                 * update distance counts for the nodes that are
                 * adjacent to o
                 */
                if (o == 0)
                        continue;
                for (w = 1; w < n; w++) {
                        if (parent[w] != k &&
                            hopcnt[o] + ADJ(o, w) < hopcnt[w]) {
                                ROUTE(k, w) = ROUTE(k, o);
                                ROUTE_ENTRY(k, w) =
                                    ROUTE_ENTRY(k, o);
                                hopcnt[w] = hopcnt[o] + ADJ(o, w);
                        }
                }
        }
        /*
         * The route to yourself is yourself.
         */
        ROUTE(k, k) = k;
        ROUTE_ENTRY(k, k) = 0; // This should not matter

        delete[] hopcnt;
        delete[] parent;
}
