#ifndef MYPACKET_H
#define MYPACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/enum.h"
#include "ns3/ipv4-address.h"
#include <map>
#include "ns3/nstime.h"
#include <vector> 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"


#define BUNDLEHEADERSIZE 21			// size bundle header
#define BUNDLEFRAGMENTHEADERSIZE 2		// size fragment header

#define PAYLOADSIZE 50000				// size bundle payload
#define PAYLOADACKSIZE 1				// size ack payload
#define PAYLOADREQUESTSIZE 1			// size bundle request payload
#define PAYLOADBACKGROUNDSIZE 1000		// size background packet

#define WIFIHEADERSIZE 24				// size header IEEE 802.11
#define LLCHEADERSIZE 8					// size header LLC
#define NETWORKHEADERSIZE 20			// size header IP
#define TCPHEADERSIZE 32				// size header TCP
#define UDPHEADERSIZE 8					// size header UDP

using namespace std;

namespace ns3 {
  namespace mypacket {

//-----------------------------------------------------------------------------
      // BUNDLE HEADER
//-----------------------------------------------------------------------------
    class BndlHeader : public Header
    {
    public:
      BndlHeader (uint8_t type = 0, Ipv4Address dst = Ipv4Address (), Ipv4Address origin = Ipv4Address (), uint32_t originSeqNo = 0, uint32_t payloadSize = 0, Time srcTimestamp = MilliSeconds (0));

      static TypeId GetTypeId ();
      TypeId GetInstanceTypeId () const;
      uint32_t GetSerializedSize () const;
      void Serialize (Buffer::Iterator start) const;
      uint32_t Deserialize (Buffer::Iterator start);
      void Print (ostream &os) const;

	  //Set - Get Proprieties
      void SetBundleType (uint8_t a) { m_bundleType = a; }
      uint8_t GetBundleType () const { return m_bundleType; }
      void SetDst (Ipv4Address a) { m_dst = a; }
      Ipv4Address GetDst () const { return m_dst; }
      void SetOrigin (Ipv4Address a) { m_origin = a; }
      Ipv4Address GetOrigin () const { return m_origin; }
      void SetOriginSeqno (uint32_t s) { m_originSeqNo = s; }
      uint32_t GetOriginSeqno () const { return m_originSeqNo; }
      void SetPayloadSize (uint32_t s) { m_payloadSize = s; }
      uint32_t GetPayloadSize () const { return m_payloadSize; }
      void SetSrcTimestamp (Time s);
      Time GetSrcTimestamp () const;
	  //const?
	  void SetPathVector (vector<BndlPath> path) { path = m_path; }
	  vector<BndlPath> GetPathVector () const { return m_path; }

      bool operator== (BndlHeader const & o) const;
    private:
      uint8_t        m_bundleType;
      Ipv4Address    m_dst;
      Ipv4Address    m_origin;
      uint32_t       m_originSeqNo;
      uint32_t       m_payloadSize;
      uint32_t       m_srcTimestamp;
	  //Maybe also a field with the size of the vector is necessary
	  vector<BndlPath>	m_path; //This will be filled by the SCGR algorithm and read by the intermediary nodes
    };

    ostream & operator<< (ostream & os, BndlHeader const &);

    //-----------------------------------------------------------------------------
          // BUNDLE FRAGMENT HEADER
    //-----------------------------------------------------------------------------
class BndlFragmentHeader : public Header
        {
        public:
    		static TypeId GetTypeId ();
    	    TypeId GetInstanceTypeId () const;
    	    uint32_t GetSerializedSize () const;
    	    void Serialize (Buffer::Iterator start) const;
    	    uint32_t Deserialize (Buffer::Iterator start);
    	    void Print (ostream &os) const;

    		BndlFragmentHeader (uint8_t currentFragmentNo = 0, uint8_t totalFragmentsNo = 0);
    		void SetCurrentFragmentNo (uint8_t currentFragmentNo) {	m_currentFragmentNo = currentFragmentNo; }
    		void SetTotalFragmentsNo (uint8_t totalFragmentsNo) {	m_totalFragmentsNo = totalFragmentsNo; }
    		uint8_t GetCurrentFragmentNo () const {	return m_currentFragmentNo; }
    		uint8_t GetTotalFragmentsNo () const {	return m_totalFragmentsNo; }

        private:
    		uint8_t m_currentFragmentNo;
    		uint8_t m_totalFragmentsNo;
        };

//-----------------------------------------------------------------------------
// BNDL PATH
//-----------------------------------------------------------------------------

	//A vector of these will be integrated into the boundle header
	class BndlPath
		{
			public:
				BndlPath (uint32_t contactTime, Ipv4Address nextNode);
				void Print (ostream &os) const;
				void SetContactTime(uint32_t contactTime){m_contactTime = contactTime;}
				uint32_t GetContactTime(){return m_contactTime;}

				void SetNextNode(Ipv4Address nextNode){m_nextNode = nextNode;}
				Ipv4Address GetNextNode(){return m_nextNode;}

			private:
				uint32_t m_contactTime;
				Ipv4Address m_nextNode;
		};
  }
}
#endif /* MYPACKET_H */
