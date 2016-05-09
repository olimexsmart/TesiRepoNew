#include "mypacket.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"

using namespace std;

namespace ns3
{
namespace mypacket
{

//-----------------------------------------------------------------------------
// BNDL
//-----------------------------------------------------------------------------
BndlHeader::BndlHeader (uint8_t type, Ipv4Address dst, Ipv4Address origin, uint32_t originSeqNo, uint32_t payloadSize, Time srcTimestamp) :
								m_bundleType(type), m_dst(dst), m_origin(origin), m_originSeqNo (originSeqNo), m_payloadSize (payloadSize)
{
	m_srcTimestamp = uint32_t (srcTimestamp.GetMilliSeconds ());
}

NS_OBJECT_ENSURE_REGISTERED (BndlHeader);

TypeId BndlHeader::GetTypeId ()
{
	static TypeId tid = TypeId ("ns3::mypacket::BndlHeader").SetParent<Header> ().AddConstructor<BndlHeader> ();
	return tid;
}

TypeId BndlHeader::GetInstanceTypeId () const
{
	return GetTypeId ();
}

uint32_t BndlHeader::GetSerializedSize () const
{
	return BUNDLEHEADERSIZE;
}

void BndlHeader::Serialize (Buffer::Iterator i) const
{
	i.WriteU8((uint8_t)m_bundleType);
	WriteTo (i, m_dst);
	WriteTo (i, m_origin);
	i.WriteHtonU32 (m_originSeqNo);
	i.WriteHtonU32 (m_payloadSize);
	i.WriteHtonU32 (m_srcTimestamp);
}

uint32_t BndlHeader::Deserialize (Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	m_bundleType = i.ReadU8();
	ReadFrom (i, m_dst);
	ReadFrom (i, m_origin);
	m_originSeqNo = i.ReadNtohU32 ();
	m_payloadSize = i.ReadNtohU32 ();
	m_srcTimestamp = i.ReadNtohU32 ();

	uint32_t dist = i.GetDistanceFrom (start);
	NS_ASSERT (dist == GetSerializedSize ());
	return dist;
}

void BndlHeader::Print (ostream &os) const
{
	os << "bundle type " << m_bundleType
			<< "Destination: ipv4 " << m_dst
			<< " source: ipv4 " << m_origin
			<< " sequence number " << m_originSeqNo
			<< " payload size " << m_payloadSize
			<< " source timestamp " << m_srcTimestamp;
}

void BndlHeader::SetSrcTimestamp (Time t)
{
	m_srcTimestamp = t.GetMilliSeconds ();
}

Time BndlHeader::GetSrcTimestamp () const
{
	Time t (MilliSeconds (m_srcTimestamp));
	return t;
}

ostream & operator<< (ostream & os, BndlHeader const & h)
{
	h.Print (os);
	return os;
}

bool BndlHeader::operator== (BndlHeader const & o) const
				{
	return (m_bundleType == o.m_bundleType &&
			m_dst == o.m_dst &&
			m_origin == o.m_origin &&
			m_originSeqNo == o.m_originSeqNo &&
			m_payloadSize == o.m_payloadSize &&
			m_srcTimestamp == o.m_srcTimestamp);
				}

//-----------------------------------------------------------------------------
// BNDL FRAGMENT
//-----------------------------------------------------------------------------

BndlFragmentHeader::BndlFragmentHeader (uint8_t currentFragmentNo, uint8_t totalFragmentsNo) :
									m_currentFragmentNo(currentFragmentNo), m_totalFragmentsNo(totalFragmentsNo)
{
}

NS_OBJECT_ENSURE_REGISTERED (BndlFragmentHeader);

TypeId BndlFragmentHeader::GetTypeId ()
{
	static TypeId tid = TypeId ("ns3::mypacket::BndlFragmentHeader")
    							.SetParent<Header> ()
    							.AddConstructor<BndlFragmentHeader> ()
    							;
	return tid;
}

TypeId BndlFragmentHeader::GetInstanceTypeId () const
{
	return GetTypeId ();
}

uint32_t BndlFragmentHeader::GetSerializedSize () const
{
	return BUNDLEFRAGMENTHEADERSIZE;
}

void BndlFragmentHeader::Serialize (Buffer::Iterator i) const
{
	i.WriteU8((uint8_t)m_currentFragmentNo);
	i.WriteU8((uint8_t)m_totalFragmentsNo);
}

uint32_t BndlFragmentHeader::Deserialize (Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	m_currentFragmentNo = i.ReadU8();
	m_totalFragmentsNo = i.ReadU8();
	uint32_t dist = i.GetDistanceFrom (start);
	NS_ASSERT (dist == GetSerializedSize ());
	return dist;
}

void BndlFragmentHeader::Print (ostream &os) const
{
	os << "Current Fragment number: " << m_currentFragmentNo<< " Total Fragments number: " << m_totalFragmentsNo;
}


//-----------------------------------------------------------------------------
// BNDL PATH
//-----------------------------------------------------------------------------
BndlPath::BndlPath(uint32_t contactTime, Ipv4Address nextNode) : m_contactTime(contactTime), m_nextNode(nextNode)
{

}

void BndlPath::Print (ostream &os) const
{
	os << "Next hop address:  " << m_nextNode
			<< "\t at time: " << m_contactTime;
}

ostream & operator<< (ostream & os, BndlPath const & h)
{
	h.Print (os);
	return os;
}

}
}
