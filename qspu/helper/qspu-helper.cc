/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>, written after OlsrHelper by Mathieu Lacage
 * <mathieu.lacage@sophia.inria.fr>
 */
#include "qspu-helper.h"

#include "ns3/qspu-routing-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/names.h"
#include "ns3/node-list.h"
#include "ns3/ptr.h"

namespace ns3
{

QspuHelper::QspuHelper()
    : Ipv4RoutingHelper()
{
    m_agentFactory.SetTypeId("ns3::qspu::RoutingProtocol");
}

QspuHelper*
QspuHelper::Copy() const
{
    return new QspuHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
QspuHelper::Create(Ptr<Node> node) const
{
    Ptr<qspu::RoutingProtocol> agent = m_agentFactory.Create<qspu::RoutingProtocol>();
    node->AggregateObject(agent);
    return agent;
}

void
QspuHelper::Set(std::string name, const AttributeValue& value)
{
    m_agentFactory.Set(name, value);
}

int64_t
QspuHelper::AssignStreams(NodeContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    Ptr<Node> node;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        node = (*i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        NS_ASSERT_MSG(ipv4, "Ipv4 not installed on node");
        Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol();
        NS_ASSERT_MSG(proto, "Ipv4 routing not installed on node");
        Ptr<qspu::RoutingProtocol> qspu = DynamicCast<qspu::RoutingProtocol>(proto);
        if (qspu)
        {
            currentStream += qspu->AssignStreams(currentStream);
            continue;
        }
        // Qspu may also be in a list
        Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting>(proto);
        if (list)
        {
            int16_t priority;
            Ptr<Ipv4RoutingProtocol> listProto;
            Ptr<qspu::RoutingProtocol> listQspu;
            for (uint32_t i = 0; i < list->GetNRoutingProtocols(); i++)
            {
                listProto = list->GetRoutingProtocol(i, priority);
                listQspu = DynamicCast<qspu::RoutingProtocol>(listProto);
                if (listQspu)
                {
                    currentStream += listQspu->AssignStreams(currentStream);
                    break;
                }
            }
        }
    }
    return (currentStream - stream);
}

} // namespace ns3
