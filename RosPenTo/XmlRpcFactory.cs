/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using CookComputing.XmlRpc;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosPenTo
{
    public class XmlRpcFactory
    {
        public static IXmlRpcMasterClient GetMasterClient(Uri masterUri)
        {
            IXmlRpcMasterClient master = XmlRpcProxyGen.Create<IXmlRpcMasterClient>();
            master.Url = masterUri.ToString();
            return master;
        }

        public static IXmlRpcSlaveClient GetSlaveClient(Uri slaveUri)
        {
            IXmlRpcSlaveClient slave = XmlRpcProxyGen.Create<IXmlRpcSlaveClient>();
            slave.Url = slaveUri.ToString();
            return slave;
        }

        public static IXmlRpcParameterClient GetParameterClient(Uri masterUri)
        {
            IXmlRpcParameterClient parameterServer = XmlRpcProxyGen.Create<IXmlRpcParameterClient>();
            parameterServer.Url = masterUri.ToString();
            return parameterServer;
        }
    }
}
