/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CookComputing.XmlRpc;

namespace RosPenTo
{
    public interface IXmlRpcSlave
    {
        [XmlRpcMethod("publisherUpdate")]
        /// <summary>
        /// Updates the current list of publishers for a specified topic.
        /// </summary>
        /// <returns>
        /// (code, statusMessage, ignore)
        /// (int, str, int)
        /// </returns>
        object[] PublisherUpdate(string caller_id, string topic, string[] publishers);

        [XmlRpcMethod("paramUpdate")]
        /// <summery>
        /// 
        /// </summery>
        /// <param name="caller_id">
        /// ROS caller ID.
        /// </param>
        /// <param name="parameter_key">
        /// Parameter name, globally resolved.
        /// </param>
        /// <param name="parameter_value">
        /// New parameter value.
        /// </param>
        /// <returns>
        /// (code, statusMessage, ignore)
        /// (int, str, int)
        /// </returns>
        object[] ParamUpdate(string caller_id, string parameter_key, object parameter_value);

        [XmlRpcMethod("requestTopic")]
        /// <summary>
        /// This method is called by a subscriber node and requests that source allocate a channel for communication.
        /// </summary>
        /// <param name="protocols">
        /// List of desired protocols for communication in order of preference.
        /// Each protocol is a list of the form:
        /// [ProtocolName, ProtocolParam1, ProtocolParam2...N]
        /// </param>
        /// <returns>
        /// (code, statusMessage, protocolParams)
        /// (int, str, [str, !XMLRPCLegalValue*] )
        /// </returns>
        object[] RequestTopic(string caller_id, string topic, object[] protocols);

        [XmlRpcMethod("getName")]
        /// <summary>
        /// (Python-Only API) Get the XML-RPC URI of this server 
        /// </summary>
        /// <param name="caller_id">
        /// ROS caller id 
        /// </param>
        /// <returns>
        /// (code, statusMessage, ROS node name)
        /// (int, str, str)
        /// </returns>
        /// 
        object[] GetName(string caller_id);

        [XmlRpcMethod("shutdown")]
        /// <summary>
        /// Stop this server.
        /// </summary>
        /// <param name="message">
        /// A message describing why the node is being shutdown.
        /// </param>
        /// <returns>
        /// (code, statusMessage, ignore)
        /// (int, str, int)
        /// </returns>
        object[] Shutdown(string caller_id, string message);

        [XmlRpcMethod("echo")]
        string Echo(string input);
    }
}
