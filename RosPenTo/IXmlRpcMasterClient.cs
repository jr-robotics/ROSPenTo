/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using CookComputing.XmlRpc;

namespace RosPenTo
{
    [XmlRpcUrl("")]
    public interface IXmlRpcMasterClient : IXmlRpcProxy
    {
        [XmlRpcMethod("getTopicTypes")]
        /// <summary>
        /// Retrieve a list of topic names and their types.
        /// </summary>
        /// <returns>
        /// (code, statusMessage, topicTypes)
        /// (int, str, [ [str,str] ])
        /// topicTypes is a list of [topicName, topicType] pairs
        /// </returns>
        object[] GetTopicTypes(string caller_id);

        [XmlRpcMethod("getSystemState")]
        /// <summary>
        /// Retrieve list representation of system state
        /// </summary>
        /// <returns>
        /// (code, statusMessage, systemState)
        /// (int, str, [ [str,[str] ], [str,[str] ], [str,[str] ] ])
        /// </returns>
        object[] GetSystemState(string caller_id);

        [XmlRpcMethod("lookupNode")]
        /// <summary>
        /// Get the XML-RPC URI of the node with the associated name/caller_id.
        /// This API is for looking information about publishers and subscribers.
        /// </summary>
        /// <returns>
        /// (code, statusMessage, URI)
        /// (int, str, str)
        /// </returns>
        object[] LookupNode(string caller_id, string node_name);

        [XmlRpcMethod("lookupService")]
        /// <summary>
        /// Lookup all provider of a particular service.
        /// </summary>
        /// <returns>
        /// (code, statusMessage, serviceUrl)
        /// (int, str, str)
        /// </returns>
        object[] LookupService(string caller_id, string service);

        [XmlRpcMethod("unregisterService")]
        /// <summary>
        /// Unregister the caller as a provider of the specified service.
        /// </summary>
        /// <param name="caller_id">ROS caller ID</param>
        /// <param name="service">Fully qualified name of service</param>
        /// <param name="service_api">
        /// API URI of service to unregister. 
        /// Unregistration will only occur if current registration matches.
        /// </param>
        /// <returns>
        /// (code, statusMessage, numUnregistered)
        /// Number of unregistrations (either 0 or 1). 
        /// If this is zero it means that the caller was not registered as a service provider. 
        /// The call still succeeds as the intended final state is reached.
        /// </returns>
        object[] UnregisterService(string caller_id, string service, string service_api);
    }

}
