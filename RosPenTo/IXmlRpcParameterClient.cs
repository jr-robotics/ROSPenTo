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

    public interface IXmlRpcParameterClient : IXmlRpcProxy
    {
        [XmlRpcMethod("getParamNames")]

        ///<summary>
        /// Get list of all parameter names stored on the parameter server.
        /// </summary>
        /// <param name="caller_id">
        /// ROS caller ID
        /// </param>
        /// <returns>
        /// (code, statusMessage, parameterNameList)
        /// (int, str, [str])
        /// </returns>
        object[] GetParamNames(string caller_id);


        [XmlRpcMethod("getParam")]
        /// <summary>
        /// Retrieve a parameter with the given key from the parameter server
        /// </summary>
        /// <param name="caller_id">
        /// ROS caller ID
        /// </param>
        /// <param name="key">
        /// Name of the parameter
        /// </param>
        /// <returns>
        /// (code, statusMessage, value)
        /// (int, string, XmlRpcValue)
        /// </returns>
        object[] GetParam(string caller_id, string key);


        [XmlRpcMethod("unsubscribeParam")]
        ///<summary>
        ///
        /// </summary>
        /// <param name="caller_id">
        /// ROS caller ID
        /// </param>
        /// <param name="caller_api">
        /// Node API URI of subscriber
        /// </param>
        /// <param name="key">
        /// 
        /// </param>
        /// <returns>
        /// 
        /// </returns>
        object[] UnsubscribeParam(string caller_id, string caller_api, string key);

        [XmlRpcMethod("subscribeParam")]
        /// <summary> 
        /// Retrieve parameter value from server and subscribe to updates to that param. See paramUpdate() in the Node API. 
        /// </summary>
        /// <param name="caller_id">
        /// ROS caller ID
        /// </param>
        /// <param name="caller_api">
        /// Node API URI of subscriber for paramUpdate callbacks 
        /// </param>
        /// <param name="key">
        /// Parameter name
        /// </param>
        object[] SubscribeParam(string caller_id, string caller_api, string key);

    }

}