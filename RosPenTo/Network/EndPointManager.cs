/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text.RegularExpressions;
using System.Collections.Generic;

namespace RosPenTo.Network
{
    public class EndPointManager
    {
        public static string ReplaceHostnameByIp(string uri)
        {
            string ipAdressPattern = @"^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$";

            //e.g. uri = "http://127.0.0.1:11311/"
            string[] splittedUri = uri.Split(':');
            string hostnameOrIp = splittedUri[1].Substring(2);

            Match result = Regex.Match(hostnameOrIp, ipAdressPattern);
            if (result.Success) // uri contains ip
                return uri;

            string ip = GetIpFromHostname(hostnameOrIp);

            return splittedUri[0] + "://" + ip + ":" + splittedUri[2];
        }

        public static Boolean IsValidMasterUri(string uri)
        {
            string ipAdressPattern = @"[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}";
            string hostNamePattern = @"[a-zA-Z0-9]*";
            string portNumberPattern = @"[1-9]+[0-9]*";

            string uriPattern = @"^http:\/\/(" + ipAdressPattern + "|" + hostNamePattern + "):" + portNumberPattern + @"\/?$";

            Match result = Regex.Match(uri, uriPattern);

            return result.Success;
        }

        public static string GetIpFromHostname(string hostname)
        {
            List<string> result = new List<string>();
            IPHostEntry hostEntry = Dns.GetHostEntry(hostname);

            // collect corresponding IPv4 addresses
            if (hostEntry.AddressList.Length > 0)
            {
                foreach (IPAddress ip in hostEntry.AddressList)
                {
                    if (ip.AddressFamily == AddressFamily.InterNetwork)
                    {
                        result.Add(ip.ToString());
                    }
                }
            }

            if (result.Count == 0)
            {
                //TODO what to do if no ip or more than one ip is found?
                // Console.WriteLine("{0} Adresses found for hostname {1}",result.Count,  hostname);
                return null;
            }

            return result[0];
        }
    }
}
