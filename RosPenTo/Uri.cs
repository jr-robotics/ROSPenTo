/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Exceptions.Uri;
using System.Net;
using System.Net.Sockets;

namespace RosPenTo
{
    public class Uri
    {
        static string protocolPattern = @"(http|https)";
        static string ipAdressPattern = @"[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}";
        static string hostnamePattern = @"[a-zA-Z0-9]+";
        static string portPattern = @"[0-9]{4,5}";
        static string uriPattern = @"^" + protocolPattern + @"\:\/\/" + @"(" + ipAdressPattern + @"|" + hostnamePattern + @")\:" + portPattern + @"[\/]{0,1}$";

        public string protocol { get; set; }
        public string ipAdress { get; set; }
        public int port { get; set; }

        public Uri(string uri)
        {
            parseUri(uri);
        }

        private void parseUri(string uri)
        {
            Match result;

            result = Regex.Match(uri, uriPattern);
            if (!result.Success)
            {
                throw new UriDoesNotMatchPatternException("Uri does not match pattern!");
            }

            string[] splittedUri = uri.Split(':');

            //1. set protocol
            protocol = splittedUri[0];

            //2. set ip
            string hostnameOrIp = splittedUri[1].Substring(2);
            result = Regex.Match(hostnameOrIp, @"^" + hostnamePattern + @"$");
            if (result.Success)
            {
                string[] addresses = getIpfromHostname(hostnameOrIp);
                if (addresses.Length != 1)
                {
                    // TODO: how to handle this?
                    // Ethernet-Adapter LAN-Verbindung & Ethernet-Adapter VirtualBox Host-Only Network
                    // both result an ipAdress to hostname
                    Console.WriteLine("Check for second ip!");
                }
                ipAdress = addresses[0];
            }
            else
            {
                ipAdress = hostnameOrIp;
            }

            //3. set port
            port = Int32.Parse(splittedUri[2].Replace("/", ""));
            
        }

        private string[] getIpfromHostname(string hostname)
        {
            List<string> result = new List<string>();
            IPHostEntry hostEntry = Dns.GetHostEntry(hostname);

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
            return result.ToArray();
        }

        public new string ToString
        {
            get
            {
                return protocol + @"://" + ipAdress + @":" + port + @"/";
            }
        }
    }
}
