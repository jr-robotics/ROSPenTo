/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System;
using System.IO;
using System.Net;
using RosPenTo.Network;

using CookComputing.XmlRpc;

namespace RosPenTo
{

    public class XmlRpcSlaveService : ListenerService, IXmlRpcSlave
    {
        public string Echo(string input)
        {
            Console.WriteLine("Echo: {0}", input);
            return input;
        }

        public object[] PublisherUpdate(string caller_id, string topic, string[] publishers)
        {
            Console.WriteLine("PublisherUpdate received");
            return null;
        }

        public object[] ParamUpdate(string caller_id, string parameter_key, object parameter_value)
        {
            Console.WriteLine("ParamUpdate received");
            return null;
        }

        public object[] RequestTopic(string caller_id, string topic, object[] protocols)
        {
            Console.WriteLine("XmlRpcSlaveService: RequestTopic received from " + caller_id);
            return null;
        }

        public object[] GetName(string caller_id)
        {
            Console.WriteLine("XmlRpcSlaveService: GetName received");
            return null;
        }

        public object[] Shutdown(string caller_id, string message)
        {
            Console.WriteLine("Shutdown received");
            return null;
        }
    }

    //http://www.cookcomputing.com/blog/archives/000572.html

    public abstract class ListenerService : XmlRpcHttpServerProtocol
    {
        public virtual void ProcessRequest(HttpListenerContext RequestContext)
        {
            try
            {
                IHttpRequest req = new ListenerRequest(RequestContext.Request);
                IHttpResponse resp = new ListenerResponse(RequestContext.Response);
                HandleHttpRequest(req, resp);
                RequestContext.Response.OutputStream.Close();
            }
            catch (Exception ex)
            {
                // "Internal server error"
                RequestContext.Response.StatusCode = 500;
                RequestContext.Response.StatusDescription = ex.Message;
            }
        }
    }

    public class ListenerRequest : CookComputing.XmlRpc.IHttpRequest
    {
        private HttpListenerRequest request;

        public ListenerRequest(HttpListenerRequest request)
        {
            this.request = request;
        }

        public Stream InputStream
        {
            get { return request.InputStream; }
        }

        public string HttpMethod
        {
            get { return request.HttpMethod; }
        }
    }

    public class ListenerResponse : CookComputing.XmlRpc.IHttpResponse
    {
        private HttpListenerResponse response;

        public ListenerResponse(HttpListenerResponse response)
        {
            this.response = response;
        }

        string IHttpResponse.ContentType
        {
            get { return response.ContentType; }
            set { response.ContentType = value; }
        }

        TextWriter IHttpResponse.Output
        {
            get { return new StreamWriter(response.OutputStream); }
        }

        Stream IHttpResponse.OutputStream
        {
            get { return response.OutputStream; }
        }

        int IHttpResponse.StatusCode
        {
            get { return response.StatusCode; }
            set { response.StatusCode = value; }
        }

        string IHttpResponse.StatusDescription
        {
            get { return response.StatusDescription; }
            set { response.StatusDescription = value; }
        }

        //public long ContentLength { set => response.ContentLength64 = value; }
        //public bool SendChunked { get => response.SendChunked; set => response.SendChunked = value; }

        public long ContentLength { set { response.ContentLength64 = value; } }

        public bool SendChunked
        {
            get
            {
                return response.SendChunked;
            }
            set
            {
                response.SendChunked = value;
            }
        }
    }
}
