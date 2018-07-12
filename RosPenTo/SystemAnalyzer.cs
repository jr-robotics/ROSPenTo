/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System.Collections.Generic;
using CookComputing.XmlRpc;
using System.Linq;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text.RegularExpressions;
using RosPenTo.Network;

namespace RosPenTo
{

    public class SystemAnalyzer
    {
        IXmlRpcMasterClient _master;
        IXmlRpcParameterClient _parameterServer;

        readonly List<Node> _nodes = new List<Node>();
        readonly List<Topic> _topics = new List<Topic>();
        readonly List<Service> _services = new List<Service>();
        readonly List<Parameter> _parameters = new List<Parameter>();
        readonly List<Communication> _communications = new List<Communication>();

        public List<Node> Nodes
        {
            get
            {
                return _nodes;
            }
        }

        public List<Topic> Topics
        {
            get
            {
                return _topics;
            }
        }

        public List<Service> Services
        {
            get
            {
                return _services;
            }
        }

        public List<Parameter> Parameters
        {
            get
            {
                return _parameters;
            }
        }

        public List<Communication> Communications
        {
            get
            {
                return _communications;
            }
        }

        public string URL
        {
            get
            {
                return _master.Url;
            }
        }

        public SystemAnalyzer(IXmlRpcMasterClient master, IXmlRpcParameterClient parameterServer)
        {
            _master = master;
            _parameterServer = parameterServer;
        }

        public void Update()
        {
            _nodes.Clear();
            _topics.Clear();
            _services.Clear();
            _parameters.Clear();
            _communications.Clear();

            Dictionary<String, Topic> foundTopics = AnalyzeTopicTypes();
            AnalyzeNodes(foundTopics);
            AnalyzeParameterNames();
            AnalyzeParameterTypesAndValues();
            SetXmlRpcUriForAllNodes();
        }

        private Dictionary<String, Topic> AnalyzeTopicTypes()
        {
            // topicTypes is a list of [topicName, topicType] pairs
            var topicTypes = (object[])GetTopicTypes()[2];

            Dictionary<String, Topic> topics = new Dictionary<String, Topic>();
            foreach (var topicTypesElement in topicTypes)
            {
                string topicName = (string)((object[])topicTypesElement)[0];
                string topicType = (string)((object[])topicTypesElement)[1];

                Topic topic = new Topic(topicName);
                topic.Type = topicType;

                topics.Add(topicName, topic);
            }

            return topics;
        }

        private void AnalyzeNodes(Dictionary<String, Topic> foundTopics)
        {
            var systemState = (object[])GetSystemState()[2];
            if (systemState == null || !systemState.Any())
                return;
            var publishersArray = (object[])systemState[0];
            var subscribersArray = (object[])systemState[1];
            var servicesArray = (object[])systemState[2];


            // +++ CREATE PUBLISHERS +++
            if (publishersArray.Any())
                ExtractNodes(publishersArray, foundTopics);

            // +++ CREATE SUBSCRIBERS +++
            if (subscribersArray.Any())
                ExtractNodes(subscribersArray, foundTopics);

            // +++ Services +++
            if (servicesArray.Any())
                ExtractServices(servicesArray);

            // +++ LINK PUBLISHERS OF NODES +++
            if (publishersArray.Any())
                LinkPublishersOfNodes(publishersArray, foundTopics);

            // +++ LINK SUBSCRIBERS OF NODES +++
            if (subscribersArray.Any())
                LinkSubscribersOfNodes(subscribersArray, foundTopics);

            // +++ GENERATE TOPICS LIST +++
            ISet<Topic> allTopics = new HashSet<Topic>();
            foreach (var node in _nodes)
            {
                allTopics.UnionWith(node.TopicPublishers.Keys);
                allTopics.UnionWith(node.TopicSubscribers.Keys);
            }
            _topics.AddRange(allTopics.ToList());

            // +++ GENERATE SERVICES LIST +++
            //TODO is it possible that 2 different nodes provide the same service?
            foreach (var node in _nodes)
                _services.AddRange(node.Services);

            // +++ GENERATE COMMUNICATIONS LIST +++
            foreach (Topic topic in Topics)
            {
                Communication comm = new Communication(topic);
                foreach (Node n in Nodes)
                {
                    // add publishers
                    if (n.TopicPublishers.ContainsKey(topic))
                        comm.Publishers.UnionWith(n.TopicPublishers[topic]);
                    // add subscribers
                    if (n.TopicSubscribers.ContainsKey(topic))
                        comm.Subscribers.UnionWith(n.TopicSubscribers[topic]);
                }
                _communications.Add(comm);
            }

        }

        public void AnalyzeParameterNames()
        {
            object[] getParameterNamesResponse = null;
            try
            {
                getParameterNamesResponse = GetParamNames("");
            }
            catch(XmlRpcFaultException e)
            {
                Console.WriteLine(e.Message);
                return;
            }

            int getParameterNamesStatusCode = (int)getParameterNamesResponse[0];
            string getParameterNamesStatusMessage = (string)getParameterNamesResponse[1];

            if (getParameterNamesStatusCode == -1) // ERROR
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + getParameterNamesStatusMessage);
                return;
            }
            else if (getParameterNamesStatusCode == 0) // FAILURE
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + getParameterNamesStatusMessage);
                return;
            }
            else if (getParameterNamesStatusCode == 1) //SUCCESS
            {
                var parameterNames = (string[])getParameterNamesResponse[2];
                if(parameterNames.Any())
                {
                    ExtractParameterNames(parameterNames);
                }
                //Console.WriteLine("ROS parameters stored");
            }
            else
            {
                Console.WriteLine("Invalid status code received");
                return;
            }
        }

        public void AnalyzeParameterTypesAndValues()
        {
            foreach (Parameter parameter in _parameters)
            {
                object[] getParamResponse = null;
                try
                {
                    getParamResponse = GetParam("", parameter.Name);
                }
                catch (XmlRpcFaultException e)
                {
                    Console.WriteLine(e.Message);
                    return;
                }

                int getParamStatusCode = (int)getParamResponse[0];
                string getParamStatusMessage = (string)getParamResponse[1];

                if(getParamStatusCode == -1)
                {
                    Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                    Console.WriteLine("StatusMessage: " + getParamStatusMessage);
                    continue;
                }
                else if(getParamStatusCode == 0)
                {
                    Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                    Console.WriteLine("StatusMessage: " + getParamStatusMessage);
                    continue;
                }
                else if(getParamStatusCode == 1)
                {
                    object value = getParamResponse[2];
                    //Console.WriteLine("{0}, Type:{1}", parameter.Name, value.GetType());
                    parameter.Type = value.GetType();
                    parameter.Value = value;
                }
                else
                {
                    Console.WriteLine("Invalid status code received");
                    continue;
                }
                //Console.WriteLine("Parameter values and types analyzed");
            }
        }
        private void ExtractNodes(object[] sourceArray, Dictionary<String, Topic> topics)
        {
            var sourceLines = sourceArray.Where(l => (l as object[]).Any())
                            .Select(ProcessLine)
                            .ToList();

            foreach (var sl in sourceLines)
            {
                foreach (string nodeName in sl.Item2)
                {
                    Node node = GetOrCreateNode(nodeName);
                    Topic topic;
                    if (!topics.TryGetValue(sl.Item1, out topic))
                        topic = new Topic(sl.Item1);

                    if (!node.TopicPublishers.ContainsKey(topic))
                        node.TopicPublishers.Add(topic, new List<Node>());

                    if (!node.TopicSubscribers.ContainsKey(topic))
                        node.TopicSubscribers.Add(topic, new List<Node>());
                }
            }
        }

        private void ExtractServices(object[] servicesArray)
        {
            var serviceLines = servicesArray.Where(l => (l as object[]).Any())
                              .Select(ProcessLine)
                              .ToList();

            foreach (var sl in serviceLines)
            {
                foreach (string serviceName in sl.Item2)
                {
                    Node node = GetOrCreateNode(serviceName);
                    node.Services.Add(new Service(sl.Item1));
                }
            }
        }

        private void ExtractParameterNames(string[] parameterNameArray)
        {
            foreach(string parameterName in parameterNameArray)
            {
                bool found = false;
                foreach (Parameter param in _parameters)
                {
                    if (param.Name.Equals(parameterName))
                    {
                        found = true;
                        break;
                    }
                }
                if(!found)
                    _parameters.Add(new Parameter(parameterName));
            }
        }

        private Node GetOrCreateNode(string name)
        {
            Node node = _nodes.Find(n => n.Name.Equals(name));
            if (node == default(Node))
            {
                node = new Node(name);
                _nodes.Add(node);
            }

            return node;
        }

        private void LinkPublishersOfNodes(object[] publishersArray, Dictionary<String, Topic> topics)
        {
            var publisherLines = publishersArray.Where(l => (l as object[]).Any())
                            .Select(ProcessLine)
                            .ToList();

            foreach (var pl in publisherLines)
            {
                Topic topic = topics[pl.Item1];
                var publishers = _nodes.Where(n => pl.Item2.Contains(n.Name));
                _nodes.Where(n => n.TopicPublishers.ContainsKey(topic)).ToList().ForEach(n => n.TopicPublishers[topic].AddRange(publishers));
            }
        }

        private void LinkSubscribersOfNodes(object[] subscribersArray, Dictionary<String, Topic> topics)
        {
            var subscriberLines = subscribersArray.Where(l => (l as object[]).Any())
                            .Select(ProcessLine)
                            .ToList();

            foreach (var sl in subscriberLines)
            {
                Topic topic = topics[sl.Item1];
                var subscribers = _nodes.Where(n => sl.Item2.Contains(n.Name));
                _nodes.Where(n => n.TopicSubscribers.ContainsKey(topic)).ToList().ForEach(n => n.TopicSubscribers[topic].AddRange(subscribers));
            }
        }

        public List<Node> GetPublishersOfTopic(Topic topic)
        {
            List<Node> publishers = new List<Node>();

            foreach (Node node in _nodes)
            {
                if (node.TopicPublishers.ContainsKey(topic))
                    publishers.AddRange(node.TopicPublishers[topic]);
            }

            return publishers;
        }

        private void SetXmlRpcUriForAllNodes()
        {
            foreach (var node in _nodes)
            {
                string uri = (string)LookupNode(node.Name)[2];
                if (!uri.Equals(""))
                    node.XmlRpcUri = new Uri(EndPointManager.ReplaceHostnameByIp(uri));
            }
        }

        private object[] GetSystemState()
        {
            return _master.GetSystemState("");
        }

        private object[] GetTopicTypes()
        {
            return _master.GetTopicTypes("");
        }

        private object[] LookupNode(string nodeName)
        {
            return _master.LookupNode("", nodeName);
        }

        public object[] LookupService(string caller_id, string service)
        {
            return _master.LookupService(caller_id, service);
        }
        
        public object[] UnregisterService(string caller_id, string service, string service_api)
        {
            return _master.UnregisterService(caller_id, service, service_api);
        }

        public object[] GetParamNames(string caller_id)
        {
            return _parameterServer.GetParamNames(caller_id);
        }

        public object[] GetParam(string caller_id, string key)
        {
            return _parameterServer.GetParam(caller_id, key);
        }

        public object[] UnsubscribeParam(string caller_id, string caller_api, string key)
        {
            return _parameterServer.UnsubscribeParam(caller_id, caller_api, key);
        }

        public object[] SubscribeParam(string caller_id, string caller_api, string key)
        {
            return _parameterServer.SubscribeParam(caller_id, caller_api, key);
        }
        #region "ROS XML CRAP Parsing"
        private static Tuple<string, IEnumerable<string>> ProcessLine(object l)
        {
            //[topic1, [topic1Publisher1...topic1PublisherN]] ... ]
            return ProcessLine((object[])l);
        }

        private static Tuple<string, IEnumerable<string>> ProcessLine(object[] l)
        {
            if (!l.Any())
                return null;
            var topicName = l[0] as string;
            var nodeNames = ((string[])l[1]).Distinct().ToList();

            return new Tuple<string, IEnumerable<string>>(topicName, nodeNames);
        }
        #endregion
    }
}
