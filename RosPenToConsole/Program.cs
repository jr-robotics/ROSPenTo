/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using RosPenTo;
using RosPenTo.Network;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RosPenToConsole
{
    class Program
    {
        private static List<string> _optionsText = new List<string>();
        private static List<Action> _optionsMethod = new List<Action>();
        private static List<SystemAnalyzer> _systemAnalyzers = new List<SystemAnalyzer>();

        private static readonly int MAX_OPTION_IF_NO_SYSTEM_ANALYZED = 3;

        static void Main(string[] args)
        {
            Console.CancelKeyPress += new ConsoleCancelEventHandler(Console_CancelKeyPress);

            if (args.Length == 0)
                runMenu();
            else
                executeCommand(args);

        }

        private static void executeCommand(string[] args)
        {
            var options = new Options();
            CommandLine.Parser.Default.ParseArgumentsStrict(args, options, () =>
            {
                Console.WriteLine("Please specify all of those parameters!\nPress any key to exit.");
                Console.ReadKey();
                Environment.Exit(1);
            });

            if (!EndPointManager.IsValidMasterUri(options.TargetSystem))
            {
                Console.WriteLine("URI '{0}' is not valid!", options.TargetSystem);
                Environment.Exit(1);
            }
            SystemAnalyzer targetSA = new SystemAnalyzer(XmlRpcFactory.GetMasterClient(new Uri(EndPointManager.ReplaceHostnameByIp(options.TargetSystem))), XmlRpcFactory.GetParameterClient(new Uri(EndPointManager.ReplaceHostnameByIp(options.TargetSystem))));
            try
            {
                targetSA.Update();
            }
            catch (Exception e)
            {
                Console.WriteLine("System '{0}' does not respond!", options.TargetSystem);
                Environment.Exit(1);
            }

            if (!EndPointManager.IsValidMasterUri(options.PentestSystem))
            {
                Console.WriteLine("URI '{0}' is not valid!", options.PentestSystem);
                Environment.Exit(1);
            }
            SystemAnalyzer pentestSA = new SystemAnalyzer(XmlRpcFactory.GetMasterClient(new Uri(EndPointManager.ReplaceHostnameByIp(options.PentestSystem))), XmlRpcFactory.GetParameterClient(new Uri(EndPointManager.ReplaceHostnameByIp(options.PentestSystem))));
            try
            {
                pentestSA.Update();
            }
            catch (Exception e)
            {
                Console.WriteLine("System '{0}' does not respond!", options.PentestSystem);
                Environment.Exit(1);
            }

            Topic topic = null;
            foreach (Topic t in targetSA.Topics)
            {
                if (t.Name.Equals(options.Topic))
                {
                    topic = t;
                    break;
                }
            }
            if (topic == null)
            {
                Console.WriteLine("Topic '{0}' not found in '{1}'", options.Topic, options.TargetSystem);
                Environment.Exit(1);
            }

            Node subscriber = null;
            foreach (Node s in targetSA.Nodes)
            {
                if (s.Name.Equals(options.Subscriber))
                {
                    subscriber = s;
                    break;
                }
            }
            if (subscriber == null)
            {
                Console.WriteLine("Subscriber '{0}' not found in '{1}'!", options.Subscriber, options.TargetSystem);
                Environment.Exit(1);
            }

            Node publisher = null;
            foreach (Node p in pentestSA.Nodes)
            {
                if (p.Name.Equals(options.Publisher))
                {
                    publisher = p;
                    break;
                }
            }
            if (publisher == null)
            {
                Console.WriteLine("Publisher '{0}' not found in '{1}'!", options.Publisher, options.PentestSystem);
                Environment.Exit(1);
            }

            if (options.AddCommand && !options.SetCommand && !options.RemoveCommand)
            {
                List<Node> publishers = new List<Node>();
                publishers.AddRange(subscriber.TopicPublishers[topic]);
                publishers.Add(publisher);
                ExecutePublisherUpdate(subscriber, topic, publishers);
            }
            else if (options.SetCommand && !options.AddCommand && !options.RemoveCommand)
            {
                List<Node> publishers = new List<Node>();
                publishers.Add(publisher);
                ExecutePublisherUpdate(subscriber, topic, publishers);
            }
            else if (options.RemoveCommand && !options.AddCommand && !options.SetCommand)
            {
                List<Node> publishers = new List<Node>();
                publishers.AddRange(subscriber.TopicPublishers[topic]);
                if (publishers.Contains(publisher))
                    publishers.Remove(publisher);
                ExecutePublisherUpdate(subscriber, topic, publishers);
            }
            else
            {
                Console.WriteLine("Please specify exactly one command!");
                Environment.Exit(1);
            }

        }

        private static void printLicense()
        {
            Console.WriteLine("RosPenTo - Penetration testing tool for the Robot Operating System(ROS)");
            Console.WriteLine("Copyright(C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH");

            Console.WriteLine("This program comes with ABSOLUTELY NO WARRANTY.\r\n"+
            "This is free software, and you are welcome to redistribute it under certain conditions.");

            Console.WriteLine("For more details see the GNU General Public License at <http://www.gnu.org/licenses/>.");
        }

        private static void runMenu()
        {
            printLicense();

            InitOptions();
            int option;
            while (true)
            {
                PrintOptions();
                string input = Console.ReadLine();

                if (_systemAnalyzers.Count == 0)
                    option = CheckInputForNumberInRange(input, MAX_OPTION_IF_NO_SYSTEM_ANALYZED);
                else
                    option = CheckInputForNumberInRange(input, _optionsText.Count);
                if (option == -1)
                    continue;

                _optionsMethod[option]();
            }
        }

        private static void InitOptions()
        {
            _optionsText.Add("Exit");
            _optionsMethod.Add(ExitOk);

            _optionsText.Add("Analyse system...");
            _optionsMethod.Add(AnalyzeSystem);

            _optionsText.Add("Print all analyzed systems");
            _optionsMethod.Add(PrintAllAnalyzedSystems);

            _optionsText.Add("Print information about analyzed system...");
            _optionsMethod.Add(PrintInformationAboutAnalyzedSystem);

            _optionsText.Add("Print nodes of analyzed system...");
            _optionsMethod.Add(PrintNodesOfSystem);

            _optionsText.Add("Print node types of analyzed system (Python or C++)...");
            _optionsMethod.Add(PrintNodeTypes);

            _optionsText.Add("Print topics of analyzed system...");
            _optionsMethod.Add(PrintTopicsOfSystem);

            _optionsText.Add("Print services of analyzed system...");
            _optionsMethod.Add(PrintServicesOfSystem);

            _optionsText.Add("Print communications of analyzed system...");
            _optionsMethod.Add(PrintCommunicationsOfSystem);

            _optionsText.Add("Print communications of topic...");
            _optionsMethod.Add(PrintCommunicationOfTopic);

            _optionsText.Add("Print parameters...");
            _optionsMethod.Add(PrintSystemParametersWithValuesAndTypes);

            _optionsText.Add("Update publishers list of subscriber (add)...");
            _optionsMethod.Add(PublisherUpdateAddPublishers);

            _optionsText.Add("Update publishers list of subscriber (set)...");
            _optionsMethod.Add(PublisherUpdateSetPublishers);

            _optionsText.Add("Update publishers list of subscriber (remove)...");
            _optionsMethod.Add(PublisherUpdateRemovePublishers);

            _optionsText.Add("Isolate service...");
            _optionsMethod.Add(ServiceIsolation);

            _optionsText.Add("Unsubscribe node from parameter (only C++)...");
            _optionsMethod.Add(UnsubscribeNodeFromParameter);

            _optionsText.Add("Update subscribed parameter at Node (only C++)...");
            _optionsMethod.Add(ParamUpdateForNode);
        }

        private static void PrintOptions()
        {
            Console.WriteLine("\nWhat do you want to do?");

            for (int index = 0; index < _optionsText.Count; index++)
            {
                if (_systemAnalyzers.Count == 0 && index >= MAX_OPTION_IF_NO_SYSTEM_ANALYZED)
                    continue;

                Console.WriteLine(index + ": " + _optionsText.ElementAt(index));
            }
        }

        private static void ExitOk()
        {
            Environment.Exit(0);
        }

        private static void AnalyzeSystem()
        {
            string input;

            while (true)
            {
                Console.WriteLine("\nPlease input URI of ROS Master: (e.g. http://localhost:11311/)");

                input = Console.ReadLine();
                if (input == null)
                    return; // ctrl-c pressed                

                if (!EndPointManager.IsValidMasterUri(input))
                {
                    Console.WriteLine("URI '{0}' is not valid!", input);
                    continue;
                }

                Uri masterUri = null;
                try
                {
                    masterUri = new Uri(EndPointManager.ReplaceHostnameByIp(input));
                }catch(Exception e)
                {
                    Console.WriteLine(e.Message);
                    continue;
                }                

                // check if system analyzer already exists
                SystemAnalyzer tempSA = null;
                foreach (SystemAnalyzer sa in _systemAnalyzers)
                {
                    if (sa.URL.ToString().Equals(masterUri.ToString()))
                    {
                        tempSA = sa;
                        break;
                    }
                }
                SystemAnalyzer systemAnalyzer = null;
                if (tempSA == null)
                    systemAnalyzer = new SystemAnalyzer(XmlRpcFactory.GetMasterClient(masterUri), XmlRpcFactory.GetParameterClient(masterUri));
                else
                    systemAnalyzer = tempSA;

                // update the system
                try
                {
                    systemAnalyzer.Update();
                }
                catch (Exception e)
                {
                    Console.WriteLine("System ({0}) does not respond!", systemAnalyzer.URL);
                    continue;
                }

                // only add new systemanalyzer
                if (tempSA == null)
                    _systemAnalyzers.Add(systemAnalyzer);
                PrintInformationAboutAnalysedSystem(systemAnalyzer);
                return;
            }
        }

        private static void PrintAllAnalyzedSystems()
        {
            Console.WriteLine("\nAnalysed Systems:");
            foreach (SystemAnalyzer sa in _systemAnalyzers)
            {
                PrintAnalysedSystemInformation(sa);
            }
        }

        private static void PrintAnalysedSystemInformation(SystemAnalyzer sa)
        {
            Console.WriteLine("System {0}: {1}", _systemAnalyzers.IndexOf(sa), sa.URL);
        }

        private static void PrintInformationAboutAnalyzedSystem()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            if (sa != null)
            {
                PrintInformationAboutAnalysedSystem(sa);
            }
        }

        private static void PrintInformationAboutAnalysedSystem(SystemAnalyzer sa)
        {
            Console.WriteLine();
            PrintAnalysedSystemInformation(sa);
            PrintNodes(sa);
            PrintTopics(sa);
            PrintServices(sa);
            PrintCommunications(sa);
            PrintParameters(sa);
        }

        private static void PrintNodesOfSystem()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            if (sa != null)
            {
                Console.WriteLine();
                PrintAnalysedSystemInformation(sa);
                PrintNodes(sa);
            }
        }

        private static void PrintTopicsOfSystem()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            if (sa != null)
            {
                Console.WriteLine();
                PrintAnalysedSystemInformation(sa);
                PrintTopics(sa);
            }
        }

        private static void PrintServicesOfSystem()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            if (sa != null)
            {
                Console.WriteLine();
                PrintAnalysedSystemInformation(sa);
                PrintServices(sa);
            }
        }

        private static void PrintCommunicationsOfSystem()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            if (sa != null)
            {
                Console.WriteLine();
                PrintAnalysedSystemInformation(sa);
                PrintCommunications(sa);
            }
        }

        private static void PrintCommunicationOfTopic()
        {
            int[] userInput = GetSystemAnalyzerAndTopicIndexFromUserInput();
            if (userInput != null)
            {
                SystemAnalyzer sa = _systemAnalyzers.ElementAt(userInput[0]);
                Topic topic = sa.Topics.ElementAt(userInput[1]);

                Console.WriteLine();
                PrintAnalysedSystemInformation(sa);
                PrintCommunicationsFiltered(sa, topic);
            }
        }

        private static void PrintNodes(SystemAnalyzer sa)
        {
            Console.WriteLine("Nodes:");
            var sortedNodesList = sa.Nodes.OrderBy(x => x.Name).ToList();
            foreach (Node node in sortedNodesList)
            {
                Console.Write("\t");
                PrintNode(sa, node);
            }
        }

        private static void PrintTopics(SystemAnalyzer sa)
        {
            Console.WriteLine("Topics:");
            var sortedTopicList = sa.Topics.OrderBy(x => x.Name).ToList();
            foreach (Topic topic in sortedTopicList)
            {
                Console.Write("\t");
                PrintTopic(sa, topic);
            }
        }

        private static void PrintServices(SystemAnalyzer sa)
        {
            Console.WriteLine("Services:");
            var sortedServiceList = sa.Services.OrderBy(x => x.Name).ToList();
            foreach (Service service in sortedServiceList)
            {
                Console.Write("\t");
                PrintService(sa, service);
            }
        }

        private static void PrintNode(SystemAnalyzer sa, Node node)
        {
            Console.WriteLine("Node {0}.{1}: {2}", _systemAnalyzers.IndexOf(sa), sa.Nodes.IndexOf(node), node);
        }

        private static void PrintTopic(SystemAnalyzer sa, Topic topic)
        {
            Console.WriteLine("Topic {0}.{1}: {2}", _systemAnalyzers.IndexOf(sa), sa.Topics.IndexOf(topic), topic);
        }

        private static void PrintService(SystemAnalyzer sa, Service service)
        {
            Console.WriteLine("Service {0}.{1}: {2}", _systemAnalyzers.IndexOf(sa), sa.Services.IndexOf(service), service);
        }

        private static void PrintCommunications(SystemAnalyzer sa)
        {
            Console.WriteLine("Communications:");
            foreach (Communication communication in sa.Communications)
            {
                PrintCommunication(sa, communication);
            }
        }

        private static void PrintCommunication(SystemAnalyzer sa, Communication communication)
        {
            Console.WriteLine("\tCommunication {0}.{1}:", _systemAnalyzers.IndexOf(sa), sa.Communications.IndexOf(communication));

            Console.WriteLine("\t\tPublishers:");
            foreach (Node publisher in communication.Publishers)
            {
                Console.Write("\t\t\t");
                PrintNode(sa, publisher);
            }

            Console.Write("\t\t");
            PrintTopic(sa, communication.Topic);

            Console.WriteLine("\t\tSubscribers:");
            foreach (Node subscriber in communication.Subscribers)
            {
                Console.Write("\t\t\t");
                PrintNode(sa, subscriber);
            }
        }

        private static void PrintCommunicationsFiltered(SystemAnalyzer sa, Topic topic)
        {
            foreach (Communication communication in sa.Communications)
            {
                if (topic == communication.Topic)
                {
                    PrintCommunication(sa, communication);
                }
            }
        }

        private static void PrintParameters(SystemAnalyzer sa)
        {
            Console.WriteLine("Parameters:");
            foreach(Parameter parameter in sa.Parameters)
            {
                PrintParameter(sa, parameter);
            }
        }
        private static void PrintSystemParametersWithValuesAndTypes()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            PrintParametersWithValuesAndTypes(sa);
        }
        private static void PrintParametersWithValuesAndTypes(SystemAnalyzer sa)
        {
            Console.WriteLine("Parameters:");
            foreach(Parameter parameter in sa.Parameters)
            {
                PrintParameterWithTypeAndValue(sa, parameter);
            }
        }
        private static void PrintParameter(SystemAnalyzer sa, Parameter parameter)
        {
            Console.WriteLine("\tParameter {0}.{1}:", _systemAnalyzers.IndexOf(sa), sa.Parameters.IndexOf(parameter));
            Console.WriteLine("\t\tName: {0}", parameter.ToString());
        }

        private static void PrintParameterWithTypeAndValue(SystemAnalyzer sa, Parameter parameter)
        {
            
            Console.WriteLine("\tParameter {0}.{1}:", _systemAnalyzers.IndexOf(sa), sa.Parameters.IndexOf(parameter));
            Console.WriteLine("\t\tName: {0}", parameter.Name);

            Console.WriteLine("\t\tType: {0}", parameter.Type);

            if (parameter.Type.IsArray)
            {
                Console.WriteLine("\t\tArrays not supported");
            }
            else
            {
                Console.WriteLine("\t\tValue: {0}", parameter.Value);
            }
        }

        private static void PrintNodeTypes()
        {
            SystemAnalyzer sa = GetSystemAnalyzerFromUserInput();
            foreach(Node node in sa.Nodes)
            {
                PrintNodeType(sa, node);
            }

        }

        private static void PrintNodeType(SystemAnalyzer sa, Node node)
        {
            
            IXmlRpcSlaveClient nodeSlave = XmlRpcFactory.GetSlaveClient(node.XmlRpcUri);
            object[] getNameResponse = null;
            try
            {
                getNameResponse = nodeSlave.GetName("/master");
                //Console.WriteLine(String.Join(",", getNameResponse));
            }
            catch(CookComputing.XmlRpc.XmlRpcFaultException)
            {
                Console.WriteLine("Node {0}.{1}: C++", _systemAnalyzers.IndexOf(sa), sa.Nodes.IndexOf(node));
                return;
            }

            int getNameStatusCode = (int)getNameResponse[0];
            string getNameStatusMessage = (string)getNameResponse[1];

            if(getNameStatusCode == -1)
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + getNameStatusMessage);
            }
            else if(getNameStatusCode == 0)
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + getNameStatusMessage);
            }
            else if(getNameStatusCode == 1)
            {
                string name = (string)getNameResponse[2];
                Console.WriteLine("Node {0}.{1}: Python", _systemAnalyzers.IndexOf(sa), sa.Nodes.IndexOf(node));
            }
            else
            {
                Console.WriteLine("Invalid status code received");
            }
        }

        private static void PublisherUpdateAddPublishers()
        {
            Console.WriteLine("To which subscriber do you want to send the publisherUpdate message?");
            int[] userInputSubscriber = GetSystemAnalyzerAndSubscriberIndexFromUserInput();
            if (userInputSubscriber == null)
                return;
            Node subscriber = _systemAnalyzers.ElementAt(userInputSubscriber[0]).Nodes.ElementAt(userInputSubscriber[1]);

            Console.WriteLine("Which topic should be affected?");
            int[] userInputTopic = GetSystemAnalyzerAndTopicIndexFromUserInput();
            if (userInputTopic == null)
                return;
            Topic topic = _systemAnalyzers.ElementAt(userInputTopic[0]).Topics.ElementAt(userInputTopic[1]);

            Console.WriteLine("Which publisher(s) do you want to add?");
            List<int[]> userInputPublishers = GetSystemAnalyzerAndPublishersIndexFromUserInput();
            List<Node> newPublishers = new List<Node>();
            foreach (int[] userInputPublisher in userInputPublishers)
            {
                newPublishers.Add(_systemAnalyzers.ElementAt(userInputPublisher[0]).Nodes.ElementAt(userInputPublisher[1]));
            }

            List<Node> allPublishers = new List<Node>();
            allPublishers.AddRange(subscriber.TopicPublishers[topic]); // adding existing publishers
            allPublishers.AddRange(newPublishers); // adding new publishers  

            ExecutePublisherUpdate(subscriber, topic, allPublishers.Distinct());
        }

        private static void PublisherUpdateSetPublishers()
        {
            Console.WriteLine("To which subscriber do you want to send the publisherUpdate message?");
            int[] userInputSubscriber = GetSystemAnalyzerAndSubscriberIndexFromUserInput();
            if (userInputSubscriber == null)
                return;
            Node subscriber = _systemAnalyzers.ElementAt(userInputSubscriber[0]).Nodes.ElementAt(userInputSubscriber[1]);

            Console.WriteLine("Which topic should be affected?");
            int[] userInputTopic = GetSystemAnalyzerAndTopicIndexFromUserInput();
            if (userInputTopic == null)
                return;
            Topic topic = _systemAnalyzers.ElementAt(userInputTopic[0]).Topics.ElementAt(userInputTopic[1]);

            Console.WriteLine("Which publisher(s) do you want to set?");
            List<int[]> userInputPublishers = GetSystemAnalyzerAndPublishersIndexFromUserInput();
            List<Node> newPublishers = new List<Node>();
            foreach (int[] userInputPublisher in userInputPublishers)
            {
                newPublishers.Add(_systemAnalyzers.ElementAt(userInputPublisher[0]).Nodes.ElementAt(userInputPublisher[1]));
            }

            ExecutePublisherUpdate(subscriber, topic, newPublishers.Distinct());
        }

        private static void PublisherUpdateRemovePublishers()
        {
            Console.WriteLine("To which subscriber do you want to send the publisherUpdate message?");
            int[] userInputSubscriber = GetSystemAnalyzerAndSubscriberIndexFromUserInput();
            if (userInputSubscriber == null)
                return;
            Node subscriber = _systemAnalyzers.ElementAt(userInputSubscriber[0]).Nodes.ElementAt(userInputSubscriber[1]);

            Console.WriteLine("Which topic should be affected?");
            int[] userInputTopic = GetSystemAnalyzerAndTopicIndexFromUserInput();
            if (userInputTopic == null)
                return;
            Topic topic = _systemAnalyzers.ElementAt(userInputTopic[0]).Topics.ElementAt(userInputTopic[1]);

            Console.WriteLine("Which publisher(s) do you want to remove?");
            List<int[]> userInputPublishers = GetSystemAnalyzerAndPublishersIndexFromUserInput();
            List<Node> deletePublishers = new List<Node>();
            foreach (int[] userInputPublisher in userInputPublishers)
            {
                deletePublishers.Add(_systemAnalyzers.ElementAt(userInputPublisher[0]).Nodes.ElementAt(userInputPublisher[1]));
            }

            List<Node> publishers = new List<Node>();
            publishers.AddRange(subscriber.TopicPublishers[topic]); // adding existing publishers
            foreach (Node deletePublisher in deletePublishers) // remove publishers
            {
                if (publishers.Contains(deletePublisher))
                    publishers.Remove(deletePublisher);
            }

            ExecutePublisherUpdate(subscriber, topic, publishers.Distinct());
        }

        private static int ExecutePublisherUpdate(Node subscriber, Topic topic, IEnumerable<Node> publishers)
        {
            List<string> publisherUris = new List<string>();
            foreach (Node publisher in publishers)
            {
                publisherUris.Add(publisher.XmlRpcUri.ToString());
            }

            Console.WriteLine("sending publisherUpdate to subscriber '{0}' over topic '{1}' with publishers '{2}'", subscriber, topic, string.Join(",", publishers));

            IXmlRpcSlaveClient subscriberSlave = XmlRpcFactory.GetSlaveClient(subscriber.XmlRpcUri);
            object[] response = null;
            try
            {
                 if(!publisherUris.Any())
                    publisherUris.Add("");
                response = subscriberSlave.PublisherUpdate("/master", topic.Name, publisherUris.ToArray());
            }
            catch (CookComputing.XmlRpc.XmlRpcFaultException e)
            {
                Console.WriteLine(e);
                return -1;
            }

            int statusCode = (int)response[0];
            string statusMessage = (string)response[1];
            if (statusCode == -1) // ERROR
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + statusMessage);
            }
            else if (statusCode == 0) // FAILURE
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + statusMessage);
            }
            else if (statusCode == 1) // SUCCESS
            {
                Console.WriteLine("PublisherUpdate completed successfully.");
            }

            return statusCode;
        }

        private static void ServiceIsolation()
        {
            Console.WriteLine("Which service do you want to isolate?");
            int[] userInputService = GetSystemAnalyzerAndServiceIndexFromUserInput();
            if (userInputService == null)
                return;
            SystemAnalyzer sa = _systemAnalyzers.ElementAt(userInputService[0]);
            Service service = sa.Services.ElementAt(userInputService[1]);

            ExecuteServiceIsolation(sa, service);
        }

        private static int ExecuteServiceIsolation(SystemAnalyzer sa, Service service)
        {
            //lookupService

            object[] lookupServiceResponse = null;
            try
            {
                lookupServiceResponse = sa.LookupService("", service.Name);
            }
            catch(CookComputing.XmlRpc.XmlRpcFaultException e)
            {
                Console.WriteLine(e);
                return -1;
            }

            int lookupServiceStatusCode = (int)lookupServiceResponse[0];
            string lookupServiceStatusMessage = (string)lookupServiceResponse[1];
            if (lookupServiceStatusCode == -1) // ERROR
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + lookupServiceStatusMessage);
                return -1;
            }
            else if (lookupServiceStatusCode == 0) // FAILURE
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + lookupServiceStatusMessage);
                return 0;
            }
            else if (lookupServiceStatusCode == 1) // SUCCESS
            {
                //get service server name and service uri
                string serviceUri = (string)lookupServiceResponse[2];
                List<Node> nodes = sa.Nodes;
                string serviceProviderName = "";
                foreach(Node node in nodes)
                {
                    if(node.Services.Contains(service))
                    {
                        serviceProviderName = node.Name;
                        break;
                    }
                }

                //unregister service
                object[] unregisterServiceResponse = null;

                try
                {
                    unregisterServiceResponse = sa.UnregisterService(serviceProviderName, service.Name, serviceUri);
                }
                catch (CookComputing.XmlRpc.XmlRpcFaultException e)
                {
                    Console.WriteLine(e);
                    return -1;
                }

                int unregisterServiceStatusCode = (int)unregisterServiceResponse[0];
                string unregisterServiceStatusMessage = (string)unregisterServiceResponse[1];

                if (unregisterServiceStatusCode == -1) // ERROR
                {
                    Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                    Console.WriteLine("StatusMessage: " + unregisterServiceStatusMessage);
                    return -1;
                }
                else if (unregisterServiceStatusCode == 0) // FAILURE
                {
                    Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                    Console.WriteLine("StatusMessage: " + unregisterServiceStatusMessage);
                    return 0;
                }
                else if (unregisterServiceStatusCode == 1) // SUCCESS
                {
                    int numUnregistered = (int)unregisterServiceResponse[2];
                    if(numUnregistered == 0)
                    {
                        Console.WriteLine(serviceProviderName + " was not registered as a service provider for " + service.Name);
                        return 0;
                    }
                    else if(numUnregistered == 1)
                    {
                        Console.WriteLine("Service Isolation completed successfully.");
                        Console.WriteLine(serviceProviderName + " has been unregistered as provider of " + service.Name);
                        return 1;
                    }
                    else
                    {
                        Console.WriteLine("Invalid number of unregistered providers: " + numUnregistered);
                        return -1;
                    }
                    
                }
                else
                {
                    Console.WriteLine("Status code for unregisterService not valid: " + lookupServiceStatusCode);
                    return -1;
                }
                
            }
            else
            {
                Console.WriteLine("Status code for lookupService not valid: " + lookupServiceStatusCode);
                return -1;
            }
        }

        private static void UnsubscribeNodeFromParameter()
        {
            int[] userInputNode = GetSystemAnalyzerAndNodeIndexFromUserInput();
            int[] userInputParameter = GetSystemAnalyzerAndParameterIndexFromUserInput();

            if (userInputNode == null || userInputParameter == null)
                return;

            if(userInputNode[0] != userInputParameter[0])
            {
                Console.WriteLine("Only supported in the same system");
                return;
            }

            SystemAnalyzer sa = _systemAnalyzers.ElementAt(userInputNode[0]);
            Node node = sa.Nodes.ElementAt(userInputNode[1]);
            Parameter parameter = sa.Parameters.ElementAt(userInputParameter[1]);

            int unsubscribeParamStatus = ExecuteUnsubscribeParam(sa, node.Name, node.XmlRpcUri.ToString(), parameter.Name);

            if(unsubscribeParamStatus == -1)
            {
                return;
            }
            else if(unsubscribeParamStatus == 0)
            {
                Console.WriteLine("Node {0}.{1} was not subscribed to Parameter {0}.{2}", userInputNode[0], userInputNode[1], userInputParameter[0], userInputParameter[1]);
            }
            else
            {
                Console.WriteLine("Node {0}.{1} successfully unsubscribed from Parameter {0}.{2}", userInputNode[0], userInputNode[1], userInputParameter[0], userInputParameter[1]);
            }
        }

        private static void ParamUpdateForNode()
        {
            int[] userInputNode = GetSystemAnalyzerAndNodeIndexFromUserInput();
            int[] userInputParameter = GetSystemAnalyzerAndParameterIndexFromUserInput();

            if (userInputNode == null || userInputParameter == null)
                return;

            if (userInputNode[0] != userInputParameter[0])
            {
                Console.WriteLine("Only supported in the same system");
                return;
            }

            SystemAnalyzer sa = _systemAnalyzers.ElementAt(userInputNode[0]);
            Node node = sa.Nodes.ElementAt(userInputNode[1]);
            Parameter parameter = sa.Parameters.ElementAt(userInputParameter[1]);

            Type parameterType = parameter.Type;

            string input;

            Console.WriteLine("Please enter value for paramUpdate");
            input = Console.ReadLine();
            if (input == null)
            {
                return; // ctrl-c pressed
            }

            IXmlRpcSlaveClient paramSubscriberSlave = XmlRpcFactory.GetSlaveClient(node.XmlRpcUri);

            int paramUpdateResult = ExecuteParamUpdate(paramSubscriberSlave, parameter, input);

            if(paramUpdateResult == -1)
            {
                Console.WriteLine("Param update for Parameter {0}.{1} at Node {0}.{2} not successful", userInputParameter[0], userInputParameter[1], userInputNode[1]);
            }
            else
            {
                Console.WriteLine("Param update for Parameter {0}.{1} at Node {0}.{2} sent", userInputParameter[0], userInputParameter[1], userInputNode[1]);
            }

        }

        private static int ExecuteParamUpdate(IXmlRpcSlaveClient client, Parameter parameter, string value)
        {
            object[] paramUpdateResponse = null;
            try
            {
                paramUpdateResponse = client.ParamUpdate("/master", parameter.Name, Convert.ChangeType(value, parameter.Type));
            }
            catch(CookComputing.XmlRpc.XmlRpcFaultException xrfe)
            {
                Console.WriteLine(xrfe.Message);
                return -1;
            }
            catch(InvalidCastException ice)
            {
                Console.WriteLine(ice.Message);
                return -1;
            }
            catch(FormatException fe)
            {
                Console.WriteLine(fe.Message);
                return -1;
            }
            catch(OverflowException oe)
            {
                Console.WriteLine(oe);
                return -1;
            }
            catch(ArgumentException ae)
            {
                Console.WriteLine(ae);
                return -1;
            }

            int paramUpdateStatusCode = (int)paramUpdateResponse[0];
            string paramUpdateStatusMessage = (string)paramUpdateResponse[1];

            if (paramUpdateStatusCode == -1)
            {
                return -1;
            }
            else if(paramUpdateStatusCode == 0)
            {
                return -1;
            }
            else if(paramUpdateStatusCode == 1)
            {
                return 1;
            }
            else
            {
                Console.WriteLine("Invalid status code received");
                return -1;
            }
        }

        private static int ExecuteSubscribeParam(SystemAnalyzer sa, string caller_id, string caller_api, string key)
        {
            object[] subscribeParamResponse = null;
            try
            {
                subscribeParamResponse = sa.SubscribeParam(caller_id, caller_api, key);
            }
            catch(CookComputing.XmlRpc.XmlRpcFaultException e)
            {
                Console.WriteLine(e.Message);
                return -1;
            }

            int subscribeParamStatusCode = (int)subscribeParamResponse[0];
            string subscribeParamStatusMessage = (string)subscribeParamResponse[1];

            if(subscribeParamStatusCode == -1)
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + subscribeParamStatusMessage);
                return -1;
            }
            else if(subscribeParamStatusCode == 0)
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + subscribeParamStatusMessage);
                return -1;    
            }
            else if(subscribeParamStatusCode == 1)
            {
                return 1;
            }
            else
            {
                Console.WriteLine("Invalid status code received");
                return -1;
            }
        }

        private static int ExecuteUnsubscribeParam(SystemAnalyzer sa, string caller_id, string caller_api, string key)
        {
            object[] unsubscribeParamResponse = null;
            try
            {
                unsubscribeParamResponse = sa.UnsubscribeParam(caller_id, caller_api, key);
            }
            catch (CookComputing.XmlRpc.XmlRpcFaultException e)
            {
                Console.WriteLine(e.Message);
                return -1;
            }

            int unsubscribeParamStatusCode = (int)unsubscribeParamResponse[0];
            string unsubscribeParamStatusMessage = (string)unsubscribeParamResponse[1];

            if (unsubscribeParamStatusCode == -1)
            {
                Console.WriteLine("Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.");
                Console.WriteLine("StatusMessage: " + unsubscribeParamStatusMessage);
                return -1;
            }
            else if (unsubscribeParamStatusCode == 0)
            {
                Console.WriteLine("Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.");
                Console.WriteLine("StatusMessage: " + unsubscribeParamStatusMessage);
                return -1;
            }
            else if (unsubscribeParamStatusCode == 1)
            {
                int numUnsubscribed = (int)unsubscribeParamResponse[2];
                if (numUnsubscribed == 0 || numUnsubscribed == 1)
                {
                    return numUnsubscribed;
                }
                else
                {
                    Console.WriteLine("Invalid number of unsubscribed parameters received");
                    return -1;
                }
            }
            else
            {
                Console.WriteLine("Invalid status code received");
                return -1;
            }
        }    
        private static int CheckInputForNumberInRange(string input, int upperBound)
        {
            int option = -1;
            try
            {
                option = Int32.Parse(input);
            }
            catch (ArgumentNullException)
            {
                return -1;

            }
            catch (FormatException)
            {
                Console.WriteLine("Please only insert numbers!");
                return -1;
            }

            if (option < 0 || option >= upperBound)
            {
                Console.WriteLine("The selected number is out of range!");
                return -1;
            }

            return option;
        }

        private static SystemAnalyzer GetSystemAnalyzerFromUserInput()
        {
            int index;
            string input;
            while (true)
            {
                Console.WriteLine("Please enter number of analysed system:");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                index = CheckInputForNumberInRange(input, _systemAnalyzers.Count);
                if (index == -1)
                    continue;

                return _systemAnalyzers.ElementAt(index);
            }
        }

        private static int[] GetSystemAnalyzerAndSubscriberIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            int[] output = new int[2];
            while (true)
            {
                Console.WriteLine("Please enter number of subscriber (e.g.: 0.0):");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split('.');
                if (inputParts.Length != 2)
                {
                    Console.WriteLine("Invalid input '{0}'!", input);
                    continue;
                }

                output[0] = CheckInputForNumberInRange(inputParts[0], _systemAnalyzers.Count);
                if (output[0] == -1)
                    continue;

                output[1] = CheckInputForNumberInRange(inputParts[1], _systemAnalyzers.ElementAt(output[0]).Nodes.Count);
                if (output[1] == -1)
                    continue;

                return output;
            }
        }

        private static int[] GetSystemAnalyzerAndServiceIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            int[] output = new int[2];
            while (true)
            {
                Console.WriteLine("Please enter number of service (e.g.: 0.0):");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split('.');
                if (inputParts.Length != 2)
                {
                    Console.WriteLine("Invalid input '{0}'!", input);
                    continue;
                }

                output[0] = CheckInputForNumberInRange(inputParts[0], _systemAnalyzers.Count);
                if (output[0] == -1)
                    continue;

                output[1] = CheckInputForNumberInRange(inputParts[1], _systemAnalyzers.ElementAt(output[0]).Services.Count);
                if (output[1] == -1)
                    continue;

                return output;
            }
        }

        private static List<int[]> GetSystemAnalyzerAndPublishersIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            string[] elementParts;
            int[] output;
            List<int[]> result = new List<int[]>();
            Boolean success = true;

            while (true)
            {
                Console.WriteLine("Please enter number of publisher(s) (e.g.: 0.0,0.1,...):");
                result.Clear();
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split(',');

                foreach (string part in inputParts)
                {
                    output = new int[2];
                    elementParts = part.Split('.');

                    output[0] = CheckInputForNumberInRange(elementParts[0], _systemAnalyzers.Count);
                    if (output[0] == -1)
                    {
                        success = false;
                        break;
                    }

                    output[1] = CheckInputForNumberInRange(elementParts[1], _systemAnalyzers.ElementAt(output[0]).Nodes.Count);
                    if (output[1] == -1)
                    {
                        success = false;
                        break;
                    }
                    result.Add(output);
                }
                if (success)
                    break;
            }
            return result;
        }

        private static int[] GetSystemAnalyzerAndTopicIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            int[] output = new int[2];
            while (true)
            {
                Console.WriteLine("Please enter number of topic (e.g.: 0.0):");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split('.');
                if (inputParts.Length != 2)
                {
                    Console.WriteLine("Invalid input '{0}'!", input);
                    continue;
                }

                output[0] = CheckInputForNumberInRange(inputParts[0], _systemAnalyzers.Count);
                if (output[0] == -1)
                    continue;

                output[1] = CheckInputForNumberInRange(inputParts[1], _systemAnalyzers.ElementAt(output[0]).Topics.Count);
                if (output[1] == -1)
                    continue;

                return output;
            }
        }

        private static int[] GetSystemAnalyzerAndNodeIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            int[] output = new int[2];
            while (true)
            {
                Console.WriteLine("Please enter number of node (e.g.: 0.0):");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split('.');
                if (inputParts.Length != 2)
                {
                    Console.WriteLine("Invalid input '{0}'!", input);
                    continue;
                }

                output[0] = CheckInputForNumberInRange(inputParts[0], _systemAnalyzers.Count);
                if (output[0] == -1)
                    continue;

                output[1] = CheckInputForNumberInRange(inputParts[1], _systemAnalyzers.ElementAt(output[0]).Nodes.Count);
                if (output[1] == -1)
                    continue;

                return output;
            }
        }

        private static int[] GetSystemAnalyzerAndParameterIndexFromUserInput()
        {
            string input;
            string[] inputParts;
            int[] output = new int[2];
            while (true)
            {
                Console.WriteLine("Please enter number of parameter (e.g.: 0.0):");
                input = Console.ReadLine();
                if (input == null)
                {
                    return null; // ctrl-c pressed
                }

                inputParts = input.Split('.');
                if (inputParts.Length != 2)
                {
                    Console.WriteLine("Invalid input '{0}'!", input);
                    continue;
                }

                output[0] = CheckInputForNumberInRange(inputParts[0], _systemAnalyzers.Count);
                if (output[0] == -1)
                    continue;

                output[1] = CheckInputForNumberInRange(inputParts[1], _systemAnalyzers.ElementAt(output[0]).Parameters.Count);
                if (output[1] == -1)
                    continue;

                return output;
            }
        }

        protected static void Console_CancelKeyPress(object sender, ConsoleCancelEventArgs args)
        {
            args.Cancel = true;
        }

    }
}
