/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using RosPenTo;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CookComputing.XmlRpc;
using System.Net;
using System.Security.Cryptography.X509Certificates;

namespace RosPenToTest
{
    class XmlRpcMasterMock : IXmlRpcMasterClient
    {
        public static XmlRpcMasterMock Empty = new XmlRpcMasterMock(new object[0], new object[0]);
        public static XmlRpcMasterMock DuplicateNodes = new XmlRpcMasterMock(
            new object[] //systemState
            { 
                new object[] //publishers
                { 
                    new object[] { "/rosout", new string[] { "/guido"} }
                },
                new object[] //subscribers
                { 
                    new object[] { "/rosin", new string[] { "/guido"} }
                },
                new object[] //services
                { 
                    new object[] {}
                }
            },
            new object[] //topicTypes
            {
                new object[]
                {
                    "/rosout","/output"
                },
                new object[]
                {
                    "/rosin","/input"
                }
            }
        );
        public static XmlRpcMasterMock EveryTopicHasAType = new XmlRpcMasterMock(
            new object[] //systemState
            {
                new object[] //publishers
                {
                    new object[] { "/rosout", new string[] { "/guido", "/sebastian", "/bernhard" } }
                },
                new object[] //subscribers
                {
                    new object[] { "/rosin", new string[] { "/guido", "/benjamin", "/thomas", "/marc", "/bernhard" } }
                },
                new object[] //services
                {
                    new object[] {}
                }
            },
            new object[] //topicTypes
            {
                new object[]
                {
                    "/rosout","/output"
                },
                new object[]
                {
                    "/rosin","/input"
                }
            }
        );

        private object[] _systemStateResult;
        private object[] _topicTypesResult;

        public XmlRpcMasterMock(object[] systemStateResult, object[] topicTypesResult)
        {
            _systemStateResult = systemStateResult;
            _topicTypesResult = topicTypesResult;
        }

        public object[] GetTopicTypes(string caller_id)
        {
            int code = 200;
            string statusMessage = "getTopicTypes";
            return new object[] { code, statusMessage, _topicTypesResult };
        }

        public object[] GetSystemState(string caller_id)
        {
            int code = 200;
            string statusMessage = "getSystemState";
            return new object[] { code, statusMessage, _systemStateResult };
        }

        public object[] LookupNode(string caller_id, string node_name)
        {
            return new object[] { 1, "uri", "http://127.0.0.1:12345/" };
        }

        public object[] LookupService(string caller_id, string service)
        {
            throw new NotImplementedException();
        }

        public object[] UnregisterService(string caller_id, string service, string service_api)
        {
            throw new NotImplementedException();
        }

        #region AutoImplementedCode
        public bool AllowAutoRedirect
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public X509CertificateCollection ClientCertificates
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public string ConnectionGroupName
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public CookieContainer CookieContainer
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public ICredentials Credentials
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool EnableCompression
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool Expect100Continue
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public WebHeaderCollection Headers
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Guid Id
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public int Indentation
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool KeepAlive
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public XmlRpcNonStandard NonStandard
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool PreAuthenticate
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public Version ProtocolVersion
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public IWebProxy Proxy
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public CookieCollection ResponseCookies
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public WebHeaderCollection ResponseHeaders
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public int Timeout
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public string Url
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseEmptyElementTags
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseEmptyParamsTag
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseIndentation
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseIntTag
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseNagleAlgorithm
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public string UserAgent
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public bool UseStringTag
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public Encoding XmlEncoding
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public string XmlRpcMethod
        {
            get
            {
                throw new NotImplementedException();
            }

            set
            {
                throw new NotImplementedException();
            }
        }

        public event XmlRpcRequestEventHandler RequestEvent;
        public event XmlRpcResponseEventHandler ResponseEvent;

        public void AttachLogger(XmlRpcLogger logger)
        {
            throw new NotImplementedException();
        }


        public string[] SystemListMethods()
        {
            throw new NotImplementedException();
        }

        public string SystemMethodHelp(string MethodName)
        {
            throw new NotImplementedException();
        }

        public object[] SystemMethodSignature(string MethodName)
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}
