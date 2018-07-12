/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using RosPenTo;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace RosPenToTest
{
    [TestClass]
    public class UriTest
    {
        [TestMethod]
        public void TestUriProtocol1()
        {
            Uri uri = new Uri("http://127.0.0.1:11311/");
            Assert.AreEqual("http", uri.protocol);
        }

        [TestMethod]
        public void TestUriIp1()
        {
            Uri uri = new Uri("http://127.0.0.1:11311/");
            Assert.AreEqual("127.0.0.1", uri.ipAdress);
        }

        [TestMethod]
        public void TestUriIp2()
        {
            Uri uri = new Uri("http://robv002:11311/");
            Assert.AreEqual("143.224.140.66", uri.ipAdress);
        }

        [TestMethod]
        public void TestUriIp3()
        {
            Uri uri = new Uri("http://143.224.140.66:11311/");
            Assert.AreEqual("143.224.140.66", uri.ipAdress);
        }

        [TestMethod]
        public void TestUriPort1()
        {
            Uri uri = new Uri("http://127.0.0.1:11311/");
            Assert.AreEqual(11311, uri.port);
        }

        [TestMethod]
        public void TestUriPort2()
        {
            Uri uri = new Uri("http://127.0.0.1:11311");
            Assert.AreEqual(11311, uri.port);
        }

        [TestMethod]
        public void TestUri1()
        {
            Uri uri = new Uri("http://127.0.0.1:11311");
            Assert.AreEqual("http://127.0.0.1:11311/", uri.ToString);
        }
    }
}
