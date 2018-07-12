/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using Microsoft.VisualStudio.TestTools.UnitTesting;
using RosPenTo;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosPenToTest
{
    [TestClass]
    public class SystemAnalyzerTest
    {
        [TestMethod]
        public void EmptySystemStateReturnsEmptyGraph()
        {
            SystemAnalyzer a = new SystemAnalyzer(XmlRpcMasterMock.Empty, XmlRpcParameterClientMock.Empty);
            a.Update();

            Assert.AreEqual(0, a.Nodes.Count);
            // Assert.IsFalse(a.Services.Any());
            Assert.IsFalse(a.Topics.Any());
        }

        [TestMethod]
        public void DuplicateNodesAreFiltered()
        {
            SystemAnalyzer a = new SystemAnalyzer(XmlRpcMasterMock.DuplicateNodes, XmlRpcParameterClientMock.Empty);
            a.Update();

            Assert.IsTrue(a.Nodes.Any());
            CollectionAssert.AllItemsAreUnique(a.Nodes);
            Assert.AreEqual(1, a.Nodes.Count);
        }

        [TestMethod]
        public void EveryTopicGetItsType()
        {
            SystemAnalyzer a = new SystemAnalyzer(XmlRpcMasterMock.EveryTopicHasAType, XmlRpcParameterClientMock.Empty);
            a.Update();

            foreach (Topic t in a.Topics)
                Assert.AreNotEqual("unknown", t.Type);
        }
    }
}
