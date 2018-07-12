/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosPenTo
{
    public class Topic
    {
        public string Name { get; private set; }
        public string Type { get; set; } = "unknown";

        public Topic(string topicName)
        {
            Name = topicName;
        }

        public override string ToString()
        {
            return string.Format("{0} (Type: {1})", Name, Type);
        }

        public int CompareTo(object obj)
        {
            if (obj == null || !(obj is Node))
                return -1;
            return string.Compare(Name, ((Node)obj).Name, StringComparison.Ordinal);
        }
    }
}
