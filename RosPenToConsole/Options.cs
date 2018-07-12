/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿using CommandLine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosPenToConsole
{
    public class Options
    {
        [Option('t', "target", Required = true, HelpText = "ROS Master URI of the target system.")]
        public string TargetSystem { get; set; }

        [Option('p', "pentest", Required = true, HelpText = "ROS Master URI of the penetration testing system.")]
        public string PentestSystem { get; set; }

        [Option("sub", Required = true, HelpText = "Name of the affected subscriber in the target system.")]
        public string Subscriber { get; set; }

        [Option("top", Required = true, HelpText = "Name of the affected topic.")]
        public string Topic { get; set; }

        [Option("pub", Required = true, HelpText = "Name of the new publisher in the penetration testing system.")]
        public string Publisher { get; set; }

        [Option("add", DefaultValue = false, MutuallyExclusiveSet = "command", HelpText = "publisherUpdate command adds publisher to existing ones.")]
        public bool AddCommand { get; set; }

        [Option("set", DefaultValue = false, MutuallyExclusiveSet = "command", HelpText = "publisherUpdate command sets new publisher.")]
        public bool SetCommand { get; set; }

        [Option("remove", DefaultValue = false, MutuallyExclusiveSet = "command", HelpText = "publisherUpdate command removes publisher from existing ones.")]
        public bool RemoveCommand { get; set; }

    }
}
