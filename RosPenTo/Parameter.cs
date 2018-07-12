/*

RosPenTo - Penetration testing tool for the Robot Operating System (ROS)
Copyright (C) 2018 JOANNEUM RESEARCH Forschungsgesellschaft mbH

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.

*/


﻿
using System;

public class Parameter
{
    public string Name { get; private set; }
    public Type Type { get; set; }
    public object Value { get; set; }

    public Parameter(string parameterName)
	{
        Name = parameterName;
	}

    public override string ToString()
    {
        return string.Format("{0}", Name);
    }
}
