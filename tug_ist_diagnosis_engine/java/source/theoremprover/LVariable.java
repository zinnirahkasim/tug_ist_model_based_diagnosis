/*
 * (c) copyright 2008, Technische Universitaet Graz and Technische Universitaet Wien
 *
 * This file is part of jdiagengine.
 *
 * jdiagengine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jdiagengine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with jdiagengine. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Joerg Weber, Franz Wotawa
 * Contact: jweber@ist.tugraz.at (preferred), or fwotawa@ist.tugraz.at
 *
 */


/**
 * LVariable: Implements an object representing a variable
 *
 * @version 0.1, DATE: 230.12.1998
 * @author Franz Wotawa
 *
 * 
 * V0.1: Creating the basic functionality (30.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;
import theoremprover.*;


public class LVariable extends LConstant
{
    // Instance creation and initialization

    LVariable()
    {
	identifier = null;
    }

    LVariable(String str)
    {
	identifier = str;
    }

    // Accessing methods

    public PropositionalTheoremProver asPropositionalSentence()
    {
	return null;
    }
}
