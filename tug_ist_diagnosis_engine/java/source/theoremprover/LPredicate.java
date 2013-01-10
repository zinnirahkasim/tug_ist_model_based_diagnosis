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
 * LPredicate: Implements an object representing a logical predicate
 *
 * @version 0.1, DATE: 230.12.1998
 * @author Franz Wotawa
 *
 * This class is used for storing the information of a logical
 * predicate. It is not intended to be used for implementing
 * logical operations or consistency checks. To do this convert
 * my instances to a more appropriate format.
 * 
 * V0.1: Creating the basic functionality (30.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;
import theoremprover.*;


public class LPredicate extends LObject
{
    // Instance variables

    public ArrayList arguments;
    public String identifier;

    // Instance creation and initialization

    LPredicate()
    {
	arguments = new ArrayList();
	identifier = null;
    }

    LPredicate(String str, ArrayList v)
    {
	arguments = v;
	identifier = str;
    }

    // Accessing methods

    public String toString()
    {
	StringBuffer str = new StringBuffer();
	str.append(identifier);
	if (arguments.size() != 0) {
	  str.append("(");
	  Iterator e = arguments.iterator();
	  int i = arguments.size();
	  while (e.hasNext()) {
	    str.append((e.next()).toString());
	    i--;
	    if (i>0) { str.append(", ");}
	  }
	  str.append(")");
	}
	return str.toString();
    }

    public PropositionalTheoremProver asPropositionalSentence()
    {
	return null;
    }

    public ABTheoremProver asABPropositionalSentence()
    {
	return null;
    }


    public ArrayList allPredicates(String str, int i, ArrayList v)
    {
	if (identifier.equals(str) && (i == arguments.size())) {
	    v.add(this);
	}
	return v;
    }
}
