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
 * LObject: Abstract class for representing logical sentences
 *
 * @version 0.1, DATE: 30.12.1998
 * @author Franz Wotawa
 *
 *
 * V0.1: Implementing the basic functionality (30.12.1998)
 */

package theoremprover;

import java.lang.*;
import java.util.*;
import theoremprover.*;

abstract class LObject extends Object
{
    // Printing methods

    public abstract String toString();

    // Conversion methods

    // Convert self to a propositional sentence
    public abstract PropositionalTheoremProver asPropositionalSentence();

  public PropositionalTheoremProver 
    asPropositionalSentence(
			    PropositionalTheoremProver tp, 
			    Hashtable pd)
  {
    return tp;
  }

  public ABTheoremProver 
    asABPropositionalSentence(
			    ABTheoremProver tp, 
			    Hashtable pd)
  {
    return tp;
  }

    public ArrayList allPredicates(String str, int i, ArrayList v)
    {
	return v;
    }
}

