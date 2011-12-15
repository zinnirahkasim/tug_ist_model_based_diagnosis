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
 * LSentence: Implements an object representing a logical sentence
 *
 * @version 0.1, DATE: 230.12.1998
 * @author Franz Wotawa
 *
 * This class is used for storing the information of a logical
 * sentence. It is not intended to be used for implementing
 * logical operations or consistency checks. To do this convert
 * my instances to a more appropriate format.
 * 
 * V0.1: Creating the basic functionality (30.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;
import theoremprover.*;


public class LSentence extends LObject
{
    // Instance variables

    public ArrayList rules;

    // Instance creation and initialization

    public LSentence()
    {
	rules = new ArrayList();
    }

    public LSentence(ArrayList v)
        {
	rules = v;
    }

    public void addRules(ArrayList newRules) {
        this.rules.addAll(newRules);
    }

    public void addRules(LSentence new_sentence) {
        this.rules.addAll(new_sentence.rules);
    }

    // Accessing methods

    public String toString()
    {
	StringBuffer str = new StringBuffer();
	Iterator e = rules.iterator();
	while (e.hasNext()) {
	    str.append((e.next()).toString());
	    str.append("\n\r");
	}
	return str.toString();
    }
  
  public PropositionalTheoremProver asPropositionalSentence() 
  { 
    PropositionalTheoremProver tp = new PropositionalTheoremProver();
    Hashtable pd = new Hashtable();

    Iterator e = rules.iterator();
    while (e.hasNext()) {
      ((theoremprover.LObject)(e.next())).asPropositionalSentence(tp,pd);
    }
    return tp;
  }


  public ABTheoremProver asABPropositionalSentence(ABTheoremProver tp) 
  { 
    Hashtable pd = new Hashtable();

    Iterator e = rules.iterator();
    while (e.hasNext()) {
      if (((theoremprover.LObject)(e.next())).asABPropositionalSentence(tp,pd) == null) {
	return null;
      }
    }
    return tp;
  }


    // Returns a list of predicates of the form str(X)
    public ArrayList allPredicates(String str)
    {
	return allPredicates(str,1);
    }

    // Returns a list of predicates of the form str(X1,..,Xi)
    public ArrayList allPredicates(String str, int i)
    {
	ArrayList v = new ArrayList();
	Iterator e = rules.iterator();
	while (e.hasNext()) {
	    v = ((theoremprover.LObject)(e.next())).allPredicates(str,i,v);
	}
	return v;
    }

}
