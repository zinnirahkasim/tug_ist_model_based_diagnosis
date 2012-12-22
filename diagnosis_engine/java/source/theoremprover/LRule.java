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
 * LRule: Implements an object representing a logical rule
 *
 * @version 0.1, DATE: 230.12.1998
 * @author Franz Wotawa
 *
 * This class is used for storing the information of a logical
 * rule. It is not intended to be used for implementing
 * logical operations or consistency checks. To do this convert
 * my instances to a more appropriate format.
 * 
 * V0.1: Creating the basic functionality (30.12.1998)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;
import theoremprover.*;


public class LRule extends LObject
{
    // Instance variables

    public ArrayList tail;
    public LObject head;

    // Instance creation and initialization

    LRule()
    {
	tail = new ArrayList();
	head = null;
    }

    LRule(ArrayList v, LObject h)
    {
	tail = v;
	head = h;
    }

    // Accessing methods

    public String toString()
    {
	StringBuffer str = new StringBuffer();
	Iterator e = tail.iterator();
	int i = tail.size();
	while (e.hasNext()) {
	    str.append((e.next()).toString());
	    i--;
	    if (i>0) { str.append(", ");}
	}
	str.append(" -> ");
	str.append(head.toString());
	str.append(".");
	return str.toString();
    }

  public PropositionalTheoremProver asPropositionalSentence()
  {
    return null;
  }


  public PropositionalTheoremProver 
    asPropositionalSentence(
			    PropositionalTheoremProver tp, 
			    Hashtable pd)
  {
    String str;
    Proposition p;
    ArrayList ant = new ArrayList();
    Iterator e = tail.iterator();

    while (e.hasNext()) {
      str = (e.next()).toString();
      if (pd.containsKey(str)) {
	p = (Proposition)pd.get(str);
      } else {
	p = new Proposition();
	p.setIdentifier(str);
	pd.put(str,p);
	tp.addProposition(p);
      }
      ant.add(p);
    }
    str = head.toString();
    if (str.equalsIgnoreCase("false")) {
      p = tp.contradiction();
    } else if (pd.containsKey(str)) {
      p = (Proposition)pd.get(str);
    } else {
      p = new Proposition();
      p.setIdentifier(str);
      pd.put(str,p);
      tp.addProposition(p);
    }
    tp.add(ant,p);
    return tp;
  }


  public ABTheoremProver asABPropositionalSentence()
  {
    return null;
  }


  public ABTheoremProver 
    asABPropositionalSentence(
			    ABTheoremProver tp, 
			    Hashtable pd)
  {
    String str;
    Assumption p;
    ArrayList ant = new ArrayList();
    Iterator e = tail.iterator();

    while (e.hasNext()) {
      str = (e.next()).toString();
      if (pd.containsKey(str)) {
	p = (Assumption)pd.get(str);
      } else {
	if ((str.length() > 0) &&
            ((Character.isUpperCase(str.charAt(0))) || (str.charAt(0) == '!'))) {
	    
            p = new Assumption();
            p.setIdentifier(str);
	    if (!tp.addAssumption(p)) return null;
	    pd.put(str,p);
	} else {
	  p = new Proposition();
	  tp.addProposition((Proposition)p);	
	  p.setIdentifier(str);
	  pd.put(str,(Proposition)p);
	}
      }
      ant.add(p);
    }
    str = head.toString();

    if ((str.length() > 0) && (Character.isUpperCase(str.charAt(0)))) {
      return null; // No assumption is allowed as succedence
    }

    if (str.equalsIgnoreCase("false")) {
      p = tp.contradiction();
    } else if (pd.containsKey(str)) {
      p = (Proposition)pd.get(str);
    } else {
      p = new Proposition();
      p.setIdentifier(str);
      pd.put(str,(Proposition)p);
      tp.addProposition((Proposition)p);
    }
    tp.add(ant,(Proposition)p);
    return tp;
  }


    public ArrayList allPredicates(String str, int i, ArrayList v)
    {
	Iterator e = tail.iterator();
	while (e.hasNext()) {
	    v = ((LPredicate)(e.next())).allPredicates(str,i,v);
	}
	v = head.allPredicates(str,i,v);
	return v;
    }
}
