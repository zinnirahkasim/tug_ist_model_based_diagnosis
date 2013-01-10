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
 * Assumption: Implements an assumption
 *
 * @version 0.1, DATE: 04.03.1999
 * @author Franz Wotawa
 *
 * This class implements an assumption used for
 * theorem proving and diagnosis.
 *
 * V0.1: Implementing the basic functionality (04.03.1999)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;    // Java util..

import theoremprover.*;

public class Assumption extends Object 
{

  // Instance variables ...

  public Object identifier;
  public boolean label;
  public ArrayList antecedence;

    // tags; can be used for any purpose
    public Object objTag;
    public int intTag;

  // Instance creation and initialization

  Assumption ()
  {
    super();
    identifier = null;
    antecedence = new ArrayList();
    label = false;
  }


    public int getIntTag() {
        return intTag;
    }

    public void setIntTag(int tag) {
        intTag = tag;
    }

    /*
    public int compareTo(Object o) {
        Assumption other = (Assumption)o;
        if (intTag < other.intTag) return -1;
        else if (intTag > other.intTag) return 1;
        else return 0;
        }*/


  public Object getIdentifier ()
  {
    return identifier;
  }

  public ArrayList getAntecedence ()
  {
    return antecedence;
  }

  public boolean getLabel()
  {
    return label;
  }

  public String toString()
  {
    if (identifier == null) {
      return new String("");
    } else {
      return identifier.toString();
    }
  }

  // Accessing and maintaining structure

  public void addAntecedence (PropositionalRule r)
  {
    antecedence.add(r);
  }

  public void setLabel(boolean l)
  {
    label = l;
  }

    public void setIdentifier(Object id)
    {
	identifier = id;
    }

    // Testing...

    public boolean isAssumption ()
    {
      return true;
    }

    // Theorem proving...

    public void propagateTrue()
    {
	Iterator e = antecedence.iterator();
	while (e.hasNext()) {
	    PropositionalRule r = (PropositionalRule)e.next();
	    r.counter(r.counter()-1);
	    if (r.counter() == 0) {
		PropositionInterface s = (PropositionInterface)r.succedence();
		s.addSupport(r);
		if (s.getLabel() != true) {
		    s.setLabel(true);
		    s.activeRule(r);
		    s.propagateTrue();
		}
	    }
	}
    }

    public void propagateFalse() {
	ArrayList v = new ArrayList();
	v = propagateFalse(v);
	Iterator ve = v.iterator();
	while (ve.hasNext()) {
	    PropositionInterface p = (PropositionInterface)ve.next();
	    p.correctLabels();
	}
    }


    public ArrayList propagateFalse(ArrayList v)
    {
	if (label == true) {
	    label = false;
	    Iterator e = antecedence.iterator();
	    while (e.hasNext()) {
		PropositionalRule r = (PropositionalRule)e.next();
		r.counter(r.counter()+1);
		PropositionInterface p = (PropositionInterface)r.succedence();
		p.removeSupport(r);
		if (p.activeRule() == r) {
		    v = p.propagateFalse(v);
		}
	    }
	}
	return v;
    }

    public void correctLabels()
    {
    }

    // Return the set of assumptions causing self to be true.
    public ArrayList collectAssumptions() {
	ArrayList result = new ArrayList();
	return collectAssumptions(result);
    }

    public ArrayList collectAssumptions(ArrayList assumptions) {
	if (! assumptions.contains(this)) {
	    assumptions.add(this);
	}
	return assumptions;
    }

    public TreeSet collectAllAssumptions() {
	TreeSet result = new TreeSet();
	return collectAllAssumptions(result);
    }

    public TreeSet collectAllAssumptions(TreeSet assumptions) {
	if (! assumptions.contains(this)) {
	    assumptions.add(this);
	}
	return assumptions;
    }

}
