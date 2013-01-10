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
 * Proposition: Implements a proposition
 *
 * @version 0.1, DATE: 19.01.1999
 * @author Franz Wotawa
 *
 * This class implements a proposition used for
 * theorem proving.
 *
 * V0.1: Creating the basic functionality (19.01.1999)
 * V0.2: Changing the structure (04.03.1999)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;    // Java util..

import theoremprover.*;

public class Proposition extends Assumption
implements PropositionInterface
{

  // Instance variables ...

  protected ArrayList succedence;
  protected PropositionalRule activeRule;
  protected ArrayList support;

  // Instance creation and initialization

  Proposition ()
  {
    super();
    identifier = null;
    succedence = new ArrayList();
    antecedence = new ArrayList();
    label = false;
    activeRule = null;
    support = new ArrayList();
  }

  // Accessing

  public ArrayList succedence ()
  {
    return succedence;
  }

  public PropositionalRule activeRule ()
  {
    return activeRule;
  }

  public ArrayList support ()
  {
    return support;
  }


  // Accessing and maintaining structure

  public void addSuccedence(PropositionalRule r)
  {
    succedence.add(r);
  }

  public void activeRule(PropositionalRule r)
  {
    activeRule = r;
  }

  public void addSupport (PropositionalRule r)
  {
    support.add(r);
  }

  public void removeSupport (PropositionalRule r)
  {
    support.remove(r);
  }

    public boolean updateActiveRule() {
        if (support.size() > 0) {
            activeRule = (PropositionalRule)support.get(0);
            return true;
        } else {
            return false;
        }
    }

  // Testing...

  public boolean isAssumption ()
  {
    return false;
  }

  // Theorem proving...

  public ArrayList propagateFalse(ArrayList v)
  {
    if (label == true) {
      label = false;
      activeRule = null;
      v.add(this);
      Iterator e = antecedence.iterator();
      while (e.hasNext()) {
	PropositionalRule r = (PropositionalRule)e.next();
	r.counter(r.counter()+1);
	Proposition p = (Proposition)r.succedence();
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
      if (label == false) {
          if (support.size() > 0) {
              PropositionalRule r = (PropositionalRule)support.get(0);
              activeRule = r;
              label = true;
              propagateTrue();
          }
      }
  }

    // Return the set of assumptions causing self to be true.
    public ArrayList collectAssumptions() {
	ArrayList result = new ArrayList();
	if (label == true) {
	    return collectAssumptions(result);
	} else {
	    return result;
	}
    }

    public ArrayList collectAssumptions(ArrayList assumptions) {
	if (activeRule != null) {
	    Iterator e = (activeRule.antecedence()).iterator();
	    while (e.hasNext()) {
		((Assumption)e.next()).collectAssumptions(assumptions);
	    }
	}
	return assumptions;
    }

    public TreeSet collectAllAssumptions() {
        TreeSet result = new TreeSet();
	if (label == true) {
	    return collectAllAssumptions(result);
	} else {
	    return result;
	}
    }

    public TreeSet collectAllAssumptions(TreeSet assumptions) {
	
        Iterator itRule = support.iterator();
        while(itRule.hasNext()) {
            PropositionalRule r = (PropositionalRule)itRule.next();
            Iterator e = (r.antecedence()).iterator();
	    while (e.hasNext()) {
		((Assumption)e.next()).collectAllAssumptions(assumptions);
	    }     
        }

	return assumptions;
    }
}
