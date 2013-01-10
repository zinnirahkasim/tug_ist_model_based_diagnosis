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
 * PropositionalTheoremProver: Implements a propositional theorem prover
 *
 * @version 0.1, DATE: 19.01.1999
 * @author Franz Wotawa
 *
 * This class implements a propositional theorem prover
 * for propositional Hornclauses. 
 *
 * V0.1: Creating the basic functionality (19.01.1999)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling
import java.util.*;    // Java util

import theoremprover.*;


public class PropositionalTheoremProver extends Object
{

  // Instance variables ...

  protected ArrayList rules;
  protected ArrayList facts;
  protected ArrayList propositions;
  protected Proposition contradiction;

  // Instance creation and initialization

  PropositionalTheoremProver ()
  {
    rules = new ArrayList();


    facts = new ArrayList();
    propositions = new ArrayList();
    contradiction = new Proposition();
    contradiction.setIdentifier("false");
  }

  // Accessing

  public ArrayList rules ()
  {
    return rules;
  }

  public ArrayList facts ()
  {
    return facts;
  }

  public ArrayList propositions ()
  {
    return propositions;
  }

  public Proposition contradiction ()
  {
    return contradiction;
  }

  // Accessing and maintaining structure

  public void addRule(PropositionalRule r)
  {
    rules.add(r);
  }

  public void addFact(PropositionalRule r)
  {
    facts.add(r);
  }

  public void addProposition(Proposition p)
  {
    propositions.add(p);
  }

  // Testing

  public boolean isConsistent ()
  {
    return (contradiction.getLabel() == false);
  }


  // Printing...

  public String toString()
  {
    StringBuffer str = new StringBuffer();
    Iterator e = facts.iterator();
    str.append("Facts:\r\n");
    while (e.hasNext()) {
      str.append((e.next()).toString());
      str.append("\r\n");
    }
    e = rules.iterator();
    str.append("Rules:\r\n");
    while (e.hasNext()) {
      str.append((e.next()).toString());
      str.append("\r\n");
    }
    return str.toString();
  }

  // Public accessing

  // Add a new rule or fact to the theorem prover and check consistency
  // It is assumed that all propositions used are added to my
  // list of propositions before calling the add method.
  public void add(ArrayList a, Proposition p)
  {
    PropositionalRule newRule = new PropositionalRule();
    int i = a.size();

    if(i > 0) {
      addRule(newRule);
    } else {
      addFact(newRule);
    }
    p.addSuccedence(newRule);
    newRule.succedence(p);
    Iterator e = a.iterator();
    while (e.hasNext()) {
      Proposition ap = (Proposition)e.next();
      ap.addAntecedence(newRule);
      newRule.addToAntecedence(ap);
      if (ap.getLabel() == true) {
	i = i - 1;
      }
    }
    newRule.counter(i);
    if (i == 0) {
      if (p.getLabel() != true) {
	p.setLabel(true);
	p.activeRule(newRule);
	// Propagate truth value
	p.propagateTrue();
      }
    }
  }


  // Remove rule or fact requires a recomputation of the truth values
  public void remove (PropositionalRule r)
  {
    // Remove rule or fact from my collections...
    if ((r.antecedence()).size() == 0) {
      facts.remove(r);
    } else {
      rules.remove(r);
    }

    // Remove related links...
    Proposition s = (Proposition)r.succedence();
    (s.succedence()).remove(r);
    s.removeSupport(r);
    Iterator e = (r.antecedence()).iterator();
    while (e.hasNext()) {
      Assumption a = (Assumption)e.next();
      (a.getAntecedence()).remove(r);
    }

    // Change truth values if necessary...
    if (s.activeRule() == r) {
      ArrayList v = new ArrayList();
      v = s.propagateFalse(v);
      Iterator ve = v.iterator();
      while (ve.hasNext()) {
	Proposition p = (Proposition)ve.next();
	p.correctLabels();
      }
    }
  }

  // Checks consistency 
  public boolean checkConsistency ()
  {
    return isConsistent();
  }

}
