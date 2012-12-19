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
 * ABTheoremProver: Implements a propositional theorem prover
 *                  extended by assumptions
 *
 * @version 0.1, DATE: 04.03.1999
 * @author Franz Wotawa
 *
 * This class implements a propositional theorem prover
 * for propositional Hornclauses extended by assumptions
 *
 * V0.1: Creating the basic functionality (04.03.1999)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.awt.*;     // User Interface components
import java.io.*;      // IO specific classes (File, Stream,..)
import java.awt.event.*;   // Event handling
import java.util.*;    // Java util

import theoremprover.*;


public class ABTheoremProver extends PropositionalTheoremProver
{

  // Instance variables ...

  protected ArrayList assumptions;

  // Instance creation and initialization

  public ABTheoremProver ()
  {
    assumptions = new ArrayList();
    rules = new ArrayList();
    facts = new ArrayList();
    propositions = new ArrayList();
    contradiction = new Proposition();
    contradiction.setIdentifier("false");
  }

  // Accessing

  public ArrayList getAssumptions ()
  {
    return assumptions;
  }

  // Accessing and maintaining structure

  public boolean addAssumption(Assumption a)
  {
      //System.out.println("add assumption: " + a.toString());
      assumptions.add(a);
      return true;
  }

  // Printing...

  public String toString()
  {
    StringBuffer str = new StringBuffer();
    Iterator e = assumptions.iterator();
    str.append("Assumptions:\r\n");
    while (e.hasNext()) {
      str.append((e.next()).toString());
      str.append("\r\n");
    }
    e = facts.iterator();
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
    public void add(ArrayList a, Proposition p) {
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
            Assumption ap = (Assumption)e.next();
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

    // Sets the assumption a to the passed value and propagates this value.
    public void setAssumption(Assumption a, boolean value) {

        if (value) {
            if (a.getLabel() != true) {
                a.setLabel(true);
                a.propagateTrue();
            } 
        } else {
            
            if(a.getLabel() != false) {
                ArrayList v = new ArrayList();
                v = a.propagateFalse(v); 
                Iterator ve = v.iterator();                                
                while (ve.hasNext()) {
                    Proposition p = (Proposition)ve.next();
                    p.correctLabels();
                }
            }

        }

    }

  // Checks consistency assuming that all assumptions are true.
  public boolean checkConsistency ()
  {
    Iterator e = assumptions.iterator();
    while (e.hasNext()) {
      Assumption a = (Assumption)e.next();
      setAssumption(a, true);
    }
    return isConsistent();
  }

    /* 
     * Checks consistency; assumptions in posAssumptions are set to true,
     * ass. in negAssumptions to false. 
     * This method assumes that these two lists contain all used assumptions; if
     * there is any assumption x which is in neither list, then the label of x remains unchanged.
     */ 
    public boolean checkConsistency(ArrayList posAssumptions, ArrayList negAssumptions) {

        Iterator e = posAssumptions.iterator();
        while (e.hasNext()) {
            Assumption a = (Assumption)e.next();
            setAssumption(a, true);
        }

        e = negAssumptions.iterator();
        while (e.hasNext()) {
            Assumption a = (Assumption)e.next();
            setAssumption(a, false);
        }
        
        return isConsistent();
    }

    /*
     * Checks consistency. 
     * posAssPrefixes contains strings; each assumption which starts with
     * one of the prefixes is assumed true; the other assumptions are assumed
     * false.
     * E.g., posAssPrefixes could contain a string "NAB".
     */
    public boolean checkConsistency(ArrayList posAssPrefixes) {

        Iterator itAss = assumptions.iterator();
        while (itAss.hasNext()) {
            Assumption a = (Assumption)itAss.next();
            String aStr = (String)a.getIdentifier();

            Iterator itPre = posAssPrefixes.iterator();
            boolean positive = false;

            while(itPre.hasNext()) {
                String prefix = (String)itPre.next();
                if (aStr.startsWith(prefix)) {
                    positive = true;
                    break;
                }
            }

            if (positive) setAssumption(a, true);
            else setAssumption(a, false);
        }

        return isConsistent();
    }

    /*
     * Returns a conflict set as an ArrayList of Assumption.
     */
    public ArrayList getConflictSet() {
        return contradiction.collectAssumptions();
    }
}
