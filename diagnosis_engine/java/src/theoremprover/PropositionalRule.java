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
 * PropositionalRule: Implements a propositional rule
 *
 * @version 0.1, DATE: 19.01.1999
 * @author Franz Wotawa
 *
 * This class implements a rule used for
 * theorem proving.
 *
 * V0.1: Creating the basic functionality (19.01.1999)
 */

package theoremprover;

import java.lang.*;    // Java language classes 
import java.util.*;    // Java util...

import theoremprover.*;


public class PropositionalRule extends Object
{

  // Instance variables ...

  protected ArrayList antecedence;
  protected Object succedence;
  protected int counter;

  // Instance creation and initialization

  PropositionalRule ()
  {
    antecedence = new ArrayList();
    succedence = null;
    counter = 0;
  }

  // Accessing

  public String toString()
  {
    StringBuffer str = new StringBuffer();
    Iterator e = antecedence.iterator();
    int i = antecedence.size();
    while (e.hasNext()) {
      str.append((e.next()).toString());
      i--;
      if (i>0) { str.append(", ");}
    }
    str.append(" -> ");
    str.append(succedence.toString());
    str.append(".");
    return str.toString();
  }

  // Accessing and maintaining structure

  public void addToAntecedence(Object p)
  {
    antecedence.add(p);
  }

  public ArrayList antecedence()
  {
    return antecedence;
  }

  public void counter(int i)
  {
    counter = i;
  }

  public int counter()
  {
    return counter;
  }

  public void succedence(Object p)
  {
    succedence = p;
  }

  public Object succedence()
  {
    return succedence;
  }
}
