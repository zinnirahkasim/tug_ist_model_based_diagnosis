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
 * PropositionInterface: The interface for propositions
 *
 * @version 1.0, DATE: 19.05.1999
 * @author Franz Wotawa
 *
 */

package theoremprover;

import java.lang.*;
import java.util.*;
import java.awt.*;

public interface PropositionInterface
{
  public void setLabel(boolean l);
  public boolean getLabel();
  public void propagateTrue();
  public ArrayList propagateFalse(ArrayList v);
  public void correctLabels();
  public void addSupport(PropositionalRule r);
  public void removeSupport(PropositionalRule r);
  public void activeRule(PropositionalRule r);
  public boolean updateActiveRule();
  public PropositionalRule activeRule();
}

