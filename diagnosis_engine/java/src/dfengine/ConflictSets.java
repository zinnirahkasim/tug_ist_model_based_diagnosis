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


package dfengine;

import java.util.*;

import utils.*;



public class ConflictSets {

    protected ArrayList csets = new ArrayList();

    public void add(ConflictSet cs) {
        assert(cs.size() > 0);

        cs.subst(Mode.MODE_AB, Mode.MODE_IF);  // normalization: AB -> IF
        csets.add(cs);
    }

    public Iterator iterator() {
        return csets.iterator();
    }

    public ConflictSet conflictsWith(ModeAssignment ma) {
        Iterator itCS = csets.iterator();
        while (itCS.hasNext()) {
            ConflictSet cs = (ConflictSet)itCS.next();
            if (cs.conflictsWith(ma)) return cs;
        }

        return null;
    }

}
