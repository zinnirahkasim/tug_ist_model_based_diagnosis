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



public class ConflictSet extends ModeAssignment {

    /*
     * This algorithm determines if this conflict set conflicts with ma, i.e.,
     * if "SDD |_| OBS |_| ma " must be inconsistent.
     *
     * Note: if a component c is not in ma, then NAB(c) is assumed to be implicitely in ma.
     *
     * Precondition: ma must not contain any "AB".
     */
    public boolean conflictsWith(ModeAssignment ma) {

        assert(!ma.hasMode(Mode.MODE_AB));

        Iterator itPairs = modes.values().iterator();
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            Component c = pair.mode.getComponent();
            Mode om = ma.getMode(c);
            if (om == null) om = c.getModeNAB();

            if ((om != pair.mode) && !(((pair.mode.getType() == Mode.MODE_IF)) && (om.getType() == Mode.MODE_DF))) {
                return false;
            }
        }

        //System.out.println("CS conflicts with mode assignment! CS: [" + this.toStringShort() + "];  MA: [" + ma.toStringShort() + "]");

        return true;
    }

}
