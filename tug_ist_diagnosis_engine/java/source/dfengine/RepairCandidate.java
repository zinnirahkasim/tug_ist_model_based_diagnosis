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

public class RepairCandidate  {

    protected RepairOrderDAG order;


    public RepairCandidate(ModeAssignment ma) {
        order = new RepairOrderDAG(ma);
    }

    public int size() {
        return order.getNumNodes();
    }

    public String toString() {
        return order.toString();
    }

    boolean equalComponents(ModeAssignment ma) {
        
        if (ma.size() != order.getNumNodes()) return false;
        else {
            return order.equalComponents(ma.getComponents());
        }

    }

    /*
     * Returns true iff the ordering of components in this candidate
     * implies the ordering of components in ma.
     * In other words: iff the order in this candidate is equal to or stronger
     * than the order in ma.
     *
     * Complexity: provided that the transitive closure of this candidate is already computed:
     *   O( (log m)^2 * m )  [m...number of modes in ma]
     */
    boolean impliesOrderOf(ModeAssignment ma) {
        
        // precondition
        assert(equalComponents(ma));

        boolean result = true;

        // iterate through edges in ma, for each edge c_i -> c_j:
        // if c_i is not an ancestor of c_j in this candidate, then
        // return false

        Iterator itDFModes = ma.getModeIterator(Mode.MODE_DF);
        while (itDFModes.hasNext() && result) {  // O(m)
            Mode dfMode = (Mode)itDFModes.next();
            Component comp = dfMode.component;
            Component parent = dfMode.parent;

            RepairOrderDAGNode n_comp = order.getNode(comp);  // O(log m)
            RepairOrderDAGNode n_parent = order.getNode(parent);  // O(log m)
            GraphMatrix transClosure = order.getTransitiveClosure();
            if (!order.isAncestor(n_parent, n_comp, transClosure)) result = false;
        }

        return result;
    }

    void mergeWith(ModeAssignment ma) {
        
        System.out.println("MERGE: " + order + "  WITH " + ma.toStringShort());

        // precondition
        assert(equalComponents(ma));

        Iterator itDFModes = ma.getModeIterator(Mode.MODE_DF);
        while (itDFModes.hasNext()) {
            Mode dfMode = (Mode)itDFModes.next();
            Component comp = dfMode.component;
            Component parent = dfMode.parent;

            if (!order.hasEdge(parent, comp)) {
                order.addEdge(parent, comp);
            }   
        }

        System.out.println("MERGE results in: " + order);

        // postcondition
        assert(impliesOrderOf(ma));
    }

}
