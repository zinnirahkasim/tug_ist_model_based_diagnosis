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

public class RepairOrderDAGNode extends DoubleLinkedDAGNode {

    Component comp;

    public RepairOrderDAGNode(Component comp) {
        this.comp = comp;
    }

    /*
     * Creates a clone of node.
     */
    public RepairOrderDAGNode(RepairOrderDAGNode node) {
        super(node);
        this.comp = node.comp;
    }

    public Object clone() {
        RepairOrderDAGNode node = new RepairOrderDAGNode(this);
        return node;
    }

    public String toString() {
        return comp.getName();
    }

    protected boolean invariant() {
        
        Iterator itParents = getParentsIterator();
        while (itParents.hasNext()) {
            RepairOrderDAGNode parentNode = (RepairOrderDAGNode)itParents.next();
            if (!comp.hasFDGParent(parentNode.comp)) return false;
        }

        return true;
    }

    public boolean equals(Object o) {
        if (!(o instanceof RepairOrderDAGNode)) return false;
        else {
            RepairOrderDAGNode otherNode = (RepairOrderDAGNode)o;
            boolean result = (comp.equals(otherNode.comp));
            assert(result == (comp.name.equals(otherNode.comp.name)));
            return result;
        }
    }
}
