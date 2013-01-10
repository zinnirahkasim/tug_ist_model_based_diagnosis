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

public class ModeAssignmentDAGNode extends DoubleLinkedDAGNode {

    Component comp;

    public ModeAssignmentDAGNode(Component comp) {
        this.comp = comp;
    }

    /*
     * Creates a clone of node.
     */
    public ModeAssignmentDAGNode(ModeAssignmentDAGNode node) {
        super(node);
        this.comp = node.comp;
    }

    public Object clone() {
        ModeAssignmentDAGNode node = new ModeAssignmentDAGNode(this);
        return node;
    }

    public Component getFailurePred() {

        if (parents.size() == 0) return null;
        else return ((ModeAssignmentDAGNode)getFirstParent()).comp;

    }

    public String toString() {
        return comp.getName();
    }

    protected boolean invariant() {
        if (parents.size() > 1) return false;
        
        Component parent = getFailurePred();
        if (parent != null) {
            if (!comp.hasFDGParent(parent)) return false;
        }

        return true;
    }
    
}
