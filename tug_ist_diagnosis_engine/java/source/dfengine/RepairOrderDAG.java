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

public class RepairOrderDAG extends DoubleLinkedDAG {

    /*
     * Map from Component to RepairOrderNode.
     */
    protected TreeMap components = new TreeMap();

    protected GraphMatrix transClosure = null;


    public RepairOrderDAG(ModeAssignment ma) {
        createFromModeAssignment(ma);

        assert(invariant());
    }

    /*
     * Util method, internally used.
     */
    protected void addNode(Component comp) {
        assert(invariant());

        RepairOrderDAGNode newNode = new RepairOrderDAGNode(comp);
        super.addNode(newNode);
        components.put(comp, newNode);

        assert(invariant());
    }

    public RepairOrderDAGNode getNode(Component comp) {
        Object o = components.get(comp);
        if (o == null) return null;
        else return (RepairOrderDAGNode)o;
    }

    public void addEdge(Component from, Component to) {

        assert(invariant());

        RepairOrderDAGNode fromNode = getNode(from);
        RepairOrderDAGNode toNode = getNode(to);
        
        addEdge(fromNode, toNode);
        updateTransClosureForNewEdge(getTransitiveClosure(), fromNode, toNode, false);
        
        // postcondition: assert that updated transClosure conforms to DAG
        assert(hasEdge(from, to));
        assert(correctTransitiveClosure(getTransitiveClosure(), false));
        // invariant
        assert(invariant());
    }

    public boolean hasEdge(Component from, Component to) {
        return hasEdge(getNode(from), getNode(to));
    }

    protected void createFromModeAssignment(ModeAssignment ma) {
        
        System.out.println("createFromModeAssignment(): ma = " + ma.toStringShort());

        // create nodes

        Iterator itModes = ma.iterator();
        while (itModes.hasNext()) {
            Mode mode = (Mode)itModes.next();
            addNode(mode.component);           
        }

        // create edges

        itModes = ma.iterator();
        while (itModes.hasNext()) {
            Mode mode = (Mode)itModes.next();
            if (mode.type == Mode.MODE_DF) {
                RepairOrderDAGNode node = getNode(mode.component);
                RepairOrderDAGNode parentNode = getNode(mode.parent);
                addEdge(parentNode, node);
            }
        }

        transClosure = null;

        System.out.println("createFromModeAssignment() results in: " + this);
    }

    protected boolean invariant() {

        // is transitive closure correct?
        if ((transClosure != null) && !correctTransitiveClosure(transClosure, false)) {
            System.err.println("ERROR: invariant of RepairOrderDAG violated: incorrect transitive closure!");
            return false;
        }

        // check if invariants of all nodes hold
        Iterator itNodes = iterator();
        while (itNodes.hasNext()) {
            RepairOrderDAGNode n = (RepairOrderDAGNode)itNodes.next();
            if (!n.invariant()) {
                System.err.println("ERROR: invariant of RepairOrderDAG violated: invariant of a node does not hold!");
                return false;
            }
        }

        // check if there is more than one node for a specific component (must not happen)
        if (!uniqueComponents()) {
            System.err.println("ERROR: invariant of RepairOrderDAG violated: components are not unique!");
            return false;
        }

        return true;
    }

    public GraphMatrix getTransitiveClosure() {
        if (transClosure == null) transClosure = computeTransitiveClosure(false);
        return transClosure;
    }

    public boolean equalComponents(Set comps) {
        
        return (components.keySet().equals(comps));

    }

    // Returns true iff there is at most one node for each component. For test purposes.
    protected boolean uniqueComponents() {
        for (int i = 0; i < nodes.size(); ++i) {
            for (int j = i + 1; j < nodes.size(); ++j) {
                if (nodes.get(i).equals(nodes.get(j))) return false;
            }
        }
        return true;
    }

    public String toString() {
        if (!uniqueComponents()) {
            System.err.println("ERROR: invariant of RepairOrderDAG violated: components are not unique!");
        }

        StringBuffer result = new StringBuffer();
        ToStringVisitor visitor = new ToStringVisitor(result);
        visitRoots(visitor, true);
        return result.toString();
    }

}


/*
 * For each visited node n: iterate through parents n_p and
 * add strings of format " " to the StringBuffer which is passed
 * to the constructor.
 */
class ToStringVisitor extends DoubleLinkedDAGVisitor {

    StringBuffer str;

    public ToStringVisitor(StringBuffer str) {
        this.str = str;
    }

    public boolean wantMoreNodes() {
        return true;
    }

    public void visit(DoubleLinkedDAGNode node) {
        Iterator itParents = node.getParentsIterator();
        if (!itParents.hasNext()) {
            if (str.length() > 0) str.append("; ");
            str.append("IF(" + node.toString() + ")");
        }
        while (itParents.hasNext()) {
            DoubleLinkedDAGNode parentNode = (DoubleLinkedDAGNode)itParents.next();
            if (str.length() > 0) str.append("; ");
            str.append("DF(" + parentNode.toString() + ", " + node.toString() + ")");
        }
    }

    public String getStr() {
        return str.toString();
    }

}
