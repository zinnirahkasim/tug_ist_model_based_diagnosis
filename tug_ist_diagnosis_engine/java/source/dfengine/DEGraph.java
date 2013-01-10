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


public class DEGraph extends DoubleLinkedDAG {

    ArrayList rootNodes = new ArrayList();

    TreeSet unexpandedAlphaNodes = new TreeSet();


    protected String PRINT_OFFSET = "    ";
    

    public Object clone() {
        throw new UnsupportedOperationException("class DEGraph does not support clone()");
    }
    
    public String toStringShort() {
        
        StringBuffer result = new StringBuffer();

        Iterator itRootNodes = rootNodes.iterator();
        while (itRootNodes.hasNext()) {
            DENode rootNode = (DENode)itRootNodes.next();
            System.out.println("DEGraph.toStringShort(): root node: " + rootNode.toStringShort()
                + ";  num children: " + rootNode.getChildren().size());
            printSubgraph(result, rootNode, 0);
        }

        return result.toString();
    }

    // util method for toStringShort()
    protected void printSubgraph(StringBuffer buf, DENode n, int depth) {
        
        for (int i = 0; i < depth; ++i) {
            buf.append(PRINT_OFFSET);
        }

        buf.append(n.toStringShort());
        buf.append('\n');

        Iterator itChildren = n.getChildrenIterator();
        while (itChildren.hasNext()) {
            DENode child = (DENode)itChildren.next();
            printSubgraph(buf, child, depth + 1);
        }
    }

    public void addRootNode(DENode newRootNode) {

        assert(invariant());
        
        addNode(newRootNode);
        rootNodes.add(newRootNode);

        assert(invariant());
    }

    public void addChildNode(DENode n, DENode child) {
        assert(invariant());  // inv
        assert((n.getID() >= 0) && hasNode(n.getID()));  // precond.

        addNode(child);
        addEdge(n, child);

        assert(invariant());
    }

    public TreeSet getUnexpandedAlphaNodes() {
        return unexpandedAlphaNodes;
    }

    /*
     * If a node with a mode assignment equal to ma already exists: return this node.
     *
     * Return null otherwise. 
     * Children of exclRootNode are NOT considered in the search!
     */
    public DENode alreadyExists(ModeAssignment ma, DENode exclRootNode) {
        
        Iterator itRootNodes = rootNodes.iterator();
        while (itRootNodes.hasNext()) {
            DENode rootN = (DENode)itRootNodes.next();
            if (rootN != exclRootNode) {
                SearchMAVisitor visitor = new SearchMAVisitor(ma);
                visitChildrenPreOrder(rootN, visitor, true);
                
                if (visitor.foundNode != null) {
                    return visitor.foundNode;
                }
            }
        }
        
        return null;
    }

    /*
     * Returns true iff all the MAs in the DEG are unique.
     */
    protected boolean checkUniqueModesHypothesis() {
        
        for (int i = nodes.size() - 1; i >= 0; --i) {
            DENode n = (DENode)nodes.get(i);
            
            for (int j = i - 1; j >= 0; --j) {
                DENode n1 = (DENode)nodes.get(j);
                if (n.getModeAssignment().equals(n1.getModeAssignment())) {
                    
                    System.out.println("WARNING: mode assignments not unique: ["
                                       + n.getModeAssignment().toStringShort()
                                       + "]  [" + n1.getModeAssignment().toStringShort()
                                       + "]");
                    return false;
                }
            }
        }

        return true;
    }

    protected boolean invariant() {

        // for all nodes n in unexpandedAlphaNodes: n.hasChildren() must be false,
        // and n.expanded(alpha) must also be false.

        Iterator unIt = unexpandedAlphaNodes.iterator();
        while (unIt.hasNext()) {
            DENode n = (DENode)unIt.next();
            if (!n.expanded(DENode.TYPE_BETA) && n.hasChildren()) return false;

            if (n.expanded(DENode.TYPE_ALPHA)) return false;
        }

        // check invariant of all nodes

        Iterator itNodes = iterator();
        while (itNodes.hasNext()) {
            DENode n = (DENode)itNodes.next();
            if (!n.invariant()) return false;
        }

        // ensure that mode assignments are unique in the graph
        if (!checkUniqueModesHypothesis()) return false;

        return true;
    }

    /*
     * Marks n and all of its descendants as inconsistent
     */
    public void propagateInconsToDesc(DENode n) {
        assert(invariant());
        
        MarkInconsistentVisitor visitor = new MarkInconsistentVisitor();
        visitChildrenPreOrder(n, visitor, true);
        assert((n.consistent() == DENode.STATUS_FALSE) && n.descInconsistent());

        assert(invariant());
    }


}

/*
 * Searches a mode assignment. If found: returned in "foundNode".
 */
class SearchMAVisitor extends DoubleLinkedDAGVisitor {

    ModeAssignment ma;

    DENode foundNode = null;

    public SearchMAVisitor(ModeAssignment ma) {
        this.ma = ma;
    }

    public boolean wantMoreNodes() {
        return (foundNode == null);
    }

    public void visit(DoubleLinkedDAGNode n) {

        DENode den = (DENode)n;
        if (ma.equals(den.getModeAssignment())) {
            foundNode = den;
        }
    }

}

/*
 * Marks nodes as "inconsistent".
 */
class MarkInconsistentVisitor extends DoubleLinkedDAGVisitor {

    public boolean wantMoreNodes() {
        return true;
    }

    public void visit(DoubleLinkedDAGNode n) {
        DENode den = (DENode)n;
        den.setDescInconsistent();
        den.setConsistent(DENode.STATUS_FALSE);
    }
}
