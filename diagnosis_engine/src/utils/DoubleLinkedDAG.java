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


package utils;

import java.util.*;

public class DoubleLinkedDAG implements Cloneable {

    /*
     * Marks used for cycle detection (see method findCycle()).
     */
    protected static int MARK_NOT_VISITED = 0;
    protected static int MARK_BEING_VISITED = 1;
    protected static int MARK_DONE_VISITED = 2;

    protected ArrayList nodes = new ArrayList();

    protected int numEdges = 0;

    /*
     * Used by findCycle(); each entry is one of the MARK_X constants (e.g., MARK_VISITED).
     * Entry i is the visit-status of the node whose id is equal to i.
     */
    protected int[] nodeMarks;

    /*
     * Used by the methods implementing the Visitor-pattern: bit i denotes whether the node with id = i
     * has already been visited.
     */
    protected BitSet nodesVisited;


    /*
     * If a transitive closure is computed (method computeTransitiveClosure()) and
     * paramter \a computeMinMaxDist is \c true, then the closure (class GraphMatrix) contains
     * tags which are of type MinMaxDistance.
     */
    public class MinMaxDistance {
        public int minDist = -1;
        public int maxDist = -1;

        public MinMaxDistance(int minDist, int maxDist) {
            this.minDist = minDist;
            this.maxDist = maxDist;
        }

        public String toString() {
            return "min: " + minDist + "; max: " + maxDist;
        }
    }

    /*
     * Creates an empty DAG.
     */
    public DoubleLinkedDAG() {
    }

    /*
     * The new DAG is a (deep) clone of dag.
     */
    public DoubleLinkedDAG(DoubleLinkedDAG dag) {
        numEdges = dag.numEdges;
        nodes.ensureCapacity(dag.nodes.size());

        Iterator itNodes = dag.nodes.iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();
            DoubleLinkedDAGNode clonedN = (DoubleLinkedDAGNode)n.clone();
            clonedN.allNodes = nodes;
            nodes.add(clonedN);
        }
    }

    /*
     * Returns a (deep) clone of this DAG.
     */
    public Object clone() {
        DoubleLinkedDAG clonedDAG = new DoubleLinkedDAG(this);
        return clonedDAG;
    }

    /**
     * Add the node to the DAG and automaticall assigns an integer ID to the node.
     *
     * Returns the node ID.
     */
    public int addNode(DoubleLinkedDAGNode node) {
        assert(node != null);

        nodes.add(node);
        node.id = nodes.size() - 1;
        node.allNodes = nodes;

        return node.id;
    }

    public boolean hasNode(int id) {
        assert(id >= 0);

        return (id < nodes.size());
    }

    /**
     * Checks if \a node is in the DAG.
     *
     * Compares the reference of \a node with the nodes added to this DAG. I.e., the method
     * \c DoubleLinkedDAGNode.equals() is not used.
     */
    public boolean hasNode(DoubleLinkedDAGNode node) {
        Iterator itNodes = nodes.iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();
            if (node == n) return true;
        }
        return false;
    }

    /**
     * Adds a new edge to the DAG.
     *
     * Precondition: both nodes must already exist in the DAG, but there must not
     * be an edge from "from" to "to".
     */
    public void addEdge(DoubleLinkedDAGNode from, DoubleLinkedDAGNode to) {
        
        // precondition
        assert(nodes.contains(from) && nodes.contains(to));
        assert(!from.getChildren().contains(to.id) && !to.getParents().contains(from.id));
        
        from.getChildren().addSorted(to.id);
        to.getParents().addSorted(from.id);

        ++numEdges;
    }

    public boolean hasEdge(DoubleLinkedDAGNode from, DoubleLinkedDAGNode to) {
        // precondition
        assert(nodes.contains(from) && nodes.contains(to));

        return (from.getChildren().contains(to.id));
    }
    
    public void removeEdge(DoubleLinkedDAGNode from, DoubleLinkedDAGNode to) {
        assert(hasEdge(from, to));

        from.getChildren().remove(to.id);
        to.getParents().remove(from.id);

        --numEdges;
    }

    public ArrayList getNodes() {
        return nodes;
    }

    public int getNumNodes() {
        return nodes.size();
    }

    public int getNumEdges() {
        return numEdges;
    }

    public String toString() {
        return toString(null);
    }

    /*
     * Similar to toString(), but the tags in graphMatrix
     * are also included (using the toString() method of the tags).
     * graphMatrix must conform to this DAG!
     *
     * graphMatrix may be null; in this case the result is equal to the
     * result returned by toString(). 
     */
    public String toString(GraphMatrix graphMatrix) {
        StringBuffer result = new StringBuffer();

        Iterator itNodes = iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();
            result.append("node: ");
            result.append(n.toString());
            result.append(";  PARENTS: ");
            
            Iterator itParents = n.getParentsIterator();
            while (itParents.hasNext()) {
                DoubleLinkedDAGNode parent = (DoubleLinkedDAGNode)itParents.next();
                result.append(parent.toString());
                if (graphMatrix != null) {
                    Object tag = graphMatrix.getTag(parent.id, n.id);
                    if (tag != null) {
                        result.append("(");
                        result.append(tag.toString());
                        result.append(")");
                    }
                }
                result.append("; ");
            }

            result.append(";  CHILDREN: ");
            Iterator itChildren = n.getChildrenIterator();
            while (itChildren.hasNext()) {
                DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                result.append(child.toString());
                if (graphMatrix != null) {
                    Object tag = graphMatrix.getTag(n.id, child.id);
                    if (tag != null) {
                        result.append("(");
                        result.append(tag.toString());
                        result.append(")");
                    }
                }
                result.append("; ");
            }
        
            result.append("\n");
        }

        return result.toString();
    }

    public String toStringShort() {
        
        StringBuffer result = new StringBuffer();
        AddNodeAsStringVisitor visitor = new AddNodeAsStringVisitor(result);
        visitRoots(visitor, false);
        return result.toString();        
    }

    /*
     * The iteration order is ascendent wrt the ID of the nodes.
     *
     * Thus, the iteration order has nothing to do with the structure of the
     * graph.
     */
    public Iterator iterator() {
        return nodes.iterator();
    }

    /*
     * \c index must be in the range 0..getNumNodes() - 1.
     */
    public final DoubleLinkedDAGNode getNode(int index) {
        return (DoubleLinkedDAGNode)nodes.get(index);
    }

    /*
     * The visitor recursively visits this node and all of its descendants in the DAG in the
     * following order: visit(this node) - visit(first child) - .. - visit(last child) 
     *
     * IMPORTANT: if the subgraph with "node" as root has diamond structures, then it 
     * could happen that nodes in this subgraph are visited multiple times. If 
     * visitOnlyOnce is true, then it is guaranteed that each child is visited only once.
     */
    public void visitChildrenPreOrder(DoubleLinkedDAGNode node,
                                      DoubleLinkedDAGVisitor visitor, boolean visitOnlyOnce) {

        if (visitOnlyOnce) nodesVisited = new BitSet(nodes.size());
        visitChildrenPreOrder_Recursive(node, visitor, visitOnlyOnce);
        nodesVisited = null;
    }

    public void visitChildrenPreOrder_Recursive(DoubleLinkedDAGNode node,
                                                DoubleLinkedDAGVisitor visitor, boolean visitOnlyOnce) {

        if (!visitOnlyOnce || !nodesVisited.get(node.id)) {

            visitor.visit(node);
            if (visitOnlyOnce) {
                nodesVisited.set(node.id, true);
            }
            boolean moreNodes = visitor.wantMoreNodes();
            
            if (moreNodes) {
                
                Iterator itChildren = node.getChildrenIterator();
                while (moreNodes && itChildren.hasNext()) {
                    DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                    visitChildrenPreOrder_Recursive(child, visitor, visitOnlyOnce);
                    moreNodes = visitor.wantMoreNodes();
                }
            }
        }
    }

    /*
     * Calls visitChildrenPreOrder() for all root nodes of this DAG.
     *
     * A "root" is a node without parents. Note that the roots are not explicitely stored,
     * i.e., in order to find the roots it is necessary to iterate through all nodes.
     *
     * If visitOnlyOnce is true, then it is guarenteed that each node is visited only once for each root, 
     * see alse visitChildrenPreOrder(). However, if a node is reachable from k different roots, then
     * this node is visited k times.
     */
    public void visitRoots(DoubleLinkedDAGVisitor visitor, boolean visitOnlyOnce) {
        Iterator itNodes = iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode node = (DoubleLinkedDAGNode)itNodes.next();
            if (!node.hasParents()) {
                visitChildrenPreOrder(node, visitor, visitOnlyOnce);
            }
        }
    }

    /*
     * Util method for computeMaxPathLen(); recursive.
     *
     * Computes the longest path in those subgraph whose root passed as parameter.
     */
    protected int computeMaxPathLen_Recursive(DoubleLinkedDAGNode root) {
        int maxLen = 0;
     
        Iterator itChildren = root.getChildrenIterator();
        while (itChildren.hasNext()) {
            DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
            int tmpMax = 1 + computeMaxPathLen_Recursive(child);
            if (tmpMax > maxLen) maxLen = tmpMax;
        }

        return maxLen;
    }

    /*
     * Returns the length of the longest path in the DAG.
     *
     * If there is no edge in the path, then it returns 0. If the longest path
     * has n edges, then it returns n.
     */
    public int computeMaxPathLen() {
        int maxLen = 0;

        Iterator itNodes = iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();

            if (!n.hasParents()) {   // perform recursive computation only for root nodes
                int tmpMax = computeMaxPathLen_Recursive(n);
                if (tmpMax > maxLen) maxLen = tmpMax;
            }
        }

        return maxLen;
    }

    /*
     * Util method for computeTransitiveClosure().
     * Adds an edge in closure from n to descendant and all descendants d of descendant.
     */
    protected void transClosureRecursion(GraphMatrix closure, DoubleLinkedDAGNode n,
                                         DoubleLinkedDAGNode descendant,
                                         boolean computeMinMaxDist, int distance) {

        boolean recursionRequired;  // is it necessary to perform recursive call for children?
        
        // set edge in closure, set tag in closure to length of min. path from n to desc.
        
        // NOTE: edge in closure already exists => edges to descendants of "descendant" exist as well!!
        // => recursion only necessary if computeMinMaxDistance=true and if new distances are smaller/greater
        // than old ones!

        if (closure.hasEdge(n.id, descendant.id)) {
            if (computeMinMaxDist) {
                MinMaxDistance d = (MinMaxDistance)closure.getTag(n.id, descendant.id);
                assert(d.minDist <= d.maxDist);

                if (distance < d.minDist) {
                    System.out.println("distance < d.minDist: " + distance + " < " + d.minDist);
                    closure.setTag(n.id, descendant.id, new MinMaxDistance(distance, d.maxDist));
                    recursionRequired = true;
                } else if (distance > d.maxDist) {
                    System.out.println("distance > d.maxDist: " + distance + " > " + d.maxDist);
                    closure.setTag(n.id, descendant.id, new MinMaxDistance(d.minDist, distance));
                    recursionRequired = true;
                } else recursionRequired = false;
                
            } else { 
                recursionRequired = false;
            }
        } else {
            closure.addEdge(n.id, descendant.id);
            if (computeMinMaxDist) closure.setTag(n.id, descendant.id, new MinMaxDistance(distance, distance));

            recursionRequired = true;
        }

        // if required: recursive call for all children

        if (recursionRequired) {
            Iterator itChildren = descendant.getChildrenIterator();
            while (itChildren.hasNext()) {
                DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                transClosureRecursion(closure, n, child, computeMinMaxDist, distance + 1);
            }
        }
    }

    /*
     * Computes a graph which has an edge from n1 -> n2 iff 
     * there is a directed path from n1 to n2.
     *
     * If computeMinMaxDist is true, then tags are assigned to all edges in 
     * the closure. These tags are instances of MinMaxDistance (nested class of DoubleLinkedDAG).
     *
     * Complexity: O(n^2)
     */
    public GraphMatrix computeTransitiveClosure(boolean computeMinMaxDist) {

        final GraphMatrix result = new GraphMatrix(nodes.size());

        // iterate through all nodes n

        int progress = 0;
        Iterator itNodes = nodes.iterator();
        while (itNodes.hasNext()) {
            final DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();

            // iterate through children of n

            Iterator itChildren = n.getChildrenIterator();
            while (itChildren.hasNext()) {
                DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                
                // perform recursive algorithm on all children
                transClosureRecursion(result, n, child, computeMinMaxDist, 1);
            }
            
            ++progress;
        }
        
        // postcondition: deactivated, because often too time-consuming!
        //assert(correctTransitiveClosure(result, computeMinMaxDist));

        return result;

    }  // computeTransitiveClosure()

    /*
     * For test purposes only. Returns true iff transClosure is the 
     * transitive closure of this DAG.
     *
     * This method can be called in assert-statements. By default, no method in DoubleLinkedDAG 
     * performs this check, is it may be extremely time-consuming.
     *
     * This method is able to check if the min. and max. distances between nodes, which are supposed to
     * be stored as tags (instances of MinMaxDistance) are correct.
     */
    public boolean correctTransitiveClosure(GraphMatrix transClosure, boolean checkMinMaxDist) {
        
        // for all nodes n1 and n2: if n1 is an ancestor of n2, then there
        // must be an edge in transClosure from n1 to n2. And vice versa.
        // The minimal distances are also checked, if demanded.

        for (int i = 0; i < nodes.size(); ++i) {
            for (int j = i + 1; j < nodes.size(); ++j) {
                
                DoubleLinkedDAGNode ni = (DoubleLinkedDAGNode)nodes.get(i);
                DoubleLinkedDAGNode nj = (DoubleLinkedDAGNode)nodes.get(j);

                if (isAncestor(ni, nj)) {
                    if (!transClosure.hasEdge(ni.id, nj.id)) return false; 
                } else if (isAncestor(nj, ni)) {
                    if (!transClosure.hasEdge(nj.id, ni.id)) return false; 
                }
            }
        }

        // for all edges from ni to nj in transClosure: ni must be an ancestor of nj
        Iterator itEdges = transClosure.getEdgeIterator();
        while (itEdges.hasNext()) {
            GraphMatrix.Edge e = (GraphMatrix.Edge)itEdges.next();
            DoubleLinkedDAGNode ni = (DoubleLinkedDAGNode)nodes.get(e.from);
            DoubleLinkedDAGNode nj = (DoubleLinkedDAGNode)nodes.get(e.to);

            if (!isAncestor(ni, nj)) return false; 
            else if (checkMinMaxDist) {
                Object tag = transClosure.getTag(ni.id, nj.id);
                assert(tag != null);
                MinMaxDistance minMaxDist = (MinMaxDistance)tag;

                int computedMinDist = computeMinDistance(ni, nj);
                if (minMaxDist.minDist != computedMinDist) return false; 

                int computedMaxDist = computeMaxDistance(ni, nj);
                if (minMaxDist.maxDist != computedMaxDist) return false;
            }
            
        }

        return true;

    }  //correctTransitiveClosure()

    /*
     * Returns the minimal distance between fromNode and toNode.
     *
     * Returns -1 if there is no path, and 0 if fromNode == toNode. The algorithm is recursive.
     */
    public int computeMinDistance(DoubleLinkedDAGNode fromNode, DoubleLinkedDAGNode toNode) {
        
        int minDist;

        if (fromNode == toNode) minDist = 0;
        else {
            Iterator itChildren = fromNode.getChildrenIterator();
            if (!itChildren.hasNext()) minDist = -1;  // there is no path from fromNode to toNode
            else {
                minDist = -1;

                while (itChildren.hasNext()) {
                    DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                    int childDist = computeMinDistance(child, toNode);
                    if (childDist == 0) {
                        minDist = 1;
                        break;
                    } else {
                        if (childDist > 0) {
                            if (minDist == -1) minDist = childDist + 1;
                            else if (childDist + 1 < minDist) minDist = childDist + 1;
                        }
                    }
                }
            }
        }

        return minDist;
    }

    
    /*
     * Returns the maximal distance between fromNode and toNode.
     *
     * Returns -1 if there is no path, and 0 if fromNode == toNode. The algorithm is recursive.
     */
    public int computeMaxDistance(DoubleLinkedDAGNode fromNode, DoubleLinkedDAGNode toNode) {
        
        int maxDist;

        if (fromNode == toNode) maxDist = 0;
        else {
            Iterator itChildren = fromNode.getChildrenIterator();
            if (!itChildren.hasNext()) maxDist = -1;  // there is no path from fromNode to toNode
            else {
                maxDist = -1;

                while (itChildren.hasNext()) {
                    DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                    int childDist = computeMaxDistance(child, toNode);
                    
                    if (childDist == 0) {
                        if (maxDist == -1) maxDist = 1;
                        else assert(maxDist >= 1);
                    } else if (childDist > 0) {
                        if (maxDist == -1) maxDist = childDist + 1;
                        else if (childDist + 1 > maxDist) maxDist = childDist + 1;
                    }

                }
            }
        }

        return maxDist;
    }
    /*
     * When a new edge from "from" to "to" was created, then this method updates "transClosure".
     *
     * See also computeTransitiveClosure().
     */
    public void updateTransClosureForNewEdge(GraphMatrix transClosure, 
                                             DoubleLinkedDAGNode fromNode, DoubleLinkedDAGNode toNode, 
                                             boolean computeMinDistance) {

        // iterate through all nodes n; if either n is fromNode or fromNode is reachable from n:
        //     add an edge in transClosure from n to toNode and all descendants of toNode

        Iterator itNodes = nodes.iterator();
        while (itNodes.hasNext()) 
        {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();
            if ((n == fromNode) || (transClosure.hasEdge(n.id, fromNode.id))) {
                transClosureRecursion(transClosure, n, toNode, computeMinDistance, 1);
            }
        }

     }


    /*
     * Returns true iff possAnc is an ancestor of node.
     *
     * This method performes a recursive search. If the transitive closure of this DAG
     * has already been computed, then use the overloaded method below.
     *
     * Complexity: O(n)  [n..#nodes]
     */
    public boolean isAncestor(DoubleLinkedDAGNode possAnc, DoubleLinkedDAGNode possDesc) {
        
        // precondition
        assert(nodes.contains(possAnc) && nodes.contains(possDesc));

        SearchNodeVisitor visitor = new SearchNodeVisitor(possDesc);
        
        Iterator itChildren = possAnc.getChildrenIterator();
        while (itChildren.hasNext() && !visitor.foundNode()) {
            DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();            
            visitChildrenPreOrder(child, visitor, true);
        }

        return visitor.foundNode();
    }

    /*
     * Returns true iff possAnc is an ancestor of node.
     *
     * If the transitive closure is not known, then use the overloaded method above.
     * Complexity: O(const)
     */
    public boolean isAncestor(DoubleLinkedDAGNode possAnc, DoubleLinkedDAGNode possDesc,
                              GraphMatrix transitiveClosure) {
        
        // precondition
        assert(nodes.contains(possAnc) && nodes.contains(possDesc));

        boolean result = transitiveClosure.hasEdge(possAnc.id, possDesc.id);

        // postcondition
        assert(result == isAncestor(possAnc, possDesc));
        
        return result;
    }

    /*
     * Implements depth-first search: a cycle is detected if n is already marked as MARK_BEING_VISITED.
     * If this happens, then n is returned. If the recursive call for a child returns a node n1, then
     * n1 is the first node encountered by the search which occurs a second time on one path.
     *
     * If a cycle is found, then all of its elements are returned in foundCycle.
     */
    protected DoubleLinkedDAGNode findCycle_Recursive(DoubleLinkedDAGNode n, ArrayList foundCycle) {
        
        int nodeStatus = nodeMarks[n.id];

        if (nodeStatus == MARK_BEING_VISITED) {  // cycle detected!
            foundCycle.add(n);
            return n;
        
        } else {

            if (nodeStatus == MARK_DONE_VISITED) {  // if there are "diamond" structures: n is visited again
                return null;
            
            } else {
                nodeMarks[n.id] = MARK_BEING_VISITED;

                // iterate through children, recursive calls

                Iterator itChildren = n.getChildrenIterator();
                while (itChildren.hasNext()) {
                    DoubleLinkedDAGNode child = (DoubleLinkedDAGNode)itChildren.next();
                    DoubleLinkedDAGNode cycleNode = findCycle_Recursive(child, foundCycle);
                    
                    if (cycleNode != null) {   // child is part of a cycle

                        if (cycleNode == n) {  // n is the "first" node of the cycle
                            foundCycle.add(n);
                            return null;

                        } else {  // n is not the first node of the cycle
                            foundCycle.add(n);
                            return n;
                        }
                    }
                }
            }

            nodeMarks[n.id] = MARK_DONE_VISITED;
            return null;
        }
        
    }  // findCycle_Recursive()

    /*
     * Searches for a cycle and, if one is found, returns a list of nodes on this cycle.
     *
     * If a cycle is found which starts with node n then n will be the first AND the last 
     * element of the returned list, i.e., this list contains n twice!
     */
    public List findCycle() {

        ArrayList foundCycle = new ArrayList();
        nodeMarks = new int[nodes.size()];

        // iterate through all nodes, if n is a root node: start depth search on n

        Iterator itNodes = nodes.iterator();
        while (itNodes.hasNext()) {
            DoubleLinkedDAGNode n = (DoubleLinkedDAGNode)itNodes.next();
            
            if (!n.hasParents()) {  // n is a root node
                
                // mark all nodes as MARK_NOT_VISITED

                for (int i = 0; i < nodeMarks.length; ++i) {
                    nodeMarks[i] = MARK_NOT_VISITED;
                }

                // start recursive search

                findCycle_Recursive(n, foundCycle);
                if (foundCycle.size() > 0) break;  // if cycle found: interrupt search
            }
        }

        if (foundCycle.size() == 0) return null;
        else return foundCycle;
    
    }  // findCyle()
    
    /*
     * Iterates through all ancestors of node. The transitive closure of this DAG
     * must be provided.
     *
     * The iterator returns objects of type DoubleLinkedDAGNode.
     *
     * UNTESTED!
     */
    public Iterator getAncestorIterator(DoubleLinkedDAGNode node, GraphMatrix transitiveClosure) {
        return new IntegerIterator(transitiveClosure.getAncestorIterator(node.id));
    }
    
    /*
     * Analogous to getAncestorIterator().
     *
     * UNTESTED!
     */
    public Iterator getDescendantIterator(DoubleLinkedDAGNode node, GraphMatrix transitiveClosure) {
        return new IntegerIterator(transitiveClosure.getDescendantIterator(node.id));
    }


    /*
     * Iterates through those nodes of the DAG whose integer indexes are passed to
     * the constructor of IntegerIterator.
     */
    protected class IntegerIterator implements Iterator {

        Iterator itNodes;

        /*
         * The argument is an iterator of Integer objects.
         */
        public IntegerIterator(Iterator itNodes) {
            this.itNodes = itNodes;
        }

        public boolean hasNext() {
            return itNodes.hasNext();
        }

        public Object next() {
            assert(hasNext());

            Integer index = (Integer)itNodes.next();
            return nodes.get(index.intValue());
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }
    }

}

class SearchNodeVisitor extends DoubleLinkedDAGVisitor {

    private DoubleLinkedDAGNode searchNode;

    private boolean foundNode = false;


    public SearchNodeVisitor(DoubleLinkedDAGNode searchNode) {
        this.searchNode = searchNode;
    }

    public boolean wantMoreNodes() {
        return (!foundNode);
    }

    public void visit(DoubleLinkedDAGNode node) {
        if (node == searchNode) foundNode = true;
    }

    public boolean foundNode() {
        return foundNode;
    }

}

/*
 * For each visited node n: iterate through parents n_p and
 * add strings of format "n_p.toString() => n.toString; " to the StringBuffer which is passed
 * to the constructor.
 */
class AddNodeAsStringVisitor extends DoubleLinkedDAGVisitor {

    StringBuffer str;

    public AddNodeAsStringVisitor(StringBuffer str) {
        this.str = str;
    }

    public boolean wantMoreNodes() {
        return true;
    }

    public void visit(DoubleLinkedDAGNode node) {
        Iterator itParents = node.getParentsIterator();
        if (!itParents.hasNext()) {
            str.append(node.toString() + "; ");
        }
        while (itParents.hasNext()) {
            DoubleLinkedDAGNode parentNode = (DoubleLinkedDAGNode)itParents.next();
            str.append(parentNode.toString() + " => " + node.toString() + "; ");
        }
    }

    public String getStr() {
        return str.toString();
    }

}

