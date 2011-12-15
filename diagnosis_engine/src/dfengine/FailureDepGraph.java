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

public class FailureDepGraph extends DoubleLinkedDAG {

    protected GraphMatrix indirectDeps;

    protected GraphMatrix commonAncestorGraph;

    protected boolean useProbabilities;

    public FailureDepGraph(boolean useProbabilities) {
        this.useProbabilities = useProbabilities;
    }

    public String toString() {
        StringBuffer result = new StringBuffer(super.toString());
        
        result.append("\n\nTRANSITIVE CLOSURE:\n\n");
        Iterator itEdges = indirectDeps.getEdgeIterator();
        while (itEdges.hasNext()) {
            GraphMatrix.Edge e = (GraphMatrix.Edge)itEdges.next();
            FailureDepNode fromNode = (FailureDepNode)getNode(e.from);
            FailureDepNode toNode = (FailureDepNode)getNode(e.to);
            result.append(fromNode + " => " + toNode);
            
            int dist = ((DoubleLinkedDAG.MinMaxDistance)indirectDeps.getTag(e.from, e.to)).minDist;
            result.append(" (" + dist + ")\n");
        }
        
        result.append("\n\nCOMMON ANCESTOR GRAPH:\n\n");
        itEdges = commonAncestorGraph.getEdgeIterator();
        while (itEdges.hasNext()) {
            GraphMatrix.Edge e = (GraphMatrix.Edge)itEdges.next();
            FailureDepNode fromNode = (FailureDepNode)getNode(e.from);
            FailureDepNode toNode = (FailureDepNode)getNode(e.to);
            result.append(fromNode + " => " + toNode);
            
            int dist = ((Integer)commonAncestorGraph.getTag(e.from, e.to)).intValue();
            result.append(" (" + dist + ")\n");
        }

        return result.toString();
    }

    public Object clone() {
        throw new UnsupportedOperationException("class FailureDepGraph does not support clone()");
    }

    boolean useProbabilities() {
        return useProbabilities;
    }

    /**
     * Create a FailureDepNode for the passed component and add it to the graph.
     *
     * Returns the new node.
     */
    public FailureDepNode addNode(Component comp) {
        assert(!useProbabilities || (comp.getProbIF() >= 0.0));

        FailureDepNode newNode = new FailureDepNode(comp);
        super.addNode(newNode);
        comp.fdNode = newNode;
        return newNode;
    }

    /**
     * Creates a directed edge between two components.
     *
     * The passed components must be added to the graph before the edge can be created.
     */
    public void addEdge(Component from, Component to) {
        assert(!useProbabilities && (from.fdNode != null) && (to.fdNode != null));

        super.addEdge(from.fdNode, to.fdNode);
    }

    /**
     * Creates a directed edge between two components, and assigns a probability to the edge.
     *
     * The passed components must be added to the graph before the edge can be created.
     */
    public void addEdge(Component from, Component to, double prob_df) {
        assert(useProbabilities && (from.fdNode != null) && (to.fdNode != null));

        super.addEdge(from.fdNode, to.fdNode);
        to.fdNode.prob_df.put(new Integer(from.fdNode.getID()), new Double(prob_df));
    }

    /*
     * Does not need to be called explicitely.
     * 
     * The indirect dependencies are computed implicitely at the first
     * invokation of hasIndirectDependency(). However, the
     * computation can be triggered earlier by calling this method.
     */
    public void computeIndirectDeps() {
        indirectDeps = computeTransitiveClosure(true);
    }

    /*
     * True iff there is a path from "from" to "to".
     */
    public boolean hasIndirectDependency(Component from, Component to) {
        if (indirectDeps == null) computeIndirectDeps();

        return (indirectDeps.hasEdge(from.getFDNode().getID(), to.getFDNode().getID()));        
    }

    /*
     * Returns true iff c1 and c2 have a common ancestor in the FDG,
     * but c1 and c2 are not (directly or indirectly) dependent on each other.
     */
    public boolean haveCommonAncestor(Component c1, Component c2) {
        int id1 = c1.getFDNode().getID();
        int id2 = c2.getFDNode().getID();
        
        return commonAncestorGraph.hasEdge(id1, id2);
    }

    /*
     * The resulting graph has an edge from n1 -> n2 iff
     * n1 and n2 have a common ancestor a in the FDG, 
     * but n1 and n2 are not indirect dep. on each other
     * (i.e., there is no path in the transitive closure
     * between n1 and n2).
     *
     * Note that the resulting graph is undirected, i.e.,
     * for each edge n1 -> n2 there must also be an edge
     * n2 -> n1.
     *
     * If computeMinDistance: the edge n1 -> n2 is labelled
     * with an integer md= max{d(a, n1), d(a, n2)}  [d..distance],
     * and a is the "closest" ancestor.
     *
     * Must be called AFTER computeIndirectDeps()!
     */
    public void computeCommonAncestorGraph(boolean computeMinDistances) {
        assert(indirectDeps != null);

        commonAncestorGraph = new GraphMatrix(nodes.size());

        for (int i = 0; i < nodes.size(); ++i) {

            for (int j = i + 1; j < nodes.size(); ++j) {

                if (!indirectDeps.hasEdge(i, j)
                    && !indirectDeps.hasEdge(j, i)) {

                    for (int k = 0; k < nodes.size(); ++k) {
                        if ((k != i) && (k != j)
                            && indirectDeps.hasEdge(k, i)
                            && indirectDeps.hasEdge(k, j)) {

                            if (computeMinDistances) {
                                Object tag_i = indirectDeps.getTag(k, i);
                                Object tag_j = indirectDeps.getTag(k, j);
                                assert((tag_i != null) && (tag_j) != null);
                                int di = ((DoubleLinkedDAG.MinMaxDistance)tag_i).minDist;
                                int dj = ((DoubleLinkedDAG.MinMaxDistance)tag_j).minDist;
                                int maxd;
                                if (di > dj) maxd = di;
                                else maxd = dj;

                                if (commonAncestorGraph.hasEdge(i, j)) {
                                    assert(commonAncestorGraph.hasEdge(j, i));

                                    Integer dk = (Integer)commonAncestorGraph.getTag(i, j);
                                    if (maxd < dk.intValue()) {
                                        commonAncestorGraph.setTag(i, j, new Integer(maxd));
                                        commonAncestorGraph.setTag(j, i, new Integer(maxd));
                                    }
                                } else {
                                    commonAncestorGraph.addEdge(i, j);
                                    commonAncestorGraph.addEdge(j, i);
                                    commonAncestorGraph.setTag(i, j, new Integer(maxd));
                                    commonAncestorGraph.setTag(j, i, new Integer(maxd));
                                }
                            } else {
                                commonAncestorGraph.addEdge(i, j);  // edges are "mutual" (2-way)
                                commonAncestorGraph.addEdge(j, i);
                                break;
                            }                            

                        }  // for k
                    }
                }  // for j
            }  // for i
        }

    }  // computeCommonAncestorGraph()

    /*
     * Returns true iff either c1 is an ancestor of c2, or vice versa, or if they
     * have a common ancestor. In the first two cases, only pathes between c1 and c2
     * with a length of <= maxPathLen are considered. In the third case, maxPathLen
     * refers to the path between the common ancestor and that component c1 or c2
     * which has a larger distance to the ancestor.
     * 
     */
    public boolean isAncestorConnected(Component c1, Component c2, int maxPathLen) {
        
        boolean result = isAncestorConnected_Impl(c1, c2, maxPathLen);
        
        // assert: this relation is commutative
        assert(result == isAncestorConnected_Impl(c2, c1, maxPathLen));
        
        return result;
    }

    protected boolean isAncestorConnected_Impl(Component c1, Component c2, int maxPathLen) {
        int id1 = c1.getFDNode().getID();
        int id2 = c2.getFDNode().getID();

        boolean result = false;

        if (indirectDeps.hasEdge(id1, id2)) {
            int dist = ((DoubleLinkedDAG.MinMaxDistance)indirectDeps.getTag(id1, id2)).minDist;
            if (dist <= maxPathLen) result = true;
        } else if (indirectDeps.hasEdge(id2, id1)) {
            int dist = ((DoubleLinkedDAG.MinMaxDistance)indirectDeps.getTag(id2, id1)).minDist;
            if (dist <= maxPathLen) result = true;
        } else if (commonAncestorGraph.hasEdge(id1, id2)) {
            int dist = ((Integer)commonAncestorGraph.getTag(id1, id2)).intValue();
            if (dist <= maxPathLen) result = true;
        }

        return result;
    }

    /*
     * Returns an ArrayList (AL) of AL of Component.
     *
     * Note that the linked list "comps" is be empty after this algorithm.
     * Complexity: O(m^2), where m = comps.size().
     *
     * maxPathLen restricts the max. length of pathes between components.
     */
    public ArrayList computeTransPiPartitions(LinkedList comps, int maxPathLen) {
    
        ArrayList result = new ArrayList();

        while (comps.size() > 0) {
         
            Component c = (Component)comps.removeFirst();
            ArrayList newPart = new ArrayList();
            newPart.add(c);

            boolean progress;
            do {
                progress = false;
                
                Iterator itR = comps.iterator();
                while (itR.hasNext()) {
                    Component cr = (Component)itR.next();

                    Iterator itP = newPart.iterator();
                    while (itP.hasNext()) {
                        Component cp = (Component)itP.next();
                        if (isAncestorConnected(cr, cp, maxPathLen)) {
                            newPart.add(cr);
                            itR.remove();
                            progress = true;
                            break;
                        }
                    }
                }


            } while (progress);
   
            result.add(newPart);

        }  //  while (comps.size() > 0)      

        return result;

    }  // computeTransPiPartitions()
}
