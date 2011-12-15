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
 * @version 1, DATE: 16.03.2005
 * @author Joerg Weber
 *
 */

package hittingsetalg;

import java.io.*;      // IO specific classes (File, Stream,..)
import java.util.*;    // Java util

import theoremprover.*;
import utils.SortedIntList;

/**
 * This class encapsulates Reiter's Hitting Set algorithm.
 *
 * There are 2 possibilities:
 *   1) the conflict sets are provided by the user of this class, i.e. no more calls to
 *      the TP are performed by methods of this class.
 *   2) the conflict sets are computed incrementally while creating the HS-DAG, i.e. an instance 
 *      of ABTheoremProver must be provided by the user of this class.
 */
public class MinHittingSets {
    
    protected final static int CS_COMPUTING = 0;
    
    public final static int CS_ALL_MIN_DIAGS_COMPUTED = 1;

    public final static int CS_MAX_HS_SIZE_REACHED = 2;

    public final static int CS_MAX_NUM_MIN_HS_REACHED = 3;


    // The computed min. hitting sets. Each one is a HSNode.
    protected ArrayList computedMinHS = new ArrayList();
    
    // max. size of desired min. hitting sets; -1 means all min. hitting sets are computed.
    protected int maxHSSize;

    // When maxNumMinHS are computed, then the algorithm stops. If -1: all min. hitting sets are computed.
    protected int maxNumMinHS;

    // True if the algorithm can rely on the fact that all conflicts are minimal.
    protected boolean conflictsAreMinimal;

    /* 
     * Conflict sets, either passed to a constructor or computed incrementally.
     *
     * ArrayList of SortedIntList.
     */
    protected ArrayList conflictSets = null;

    // The theorem prover. Remains "null" if the conflict sets are provided by the user of this class.
    protected ABTheoremProver theoremProver = null;

    // The assumptions of theoremProver.
    protected ArrayList assumptions = null;

    // The set COMP. Is actually the set {0,..,n} where n = assumption.size() - 1.
    // Only used when computeIncr=true.
    protected SortedIntList components = null;

    // Indicates if the conflict sets are computed incrementally or provided by the user of this class.
    protected boolean computeIncr;

    // The root node.
    protected HSNode rootNode = null;

    // ArrayList of LinkedList of HSNode. Contains the levels of the DAG.
    protected ArrayList levels = new ArrayList();

    protected int computationState = -1;

    protected HSNode attemptedPruneNode = null;


    /**
     * Conctructor of MinHittingSets, receiving a list of conflict sets.
     *
     * The constructor initializes the object instance. In order to compute the hitting sets,
     * call compute(). Note that an instance of MinHittingSets which is created by this constructor
     * does not allow to compute the hitting sets incrementally!
     *
     * @param conflictsAreMinimal If true, then all provided conflicts are minimal. As a consequence,
     *   the computationally expensive "Pruning" rule is not required.
     * @param conflictSets A list of SortedIntList objects, where a SortedIntList is a linked list 
     *   containing integers representing components.
     */
    public MinHittingSets(boolean conflictsAreMinimal, ArrayList conflictSets) {
        // precondition
        assert(conflictSets.size() >= 1);

        computeIncr = false;
        this.conflictsAreMinimal = conflictsAreMinimal;
        this.conflictSets = conflictSets;
    }

    /**
     * Conctructor of MinHittingSets, receiving a theorem prover.
     *
     * The constructor initializes the object instance. A subsequent call to compute() will
     * compute the min. hitting sets incrementally.
     * If conflictsAreMinimal is true, then the algorithm relies on the theorem prover to 
     * compute only minmal conflicts.
     */
    public MinHittingSets(boolean conflictsAreMinimal,
                          ABTheoremProver tp) {
        // precondition
        assert((tp != null) && (tp.getAssumptions().size() > 0));
        
        computeIncr = true;
        this.conflictsAreMinimal = conflictsAreMinimal;
        this.conflictSets = new ArrayList();
        this.theoremProver = tp;
        
        // for all assumptions: set an integer tag
        assumptions = tp.getAssumptions();
        components = new SortedIntList();
        Iterator it = assumptions.iterator();
        int tag = 0;
        System.out.println("Assumption tags: ");
        while(it.hasNext()) {
            Assumption a = (Assumption)it.next();
            a.setIntTag(tag);
            System.out.println((new Integer(tag)).toString() + ": " + (String)a.identifier);
            components.addSorted(tag);
            ++tag;
        }

        System.out.println();
    }

    /**
     * Returns the computed min. hitting sets.
     *
     * The return value is an ArrayList of ArrayList of Assumption.
     * IMPORTANT: this method can be used only if a theorem prover is used, i.e., if the conflict
     * sets are computed dynamically!
     * If no TP is used, then getMinHSAsIntegers() must be used!
     */
    public ArrayList getMinHS() {

        if (theoremProver == null) {
            throw new UnsupportedOperationException("class MinHittingSets: if no theorem prover is used "
                                                    + "for the dynamic computation of conflicts sets, then "
                                                    + "getMinHSAsIntegers() must be used instead of "
                                                    + "getMinHS() !!");
        }                                       

        ArrayList result = new ArrayList();
        result.ensureCapacity(computedMinHS.size());

        Iterator itMinHS = computedMinHS.iterator();
        while (itMinHS.hasNext()) {
            HSNode node = (HSNode)itMinHS.next();
            if (node.state != HSNode.STATE_PRUNED) {
                assert(node.state == HSNode.STATE_MINIMAL);

                ArrayList candidate = new ArrayList();
                Iterator itAss = node.edgeLabels.iterator();
                while (itAss.hasNext()) {
                    Integer index = (Integer)itAss.next();
                    candidate.add(theoremProver.getAssumptions().get(index.intValue()));
                }
                result.add(candidate);

            }
        }

        return result;
    }

    /*
     * This method is used when no theorem prover is used, i.e., the conflict sets
     * were computed before executing the HS algorithm.
     *
     * Returns an ArrayList of SortedIntList, where eac SortedIntList represents a
     * min. diagnosis.
     */
    public ArrayList getMinHSAsIntLists() {
        ArrayList result = new ArrayList();
        result.ensureCapacity(computedMinHS.size());

        Iterator itMinHS = computedMinHS.iterator();
        while (itMinHS.hasNext()) {
            HSNode node = (HSNode)itMinHS.next();
            if (node.state != HSNode.STATE_PRUNED) {
                assert(node.state == HSNode.STATE_MINIMAL);

                result.add(node.edgeLabels);
            }
        }

        return result;
    }

    /*
     * Returns an ArrayList of ArrayList of Assumption representing the conflicts.
     *
     * This method can only be used when the theorem prover is used, i.e., these conflicts were
     * computed incrementally.
     */
    public ArrayList getConflictsAsAss() {
        ArrayList result = new ArrayList();

        Iterator itConflicts = conflictSets.iterator();
        while (itConflicts.hasNext()) {
            SortedIntList cs = (SortedIntList)itConflicts.next();
            if (cs != null) {  // there may be "null" entries in conflictSet due to pruning
                ArrayList csAss = new ArrayList(cs.size());

                Iterator itCS = cs.iterator();
                while (itCS.hasNext()) {
                    int index = ((Integer)itCS.next()).intValue();
                    Assumption a = (Assumption)assumptions.get(index);
                    csAss.add(a);
                }
                
                result.add(csAss);
            }
        }

        return result;
    }

    /*
     * @param maxHSSize The max. size of min. hitting sets to be computed, 
     *   -1 means all min. hitting sets are computed.
     * @param maxNumMinHS The max. number of computed hitting sets. The algorithm stops
     *   as soon as maxNumMinHS hitting sets have been computed. If -1: all min. hitting sets 
     *   are computed.
     */
    /*
     * Returns one of the CS_xxx constants.
     */
    public int compute(int maxHSSize, int maxNumMinHS) {
        
        assert((maxHSSize >= 1) && (maxNumMinHS >= 1));

        // some initializations

        computationState = CS_COMPUTING;
        this.maxHSSize = maxHSSize;
        this.maxNumMinHS = maxNumMinHS;
        LinkedList lastLevelNodes  = new LinkedList(); // the nodes of the last level in the DAG
        int lastLevel = 0;
        levels.add(lastLevelNodes);

        // create root node

        rootNode = new HSNode();
        rootNode.edgeLabels = new SortedIntList();  // H(n) is empty for the root node
        lastLevelNodes.add(rootNode);

        // create one DAG level after the other (breadth-first)

        while(computationState == CS_COMPUTING) {

            //System.out.println();
            //System.out.println("iterate through last level: " + lastLevel + "; expand nodes: " + expandNodes);

            LinkedList newLevelNodes = new LinkedList();
            levels.add(newLevelNodes);
            processLastLevel(lastLevel, lastLevelNodes, newLevelNodes, true);
            //if (expandNodes) System.out.println("total # of nodes in new level: " + newLevelNodes.size());
            
            lastLevelNodes = newLevelNodes;
            ++lastLevel;

            if (computationState == CS_COMPUTING) {
                assert(lastLevelNodes.size() > 0);
                if (lastLevel - 1 == maxHSSize) computationState = CS_MAX_HS_SIZE_REACHED;
            }   
        }

        assert(computationState != CS_COMPUTING);
        return computationState;

    }  // compute()

    public int computeMore(int newMaxHSSize, int newMaxNumMinHS) {
        assert((newMaxHSSize >= maxHSSize) && (newMaxNumMinHS >= maxNumMinHS));

        if (computationState == CS_ALL_MIN_DIAGS_COMPUTED) return CS_ALL_MIN_DIAGS_COMPUTED;
        else {
            
            this.maxHSSize = newMaxHSSize;
            this.maxNumMinHS = newMaxNumMinHS; 
            int lastLevel;
            LinkedList lastLevelNodes;
            LinkedList newLevelNodes;

            if (computationState == CS_MAX_HS_SIZE_REACHED) {
                lastLevel = levels.size() - 1;
                lastLevelNodes = (LinkedList)levels.get(lastLevel);
                newLevelNodes = null;
            } else {
                assert(computationState == CS_MAX_NUM_MIN_HS_REACHED);
                lastLevel = levels.size() - 2;
                lastLevelNodes = (LinkedList)levels.get(lastLevel);
                newLevelNodes = (LinkedList)levels.get(lastLevel + 1);
            }
            
            if (computedMinHS.size() == maxNumMinHS) computationState = CS_MAX_NUM_MIN_HS_REACHED;
            else if (lastLevel - 1 == maxHSSize) computationState = CS_MAX_HS_SIZE_REACHED;
            else computationState = CS_COMPUTING;

            while(computationState == CS_COMPUTING) {
                 
                if (newLevelNodes == null) {
                    newLevelNodes = new LinkedList();
                    levels.add(newLevelNodes);
                }
                
                processLastLevel(lastLevel, lastLevelNodes, newLevelNodes, true);
                lastLevelNodes = newLevelNodes;
                ++lastLevel;
                
                if (computationState == CS_COMPUTING) {
                    if (lastLevelNodes.size() == 0) computationState = CS_ALL_MIN_DIAGS_COMPUTED;
                    else if (lastLevel - 1 == maxHSSize) computationState = CS_MAX_HS_SIZE_REACHED;
                } 
            }

            assert(computationState != CS_COMPUTING);
            return computationState;
        }
    }

    // Performs the call: "TP(SD, COMP-H(n), OBS)" (in terms of Reiter's formalization).
    // Returns a new conflict set or, if no inconsistency was detected, return "null".
    protected SortedIntList callTheoremProver(SortedIntList edgeLabels) {

        SortedIntList assumptionSet = components.subtract(edgeLabels);  // compute COMP-H(n)

        // set assumptions in COMP-H(n) to "true"

        Iterator itAss = assumptionSet.iterator();
        while(itAss.hasNext()) {
            int assIndex = ((Integer)itAss.next()).intValue();
            Assumption a = (Assumption)assumptions.get(assIndex);
            
            if (a.getLabel() != true) {
                a.setLabel(true);
                a.propagateTrue();
            }
        }

        // set assumptions in H(n) to "false"
        
        Iterator itNegAss = edgeLabels.iterator();
        while(itNegAss.hasNext()) {
            int negassIndex = ((Integer)itNegAss.next()).intValue();
            Assumption a = (Assumption)assumptions.get(negassIndex);

            if (a.getLabel() != false)
            {
                ArrayList v = new ArrayList();
                v = a.propagateFalse(v); 
                Iterator ve = v.iterator();                                
                while (ve.hasNext()) {
                    Proposition p = (Proposition)ve.next();
                    p.correctLabels();
                }
            }
        }

        //System.out.println("CALL THEOREM PROVER: true: " + assumptionSet + ";  false: " + edgeLabels);

        // consistency check

        if (theoremProver.isConsistent()) {
            //System.out.println("TP: consistent!");
            return null;  // consistent!!
        } else {

            // determine conflict set

            ArrayList conflicts = theoremProver.contradiction().collectAssumptions();
            SortedIntList cs = new SortedIntList();
            Iterator itConflicts = conflicts.iterator();
            while(itConflicts.hasNext()) {
                Assumption a = (Assumption)itConflicts.next();
                cs.addSorted(a.getIntTag());
            }
            //System.out.println("TP returns conflict: " + cs);
            
            return cs;
        }

    }  // callTheoremProver()


    /*
     * This node and all of its children are "removed", except of those nodes with another
     * parent which is not removed.
     */
    protected void prune(HSNode node, HSNode prunedParent) {

        assert(nodeInvariant(node));

        if ((node.state == HSNode.STATE_OPEN) || (node.state == HSNode.STATE_MINIMAL)) {

            boolean hasAliveParents;  // are there any parents which are NOT relabelled/removed?
            if (node.parents.size() == 1) hasAliveParents = false;
            else {
                assert(node.parents.size() > 1);
                
                hasAliveParents = false;

                Iterator itParents = node.parents.iterator();
                while (itParents.hasNext()) {
                    HSNode parent = (HSNode)itParents.next();
                    if ((parent != prunedParent) && (parent.state != HSNode.STATE_PRUNED)) {
                        hasAliveParents = true;
                        break;
                    }
                }
            }

            if (!hasAliveParents) {  // prune only if there are no alive parents!
                
                node.state = HSNode.STATE_PRUNED;
                
                //System.out.println("REMOVE node: " + node);

                Iterator itChildren = node.children.iterator();
                while (itChildren.hasNext()) {
                    HSNode child = ((EdgeNodePair)itChildren.next()).node;
                    if (child.state != HSNode.STATE_PRUNED) {
                        prune(child, node);
                    }
                }
            }
            else {
                //System.out.println("DONT remove node (alive parents): " + node);
            }
        }

        assert(nodeInvariant(node));

    }  // prune()


    /*
     * Called during pruning. node is to be relabelled by newNode.label. 
     * Some of its children may be pruned.
     */
    protected void relabel(HSNode node, HSNode newNode) {

        // precondition
        //assert(node.conflictSetIndex != newNode.conflictSetIndex);
        assert((node.state != HSNode.STATE_MINIMAL) &&  newNode.label.properSubsetOf(node.label));
        assert(nodeInvariant(node));
        assert(nodeInvariant(newNode));
        assert(newNode == attemptedPruneNode);

        // relabel node and interchange conflict sets

        SortedIntList removeLabel = node.label.subtract(newNode.label);
        node.label = newNode.label;
        conflictSets.set(node.conflictSetIndex, newNode.label);
        //conflictSets.set(newNode.conflictSetIndex, null);
        newNode.conflictSetIndex = node.conflictSetIndex;                

        // iterate through children: if child is to be removed, call prune() for it

        int prunedChildrenCount = 0;
        Iterator itChildren = node.children.iterator();
        while (itChildren.hasNext()) {
            EdgeNodePair child = (EdgeNodePair)itChildren.next();

            if (removeLabel.contains(child.edge)) {  // determine if child is to be removed
                
                if (child.node.state != HSNode.STATE_PRUNED) {
                    prune(child.node, node);
                }

                ++prunedChildrenCount;
            }    
            
            
        }
        assert(prunedChildrenCount == removeLabel.size());
        
        assert(nodeInvariant(node));
        assert(nodeInvariant(newNode));
        assert(node.label.equals(newNode.label));
    }  // relabel();


    // returns number of children which are NOT pruned
    protected int computeNumUnprunedChildren(HSNode node) {
        int n = 0;
        Iterator itChildren = node.children.iterator();
        while (itChildren.hasNext()) {
            HSNode child = ((EdgeNodePair)itChildren.next()).node;
            if (child.state != HSNode.STATE_PRUNED) ++n;
        }

        return n;
    }


    // node has just been labelled, now check if DAG can be pruned using this label
    protected void attemptPrune(HSNode node) {
        
        // precondition
        assert((node.state == HSNode.STATE_OPEN) && (node.label != null)               
               && (node.label.size() > 0) && (node != rootNode) && nodeInvariant(node));

        attemptedPruneNode = node;
        int oldConflictSetIndex = node.conflictSetIndex;
        boolean pruned = false;  // set to true when pruning is done

        // always try to relabel root node
        rootNode.relevantForPruning = true;
        boolean relevantNodesExist = true;

        // iterate through levels; stop when there are no more nodes which are candidates for relabelling

        Iterator itLevels = levels.iterator();
        while(itLevels.hasNext() && relevantNodesExist) {
        
            relevantNodesExist = false;

            LinkedList level = (LinkedList)itLevels.next();
            Iterator itLevelNodes = level.iterator();

            SortedIntList intersectionSet = new SortedIntList();

            // iterate through nodes of a level

            while(itLevelNodes.hasNext()) {

                HSNode otherNode = (HSNode)itLevelNodes.next();
                
                if ((otherNode.state == HSNode.STATE_OPEN) && (otherNode.label != null) 
                    && otherNode.relevantForPruning) {
                    
                    // determine if we can prune; if not: are there irrelevant subtrees?

                    intersectionSet.clear();
                    boolean intersects = otherNode.label.intersection(node.label, intersectionSet);
                    assert((!intersects && (intersectionSet.size() == 0))
                           || (intersects && (intersectionSet.size() != 0)));
                   
                    // Note: "A.properSubsetOf(B)" is equivalent to "A.intersects(B) and (A = intersection(A, B))
                    // and (size(A) < size(B))"
                    boolean canPrune = (intersects && (intersectionSet.size() == node.label.size())
                                        && (node.label.size() < otherNode.label.size()));
                    assert(!canPrune || node.label.properSubsetOf(otherNode.label));

                    if (canPrune) {  // prune!

                        relabel(otherNode, node);
                        pruned = true;

                    } else {  // cannot prune: investigate which subtrees are not relevant for further search
                        
                        if ((intersectionSet.size() > 0)        // some subtrees are not relevant
                            && (otherNode.children.size() > 0)) {   // .. and otherNode has already been expanded
                            
                            //System.out.println("IGNORE children of node: " + otherNode + ";   child edges: " + intersectionSet); 

                            Iterator itIntersection = intersectionSet.iterator();
                            int nextItem;
                            if (itIntersection.hasNext()) 
                                nextItem = ((Integer)itIntersection.next()).intValue();
                            else nextItem = Integer.MAX_VALUE;
                            
                            Iterator itChildren = otherNode.children.iterator();
                            while (itChildren.hasNext()) {
                                EdgeNodePair child = (EdgeNodePair)itChildren.next();
                                
                                if (child.edge == nextItem) {
                                    child.node.relevantForPruning = false;
                                    assert(node.label.contains(child.edge));
                                    if (itIntersection.hasNext()) 
                                        nextItem = ((Integer)itIntersection.next()).intValue();
                                    else nextItem = Integer.MAX_VALUE;
                                } else {
                                    child.node.relevantForPruning = true;
                                    relevantNodesExist = true;
                                }
                            }

                            assert(!itIntersection.hasNext());
                            
                        } else {  // all subtrees are relevant
                            
                            Iterator itChildren = otherNode.children.iterator();
                            while (itChildren.hasNext()) {
                                HSNode child = ((EdgeNodePair)itChildren.next()).node;
                                child.relevantForPruning = true;
                            }
                            relevantNodesExist = true;

                        }

                    }  // !canPrune

                }  // node is open and relevant for pruning

                else if (!otherNode.relevantForPruning) {  // if node not relevant => children are not rel., too

                    Iterator itChildren = otherNode.children.iterator();
                    while(itChildren.hasNext()) {
                        HSNode child = ((EdgeNodePair)itChildren.next()).node;
                        child.relevantForPruning = false;
                    }

                }

            }  // iterate through nodes of a level

        }  // iterate through levels

        if (pruned) conflictSets.set(oldConflictSetIndex, null);
        attemptedPruneNode = null;

        assert(nodeInvariant(node));

    }  // attemptPrune()

    /*
     * Searches in the list of already computed conflict sets for a conflict set cs
     * which refutes edgeLabel (= H(n) in Reiter's notation), i.e., cs shows that H(n)
     * is inconsistent with SD and OBS.
     *
     * If no such conflict is found, then this method returns -1. Otherwise, it returns
     * an index to the collection conflictSets such that edgeLabel does not intersect conflictSets[index].
     */
    protected int searchRefutingCS(SortedIntList edgeLabel) {
        for (int i = 0; i < conflictSets.size(); ++i) {
            Object o = conflictSets.get(i);
            if (o != null) {  // the conflict set at this index may have been removed by pruning!
                SortedIntList cs = (SortedIntList)conflictSets.get(i);
                assert(cs.size() > 0);
                if (!edgeLabel.intersects(cs)) return i;
            }
        }

        return -1;
    }
    

    /*
     * Compute label of node: either it is detected that this node is a min. HS,
     * or it is labelled by a new conflict set. The CS is either taken from the 
     * provided conflict sets, or it is computed by calling the theorem prover.
     */
    protected void computeLabel(HSNode node) {

        assert((node.state == HSNode.STATE_OPEN) && (node.label == null)
            && nodeInvariant(node));

        // check if node can be closed

        if (canClose(node)) {
            node.state = HSNode.STATE_CLOSED;
            //System.out.println("CLOSE node: " + node);
        }

        else {

            if (computeIncr) {
                
                int existingCSIndex = searchRefutingCS(node.edgeLabels);
                if (existingCSIndex >= 0) {
                    assert(callTheoremProver(node.edgeLabels) != null);
                    SortedIntList existingCS = (SortedIntList)conflictSets.get(existingCSIndex);
                    node.label = existingCS;
                    node.conflictSetIndex = existingCSIndex;
                    //System.out.println("REFUTE using existing conflict set: {" + existingCS + "} refutes " + node);
                    if (!conflictsAreMinimal && (node != rootNode)) attemptPrune(node);
                }
                else {
                    SortedIntList cs = callTheoremProver(node.edgeLabels);
                    if (cs == null) {
                        node.state = HSNode.STATE_MINIMAL;
                    }
                    else {
                        assert(cs.size() > 0);
                        conflictSets.add(cs);
                        node.label = cs;
                        node.conflictSetIndex = conflictSets.size() - 1;
                        //System.out.println("computed label of node: " + node);
                        if (!conflictsAreMinimal && (node != rootNode)) attemptPrune(node);
                    }
                }

            }
            
            else {  // !computeIncr
                
                if (node.edgeLabels.size() == 0) {  // true only for root node
                    node.label = (SortedIntList)conflictSets.get(0);
                    node.conflictSetIndex = 0;
                }
                
                else {  // nodes which are not the root node
                    
                    Iterator itCS = conflictSets.iterator();
                    int csIndex = 0;
                    boolean hitsAllCS = true;  // true if H(n) (n=node) hits all conflict sets
                    
                    // try to find a conflict set which is not hit by H(n)
                    while(itCS.hasNext() && hitsAllCS) {
                        SortedIntList cs = (SortedIntList)itCS.next();
                        
                        if (!node.edgeLabels.intersects(cs)) {
                            hitsAllCS = false;
                            node.label = cs;
                            node.conflictSetIndex = csIndex;
                            assert(cs.equals(conflictSets.get(csIndex)));
                            //System.out.println("computed label of node: " + node);
                        } else ++csIndex;
                    }
                    
                    if (hitsAllCS) {
                        node.state = HSNode.STATE_MINIMAL;
                        //System.out.println("new minimal hitting set: " + node);
                    } else {
                        if (!conflictsAreMinimal && (node != rootNode)) attemptPrune(node);
                    }
                    
                }
                
            }  // !computeIncr
            


        }  // node cannot be closed

        assert(nodeInvariant(node));

    }  // computeLabel()


    // Returns true iff the node can be closed (the 2nd pruning rule).
    protected boolean canClose(HSNode node) {
        
        assert(nodeInvariant(node));

        Iterator itMinHS = computedMinHS.iterator();
        while (itMinHS.hasNext()) {
            HSNode n1 = (HSNode)itMinHS.next();
            if (n1.state == HSNode.STATE_MINIMAL) {
                SortedIntList hs = n1.edgeLabels;
                if (hs.subsetOf(node.edgeLabels)) {
                    assert(hs.properSubsetOf(node.edgeLabels));
                    return true;
                }
            } else assert(n1.state == HSNode.STATE_PRUNED);
        }

        return false;
    }

    /* 
     * Returns the new nodes which were created out of the nodes of the last level.
     * If expandNodes is true, then no new nodes are created (only the nodes of the last
     * level are checked if they are min. HS);
     */
    protected void processLastLevel(int lastLevel, LinkedList lastLevelNodes, 
                                    LinkedList newLevelNodes, boolean expandNodes) {

        //System.out.println("processLastLevel: ");

        boolean hasOpenNodes = false;

        Iterator itLastNodes = lastLevelNodes.iterator();
        while(itLastNodes.hasNext()) {

            HSNode node = (HSNode)itLastNodes.next();
            assert(nodeInvariant(node));
            if (node.state == HSNodeFM.STATE_OPEN) {

                computeLabel(node);

                if (node.state == HSNodeFM.STATE_MINIMAL) {
                    computedMinHS.add(node);
                    if (computedMinHS.size() == maxNumMinHS) {
                        computationState = CS_MAX_NUM_MIN_HS_REACHED;
                        break;
                    }
                } else if (node.state == HSNodeFM.STATE_OPEN) {
                    hasOpenNodes = true;
                    if (expandNodes) {
                        expandNode(node, newLevelNodes, lastLevel);
                    }
                }
            }

            assert(nodeInvariant(node));

        }  // while(itLastNodes.hasNext())

        if ((computationState == CS_COMPUTING) && !hasOpenNodes) {
            computationState = CS_ALL_MIN_DIAGS_COMPUTED;
        }
    }

    /*
     * Expand a node: generate child nodes and set their edgeLabels field.
     * The new nodes are added to newNodes.
     */
    protected void expandNode(HSNode node, LinkedList newNodes, int lastLevel) {
        // precondition
        assert((node.label != null) && (node.label.size() >= 1) && nodeInvariant(node));

        int oldNewNodesSize = newNodes.size();

        int newEdge;

        // iterate through the conflict set of this node

        Iterator itLabel = node.label.iterator();
        while(itLabel.hasNext()) {

            // compute H(n) for the new node

            newEdge = ((Integer)itLabel.next()).intValue();            
            SortedIntList newEdgeLabels = (SortedIntList)node.edgeLabels.clone();
            newEdgeLabels.addSorted(newEdge);
            
            //System.out.println("new H(n): " + newEdgeLabels);

            // check if "Reusing" rule can be applied. Iterate through new nodes only.
            // Consider only nodes which were NOT created by expansion of node (ie by this method call).

            boolean reused = false;
            Iterator itNewNodes = newNodes.iterator();
            for (int i = 0; i < oldNewNodesSize; ++i) {
                HSNode m = (HSNode)itNewNodes.next();
                if (newEdgeLabels.equals(m.edgeLabels)) {  // reuse of node m!!
                    node.children.add(new EdgeNodePair(newEdge, m));
                    m.parents.add(node);
                    reused = true;
                    //System.out.println("REUSE node m: " + m);
                    break;
                }
            }

            if (!reused) {  // no reuse possible: create new node

                HSNode newNode = new HSNode();
                newNode.edgeLabels = newEdgeLabels;

                node.children.add(new EdgeNodePair(newEdge, newNode));
                newNode.parents.add(node);
                newNodes.add(newNode);

                //System.out.println("NO reuse possible, create new node: " + newNode);
            }

        }  // while(itLabel.hasNext())
 
        assert(nodeInvariant(node));
        
    }  // expandNode()

    public boolean checkMinimalityHS() {

        for (int i = 0; i < computedMinHS.size(); ++i) {

            HSNode node = (HSNode)computedMinHS.get(i);

            if (node.state == HSNode.STATE_MINIMAL) {

                for (int j = i + 1; j < computedMinHS.size(); ++j) {
                    
                    HSNode otherNode = (HSNode)computedMinHS.get(j);
                    
                    if ((otherNode.state == HSNode.STATE_MINIMAL) 
                        && node.edgeLabels.subsetOf(otherNode.edgeLabels) 
                        || otherNode.edgeLabels.subsetOf(node.edgeLabels)) return false;
                    
                }
            }
        }

        return true;
    }

    public boolean hitsAllConflictSets() {

        for (int i = 0; i < computedMinHS.size(); ++i) {

            HSNode node = (HSNode)computedMinHS.get(i);
            if (node.state == HSNode.STATE_MINIMAL) {
                
                Iterator itCS = conflictSets.iterator();
                while (itCS.hasNext()) {

                    SortedIntList cs = (SortedIntList)itCS.next();

                    if (!node.edgeLabels.intersects(cs)) return false;
                    
                }

            }
        }

        return true;

    }

    protected boolean nodeInvariant(HSNode node) {

        boolean a = ((node.label == null) || !node.label.intersects(node.edgeLabels));
        boolean b = ((node.conflictSetIndex == -1) 
                    || (node.label.equals(conflictSets.get(node.conflictSetIndex)))
                    || ((attemptedPruneNode != null) 
                        && attemptedPruneNode.label.equals(conflictSets.get(node.conflictSetIndex))));
        boolean c = ((node == rootNode) || (node.parents.size() >= 1));

        boolean result = (a && b && c);

        if (!result) {
            if (!a) {
                System.out.println("NODE INVARIANT VIOLATED: a");
                System.out.println("node.label: " + node.label);
                System.out.println("node.edgeLabels: " + node.edgeLabels);
            }
            if (!b) System.out.println("NODE INVARIANT VIOLATED: b");
            if (!c) System.out.println("NODE INVARIANT VIOLATED: c");
        }

        if (!b) {
            System.out.println("node: " + node + "   |  " 
                               + (SortedIntList)conflictSets.get(node.conflictSetIndex)
                + "   conflictSetIndex: " + node.conflictSetIndex);
            System.out.print("attemptedPruneNode: ");
            if (attemptedPruneNode != null) {
                System.out.print("" + attemptedPruneNode);
                System.out.println("   conflictSetIndex: " + attemptedPruneNode.conflictSetIndex);
                System.out.println(conflictSets.get(attemptedPruneNode.conflictSetIndex));
            } else System.out.println("null");
        }

        return result; 
                 
                /*&& ((node.children.size() == 0) || (node.children.size() == node.label.size())
                    || (node.label.size() == computeNumUnprunedChildren(node))
                    || (node.state == HSNode.STATE_PRUNED))*/
    }
}


class EdgeNodePair {

    public int edge = -1;

    public HSNode node;

    public EdgeNodePair(int edge, HSNode node) {
        this.edge = edge;
        this.node = node;
    }

}

// A node in the HS-DAG
class HSNode {

    final static int STATE_OPEN = 0;  // The node is neither closed nor is it a min. HS
    final static int STATE_CLOSED = 1;  // The node is closed (see the "Close" rule in Reiter).
    final static int STATE_MINIMAL = 2;  // The node is a min HS
    final static int STATE_PRUNED = 3;  // Removed by pruning algorithm.

    int state = STATE_OPEN;

    // the H(n), in terms of Reiter's formalization.
    SortedIntList edgeLabels;  
    
    // parent nodes
    ArrayList parents = new ArrayList();
    
    // child nodes: each element is of type EdgeNodePair
    ArrayList children = new ArrayList();

    SortedIntList label;
   
    // The index of label in the conflictSets data structure.
    int conflictSetIndex = -1;

    // flag during pruning algorithm; if false: this node (and all of its children!) do not
    // need to be considered for pruning.
    boolean relevantForPruning = true;

    public String toString() {

        String result = "{" + edgeLabels + "}; ";
        
        if ((state == STATE_OPEN) || (state == STATE_PRUNED)) {
            if (label == null) result = result + "{??}";
            else {
                result = result + "{" + label + "}";
            }
            if (state == STATE_PRUNED) result = result + "  PRUNED";
        } 
        else if (state == STATE_CLOSED) {
            result = result + "X";
        } else if (state == STATE_MINIMAL) {
            result = result + "!!!";
        }

        result = result + " csIndex: " + conflictSetIndex;

        result = result + "  #(parents): " + parents.size() 
            + "  #(children): " + children.size();

        return result;
    }


}
