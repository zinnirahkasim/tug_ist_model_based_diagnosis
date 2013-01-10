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
 * @version 1, DATE: 26.07.2006
 * @author Joerg Weber
 *
 */

package hittingsetalg;

import java.io.*;      // IO specific classes (File, Stream,..)
import java.util.*;    // Java util

import theoremprover.*;
import utils.SortedIntList;

/**
 * This class encapsulates an implementation of Reiter's Hitting Set algorithm which supports fault models.
 *
 */
public class MinHittingSetsFM {

    protected final static int CS_COMPUTING = 0;
    
    public final static int CS_ALL_MIN_DIAGS_COMPUTED = 1;

    public final static int CS_MAX_HS_SIZE_REACHED = 2;

    public final static int CS_MAX_NUM_MIN_HS_REACHED = 3;
    

    // The computed min. hitting sets. Each one is a HSNodeFM.
    protected ArrayList computedMinHS = new ArrayList();
    
    // max. size of desired min. hitting sets; -1 means all min. hitting sets are computed.
    protected int maxHSSize;

    // When maxNumMinHS are computed, then the algorithm stops. If -1: all min. hitting sets are computed.
    protected int maxNumMinHS;

    // True if the algorithm can rely on the fact that all conflicts are minimal.
    protected boolean conflictsAreMinimal;

    // List of ConflictSet instances.
    protected ArrayList fmConflictSets = new ArrayList();

    // The theorem prover. Remains "null" if the conflict sets are provided by the user of this class.
    protected ABTheoremProver theoremProver = null;

    // objects of class Component; sorted by name (string)
    protected TreeMap components = new TreeMap();

    // list of Component's. The index in the ArrayList corresponds to Component.id.
    protected ArrayList componentList = new ArrayList();

    // The set COMP. Is actually the set {0,..,n-1} where n is the number of components.
    protected SortedIntList componentIdSet = new SortedIntList();

    // The root node.
    protected HSNodeFM rootNode = null;

    // ArrayList of LinkedList of HSNodeFM. Contains the levels of the DAG.
    protected ArrayList levels = new ArrayList();

    protected int computationState = -1;

    protected HSNodeFM attemptedPruneNode = null;

    protected int numTPCalls;
    protected int numPrunings;
    protected int numReuses;
    protected int numExpansions;

    // the string identifier of the AB assumption, e.g.: "AB", "N_OK", "ABNORMAL",..
    //protected String assumptionAB;

    // e.g.: "NAB", "N_AB", "OK", "GOOD", "NOMINAL", ..
    //protected String assumptionNAB;


    /**
     * Conctructor of MinHittingSets, receiving a theorem prover.
     *
     * The constructor initializes the object instance. A subsequent call to compute() will
     * compute the min. hitting sets incrementally.
     * If conflictsAreMinimal is true, then the algorithm relies on the theorem prover to 
     * compute only minmal conflicts.
     *
     * Throws IllegalAssumption if an assumption different from the two passed assumptions is used.
     *
     * @param maxHSSize The max. size of min. hitting sets to be computed, 
     *   -1 means all min. hitting sets are computed.
     * @param maxNumMinHS The max. number of computed hitting sets. The algorithm stops
     *   as soon as maxNumMinHS hitting sets have been computed. If -1: all min. hitting sets 
     *   are computed.
     * @param conflictsAreMinimal If true, then all provided conflicts are minimal. As a consequence,
     *   the computationally expensive "Pruning" rule is not required.
     * @param assumptionAB the string identifier of the AB assumption, e.g.: "AB", "N_OK", "ABNORMAL",..
     * @param assumptionNAB e.g.: "NAB", "N_AB", "OK", "GOOD", "NOMINAL",..
     */
    public MinHittingSetsFM(boolean conflictsAreMinimal, ABTheoremProver tp, 
                            String assumptionAB, String assumptionNAB) 
        throws IllegalAssumption {

        // precondition
        assert(tp != null);
        
        this.conflictsAreMinimal = conflictsAreMinimal;
        this.theoremProver = tp;

        // extract components from assumptions

        Iterator it = tp.getAssumptions().iterator();
        int id = 0;
        
        while(it.hasNext()) {
            Assumption a = (Assumption)it.next();
            
            String ass = (String)a.identifier;
            boolean ass_ab = true;
            String compName = null;

            if (ass.matches(assumptionAB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {

                compName = ass.substring(assumptionAB.length() + 1,
                                                ass.length() - 1);
                ass_ab = true;
            } else if (ass.matches(assumptionNAB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {

                compName = ass.substring(assumptionNAB.length() + 1,
                                         ass.length() - 1);
                ass_ab = false;
            } else throw new IllegalAssumption(ass);

            if (compName.length() == 0) throw new IllegalAssumption(ass);

            Object obj = components.get(compName);
            Component c;
            if (obj == null) {
                c = new Component(compName, id);
                //System.out.println("add new component: " + compName);
                components.put(compName, c);
                componentList.add(c);
                ++id;
            } else {
                c = (Component)obj;
            }
            if (ass_ab) c.ab = a; else c.nab = a;
            a.objTag = c;
        
        }

        for (int i = 0; i < components.size(); ++i) componentIdSet.addSorted(i);
        
    }  // constructo

    /**
     * Returns the computed min. hitting sets.
     *
     * The return value is an ArrayList of ArrayList of Assumption (only "NAB"-assumptions, no "AB"-assumptions!).
     */
    public ArrayList getMinHS() {
        ArrayList result = new ArrayList();
        result.ensureCapacity(computedMinHS.size());
        
        Iterator itMinHS = computedMinHS.iterator();
        while (itMinHS.hasNext()) {
            HSNodeFM node = (HSNodeFM)itMinHS.next();
            if (node.state != HSNodeFM.STATE_PRUNED) {
                assert(node.state == HSNodeFM.STATE_MINIMAL);
                
                ArrayList candidate = new ArrayList();
                Iterator itComps = node.edgeLabels.iterator();
                while (itComps.hasNext()) {
                    Integer compId = (Integer)itComps.next();
                    Component c = (Component)componentList.get(compId.intValue());
                    candidate.add(c.nab);
                }
                if (candidate.size() > 0) {  // == 0 if root node is already consistent
                    result.add(candidate);
                }
            }
        }
        
        return result;
    }

    /**
     * Returns an ArrayList of ArrayList of Assumption: the conflict sets.
     *
     * Each assumption is either a NAB or AB assumption.
     */
    public ArrayList getConflictsAsAss() {
        ArrayList result = new ArrayList();

        Iterator itConflicts = fmConflictSets.iterator();
        while (itConflicts.hasNext()) {
            ConflictSet cs = (ConflictSet)itConflicts.next();
            if (cs != null) {  // may be null due to pruning
                ArrayList csAss = new ArrayList();
                
                Iterator itAbComp = cs.abComps.iterator();
                while (itAbComp.hasNext()) {
                    int compIndex = ((Integer)itAbComp.next()).intValue();
                    Component c = (Component)componentList.get(compIndex);
                    csAss.add(c.ab);
                }
                
                Iterator itNabComp = cs.nabComps.iterator();
                while (itNabComp.hasNext()) {
                    int compIndex = ((Integer)itNabComp.next()).intValue();
                    Component c = (Component)componentList.get(compIndex);
                    csAss.add(c.nab);
                }

                result.add(csAss);
            }
            
        }

        return result;
    }

    /*
     * Returns one of the CS_xxx constants.
     */
    public int compute(int maxHSSize, int maxNumMinHS) {
        
        assert((maxHSSize >= 1) && (maxNumMinHS >= 1));

        // some initializations

        computationState = CS_COMPUTING;
        numTPCalls = 0;
        numPrunings = 0;
        numReuses = 0;
        numExpansions = 0;
        this.maxHSSize = maxHSSize;
        this.maxNumMinHS = maxNumMinHS;
        LinkedList lastLevelNodes  = new LinkedList(); // the nodes of the last level in the DAG
        int lastLevel = 0;
        levels.add(lastLevelNodes);

        // create root node

        rootNode = new HSNodeFM();
        rootNode.edgeLabels = new SortedIntList();  // H(n) is empty for the root node
        lastLevelNodes.add(rootNode);

        // create one DAG level after the other (breadth-first)

        while(computationState == CS_COMPUTING) {

            boolean expandNodes = (lastLevel < maxHSSize);

            LinkedList newLevelNodes; 
            if (expandNodes) {
                newLevelNodes = new LinkedList();
                levels.add(newLevelNodes);
            } else {
                newLevelNodes = null;
            }
            
            processLastLevel(lastLevel, lastLevelNodes, newLevelNodes, expandNodes);
            //if (expandNodes) System.out.println("total # of nodes in new level: " + newLevelNodes.size());

            if (expandNodes) {
                if (computationState == CS_COMPUTING) {
                    lastLevelNodes = newLevelNodes;
                    ++lastLevel;
                    assert(lastLevelNodes.size() > 0);
                }
            } else {
                if (computationState == CS_COMPUTING) {
                    computationState = CS_MAX_HS_SIZE_REACHED;
                }
            }
  
        }

        assert(computationState != CS_COMPUTING);
        System.out.println("compute(): computationState: " + computationState);
        System.out.println("\nNumber of TP calls: " + numTPCalls);
        System.out.println("\nNumber prunings: " + numPrunings);
        System.out.println("\nNumber reuses: " + numReuses);
        System.out.println("\nNumber expansions: " + numExpansions);
        return computationState;

    }  // compute()

    public int computeMore(int newMaxHSSize, int newMaxNumMinHS) {
        
        assert((newMaxHSSize >= maxHSSize) && (newMaxNumMinHS >= maxNumMinHS));

        //System.out.println("computationState: " + computationState);

        if (computationState == CS_ALL_MIN_DIAGS_COMPUTED) return CS_ALL_MIN_DIAGS_COMPUTED;
        else {
            
            this.maxHSSize = newMaxHSSize;
            this.maxNumMinHS = newMaxNumMinHS; 
            int lastLevel = -1;
            LinkedList lastLevelNodes = null;
            LinkedList newLevelNodes = null;

            if (computationState == CS_MAX_HS_SIZE_REACHED) {
                lastLevel = levels.size() - 1;
                boolean expandNodes = (lastLevel < maxHSSize);
                if (expandNodes) {

                    if (computedMinHS.size() < maxNumMinHS) {

                        computationState = CS_COMPUTING;
                        
                        lastLevelNodes = (LinkedList)levels.get(lastLevel);
                        assert(lastLevelNodes != null);
                        newLevelNodes = new LinkedList();
                        levels.add(newLevelNodes);
                        
                        expandLastLevel(lastLevel, lastLevelNodes, newLevelNodes);
                        ++lastLevel;
                        lastLevelNodes = newLevelNodes;
                        newLevelNodes = null;
                    
                    } else {
                        computationState = CS_MAX_NUM_MIN_HS_REACHED;
                    }

                } else {
                    computationState = CS_MAX_HS_SIZE_REACHED;
                }
                
            } else {       
                assert(computationState == CS_MAX_NUM_MIN_HS_REACHED);

                if (computedMinHS.size() == maxNumMinHS) computationState = CS_MAX_NUM_MIN_HS_REACHED;
                else {
                    computationState = CS_COMPUTING;
                    lastLevel = levels.size() - 2;
                    lastLevelNodes = (LinkedList)levels.get(lastLevel);
                    newLevelNodes = (LinkedList)levels.get(lastLevel + 1);
                }
            }

            while(computationState == CS_COMPUTING) {
                 
                assert(lastLevelNodes != newLevelNodes);
                
                boolean expandNodes = (lastLevel < maxHSSize);
                if (expandNodes) {
                    if (newLevelNodes == null) {
                        newLevelNodes = new LinkedList();
                        levels.add(newLevelNodes);
                    }
                } else {
                    newLevelNodes = null;
                }
                
                processLastLevel(lastLevel, lastLevelNodes, newLevelNodes, expandNodes);
                
                if (expandNodes) {
                    //System.out.println("expand nodes, state = " + computationState);
                    if (computationState == CS_COMPUTING) {
                        lastLevelNodes = newLevelNodes;
                        newLevelNodes = null;
                        ++lastLevel;
                        assert(lastLevelNodes.size() > 0);
                    }
                } else {
                    //System.out.println("not expand nodes, state = " + computationState);
                    if (computationState == CS_COMPUTING) {
                        computationState = CS_MAX_HS_SIZE_REACHED;
                    }
                }

            }

            assert(computationState != CS_COMPUTING);
            //System.out.println("computeMore(): computationState: " + computationState);
            return computationState;
        }        
        
    }  // computeMore()
    

    // Performs the call: "TP(SD, COMP-H(n), OBS)" (in terms of Reiter's formalization).
    // Returns false if an inconsistency is detected.
    // If inconsistent: all NAB components in the conflict set are returned in nabConflicts,
    // the AB comp. in abConflicts.
    protected ConflictSet callTheoremProver(SortedIntList edgeLabels, ConflictSet label) {

        //System.out.println("Reiter: callTheoremProver(): size(edgeLabels) = " 
        //                   + edgeLabels.size());

        SortedIntList nabAssumptions = componentIdSet.subtract(edgeLabels);  // compute COMP-H(n)

        // set components in COMP-H(n) to "nab"

        Iterator itAss = nabAssumptions.iterator();
        while(itAss.hasNext()) {
            int assIndex = ((Integer)itAss.next()).intValue();
            Component c = (Component)componentList.get(assIndex);
            
            assert(c.nab != null);
            theoremProver.setAssumption(c.nab, true);

            if (c.ab != null) theoremProver.setAssumption(c.ab, false);
        }

        // set components in H(n) to "ab"
        
        Iterator itNegAss = edgeLabels.iterator();
        while(itNegAss.hasNext()) {
            int negassIndex = ((Integer)itNegAss.next()).intValue();
            Component c = (Component)componentList.get(negassIndex);

            assert(c.nab != null);
            theoremProver.setAssumption(c.nab, false);
            
            if (c.ab != null) theoremProver.setAssumption(c.ab, true); 
         }

        //System.out.println("CALL THEOREM PROVER: set add. to false: " + edgeLabels);

        // consistency check

        ++numTPCalls;
        if (theoremProver.isConsistent()) {
            //System.out.println("TP: consistent!");
            return null;  // consistent!!
        } else {

            // determine conflict set (set of assumptions) and add them to fmConflictSets

            ArrayList conflictingAss = theoremProver.contradiction().collectAssumptions();
            ConflictSet cs = new ConflictSet();

            Iterator itConflictingAss = conflictingAss.iterator();
            //System.out.print("TP returns conflict: ");
            while(itConflictingAss.hasNext()) {
                Assumption a = (Assumption)itConflictingAss.next();
                //System.out.print((String)a.getIdentifier() + ", ");

                assert(a.objTag != null);
                Component c = (Component)a.objTag;
                if (c.nab == a) {
                    cs.nabComps.addSorted(c.id);
                    label.nabComps.addSorted(c.id);  
                }
                else {
                    assert(c.ab == a);
                    cs.abComps.addSorted(c.id);
                    label.abComps.addSorted(c.id);
                }
            }

            return cs;
        }

    }  // callTheoremProver()


    /*
     * This node and all of its children are "removed", except of those nodes with another
     * parent which is not removed.
     */
    protected void prune(HSNodeFM node, HSNodeFM prunedParent) {

        assert(nodeInvariant(node));

        if ((node.state == HSNodeFM.STATE_OPEN) || (node.state == HSNodeFM.STATE_EXPANDED)
            || (node.state == HSNodeFM.STATE_MINIMAL)) {

            boolean hasAliveParents;  // are there any parents which are NOT relabelled/removed?
            if (node.parents.size() == 1) hasAliveParents = false;
            else {
                assert(node.parents.size() > 1);
                
                hasAliveParents = false;

                Iterator itParents = node.parents.iterator();
                while (itParents.hasNext()) {
                    HSNodeFM parent = (HSNodeFM)itParents.next();
                    if ((parent != prunedParent) && (parent.state != HSNodeFM.STATE_PRUNED)) {
                        hasAliveParents = true;
                        break;
                    }
                }
            }

            if (!hasAliveParents) {  // prune only if there are no alive parents!
                
                node.state = HSNodeFM.STATE_PRUNED;
                ++numPrunings;

                //System.out.println("REMOVE node: " + node);

                Iterator itChildren = node.children.iterator();
                while (itChildren.hasNext()) {
                    HSNodeFM child = ((EdgeNodePairFM)itChildren.next()).node;
                    if (child.state != HSNodeFM.STATE_PRUNED) {
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
    protected void relabel(HSNodeFM node, HSNodeFM newNode) {

        // precondition
        assert((node.state != HSNodeFM.STATE_MINIMAL) 
               &&  newNode.label.nabComps.properSubsetOf(node.label.nabComps)
               && newNode.label.abComps.subsetOf(node.label.abComps));
           
        assert(nodeInvariant(node));
        assert(nodeInvariant(newNode));
        
        // relabel node

        SortedIntList removeLabel = node.label.nabComps.subtract(newNode.label.nabComps);
        node.label = newNode.label;
        fmConflictSets.set(node.conflictSetIndex, new ConflictSet(newNode.label));
        newNode.conflictSetIndex = node.conflictSetIndex;

        // iterate through children: if child is to be removed, call prune() for it

        int prunedChildrenCount = 0;
        Iterator itChildren = node.children.iterator();
        while (itChildren.hasNext()) {
            EdgeNodePairFM child = (EdgeNodePairFM)itChildren.next();

            if (removeLabel.contains(child.edge)) {  // determine if child is to be removed
                
                if (child.node.state != HSNodeFM.STATE_PRUNED) {
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
    protected int computeNumUnprunedChildren(HSNodeFM node) {
        int n = 0;
        Iterator itChildren = node.children.iterator();
        while (itChildren.hasNext()) {
            HSNodeFM child = ((EdgeNodePairFM)itChildren.next()).node;
            if (child.state != HSNodeFM.STATE_PRUNED) ++n;
        }

        return n;
    }


    // node has just been labelled, now check if DAG can be pruned using this label
    protected void attemptPrune(HSNodeFM node) {

        // precondition
        assert(((node.state == HSNodeFM.STATE_OPEN) || (node.state == HSNodeFM.STATE_EXPANDED))
               && ((node.label.nabComps.size() > 0) || (node.label.abComps.size() > 0))
               && (node != rootNode) && nodeInvariant(node));

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

            SortedIntList nabIntersectionSet = new SortedIntList();
  
            // iterate through nodes of a level

            while(itLevelNodes.hasNext()) {

                HSNodeFM otherNode = (HSNodeFM)itLevelNodes.next();
                
                if (((otherNode.state == HSNodeFM.STATE_OPEN) || (otherNode.state == HSNodeFM.STATE_EXPANDED)) 
                    && (otherNode.label != null) 
                    && otherNode.relevantForPruning) {
                    
                    // determine if we can prune; if not: are there irrelevant subtrees?

                    // does the NAB part of the labels of the two nodes intersect?
                    nabIntersectionSet.clear();
                    boolean intersects = otherNode.label.nabComps.intersection(node.label.nabComps, nabIntersectionSet);

                    assert((!intersects && (nabIntersectionSet.size() == 0))
                           || (intersects && (nabIntersectionSet.size() != 0)));
                   
                    // "A.properSubsetOf(B)" is equivalent to "A.intersects(B) and (A = intersection(A, B))
                    // and (size(A) < size(B))"
                    boolean canPrune = (intersects && (nabIntersectionSet.size() == node.label.nabComps.size())
                                        && (node.label.nabComps.size() < otherNode.label.nabComps.size())
                                        && node.label.abComps.subsetOf(otherNode.label.abComps));
                  
                    assert(canPrune && node.label.nabComps.properSubsetOf(otherNode.label.nabComps)
                           && node.label.abComps.subsetOf(otherNode.label.abComps)
                           ||
                           !canPrune && (!node.label.nabComps.properSubsetOf(otherNode.label.nabComps)
                                         || !node.label.abComps.subsetOf(otherNode.label.abComps))
                        );

                    if (canPrune) {  // prune!

                        //System.out.println("MinHittingSetsFM: prune node, relabel " + otherNode + "  with " + node);

                        relabel(otherNode, node);
                        pruned = true;

                    } else {  // cannot prune: investigate which subtrees are not relevant for further search
    
                        if ((nabIntersectionSet.size() > 0)        // some subtrees are not relevant
                            && (otherNode.children.size() > 0)) {   // .. and otherNode has already been expanded
                            
                            //System.out.println("IGNORE children of node: " + otherNode + ";   child edges: " + nabIntersectionSet); 

                            Iterator itIntersection = nabIntersectionSet.iterator();
                            int nextItem;
                            if (itIntersection.hasNext()) 
                                nextItem = ((Integer)itIntersection.next()).intValue();
                            else nextItem = Integer.MAX_VALUE;
                            
                            Iterator itChildren = otherNode.children.iterator();
                            while (itChildren.hasNext()) {
                                EdgeNodePairFM child = (EdgeNodePairFM)itChildren.next();
                                
                                if (child.edge == nextItem) {
                                    child.node.relevantForPruning = false;
                                    assert(node.label.nabComps.contains(child.edge));
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
                                HSNodeFM child = ((EdgeNodePairFM)itChildren.next()).node;
                                child.relevantForPruning = true;
                            }
                            relevantNodesExist = true;

                        }

                    }  // !canPrune

                }  // node is open and relevant for pruning

                else if (!otherNode.relevantForPruning) {  // if node not relevant => children are not rel., too

                    Iterator itChildren = otherNode.children.iterator();
                    while(itChildren.hasNext()) {
                        HSNodeFM child = ((EdgeNodePairFM)itChildren.next()).node;
                        child.relevantForPruning = false;
                    }

                }

            }  // iterate through nodes of a level

        }  // iterate through levels

        if (pruned) fmConflictSets.set(oldConflictSetIndex, null);
        attemptedPruneNode = null;

        assert(nodeInvariant(node));

    }  // attemptPrune()

    /*
     * Searches in the list of already computed conflict sets for a conflict set cs
     * which refutes edgeLabel (= H(n) in Reiter's notation), i.e., cs shows that H(n)
     * is inconsistent with SD and OBS.
     *
     * If no such conflict is found, then this method returns -1. Otherwise, it returns
     * an index to the collection conflictSets such that the following holds for cs (= conflictsSets[index]):
     *   cs.abComps is an improper subset of H(n)  [= edgeLabel]
     *   AND cs.nabComps does not intersect H(n)
     *   
     */
    protected int searchRefutingCS(SortedIntList edgeLabel) {
        for (int i = 0; i < fmConflictSets.size(); ++i) {

            Object o = fmConflictSets.get(i);
            if (o != null) {  // the conflict set at this index may have been removed by pruning!
                ConflictSet cs = (ConflictSet)fmConflictSets.get(i);
                assert((cs.nabComps.size() > 0) || (cs.abComps.size() > 0));
                
                /*
                System.out.println("searchRefutingCS: cs.nabComps: " + cs.nabComps + ";  abComps: " + cs.abComps);
                System.out.println("searchRefutingCS: edgeLabel: " + edgeLabel);

                if (cs.abComps.subsetOf(edgeLabel)) {
                    System.out.println("SUBSET");
                }
                else System.out.println("NOT SUBSET");

                if (cs.nabComps.intersects(edgeLabel)) {
                    System.out.println("INTERSECTS");
                }
                else System.out.println("NOT INTERSECTS");
                */
                
                if (cs.abComps.subsetOf(edgeLabel)
                    && !cs.nabComps.intersects(edgeLabel)) {
                    
                    return i;
                }

                
                    
            }
        }

        return -1;
    }

    /*
     * Compute label of node: either it is detected that this node is a min. HS,
     * or it is labelled by a new conflict set. The CS is either taken from the 
     * provided conflict sets, or it is computed by calling the theorem prover.
     */
    protected void computeLabel(HSNodeFM node) {

        assert(node.state == HSNodeFM.STATE_OPEN);
        assert(node.label == null);
        assert(nodeInvariant(node));

        // check if node can be closed

        if (canClose(node)) {
            node.state = HSNodeFM.STATE_CLOSED;
            //System.out.println("CLOSE node: " + node);
        }

        else {

            // before making call to TP: check if there is already a conflict set which refutes this node

            int existingCSIndex = searchRefutingCS(node.edgeLabels);
            if (existingCSIndex >= 0) {
                assert(callTheoremProver(node.edgeLabels, new ConflictSet()) != null);
                ConflictSet existingCS = (ConflictSet)fmConflictSets.get(existingCSIndex);
                assert(existingCS != null);
                node.label = existingCS;
                node.conflictSetIndex = existingCSIndex;
                //System.out.println("REFUTE mode assignment using an existing conflict set: {" + existingCS + "} refutes " + node);
                //if (!conflictsAreMinimal && (node != rootNode)) attemptPrune(node);
                
            
            } else {  // no refuting conflict set found

                ConflictSet label = new ConflictSet();
                ConflictSet csForDB = callTheoremProver(node.edgeLabels, label);
                boolean consistent = (csForDB == null);
                if (consistent) {
                    node.state = HSNodeFM.STATE_MINIMAL;
                }
                else {
                    fmConflictSets.add(csForDB);
                    if (label.nabComps.size() == 0) {  // conflict: if only AB's, no NAB's => close node
                        node.state = HSNodeFM.STATE_CLOSED;
                    }
                    else {
                        node.label = label;
                        node.conflictSetIndex = fmConflictSets.size() - 1;
                        //System.out.println("computed label of node: " + node);
                        if (!conflictsAreMinimal && (node != rootNode)) attemptPrune(node);
                    }
                }
            }
            
        }  // node cannot be closed

        assert(nodeInvariant(node));

    }  // computeLabel()


    // Returns true iff the node can be closed (the 2nd pruning rule).
    protected boolean canClose(HSNodeFM node) {
        
        assert(nodeInvariant(node));

        Iterator itMinHS = computedMinHS.iterator();
        while (itMinHS.hasNext()) {
            HSNodeFM n1 = (HSNodeFM)itMinHS.next();
            if (n1.state == HSNodeFM.STATE_MINIMAL) {
                SortedIntList hs = n1.edgeLabels;
                if (hs.subsetOf(node.edgeLabels)) {
                    assert(hs.properSubsetOf(node.edgeLabels));
                    return true;
                }
            } else assert(n1.state == HSNodeFM.STATE_PRUNED);
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

            HSNodeFM node = (HSNodeFM)itLastNodes.next();
            assert(nodeInvariant(node));
            if (node.state == HSNodeFM.STATE_OPEN) {

                if (node.label == null) {  // node may already be refuted by a CS
                    computeLabel(node);
                }

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
            } else if (node.state == HSNodeFM.STATE_EXPANDED) {
                hasOpenNodes = true;
            }

            assert(nodeInvariant(node));

        }  // while(itLastNodes.hasNext())

        if ((computationState == CS_COMPUTING) && !hasOpenNodes) {
            computationState = CS_ALL_MIN_DIAGS_COMPUTED;
        }
    }

    protected void expandLastLevel(int lastLevel, LinkedList lastLevelNodes, 
                                   LinkedList newLevelNodes) {

        boolean hasOpenNodes = false;

        Iterator itLastNodes = lastLevelNodes.iterator();
        while(itLastNodes.hasNext()) {

            HSNodeFM node = (HSNodeFM)itLastNodes.next();
            assert(nodeInvariant(node));
            
            if (node.state == HSNodeFM.STATE_OPEN) {
                assert(node.label != null);  // this method presumes that a TP call has already been performed 
                hasOpenNodes = true;
                expandNode(node, newLevelNodes, lastLevel);               
            }
                
            assert(nodeInvariant(node));

        }  // while(itLastNodes.hasNext())

        assert(computationState == CS_COMPUTING);
        if (!hasOpenNodes) {
            computationState = CS_ALL_MIN_DIAGS_COMPUTED;
        }
    }


    /*
     * Expand a node: generate child nodes and set their edgeLabels field.
     * The new nodes are added to newNodes.
     */
    protected void expandNode(HSNodeFM node, LinkedList newNodes, int lastLevel) {
        // precondition
        assert((node.label != null) && nodeInvariant(node));

        ++numExpansions;

        int oldNewNodesSize = newNodes.size();
        int newEdge;

        // iterate through the conflict set of this node

        Iterator itLabel = node.label.nabComps.iterator();
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
                HSNodeFM m = (HSNodeFM)itNewNodes.next();
                if (newEdgeLabels.equals(m.edgeLabels)) {  // reuse of node m!!
                    node.children.add(new EdgeNodePairFM(newEdge, m));
                    m.parents.add(node);
                    reused = true;
                    ++numReuses;
                    //System.out.println("REUSE node m: " + m);
                    break;
                }
            }

            if (!reused) {  // no reuse possible: create new node

                HSNodeFM newNode = new HSNodeFM();
                newNode.edgeLabels = newEdgeLabels;

                node.children.add(new EdgeNodePairFM(newEdge, newNode));
                newNode.parents.add(node);
                newNodes.add(newNode);

                //System.out.println("NO reuse possible, create new node: " + newNode);
            }

        }  // while(itLabel.hasNext())
 
        node.state = HSNodeFM.STATE_EXPANDED;
        assert(nodeInvariant(node));
        
    }  // expandNode()

    // for testing purposes only.
    public boolean checkMinimalityHS() {

        for (int i = 0; i < computedMinHS.size(); ++i) {

            HSNodeFM node = (HSNodeFM)computedMinHS.get(i);

            if (node.state == HSNodeFM.STATE_MINIMAL) {

                for (int j = i + 1; j < computedMinHS.size(); ++j) {
                    
                    HSNodeFM otherNode = (HSNodeFM)computedMinHS.get(j);
                    
                    if ((otherNode.state == HSNodeFM.STATE_MINIMAL) 
                        && node.edgeLabels.subsetOf(otherNode.edgeLabels) 
                        || otherNode.edgeLabels.subsetOf(node.edgeLabels)) return false;
                    
                }
            }
        }

        return true;
    }

    // for testing purposes only.
    protected boolean nodeInvariant(HSNodeFM node) {

        boolean a = ((node.label == null) || !node.label.nabComps.intersects(node.edgeLabels));
        boolean b = (node.conflictSetIndex == -1) || (node.label.equals(fmConflictSets.get(node.conflictSetIndex)));
        boolean c = ((node == rootNode) || (node.parents.size() >= 1));

        boolean result = (a && b && c);

        if (!result) {
            if (!a) System.out.println("NODE INVARIANT VIOLATED: a");
            if (!b) {
                System.out.println("NODE INVARIANT VIOLATED: b");
                System.out.println("node: " + node);
                System.out.println("fmConflictSets.get(node.conflictSetIndex): " + fmConflictSets.get(node.conflictSetIndex));
            }
            if (!c) System.out.println("NODE INVARIANT VIOLATED: c");
        }

        return result; 
                 
                /*&& ((node.children.size() == 0) || (node.children.size() == node.label.size())
                    || (node.label.size() == computeNumUnprunedChildren(node))
                    || (node.state == HSNodeFM.STATE_PRUNED))*/
    }
}


/*
 * Internally used by MinHittingSetsFM.
 * A record which encapsulates a component.
 */
class Component {

    public Component(String name, int id) {
        this.name = name;
        this.id = id;
    }

    // The name of the component. E.g., if there is an assumption "NAB(C1)", then name = "C1".
    String name;

    /* 
     * In internal integer ID is assigned to each component. 
     * This allows for efficient set operations (e.g., create the diff of two integer sets).
     */
    int id;

    // an assumption of the style "NAB(x)"
    Assumption nab;

    // an assumption of the style "AB(x)"
    Assumption ab;


}

class ConflictSet {

    SortedIntList abComps;

    SortedIntList nabComps;

    public ConflictSet(SortedIntList abComps, SortedIntList nabComps) {
        this.abComps = abComps;
        this.nabComps = nabComps;
    }

    public ConflictSet() {
        abComps = new SortedIntList();
        nabComps = new SortedIntList();
    }

    public ConflictSet(ConflictSet other) {
        abComps = (SortedIntList)other.abComps.clone();
        nabComps = (SortedIntList)other.nabComps.clone();
    }

    public String toString() {
        return "AB: [" + abComps + "];  NAB: [" + nabComps + "]";
    }

    public boolean equals(Object o) {
        ConflictSet other = (ConflictSet)o;
        return (abComps.equals(other.abComps) && nabComps.equals(other.nabComps));
    }
}

class EdgeNodePairFM {

    public int edge = -1;

    public HSNodeFM node;

    public EdgeNodePairFM(int edge, HSNodeFM node) {
        this.edge = edge;
        this.node = node;
    }

}

// A node in the HS-DAG
class HSNodeFM {

    final static int STATE_OPEN = 0;  // The node is neither closed nor is it a min. HS
    final static int STATE_CLOSED = 1;  // The node is closed (see the "Close" rule in Reiter).
    final static int STATE_MINIMAL = 2;  // The node is a min HS
    final static int STATE_PRUNED = 3;  // Removed by pruning algorithm.
    final static int STATE_EXPANDED = 4;


    int state = STATE_OPEN;

    // the H(n), in terms of Reiter's formalization.
    SortedIntList edgeLabels;  
    
    // parent nodes
    ArrayList parents = new ArrayList();
    
    // child nodes: each element is of type EdgeNodePair
    ArrayList children = new ArrayList();

    ConflictSet label;

    // The index of label in the conflictSets data structure.
    int conflictSetIndex = -1;

    // flag during pruning algorithm; if false: this node (and all of its children!) do not
    // need to be considered for pruning.
    boolean relevantForPruning = true;

    public String toString() {

        String result = "{" + edgeLabels + "}; ";
        
        if ((state == STATE_OPEN) || (state == STATE_PRUNED) || (state == STATE_EXPANDED)) {
            if (label == null) result = result + "{??}";
            else {
                result += "label NAB: {" + label.nabComps + "}";
                if (label.abComps.size() > 0) result += ", label AB: {" + label.abComps + "}";
            }
            if (state == STATE_PRUNED) result = result + "  PRUNED";
        } 
        else if (state == STATE_CLOSED) {
            result = result + "X";
        } else if (state == STATE_MINIMAL) {
            result = result + "!!!";
        }

        result = result + "  #(parents): " + parents.size() 
            + "  #(children): " + children.size();

        return result;
    }


}
