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

public class ModeAssignment implements Cloneable {

    /*
     * A pair of type (Mode, ModeAssignmentDAGNode).
     * This is the type of values in the modes map (below).
     */
    protected class ModeNodePair {

        Mode mode;
        
        ModeAssignmentDAGNode node;  // only valid if maDag != null
        
        ModeNodePair(Mode m, ModeAssignmentDAGNode n) {
            mode = m;
            node = n;
        }

        public boolean equals(Object o) {
            if (!(o instanceof ModeNodePair)) return false;
            ModeNodePair other = (ModeNodePair)o;
            return mode.equals(other.mode);
        }
    }

    /*
     * Map from Component to ModeNodePair.
     */
    protected TreeMap modes = new TreeMap();

    /*
     * This DAG represents the dependencies between components.
     * Created only on demand; if it is null, then it has not yet been created.
     *
     * As soon as it is created (i.e., maDag != null), all methods have to ensure its
     * consistency with this MA.
     */
    protected ModeAssignmentDAG maDag = null;



    /**
     * Creates an empty mode assignment;
     */
    public ModeAssignment() {
    }

    /*
     * Returns the MA-DAG for this MA.
     *
     * If the MA-DAG has not yet been created, then this is done now (lazy creation).
     */
    public ModeAssignmentDAG getMaDag() {
        if (maDag == null) {
            maDag = createModeAssignmentDAG();
        }
        
        assert(invariant());

        return maDag;
    }

    public Object clone() {

        assert(invariant());

        ModeAssignment newMA = new ModeAssignment();
        newMA.maDag = null;

        Iterator itPairs = modes.values().iterator();
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            ModeNodePair newPair = new ModeNodePair(pair.mode, null);
            newMA.modes.put(pair.mode.component, newPair);
        }

        assert(newMA.invariant());

        return newMA;
    }

    public boolean equals(Object o) {

        if (!(o instanceof ModeAssignment)) return false;
        else {
            ModeAssignment other = (ModeAssignment)o;
            return modes.equals(other.modes);            
        }
    }

    /*
     * Returns the number of modes in this MA.
     */
    public int size() {
        return modes.size();
    }

    /*
     * Returns all components in this MA.
     */
    public Set getComponents() {
        return modes.keySet();
    }

    /*
     * Returns true iff the two MAs are of equal size and contain the same components.
     */
    public boolean equalComponents(ModeAssignment other) {
        boolean result;

        if (size() == other.size()) {
            Set comps1 = modes.keySet();
            Set comps2 = other.modes.keySet();
            result = comps1.equals(comps2);

        } else result = false;
        
        assert(postcond_equalComponents(other, result));
        return result;
    }

    /*
     * For test purposes only: this is the postcondition of equalComponents().
     * "result" is the return value of equalComponents() which is to be checked.
     * Returns true iff "result" is correct.
     */
    protected boolean postcond_equalComponents(ModeAssignment other, boolean result) {
        
        boolean correctResult = true;

        Iterator itModes = iterator();
        while (itModes.hasNext()) {
            
            Mode mode = (Mode)itModes.next();
            Mode otherMode = other.getMode(mode.getComponent()); 
            if (otherMode == null) {
                correctResult = false;
                break;
            }
        }

        if (correctResult) {
            Iterator itOtherModes = other.iterator();
            while (itOtherModes.hasNext()) {
                Mode otherMode = (Mode)itOtherModes.next();
                Mode mode = getMode(otherMode.getComponent());
                if (mode == null) {
                    correctResult = false;
                    break;
                }
            }
        }

        return (result == correctResult);
    }

    /*
     * For debugging.
     */
    public String toStringShort() {
        StringBuffer result = new StringBuffer();

        Iterator itValues = modes.values().iterator();
        while (itValues.hasNext()) {
            Mode m = ((ModeNodePair)itValues.next()).mode;
            if (result.length() > 0) result.append("; ");
            result.append(m.toStringShort());
        }

        return result.toString();
    }

    /*
     * Substitutes certain modes.
     *
     * fromMode: one of Mode.MODE_NAB/AB/IF.
     * toMode: one of Mode.MODE_NAB/AB/IF
     *
     * Returns: true iff any modes have been substituted.
     */
    public boolean subst(int fromMode, int toMode) {
        assert((fromMode != Mode.MODE_DF) && (toMode != Mode.MODE_DF));
        assert(invariant());
        
        boolean result = false;

        Set entries = modes.entrySet();
        Iterator itEntries = entries.iterator();
        while (itEntries.hasNext()) {
            Map.Entry entry = (Map.Entry)itEntries.next();
            
            ModeNodePair pair = (ModeNodePair)entry.getValue();
            if (pair.mode.type == fromMode) {
                Mode newMode = pair.mode.getComponent().getMode(toMode, null);
                pair.mode = newMode;
                
                result = true;
            }
        }
        
        assert(invariant());
        return result;
    }

    /*
     * Sets the mode of comp to toMode. 
     *
     * comp must already exist in this mode assignment, AssertionError otherwise.
     * "parent" is ignored if toMode != Mode.MODE_DF.
     *
     * Returns "true".
     */
    public boolean subst(Component comp, int toMode, Component parent) {
        assert(modes.containsKey(comp));
        assert(invariant());

        ModeNodePair pair = (ModeNodePair)modes.get(comp);
        
        // remove edge in MA-DAG, if necessary

        if ((maDag != null) && (pair.mode.type == Mode.MODE_DF)) {
            Component oldParent = pair.mode.getParent();
            ModeAssignmentDAGNode oldParentNode = ((ModeNodePair)modes.get(oldParent)).node;
            maDag.removeEdge(oldParentNode, pair.node);
        }

        // substitute

        Mode newMode = comp.getMode(toMode, parent);
        pair.mode = newMode;

        // create edge in MA-DAG, if necessary

        if ((maDag != null) && (toMode == Mode.MODE_DF)) {
            ModeAssignmentDAGNode parentNode = ((ModeNodePair)modes.get(parent)).node;
            maDag.addEdge(parentNode, pair.node);
        }

        assert(invariant());
        return true;
    }

    /*
     * Like subst(Component, int, Component), but the component may not be in this assignment yet.
     */
    public boolean setMode(Component comp, int toMode, Component parent) {
        assert(invariant());

        Object o = modes.get(comp);
        
        if (o != null) {   // substitute; as above
            
            ModeNodePair pair = (ModeNodePair)o;

            if ((maDag != null) && (pair.mode.type == Mode.MODE_DF)) {
                Component oldParent = pair.mode.getParent();
                ModeAssignmentDAGNode oldParentNode = ((ModeNodePair)modes.get(oldParent)).node;
                maDag.removeEdge(oldParentNode, pair.node);
            }
            
            // substitute
            
            Mode newMode = comp.getMode(toMode, parent);
            pair.mode = newMode;
            
            // create edge in MA-DAG, if necessary
            
            if ((maDag != null) && (toMode == Mode.MODE_DF)) {
                ModeAssignmentDAGNode parentNode = ((ModeNodePair)modes.get(parent)).node;
                maDag.addEdge(parentNode, pair.node);
            }
 
            assert(invariant());
            return true;
        }

        else {   // add new component

            ModeAssignmentDAGNode newNode = null;
            if (maDag != null) newNode = new ModeAssignmentDAGNode(comp);
            
            ModeNodePair pair = new ModeNodePair(comp.getMode(toMode, parent), newNode);
            modes.put(comp, pair);
            
            if (maDag != null) {
                maDag.addNode(newNode);
                if (toMode == Mode.MODE_DF) {
                    ModeAssignmentDAGNode parentNode = ((ModeNodePair)modes.get(parent)).node;
                    maDag.addEdge(parentNode, newNode);
                }
            }

            assert(invariant());
            return false;
        }

    }  // setMode()

    /*
     * Sets the mode of all passed components to toMode.
     *
     * components is an ArrayList of Component. It is not required that a component
     * in this list must already be an element of this mode assignment.
     *
     * toMode must be != Mode.MODE_DF.
     *
     */
    public void setModes(ArrayList components, int toMode) {
        assert(toMode != Mode.MODE_DF);

        Iterator itComp = components.iterator();
        while (itComp.hasNext()) {
            Component c = (Component)itComp.next();
            setMode(c, toMode, null);
        }
    }

    /*
     * Returns the mode of the passed component.
     *
     * If the component is not yet in this assignment, then null is returned.
     */
    public Mode getMode(Component comp) {
        Object o = modes.get(comp);
        if (o == null) return null;
        else {
            return ((ModeNodePair)o).mode;
        }
    }

    public ModeAssignmentDAGNode getNode(Component comp) {
        assert(maDag != null);

        Object o = modes.get(comp);
        if (o == null) return null;
        else {
            return ((ModeNodePair)o).node;
        }
    }

    /*
     * Returns the mode type of the passed component.
     *
     * If the component is not in the mode assignment, then the default mode,
     * defined by defaultModeType, is returned.
     */
    public int getMode(Component comp, int defaultModeType) {
        Object o = modes.get(comp);
        
        if (o == null) return defaultModeType;
        else {
            return ((ModeNodePair)o).mode.type;
        }
    }

    /*
     * Returns the number of modes of type modeType.
     */
    protected int countNumModes(int modeType) {
        int result = 0;
        
        Iterator itModes = getModeIterator(modeType);
        while (itModes.hasNext()) {
            itModes.next();
            ++result;
        }
        
        return result;
    }

    /*
     * Returns the number of primary failed components.
     */
    public int getNumPFCs() {
        int result = (modes.size() - getNumSFCs());
        assert(result == countNumModes(Mode.MODE_AB) + countNumModes(Mode.MODE_IF));
        return result;
    }

    /*
     * Returns the number of secondary failed components.
     */
    public int getNumSFCs() {
        if (maDag == null) {
            return countNumModes(Mode.MODE_DF);
        } else {
            int result = maDag.getNumEdges();
            assert(result == countNumModes(Mode.MODE_DF));
            return result;
        }
    }

    public boolean hasMode(int modeType) {
        Collection values = modes.values();
        Iterator itPairs = values.iterator();
        
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            if (pair.mode.getType() == modeType) return true;
        }

        return false;
    }
   
    /*
     * Returns the #IF of the longest DF chain.
     */
    public int computeLongestDFChain() {
        maDag = getMaDag();
        return maDag.computeMaxPathLen();
    }

    /*
     * The returned iterator iterates through all modes.
     */
    public Iterator iterator() {
        return new ModeIterator();
    }

    /*
     * Returns an iterator whose elements m are of type Mode
     * and m.type == modeType.
     */
    public Iterator getModeIterator(int modeType) {
        return new ModeIterator(modeType);
    }

    /*
     * Like getModeIterator(int), but multiple mode types can be specified.
     */
    public Iterator getModeIterator(SortedIntList modeTypes) {
        return new ModeIterator(modeTypes);
    }

    protected ModeAssignmentDAG createModeAssignmentDAG() {
        
        ModeAssignmentDAG dag = new ModeAssignmentDAG();

        // first iteration through modes: create nodes of DAG

        Iterator itPairs = modes.values().iterator();
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            ModeAssignmentDAGNode n = new ModeAssignmentDAGNode(pair.mode.getComponent());
            dag.addNode(n);
            pair.node = n;
        }

        // second iteration: create edges

        itPairs = modes.values().iterator();
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            if (pair.mode.type == Mode.MODE_DF) {
                Component parentComp = pair.mode.parent;
                ModeAssignmentDAGNode parentNode = ((ModeNodePair)modes.get(parentComp)).node;
                dag.addEdge(parentNode, pair.node);
            } 
        }

        assert(correctMaDag(dag));
        return dag;

    }  // createModeAssignmentDAG()

    protected boolean invariant() {
        if ((maDag != null) && !correctMaDag(maDag)) return false;

        return true;
    }

    /*
     * For test purposes only: returns true iff "dag" is a correct MA-DAG of this MA.
     */
    protected boolean correctMaDag(ModeAssignmentDAG dag) {
        
        if (!dag.invariant()) return false;

        // iterate through modes, check if DAG conforms

        Iterator itPairs = modes.values().iterator();
        while (itPairs.hasNext()) {
            ModeNodePair pair = (ModeNodePair)itPairs.next();
            if (pair.node == null) return false;
            if (!dag.getNodes().contains(pair.node)) return false;
            
            if (pair.mode.type == Mode.MODE_DF) {
                Component parentComp = pair.mode.parent;
                ModeAssignmentDAGNode parentNode = ((ModeNodePair)modes.get(parentComp)).node;
                if (!dag.hasEdge(parentNode, pair.node)) return false;
            }
        }
        
        // iterate through DAG, check if modes conform
        
        Iterator itNodes = dag.iterator();
        while (itNodes.hasNext()) {
            ModeAssignmentDAGNode n = (ModeAssignmentDAGNode)itNodes.next();
            if (!modes.containsKey(n.comp)) return false;
            
            Component parentComp = n.getFailurePred();
            if (parentComp != null) {
                ModeNodePair pair = (ModeNodePair)modes.get(n.comp);
                if ((pair.mode.type != Mode.MODE_DF) || (pair.mode.parent != parentComp)) return false;
            }
        }
        

        return true;

    }  // correctMaDag()

    /*
     * Returns true iff there is a component c in this MA which has an ancestor c_a
     * s.t. c_a is NOT an ancestor of c in the other MA.
     *
     * This method presumes that both MAs contain the same components.
     *
     * Complexity: O( (log(m))^2 * m^2) with m = #modes in this MA
     * [the "log(m)" stems from "other.getNode(..)"]
     */
    protected boolean hasMoreConstrainedNodeThan(ModeAssignment other, GraphMatrix transClosure,
                                                 GraphMatrix otherTransClosure) {
        assert(equalComponents(other));

        ModeAssignmentDAG dag = getMaDag();
        ModeAssignmentDAG otherDag = other.getMaDag();

        // iterate through nodes

        Iterator itNodes = dag.iterator();
        while (itNodes.hasNext()) {
            ModeAssignmentDAGNode node = (ModeAssignmentDAGNode)itNodes.next();
            ModeAssignmentDAGNode otherNode = other.getNode(node.comp);

            // iterate through ancestors of node

            Iterator itAnc = dag.getAncestorIterator(node, transClosure);
            while (itAnc.hasNext()) {
                ModeAssignmentDAGNode anc = (ModeAssignmentDAGNode)itAnc.next();
                assert(dag.isAncestor(anc, node, transClosure));
                       
                ModeAssignmentDAGNode otherAnc = other.getNode(anc.comp);
                if (!otherDag.isAncestor(otherAnc, otherNode, otherTransClosure)) return true;
            }
        }
        
        return false;
    }


    /*
     * THIS METHOD IS DEPRICATED! The algorithm is bad (complexity too high)!
     *
     * Returns -1 if this MA has a weaker order than other, -1 if it has a stricter order,
     * and 0 if both MAs have at least one component which is more constrained than in the
     * other MA.
     *
     * Complexity:  O( 2* (log(m))^2 * m^2) with m = #modes in this MA
     * [2 calls to hasMoreConstrainedNodeThan()]
     */
    public int compareOrderTo(ModeAssignment other, GraphMatrix transClosure,
                              GraphMatrix otherTransClosure) {

        assert(equalComponents(other));

        if (hasMoreConstrainedNodeThan(other, transClosure, otherTransClosure)) {
            
            if (other.hasMoreConstrainedNodeThan(this, otherTransClosure, transClosure)) {
                return 0;
            } else return 1;

        } else {
            if (other.hasMoreConstrainedNodeThan(this, otherTransClosure, transClosure)) {
                return -1;
            } else {
                assert(false);
                return 1000;                
            }
        }
    }


    /////////////////////////////////////////////////////////////////////


    protected class ModeIterator implements Iterator {
      
        protected Iterator modesIterator;

        protected Mode next = null;

        protected SortedIntList modeTypes = null;

        // Iterates through all modes.
        public ModeIterator() {
            modesIterator = modes.values().iterator();
            moveToNext();
        }

        // Iterates through modes of the specified type.
        public ModeIterator(int modeType) {
            this.modeTypes = new SortedIntList();
            this.modeTypes.addSorted(modeType);
            
            Collection values = modes.values();
            modesIterator = values.iterator();

            moveToNext();
        }        
  
        // Iterates through modes of the specified types.
        public ModeIterator(SortedIntList modeTypes) {
            this.modeTypes = modeTypes;
 
            Collection values = modes.values();
            modesIterator = values.iterator();

            moveToNext();
        }

        protected void moveToNext() {
            next = null;

            while (modesIterator.hasNext()) {
                ModeNodePair pair = (ModeNodePair)modesIterator.next();
                if ((modeTypes == null) || modeTypes.contains(pair.mode.getType())) {
                    next = pair.mode;
                    break;
                }
            }
        }

        public boolean hasNext() {
            return (next != null);
        }

        public Object next() {
            Object result = next;
            moveToNext();
            return result;
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }

    }  // nested class ModeIterator
}
