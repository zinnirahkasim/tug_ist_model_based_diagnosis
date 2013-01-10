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


public final class DENode extends DoubleLinkedDAGNode {

    public static final int TYPE_ALPHA = 0;
    public static final int TYPE_BETA = 1;

    public static final int STATUS_FALSE = 0;
    public static final int STATUS_TRUE = 1;
    public static final int STATUS_UNKNOWN = -1;

    // either TYPE_ALPHA or TYPE_BETA
    protected int type;

    // The root of the tree.
    protected DENode rootNode;

    /*
     * The length of the path between rootNode and this node.
     *
     * Note: distanceToRoot is an upper limit for the length of the longest DF-chain in the mode assignment.
     */
    protected int distanceToRoot;

    protected ModeAssignment ma;

    /*
     * Contains Mode objects indicating which modes must not change in descendants of this node.
     *
     * The meaning of a Mode m is different for alpha and beta nodes: 
     *
     * For alpha nodes, m.type = Mode.MODE_IF and this.ma contains m. Furthermore, for all descendants n of
     * this node: n.ma contains m.
     *
     * For beta nodes: m.type = Mode.MODE_DF, and this.ma must not contain m. The same holds for all
     * descendants of this node.
     *
     * Released by DiagnosisEnvironments when no longer needed (see method DENode.releaseConstCompModes()).
     * No longer needed after expanded AND consistency checked.
     */
    protected TreeSet constCompModes;

    // also stored in a ConflictSets instance => this member variable does not need much additional memory
    protected ConflictSet cs = null;

    protected boolean expandedAlpha = false;

    protected boolean expandedBeta = false;

    protected int consistent = STATUS_UNKNOWN;

    // true if a conflict set exists which proves that all descendants of this node are inconsistent
    protected boolean descInconsistent = false;

    protected int numIF = -1;

    protected int minNumIF = -1;

    protected int minimal = STATUS_UNKNOWN;


    public DENode(int type, DENode rootNode, int distanceToRoot, ModeAssignment ma, 
                  TreeSet constCompModes) {

        this.type = type;
        if (rootNode == null) {
            this.rootNode = this;
        } else {
            this.rootNode = rootNode;
        }
        this.distanceToRoot = distanceToRoot;
        this.ma = ma;
        this.constCompModes = constCompModes;
    }

    public int getType() {
        return type;
    }

    public int getDistanceToRoot() {
        return distanceToRoot;
    }

    public String getTypeAsString() {
        if (type == TYPE_ALPHA) return "ALPHA";
        else if (type == TYPE_BETA) return "BETA";
        else {
            assert(false);
            return null;
        }
    }

    public DENode getRootNode() {
        return rootNode;
    }

    public ModeAssignment getModeAssignment() {
        return ma;
    }

    public void setConflictSet(ConflictSet cs) {
        this.cs = cs;
    }

    public ConflictSet getConflictSet() {
        return cs;
    }

    public void setDescInconsistent() {
        descInconsistent = true;
    }

    public boolean descInconsistent() {
        return descInconsistent;
    }

    /*
     * In order to safe memory: dereference constCompModes (=> can be destroyed by garbage collector now)
     */
    public void releaseConstCompModes() {
        constCompModes = null;
    }

    public TreeSet getConstCompModes() {
        return constCompModes;
    }

    public void setConsistent(int status) {
        consistent = status;
    }

    public int consistent() {
        return consistent;
    }

    public void setMinimal(int status) {
        minimal = status;
    }

    public int minimal() {
        return minimal;
    }

    public void setExpanded(int expansionType) {
        if (expansionType == TYPE_ALPHA) 
        {
            assert(this.type == TYPE_ALPHA);
            expandedAlpha = true;
        } else  
        {
            assert(expansionType == TYPE_BETA);
            expandedBeta = true;
        }
    }

    public boolean expanded(int expansionType) {
        if (expansionType == TYPE_ALPHA) 
        {
            return expandedAlpha;
        }
        else {
            assert(expansionType == TYPE_BETA);
            return expandedBeta;
        }
    }

    public int getNumIF() {

        // lazy evaluation
        if (numIF == -1) numIF = ma.getNumPFCs();
        return numIF;
    }

    public void setMinNumIF(int minNumIF) {
        this.minNumIF = minNumIF;
    }

    // This method is only relevant for alpha nodes.
    public int getMinNumIF() {
        return minNumIF;
    }

    /*
     * For debugging.
     */
    public String toStringShort() {
        StringBuffer result = new StringBuffer();

        result.append(Integer.toString(id));
        result.append(": ");
       
        if (type == TYPE_ALPHA) result.append("ALPHA node: ");
        else result.append("BETA node: ");

        /*
          result.append("ROOT_DISTANCE: ");
          result.append(Integer.toString(distanceToRoot));
          result.append("  ");
        */

        result.append(ma.toStringShort());

        result.append("  [");
        if (type == TYPE_ALPHA) {
            result.append("expanded ALPHA: " + expandedAlpha + "; ");
        }
        result.append("expanded BETA: " + expandedBeta);
        result.append("; cons.: " + statusToStr(consistent));
        result.append("; minimal: " + statusToStr(minimal));
        result.append("; #<=IF: " + minNumIF);
        if (descInconsistent) result.append("; DESC. INC.");

        if (constCompModes != null) {
            result.append(";  CCM: {");

            Iterator itCCM = constCompModes.iterator();
            while (itCCM.hasNext()) {
                Mode m = (Mode)itCCM.next();
                result.append(m.toString());
                if (itCCM.hasNext()) {
                    result.append(", ");
                }
            }

            result.append("}");
        }

        result.append("]");

        return result.toString();
    }

    protected String statusToStr(int status) {
        switch (status) {
            case STATUS_TRUE: return "true";
            case STATUS_FALSE: return "false";
            default: return "?";
        }
    }

    protected boolean invariant() {

        // for all DF(c, c_i) in ma: c_i must be IF or DF

        Iterator itModes = ma.getModeIterator(Mode.MODE_DF);
        while (itModes.hasNext()) {
            Mode m = (Mode)itModes.next();
            Mode parM = ma.getMode(m.getComponent());
            if ((parM.getType() != Mode.MODE_IF) 
                && (parM.getType() != Mode.MODE_DF)) {

                System.out.println("ERROR: DENode.invariant() failed: " + 1);
                return false;
            }
        }

        // if this node is BETA: all children must be BETA as well
        
        if (type == TYPE_BETA) {
            Iterator itChildren = getChildrenIterator(); 
            while (itChildren.hasNext()) {
                DENode child = (DENode)itChildren.next();
                if (child.getType() != TYPE_BETA) {

                    System.out.println("ERROR: DENode.invariant() failed: " + 2);
                    return false;
                }
            }
        }

        // mode assignment must not contain any NAB

        if (ma.hasMode(Mode.MODE_NAB)) {
            System.out.println("ERROR: DENode.invariant() failed: " + 3);
            return false;
        }

        // See constCompModes. Checks if constCompModes contains modes of the "right" type
        // and compares constCompModes with ma.

        if (constCompModes != null) {

            // ALPHA
            if (type == TYPE_ALPHA) {
                Iterator itCCM = constCompModes.iterator();
                while (itCCM.hasNext()) {
                    Mode m = (Mode)itCCM.next();
                    if ((m.getType() != Mode.MODE_IF) || (ma.getMode(m.getComponent()) == null)) {
                        System.out.println("ERROR: DENode.invariant() failed: " + 4);
                        return false;
                    }
                }

            } else {  // BETA
                Iterator itCCM = constCompModes.iterator();
                while (itCCM.hasNext()) {
                    Mode m = (Mode)itCCM.next();
                    if ((m.getType() != Mode.MODE_DF) || (ma.getMode(m.getComponent()) == m)) {
                        System.out.println("ERROR: DENode.invariant() failed: " + 5);
                        return false;
                    }
                }       
            }           
        }

        // minNumIF must be less or equal to numIF

        if ((numIF > 0) && (minNumIF > numIF)) {
            System.out.println("ERROR: DENode.invariant() failed: " + 6);
            return false;
        }

        // for all children: child.minNumIF >= this.minNumIF

        Iterator itChildren = getChildrenIterator();
        while (itChildren.hasNext()) {
            DENode child = (DENode)itChildren.next();
            if ((minNumIF > 0) && (child.minNumIF > 0) && (child.minNumIF < this.minNumIF)) {
                System.out.println("ERROR: DENode.invariant() failed: " + 7);
                return false;
            }
        }

        // children must have the same root node as this node

        if (rootNode == null) {
            System.out.println("ERROR: DENode.invariant() failed: " + 8);
            return false;
        }
        itChildren = getChildrenIterator();
        while (itChildren.hasNext()) {
            DENode child = (DENode)itChildren.next();
            if ((child.getRootNode() == null) || (child.getRootNode() != rootNode)) {
                System.out.println("ERROR: DENode.invariant() failed: " + 9);
                return false;
            }
        }

        // this is a root node <=> no parents
        
        if ((rootNode != this) && (parents.size() == 0) 
            || (rootNode == this) && (parents.size() != 0)) {

            System.out.println("ERROR: DENode.invariant() failed: " + 10);
            return false;
        }

        // #parents <= 1
        
        if (parents.size() > 1) {
            System.out.println("ERROR: DENode.invariant() failed: " + 11);
            return false;
        }

        // if marked as "desc. inconsistent": the same must hold for children
        // furthermore: must be marked as inconsistent
        if (descInconsistent) {
            if (consistent == STATUS_TRUE) {
                System.out.println("ERROR: DENode.invariant() failed: " + 12);
                return false;
            }
            itChildren = getChildrenIterator();
            while (itChildren.hasNext()) {
                DENode child = (DENode)itChildren.next();
                if (!child.descInconsistent) {
                    System.out.println("ERROR: DENode.invariant() failed: " + 13);
                    return false;
                }
            }
        }

        // if expanded == false: must not have children
        if (!expandedAlpha && !expandedBeta && hasChildren()) {
            System.out.println("ERROR: DENode.invariant() failed: " + 15);
            return false;
        }

        // if minimal: must be consistent
        if ((minimal == STATUS_TRUE) && (consistent != STATUS_TRUE)) {
            System.out.println("ERROR: DENode.invariant() failed: " + 16);
            return false;
        }

        // if impliedByRC: the consistency of this node is not checked
        /*if (impliedByRC && (consistent != STATUS_UNKNOWN)) {
            System.out.println("ERROR: DENode.invariant() failed: " + 17);
            return false;
            }*/

        return true;

    }  // invariant

}
