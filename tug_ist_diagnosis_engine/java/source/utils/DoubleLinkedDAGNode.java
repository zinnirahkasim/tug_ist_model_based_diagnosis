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

/**
 * A node of a DoubleLinkedDAG.
 *
 * Each node has an (internally used) integer ID, which is automatically assigned by the DAG, 
 * a list of parent nodes, and a list of child nodes. 
 * The varialbe allNodes contains all nodes of the DAG, and this.id corresponds to the index 
 * of this node in allNodes. The parent and child nodes are characterized by their id only.
 */
public class DoubleLinkedDAGNode implements Comparable, Cloneable {

    /* 
     * Set by DoubleLinkedDAG only.
     */
    public int id = -1;

    /*
     * Set by DoubleLinkedDAG only.
     */
    public ArrayList allNodes;

    /*
     * The integers in the list are indexes to allNodes.
     */
    protected SortedIntList parents;

    /*
     * The integers in the list are indexes to allNodes.
     */
    protected SortedIntList children;


    /*
     * Creates an "empty" node.
     */
    public DoubleLinkedDAGNode() {
        parents = new SortedIntList();
        children = new SortedIntList();
    }

    /*
     * The new node is a (deep) clone of n.
     */
    public DoubleLinkedDAGNode(DoubleLinkedDAGNode n) {
        id = n.id;
        allNodes = n.allNodes;
        parents = (SortedIntList)n.parents.clone();
        children = (SortedIntList)n.children.clone();
    }

    /*
     * Returns a (deep) clone of the node.
     */
    public Object clone() {
        DoubleLinkedDAGNode clonedN = new DoubleLinkedDAGNode(this);
        return clonedN;
    }

    /*
     * Compares the id fields.
     */
    public int compareTo(Object other) {

        DoubleLinkedDAGNode on = (DoubleLinkedDAGNode)other;
        if (id < on.id) return -1;
        else if (id == on.id) return 0;
        else return +1;
    }

    public final int getID() {
        return id;
    }

    /*public void setID(int id) {
        this.id = id;
    }

    public void setAllNodes(ArrayList allNodes) {
        this.allNodes = allNodes;
        }*/

    
    public final SortedIntList getParents() {
        return parents;
    }

    public final SortedIntList getChildren() {
        return children;
    }

    public final boolean hasParents() {
        return (parents.size() > 0);
    }

    public final boolean hasChildren() {
        return (children.size() > 0);
    }
    
    /*
     * Returns an iterator whose elements are nodes.
     */
    public final Iterator getParentsIterator() {
        return new NodeIterator(parents);
    }

    /*
     * Returns an iterator whose elements are nodes.
     */
    public final Iterator getChildrenIterator() {
        return new NodeIterator(children);
    }

    public DoubleLinkedDAGNode getFirstParent() {
        assert(hasParents());

        return (DoubleLinkedDAGNode)allNodes.get(parents.getFirstInt());
    }

    public String toString() {
        return Integer.toString(id);
    }

    /*
     * Iterates through node objects.
     */
    protected class NodeIterator implements Iterator {
        
        Iterator silIterator;

        NodeIterator(SortedIntList nodeIDs) {
            silIterator = nodeIDs.iterator();
        }

        public boolean hasNext() {
            return (silIterator.hasNext());
        }

        /*
         * Returns a DoubleLinkedDAGNode instance.
         */
        public Object next() {
            Integer iobj = (Integer)silIterator.next();
            return allNodes.get(iobj.intValue());
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }

    }
}
