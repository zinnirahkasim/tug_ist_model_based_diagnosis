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


public class DoubleLinkedTreeNode {
    
    protected Long id = null;

    protected DoubleLinkedTreeNode parent;

    protected ArrayList children;

    /*
     * Creates a new node which has a unique id.
     *
     * No space is reserved for children.
     */
    public DoubleLinkedTreeNode() {
        children = new ArrayList(0);
    }

    /*
     * Chreates a new node and reserves capacity for the passed number of children.
     */
    public DoubleLinkedTreeNode(int childrenCapacity) {
        this.children = new ArrayList(childrenCapacity);
    }

    /*
     * Should be called by DoubleLinkedTree only.
     */
    protected void setId(long id) {
        this.id = new Long(id);
    }

    /*
     * Should be called by DoubleLinkedTree only.
     */
    protected void setParent(DoubleLinkedTreeNode parent) {
        this.parent = parent;
    }

    public long getId() {
        return id.longValue();
    }

    /*
     * Should be called by DoubleLinkedTree only.
     */
    protected void addChild(DoubleLinkedTreeNode child) {
        children.add(child);
    }

    /*
     * Trims the capacity for children to the current number of children (ensures that no memory is wasted).
     */
    public void trimChildren() {
        children.trimToSize();
    }

    public boolean hasParent() {
        return (parent != null);
    }

    /*
     * Returns the parent of the node or null if the node has no parent.
     */
    public DoubleLinkedTreeNode getParent() {
        return parent;
    }

    public Iterator getChildrenIterator() {
        return children.iterator();
    }

    public List getChildren() {
        return children;
    }

    public boolean hasChildren() {
        return (children.size() > 0);
    }    

    public boolean isLeafNode() {
        return (children.size() == 0);
    }

    public void removeChild(DoubleLinkedTreeNode childNode) {
        Iterator itChildren = children.iterator();
        while (itChildren.hasNext()) {
            Object o = itChildren.next();
            if (childNode == o) {
                itChildren.remove();
                return;
            }
        }

        assert(false);  // if childNode is not among the children of this node
    }

}
