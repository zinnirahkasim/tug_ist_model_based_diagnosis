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


public class DoubleLinkedTree {
 
    protected long lastId = -1;

    protected DoubleLinkedTreeNode root;

    protected int size = 0;



    public DoubleLinkedTree(int initialCapacity) {
    }

    public void addNode(DoubleLinkedTreeNode newNode, DoubleLinkedTreeNode parent) {
        assert((root == null) || (parent != null));

        ++lastId;
        newNode.setId(lastId);

        if (root == null) {
            root = newNode;
        }
        if (parent != null) {
            newNode.setParent(parent);
            parent.addChild(newNode);
        }

        ++size;        
        //nodes.put(newNode.id, newNode);
    }

    public int size() {
        return size;
    }

    /*
     * Returns an iterator for the entire tree.
     *
     * Pre-order iteration: the nodes are returned in the same order as they are visited by visitChildrenPreOrder().
     */
    public Iterator iterator() {
        return new DoubleLinkedTreeIterator(root);
    }

    /*
     * Like iterator(), but only startNode and its descendants are iterated.
     */
    public Iterator iterator(DoubleLinkedTreeNode startNode) {
        return new DoubleLinkedTreeIterator(startNode);
    }

    public void visitChildrenPreOrder(DoubleLinkedTreeNode node,
                                      DoubleLinkedTreeVisitor visitor) {

        visitChildrenPreOrder_Recursive(node, visitor);
    }

    public void visitChildrenPreOrder_Recursive(DoubleLinkedTreeNode node,
                                                DoubleLinkedTreeVisitor visitor) {

        visitor.visit(node);
        boolean moreNodes = visitor.wantMoreNodes();
        if (moreNodes) {                
            Iterator itChildren = node.getChildrenIterator();
            while (moreNodes && itChildren.hasNext()) {
                DoubleLinkedTreeNode child = (DoubleLinkedTreeNode)itChildren.next();
                visitChildrenPreOrder_Recursive(child, visitor);
                moreNodes = visitor.wantMoreNodes();
            }
        }
    }

    /*
     * Remove leafNode which MUST really be a leaf.
     */
    public void removeLeafNode(DoubleLinkedTreeNode leafNode) {
        assert(leafNode.isLeafNode());
        //assert(nodes.containsKey(leafNode.id));

        DoubleLinkedTreeNode parent = leafNode.getParent();
        if (parent != null) parent.removeChild(leafNode);
        else {
            assert(leafNode == root);
            root = null;
        }
        //nodes.remove(leafNode.id);
    }

    /*
     * Removes node and all of its descendants from this tree.
     */
    public void removeNodeAndDescendants(DoubleLinkedTreeNode node) {
        DoubleLinkedTreeNode parent = node.getParent();
        if (parent != null) parent.removeChild(node);
        else {
            assert(node == root);
            root = null;
        }
        //removeRecursive(node);
    }

    /*
     * Creates a list containing all nodes of this tree.
     *
     * The order of the resulting list is not guaranteed.
     * Note that this method is computationally expensive, as the list must be created from scratch. 
     */
    public List composeNodeList() {
        ArrayList result = new ArrayList(size());

        DoubleLinkedTreeIterator itNodes = new DoubleLinkedTreeIterator(root);
        while (itNodes.hasNext()) {
            Object node = itNodes.next();
            result.add(node);
        }

        return result;
    }

    /*
    protected removeRecursive(DoubleLinkedTreeNode node) {
        
        Iterator itChildren = node.getChildrenIterator();
        while (itChildren.hasNext()) {
            DoubleLinkedTreeNode child = (DoubleLinkedTreeNode)itChildren.next();
            removeRecursive(child);
        }

        nodes.remove(node.id);
        }*/
}

/*
 * For pre-order iteration of a tree.
 */
class DoubleLinkedTreeIterator implements Iterator {
    
    protected Stack nodesOnRight;

    protected DoubleLinkedTreeNode nextNode;

    
    public DoubleLinkedTreeIterator(DoubleLinkedTreeNode startNode) {
        nextNode = startNode;
        nodesOnRight = new Stack();
    }

    public boolean hasNext() {
        return (nextNode != null);
    }

    public Object next() {
        assert(hasNext());
        
        Object result = nextNode;

        List children = nextNode.getChildren();
        if (children.size() == 0) {  // no children? Take node on top of stack, if stack not empty
            if (nodesOnRight.empty()) nextNode = null;
            else nextNode = (DoubleLinkedTreeNode)nodesOnRight.pop();
        
        } else {  // there are children: the next node is the topleft child, the children on right are pushed on stack
            DoubleLinkedTreeNode firstChild = (DoubleLinkedTreeNode)children.get(0);
            nextNode = firstChild;

            for (int iChildren = 1; iChildren < children.size(); ++iChildren) {
                Object childOnRight = children.get(iChildren);
                nodesOnRight.push(childOnRight);
            }
        }

        return result;
    }
    
    public void remove() {
        throw new UnsupportedOperationException("DoubleLinkedTreeIterator does not provide a \"remove()\" method.");
    }
    
}
