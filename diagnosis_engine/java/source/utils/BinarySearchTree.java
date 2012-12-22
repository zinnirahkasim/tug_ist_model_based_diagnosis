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

public class BinarySearchTree {
    
    protected BinarySearchNode root;

    protected class BinarySearchNode {
    
        public Comparable obj;
        
        public BinarySearchNode left;  // the left node must have a value smaller than this node
        
        public BinarySearchNode right;  // the right node must have a value larger than this node
        
        public BinarySearchNode(Comparable obj) {
            this.obj = obj;
        }

        // The invariant checks if the order (left < this < right) is correct.
        protected boolean invariant() {
            if ((left != null) && !(left.obj.compareTo(this.obj) < 0)) return false;
            if ((right != null) && !(this.obj.compareTo(right.obj) < 0)) return false;
            if ((left != null) && (right != null) && !(left.obj.compareTo(right.obj) < 0)) return false;

            return true;
        }

        public boolean isLeaf() {
            return ((left == null) && (right == null));
        }

    }  // nested class BinarySearchNode


    /*!
     * Add a new object to the tree.
     *
     * Returns \c true iff \a obj has not yet been in the tree. A return value of \c false indicates that
     * the tree remains unchanged.
     */
    public boolean add(Comparable obj) {
        
        boolean result;

        assert(obj != null);  // precondition
        assert(invariant());

        if (root == null) {
            root = new BinarySearchNode(obj);
            result = true;
        }
        else result = addRecursive(root, obj);

        assert(search(obj) == obj);  // postcondition
        assert(invariant());

        return result;
    }

    protected boolean addRecursive(BinarySearchNode node, Comparable obj) {

        int comparison = obj.compareTo(node.obj);

        if (comparison == 0) {  // obj already exists
            assert(obj.equals(node.obj));
            return false;

        } else {  
            
            if (comparison < 0) {  // add to left

                if (node.left == null) {
                    node.left = new BinarySearchNode(obj);  // create new node
                    return true;
                } else {
                    return addRecursive(node.left, obj);  // recursion: add to left sub-tree
                }             

            } else {  // add to right
                
                assert(comparison > 0);

                if (node.right == null) {
                    node.right = new BinarySearchNode(obj);  // create new node
                    return true;
                } else {
                    return addRecursive(node.right, obj);  // recursion: add to right sub-tree
                }
            }
        }

    }  // addRecursive()

    public Comparable search(Comparable obj) {
        return searchRecursive(root, obj);
    }

    protected Comparable searchRecursive(BinarySearchNode node, Comparable obj) {
        
        if (node == null) return null;
        else {

            int comparison = obj.compareTo(node.obj);
            
            if (comparison == 0) {
                assert(node.equals(obj));
                return node.obj;
            } else {
                if (comparison < 0) return searchRecursive(node.left, obj);
                else {
                    assert(comparison > 0);
                    return searchRecursive(node.right, obj);
                }
            }
        }
    }

    

    public boolean delete(Comparable obj) {

        assert(false);  // method not yet supported: look for XXX for positions where code is missing!

        assert(invariant());

        boolean result;
        if (root != null) {

            int comparison = obj.compareTo(root.obj);

            if (comparison == 0) {
                assert(root.obj.equals(obj));
                root = null;
                result = true;

            } else {
                if (comparison < 0) return deleteRecursive(root, root.left, obj);
                else result = deleteRecursive(root, root.right, obj);
            } 
        } else {
            result = false;
        }

        assert(invariant());
        assert(search(obj) == null);

        return result;
    }

    protected boolean deleteRecursive(BinarySearchNode parent, BinarySearchNode node, Comparable obj) {
        
        int comparison = obj.compareTo(node.obj);

        if (comparison == 0) {  // node found!
            deleteNode(parent, node);
            return true;

        } else {  // node not yet found -> continue search

            if (comparison < 0) {
                if (node.left == null) return false;
                else return deleteRecursive(node, node.left, obj);
            } else {
                assert(comparison > 0);
                if (node.right == null) return false;
                else return deleteRecursive(node, node.right, obj);
            }
        }

    }  // deleteRecursive()
    
    protected void deleteNode(BinarySearchNode parent, BinarySearchNode nodeToDelete) {
        
        if (nodeToDelete.isLeaf()) {  // easiest case: node is a leaf

            if (nodeToDelete == parent.left) parent.left = null;
            else {
                assert(nodeToDelete == parent.right);
                parent.right = null;
            }
        
        } else {  // node is not a leaf
            
            if (nodeToDelete.left == null) {  // nodeToDelete has no left child

                if (nodeToDelete == parent.left) parent.left = nodeToDelete.right;
                else {
                    assert(nodeToDelete == parent.right);
                    parent.right = nodeToDelete.right;
                }
            
            } else if (nodeToDelete.right == null) {   // nodeToDelete has no right child

                if (nodeToDelete == parent.left) parent.left = nodeToDelete.left;
                else {
                    assert(nodeToDelete == parent.right);
                    parent.right = nodeToDelete.left;
                }
            
            } else {  // nodeToDelete has 2 children
                
                // XXX

            }

        }

    }  // deleteNode()

    protected boolean invariant() {
        return invariantRecursive(root);
    }

    protected boolean invariantRecursive(BinarySearchNode node) {
        if (node == null) return true;
        else {
            boolean result = (node.invariant() && invariantRecursive(node.left) && invariantRecursive(node.right));
            return result;
        }
    }

}
