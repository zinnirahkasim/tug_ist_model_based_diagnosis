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
import java.io.*;

/*
 * A graph, where the nodes are denoted by integers, and the edges are represented by 
 * a NxN matrix of bits (N...# of nodes). 
 */
public class GraphMatrix {

    protected int numNodes;

    protected int numEdges;

    protected BitSet[] matrix;

    protected Object[][] tags;


    public class Edge {

        public int from;
        public int to;

        public Edge(int from, int to) {
            this.from = from;
            this.to = to;
        }
    };

    public GraphMatrix(int numNodes) {
        this.numNodes = numNodes;
        numEdges = 0;
        matrix = new BitSet[numNodes];

        for (int i = 0; i < numNodes; ++i) {
            matrix[i] = new BitSet(numNodes);
        }
    }

    public int getNumNodes() {
        return numNodes;
    }

    public int getNumEdges() {
        return numEdges;
    }

    public void addEdge(int fromNode, int toNode) {
        assert((fromNode < numNodes) && (toNode < numNodes));
        matrix[fromNode].set(toNode);
    }

    /*
     * Complexity: O(const). 
     */
    public  boolean hasEdge(int fromNode, int toNode) {
        assert((fromNode < numNodes) && (toNode < numNodes));
        return matrix[fromNode].get(toNode);
    }

    protected void initTags() {
        tags = new Object[numNodes][numNodes];
    }

    // there must already be an edge between these nodes!
    public void setTag(int fromNode, int toNode, Object tag) {
        assert(hasEdge(fromNode, toNode));

        if (tags == null) {
            initTags();
        }
        tags[fromNode][toNode] = tag;
    }

    public Object getTag(int fromNode, int toNode) {
        assert(hasEdge(fromNode, toNode) && (tags != null));

        return tags[fromNode][toNode];
    }

    /*
     * Returns an iterator which iterates through all edges and which next() method returns Edge instances.
     */
    public Iterator getEdgeIterator() {
        return new EdgeIterator();
    }

    /*
     * This iterator iterates through all ancestors of node; the Iterator.next() method returns Integer instances.
     *
     * Note: the complexity of each Iterator.next() method is O(N).
     */
    public Iterator getAncestorIterator(int node) {
        return new AncestorIterator(node);
    }

    /*
     * This iterator iterates through all descendants of node; the next() method returns Integer instances.
     *
     * Note: the complexity of each Iterator.next() method is O(N).
     */
    public Iterator getDescendantIterator(int node) {
        return new DescendantIterator(node);
    }


    // ------------------------- iterator classes ----------------------------------
    

    // probably untested..
    protected class EdgeIterator implements Iterator {
        
        int from = 0;
        int to = -1;
        
        GraphMatrix.Edge next;
        
        public EdgeIterator() {
            moveToNext();
        }
        
        public boolean hasNext() {
            return (next != null);
        }
        
        public Object next() {
            if (next == null) throw new NoSuchElementException();
            GraphMatrix.Edge result = next;
            moveToNext();

            assert(hasEdge(result.from, result.to));
            return result;
        }
        
        public void remove() {
            throw new UnsupportedOperationException();
        }
        
        protected void moveToNext() {
            
            ++to;
            
            while (from < numNodes) {
                while (to < numNodes) {
                    if (matrix[from].get(to)) {
                        next = new GraphMatrix.Edge(from, to);
                        return;
                    }
                    ++to;
                }
                
                ++from;
                to = 0;
            }
            
            next = null;

        }  
        
    }  // class GraphMatrix.EdgeIterator
        
    protected class AncestorIterator implements Iterator {
        
        int desc;
        
        int nextAnc = -1;
        
        
        public AncestorIterator(int desc) {
            this.desc = desc;
            moveToNext();
        }
        
        public Object next() {
            assert(hasNext());
            
            Integer result = new Integer(nextAnc);
            moveToNext();
            return result;
        }
        
        protected void moveToNext() {
            boolean nextAncFound = false;
            
            while (!nextAncFound && (nextAnc + 1 < numNodes)) {
                ++nextAnc;
                nextAncFound = matrix[nextAnc].get(desc);
            }

            if (!nextAncFound) ++nextAnc;

            // postcondition 
            assert(!nextAncFound || hasEdge(nextAnc, desc));
        }
        
        public boolean hasNext() {
            return (nextAnc < numNodes); 
        }
        
        public void remove() {
            throw new UnsupportedOperationException();
        }
        
    };  // class GraphMatrix.AncestorIterator
    
    
    protected class DescendantIterator implements Iterator {
        
        BitSet descs;
        
        int nextDesc = -1;
        
        
        public DescendantIterator(int anc) {
            descs = matrix[anc];
            moveToNext();
        }
        
        public Object next() {
            assert(hasNext());
            
            Integer result = new Integer(nextDesc);
            moveToNext();
            return result;
        }
        
        protected void moveToNext() {
            ++nextDesc;
            nextDesc = descs.nextSetBit(nextDesc);
        }
        
        public boolean hasNext() {
            return (nextDesc != -1); 
        }
        
        public void remove() {
            throw new UnsupportedOperationException();
        }
        
    };  // class GraphMatrix.DescendantIterator

};  // class GraphMatrix
    





