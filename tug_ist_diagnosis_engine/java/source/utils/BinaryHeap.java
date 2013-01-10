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

/*
 * A binary heap can be used as a priority queue.
 *
 * The elements are Object's. They are compared using a Comparator which is passed to
 * the constructors of BinaryHeap. The root of the heap is the element which has the smallest key value.
 *
 * The heap may also contain multiple elements which are "equal" wrt the comparator, i.e., the comparator returns 0
 * when comparing those elements. However, a precondition of method add() is that the object instance to be added
 * does not yet exist in the heap.
 *
 */
public class BinaryHeap {
    
    /*
     * IMPORTANT: the logical heap index ranges from 1..size, hence heap[index-1] contains the heap
     * element at this index.
     */
    protected Object[] heap;

    protected int size;
    
    protected float incrementCapacityFactor;

    protected Comparator comparator;

    /*
     * Creates a heap with a size of 0 and the specified initial capacity. The comparator
     * is supposed to return -1 for two elements (e1, e2) if e1 has a higher "priority", i.e., if 
     * e1 should be closer to the root.
     *
     * When the heap runs out of capacity, then the capacity is doubled.
     */
    public BinaryHeap(Comparator comparator, int initialCapacity) {
        assert(initialCapacity >= 0);

        heap = new Object[initialCapacity];
        incrementCapacityFactor = 1.0f;
        size = 0;
        this.comparator = comparator;
    }

    /*
     * An incrementCapacityFactor of, e.g., 0.5 means that, when the heap runs out of capacity,
     * the capacity is increased by 50 percent.
     */
    public BinaryHeap(Comparator comparator, int initialCapacity, float incrementCapacityFactor) {
        assert((initialCapacity >= 0) && (incrementCapacityFactor > 0));

        heap = new Object[initialCapacity];
        this.incrementCapacityFactor = incrementCapacityFactor;
        size = 0;
        this.comparator = comparator;
    }

    public int getCapacity() {
        return heap.length;
    }

    /*
     * The actual number of elements in the heap.
     */
    public int size() {
        return size;
    }

    protected void incrementCapacity() {
        //assert(invariant());

        int newCap = Math.round(getCapacity() * (1 + incrementCapacityFactor));
        
        Object[] newHeap = new Object[newCap];
        System.arraycopy(heap, 0, newHeap, 0, size);
        heap = newHeap;

        //assert(invariant());
    }

    public String toString() {
        String result = "";

        Iterator it = iterator();
        while (it.hasNext()) {
            Object o = it.next();
            result = result += o.toString();
            if (it.hasNext()) result += " | ";
        }

        return result;
    }

    protected boolean invariant() {
        for (int i = 1; i < size / 2; ++i) {
            if (!invariant(i)) {
                System.out.println("BinaryHeap: invariant failed for element " + i + "; size = " + size);
                //System.out.println("[" + this + "]");
                return false;
            }
        }
        return true;
    }

    /*
     * Checks if the parent value of the heap element at index is smaller than its children.
     *
     * Remember, again, that heap[index-1] contains the heap element at index (which ranges from 1..size).
     */
    protected boolean invariant(int index) {
        assert((index >= 1) && (index <= size));
        
        for (int i = 1; i <= size; ++i) {
            if (heap[i - 1] == null) {
                System.out.println("There is a null value in the heap!!!");
                return false;
            }
        }

        int left = index * 2;
        if (left <= size) {
            if (comparator.compare(heap[index - 1], heap[left - 1]) > 0) {
                System.out.println("heap[index - 1] > heap[left - 1]) > 0 !!!");
                return false;
            } else {
                int right = index * 2 + 1;
                if (right <= size) {
                    if (comparator.compare(heap[index - 1], heap[right - 1]) > 0) {
                        System.out.println("heap[index - 1] > heap[right - 1]) !!!");
                        System.out.println("heap[index - 1]:  " + heap[index - 1]
                                           + "\nheap[right - 1]: " + heap[right - 1]);
                        return false;
                    }
                }
            }
        }

        return true;
    }

    public void add(Object newObj) {
        assert((newObj != null) && !contains(newObj));
        //assert(invariant());

        if (size + 1 > heap.length) {
            incrementCapacity();
            assert(size + 1 <= heap.length);
        }

        ++size;
        heap[size - 1] = newObj;
        
        int i = size;
        while (i > 1) {

            int parent = i / 2;
            int comp = comparator.compare(heap[parent - 1], heap[i - 1]);

            if (comp > 0) {

                //assert(invariant(i));
                swap(parent, i);
                //assert(invariant(parent));

                //assert(invariant(i));

                i = parent;
            } else {

                //assert(invariant(parent));
                //assert(invariant(i));
                break;
            }
        }

        //assert(invariant());
    }

    /*
     * Iterates through the Object's in the heap; the order in which the elements are iterates is not guaranteed.
     */
    public Iterator iterator() {
        return new BinaryHeapIterator();
    }

    public Object removeMin() {

        assert(size >= 1);
        //assert(invariant());

        Object result = heap[1 - 1];  // :)
        int oldSize = size;
        --size;

        if (size > 0) {
            int i = 1;
            Object lastElem = heap[oldSize - 1];
            heap[0] = lastElem;
            heap[oldSize - 1] = null;

            while (i < size) {
                int left = i * 2;
                if (left > size) {  // there is no left element
                    //assert(invariant(i));
                    break;
                
                } else {  // there is a left element
                    assert(heap[left - 1] != null);

                    int right = left + 1;
                    if (right <= size) {  // there is a left and a right element
                        assert((heap[left - 1] != null) && (heap[right - 1] != null));

                        int smaller;
                        if (comparator.compare(heap[left - 1], heap[right - 1]) < 0) smaller = left;
                        else smaller = right;
                        
                        if (comparator.compare(heap[i - 1], heap[smaller - 1]) > 0) {
                            swap(i, smaller);
                            i = smaller;
                        } else break;
                        
                    } else {  // there is only a left element
                        assert(heap[left - 1] != null);

                        if (comparator.compare(heap[i - 1], heap[left - 1]) > 0) {
                            swap(i, left);
                            i = left;
                        } else break;

                    }
                }
            
            } 

            assert(contains(lastElem, 1, size));
        
        } else {
            heap[0] = null;
        }
        
        assert(!contains(result, 1, size));         

        //assert(invariant());
        return result;
    
    }  // removeMin()

    protected boolean contains(Object obj) {
        return contains(obj, 1, size);
    }

    protected boolean contains(Object obj, int start, int end) {
        for (int i = start; i <= end; ++i) {
            if (heap[i - 1] == obj) return true;
        }

        return false;
    }

    protected void swap(int index1, int index2) {
        assert(index1 != index2);
        assert((heap[index1 - 1] != null) && (heap[index2 - 1] != null));
        
        Object tmp = heap[index1 - 1];
        heap[index1 - 1] = heap[index2 - 1];
        heap[index2 - 1] = tmp;
    }

    protected class BinaryHeapIterator implements Iterator {
        
        int next;

        public BinaryHeapIterator() {
            next = 1;
        }

        public boolean hasNext() {
            return (next <= size);
        }

        public Object next() {
            assert(hasNext());
            Object result = heap[next - 1];
            ++next;
            return result;
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }

    }
}
