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
 * This class implements a sorted collection allowing to add and remove
 * object. Every object has to implement a method greater.
 *
 * @version 1.0, Date 10.05.2000
 * @author Franz Wotawa
 *
 * @see utils.MSortedElementInterface
 *
 */
public class MSortedCollection extends Object {

    protected int elementCount = 0;
    protected MSortedCollectionNode topNode = null;

    /** This method creates a new instance.
	@return A new instance
    */
    public MSortedCollection() {
	topNode = new MSortedCollectionNode();
    }

    /** This method adds a new element to the collection.
	@param element, a sorted collection element.
    */
    public void addElement(MSortedElementInterface element) {
	elementCount = elementCount + 1;
	topNode.addElement(element);
    }

    /** This method adds a vector of elements to the collection.
	@param element, a sorted collection element.
    */
    public void addAllElements(ArrayList v) {
	Iterator e = v.iterator();
	while (e.hasNext()) {
	    addElement((MSortedElementInterface)e.next());
	}
    }


    /** This method removes the given element from the collection.
	@param element, a sorted collection element.
    */
    public void removeElement(MSortedElementInterface element) {
    }

    /** This method removes all elements from self.
     */
    public void removeAllElements() {
	elementCount = 0;
	topNode = new MSortedCollectionNode();
    }

    /** This method answers the number of elements stored in self.
	@return an integer
    */
    public int size() {
	return elementCount;
    }

    /** This method is for returning an enumeration of elements.
	@return all elements as Enumeration
    */
    public Iterator elements() {
	return (topNode.allElements()).iterator();
    }

    public ArrayList elementsVector() {
	return topNode.allElements();
    }

    /** This method is for returning an enumeration of elements in
	inverse order.
	@return all elements as Enumeration
    */
    public Iterator elementsInverse() {
	return (topNode.allElementsInverse()).iterator();
    }

    public ArrayList elementsVectorInverse() {
	return topNode.allElementsInverse();
    }

}









