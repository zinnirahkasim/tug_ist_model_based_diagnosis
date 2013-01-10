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

public class MSortedCollectionNode extends Object
{
    protected MSortedCollectionNode left;
    protected MSortedCollectionNode right;
    protected MSortedElementInterface object;

    public MSortedCollectionNode () {
	left = null;
	right = null;
	object = null;
    }

    public void left(MSortedCollectionNode node) {
	left = node;
    }

    public void right(MSortedCollectionNode node) {
	right = node;
    }

    public void addElement(MSortedElementInterface obj) {
	if (object == null) {
	    object = obj;
	    left = new MSortedCollectionNode();
	    right = new MSortedCollectionNode();
	} else {
	    if (object.greater(obj)) {
		left.addElement(obj);
	    } else {
		right.addElement(obj);
	    }
	}
    }

    public ArrayList allElements() {
	ArrayList v = new ArrayList();
	if (object != null) {
	    return this.allElements(v);
	} else {
	    return v;
	}
    }

    public ArrayList allElements(ArrayList v) {
	if (object != null) {
	    left.allElements(v);
	    v.add(object);
	    right.allElements(v);
	}
	return v;
    }

    public ArrayList allElementsInverse() {
	ArrayList v = new ArrayList();
	if (object != null) {
	    return this.allElementsInverse(v);
	} else {
	    return v;
	}
    }

    public ArrayList allElementsInverse(ArrayList v) {
	if (object != null) {
	    right.allElementsInverse(v);
	    v.add(object);
	    left.allElementsInverse(v);
	}
	return v;
    }

}
