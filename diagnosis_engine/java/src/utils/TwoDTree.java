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
 * This class implements a two dimensional d tree (2d-tree) which
 * can be used in multimedia and graphics applications.
 *
 * @version 1.0, Date 10.05.2000
 * @author Franz Wotawa
 *
 *
 */
public class TwoDTree extends Object {

    protected Object info;
    protected double xval;
    protected double yval;
    protected TwoDTree parent;
    protected TwoDTree llink;
    protected TwoDTree rlink;
    protected int level;

    /** This method creates a new instance.
	@return A new instance
    */
    public TwoDTree() {
	initialize();
    }

    /** This method creates a new instance using the given parameters.
	@param element, a new info object.
	@param xPos, the x position.
	@param yPos, the y position.
	@return A new instance
    */
    public TwoDTree(Object element, double xPos, double yPos) {
	initialize();
	xval = xPos;
	yval = yPos;
	info = element;
    }

    /** This method initializes the data structure 
     */
    public void initialize() {
	info = null;
	xval = 0.0d;
	yval = 0.0d;
	parent = null;
	llink = null;
	rlink = null;
	level = 0;
    }

    public Object info() {
	return info;
    }

    public void rlink(TwoDTree n) {
	rlink = n;
    }

    public TwoDTree rlink() {
	return rlink;
    }

    public void llink(TwoDTree n) {
	llink = n;
    }

    public TwoDTree llink() {
	return llink;
    }

    public void parent(TwoDTree n) {
	parent = n;
    }

    public TwoDTree parent() {
	return parent;
    }

    public int level() {
	return level;
    }

    public void level(int l) {
	level = l;
    }

    public boolean isTopNode() {
	return parent == null;
    }

    public boolean isLeafNode() {
	return (llink == null) && (rlink == null);
    }

    /** This method adds a new element to the collection.
	@param element, a new info object.
	@param xPos, the x position.
	@param yPos, the y position.
    */
    public void addElement(Object element, double xPos, double yPos) {
	addElement(element,xPos,yPos,(level%2)==1);
    }

    /** This method adds a new element to the collection using the
	level of the current node.
	@param element, a sorted collection element.
	@param xPos, the x position.
	@param yPos, the y position.
	@param evenLevel, true if the current level is even and false
	otherwise.  
    */
    public void addElement(Object element, double xPos, double yPos,
			   boolean evenlevel) {
	if ((xval == xPos) && (yval == yPos)) {
	    info = element;
	} else {
	    if (evenlevel) { // Case: even level
		if (yval < yPos) {
		    if (llink == null) {
			llink = new TwoDTree(element,xPos,yPos);
			llink.parent(this);
			llink.level(level + 1);
		    } else {
			llink.addElement(element,xPos,yPos,false);
		    }
		} else { 
		    if (rlink == null) {
			rlink = new TwoDTree(element,xPos,yPos);
			rlink.parent(this);
			rlink.level(level + 1);
		    } else {
			rlink.addElement(element,xPos,yPos,false);
		    }
		}
	    } else { // Case: odd level
		if (xval < xPos) {
		    if (llink == null) {
			llink = new TwoDTree(element,xPos,yPos);
			llink.parent(this);
			llink.level(level + 1);
		    } else {
			llink.addElement(element,xPos,yPos,true);
		    }
		} else { 
		    if (rlink == null) {
			rlink = new TwoDTree(element,xPos,yPos);
			rlink.parent(this);
			rlink.level(level + 1);
		    } else {
			rlink.addElement(element,xPos,yPos,true);
		    }
		}
	    }	    
	}
    }

    /** This method removes the given element from the collection.
	@param xPos, the element's x position.
	@param yPos, the element's y position.
    */
    public void removeElement(double xPos, double yPos) {
	TwoDTree node = this.getNode(xPos,yPos);
	if (node != null) {
	    if (node.isLeafNode()) {
		if ((node.parent()).llink() == node) {
		    node.parent().llink(null);
		} else {
		    node.parent().rlink(null);
		}
		node.parent(null);
	    } else {
		if (node.rlink() == null) {
		} else {
		}
	    }
	}
    }

    /** This method removes all elements from self.
     */
    public void removeAllElements() {
	initialize();
    }

    /** This method answers the number of elements stored in self.
	@return an integer
    */
    public int size() {
	int lcount = 0, rcount = 0;
	if (llink != null) {
	    lcount = llink.size();
	}
	if (rlink != null) {
	    rcount = rlink.size();
	}
	return 1 + lcount + rcount;
    }

    /** This method is for returning an enumeration of elements.
	@return all elements as Enumeration
    */
    public Enumeration elements() {
	return (elementsVector()).elements();
    }

    public Vector elementsVector() {
	return collectElements(new Vector());
    }

    public Vector collectElements(Vector v) {
	if (info != null) {
	    if (llink != null) {
		llink.collectElements(v);
	    }
	    v.addElement(info);
	    if (rlink != null) {
		rlink.collectElements(v);
	    }
	}
	return v;
    }

    /** This method returns the info object for the given position and
	null if no such object exists.
	@param xPos, the x position.
	@param yPos, the y position.
	@return the info object or null.  
    */
    public Object get(double xPos, double yPos) {
	TwoDTree node = getNode(xPos,yPos);
	if (node == null) {
	    return null;
	} else {
	    return node.info();
	}
    }

    public TwoDTree getNode(double xPos, double yPos) {
	return getNode(xPos,yPos,(level%2)==1);
    }

    public TwoDTree getNode(double xPos, double yPos, boolean evenlevel) {
	if ((xval == xPos) && (yval == yPos)) {
	    return this;
	} else {
	    if (evenlevel) { // Case: even level
		if (yval < yPos) {
		    if (llink == null) {
			return null;
		    } else {
			return llink.getNode(xPos,yPos,false);
		    }
		} else { 
		    if (rlink == null) {
			return null;
		    } else {
			return rlink.getNode(xPos,yPos,false);
		    }
		}
	    } else { // Case: odd level
		if (xval < xPos) {
		    if (llink == null) {
			return null;
		    } else {
			return llink.getNode(xPos,yPos,true);
		    }
		} else { 
		    if (rlink == null) {
			return null;
		    } else {
			return rlink.getNode(xPos,yPos,true);
		    }
		}
	    }	    
	}
    }

    /** This method converts the tree into a string.
	@return a string, representing the tree.
    */
    public String toString() {
	StringBuffer buf = new StringBuffer();
	this.toString(buf,0);
	return buf.toString();
    }

    public void toString(StringBuffer buf, int level) {
	for (int i = level; i>0; i--) {
	    buf.append("\t");
	}
	if (info == null) {
	    buf.append("NO INFO");
	} else {
	    buf.append(info.toString());
	    buf.append("(" + (new Integer(level)).toString() +
		       " | " + (new Double(xval)).toString());
	    buf.append("," + (new Double(yval)).toString() + ")");

	}
	buf.append("\r\n");
	if (llink != null) {
	    llink.toString(buf,level + 1);
	}
	if (rlink != null) {
	    rlink.toString(buf,level + 1);
	}
    }

    /** This method returns a vector of objects that are located
     within the specified bounds.  
     @param xPos1, the smallest x position.
     @param yPos1, the smallest y position.
     @param xPos2, the largest x position.
     @param yPos2, the largest y position.
     @return a vector containing points lying in the specified
     rectangle.  
    */
    public Vector getAll(double xPos1, double yPos1, 
			 double xPos2, double yPos2) {
	Vector v = new Vector();
	return getAll(xPos1,yPos1,xPos2,yPos2,(level%2)==1,v);
    }

    public Vector getAll(double xPos1, double yPos1, 
			 double xPos2, double yPos2,
			 boolean evenlevel, Vector v) {

	if ((xval >= xPos1) && (xval <= xPos2) &&
	    (yval >= yPos1) && (yval <= yPos2)) {
	    v.addElement(info); }

	if (evenlevel) {
	    if ((xval >= xPos1) && (llink != null)) {
		llink.getAll(xPos1,yPos1,xPos2,yPos2,false,v);
	    }
	    if ((xval <= xPos2) && (rlink != null)) {
		rlink.getAll(xPos1,yPos1,xPos2,yPos2,false,v);
	    }
	} else {
	    if ((yval >= yPos1) && (llink != null)) {
		llink.getAll(xPos1,yPos1,xPos2,yPos2,true,v);
	    }
	    if ((yval <= yPos2) && (rlink != null)) {
		rlink.getAll(xPos1,yPos1,xPos2,yPos2,true,v);
	    }
	}
	return v;
    }
}

