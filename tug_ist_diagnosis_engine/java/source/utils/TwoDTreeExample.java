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

public class TwoDTreeExample
{

    public static void main (String[] args) {
	TwoDTree top;
	Object obj;

	top = new TwoDTree("n0",10,3);
	top.addElement("n1",10,4);
	top.addElement("n2",10,2);
	top.addElement("n3",8,5);
	top.addElement("n4",12,2);

	System.out.println("TREE: ");
	System.out.println(top.toString());
	System.out.println("size = " + (new Integer(top.size())).toString());
	System.out.println("Elements ..." + (top.elementsVector()).toString());

	obj = top.get(10,2);
	if (obj == null) {
	    System.out.println("(10,2) NULL");
	} else {
	    System.out.println("(10,2) " + obj.toString());
	}

	obj = top.get(10,3);
	if (obj == null) {
	    System.out.println("(10,3) NULL");
	} else {
	    System.out.println("(10,3) " + obj.toString());
	}

	obj = top.get(12,1);
	if (obj == null) {
	    System.out.println("(12,1) NULL");
	} else {
	    System.out.println("(12,1) " + obj.toString());
	}

	obj = top.get(12,2);
	if (obj == null) {
	    System.out.println("(12,2) NULL");
	} else {
	    System.out.println("(12,2) " + obj.toString());
	}

	Vector result = top.getAll(10,2,10,3);
	System.out.println(result.toString());

	result = top.getAll(0,0,10,10);
	System.out.println(result.toString());

	result = top.getAll(12,2,12,2);
	System.out.println(result.toString());

	result = top.getAll(1,2,9,4);
	System.out.println(result.toString());

    }

}
