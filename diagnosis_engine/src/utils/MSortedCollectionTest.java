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

public class MSortedCollectionTest
{

    public static void main (String[] args) {
	MSortedCollection col = new MSortedCollection();
	MSortedInteger i;

	i = new MSortedInteger(10);
	col.addElement(i);

	i = new MSortedInteger(9);
	col.addElement(i);

	i = new MSortedInteger(11);
	col.addElement(i);

	i = new MSortedInteger(1);
	col.addElement(i);

	i = new MSortedInteger(1);
	col.addElement(i);

	Iterator e = col.elements();
	MSortedInteger obj;
	System.out.println("TEST " + ((new Integer(col.size())).toString()));
	while (e.hasNext()) {
	    obj = (MSortedInteger)e.next();
	    System.out.println(obj.toString());
	}

	e = col.elementsInverse();
	System.out.println("TEST2 " + ((new Integer(col.size())).toString()));
	while (e.hasNext()) {
	    obj = (MSortedInteger)e.next();
	    System.out.println(obj.toString());
	}

    }

}
