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

public class MSortedInteger extends Object
implements MSortedElementInterface 
{
    protected int value;

    public MSortedInteger () {
	this.value = 0;
    }

    public MSortedInteger (int value) {
	this.value = value;
    }

    public int intValue() {
	return value;
    }

    public boolean greater(Object element) {
	MSortedInteger i = (MSortedInteger)element;

	if (this.intValue() > (i.intValue())) {
	    return true;
	} else {
	    return false;
	}
    }

    public String toString() {
	return (new Integer(value)).toString();
    }
}
