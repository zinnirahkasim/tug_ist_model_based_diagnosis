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

public class IntPair implements Comparable {
    
    public int first;
    
    public int last;
    
    public IntPair(int first, int last) {
        this.first = first;
        this.last = last;
    }

    public int compareTo(Object o) {
        IntPair other = (IntPair)o;
        if (first < other.first) return -1;
        else if (first > other.first) return +1;
        else if (last < other.last) return -1;
        else if (last > other.last) return +1;
        else return 0;
    }
}
