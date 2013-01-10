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


/*!
 * A random number generated which creates unique \c int values.
 *
 * This generator remembers already generated values and never returns the same value twice.
 * Obviously, this generator should not be called too often, otherwise it will run out of available
 * values. The performance will also degenerate.
 *
 * In general, this simple implementation is not very fast..
 */
public class UniqueRandomNumbers {
    
    protected TreeSet generatedValues = new TreeSet();

    protected Random random = new Random();

    public int nextInt() {
        int value = random.nextInt();
        boolean exists = generatedValues.contains(new Integer(value));
        while (exists) {
            value = random.nextInt();
            exists = generatedValues.contains(new Integer(value));
        }

        generatedValues.add(new Integer(value));
        return value;
    }

}
