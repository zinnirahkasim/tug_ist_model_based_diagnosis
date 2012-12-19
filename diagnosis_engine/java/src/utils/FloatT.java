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

/*!
 * This class provides some common tools for floating point computations.
 */
public class FloatT {

    //! Default precision for comparison of double values.
    public static double DEF_DOUBLE_COMP_PREC = 1.0e-10;

    //! Precision for comparison of double values, can be modified.
    public static double DOUBLE_COMP_PREC = DEF_DOUBLE_COMP_PREC;


    public static boolean eq(double x, double y) {
        return (Math.abs(x - y) <= DOUBLE_COMP_PREC);
    } 

    public static boolean abs_lt(double x, double y) {
        return (Math.abs(x) + DOUBLE_COMP_PREC < Math.abs(y));
    }

    public static boolean abs_gt(double x, double y) {
        return (Math.abs(x) - DOUBLE_COMP_PREC > Math.abs(y));
    }
    
}
