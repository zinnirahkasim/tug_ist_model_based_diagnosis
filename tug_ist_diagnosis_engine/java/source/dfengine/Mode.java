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


package dfengine;

import java.util.*;

import theoremprover.Assumption;

/*
 * Note that Mode objects are immutable: once they are created, they cannot be changed.
 */
public class Mode implements Cloneable, Comparable {

    public final static int MODE_NAB = 0;
    public final static int MODE_AB = 1;
    public final static int MODE_IF = 2;
    public final static int MODE_DF = 3;

    /**
     * one of the MODE_x constants
     */
    protected int type;

    protected Component component;

    /**
     * Only relevant if type = MODE_DF.
     */
    protected Component parent;

    /*
     * The TP assumption which corresponds to this mode.
     *
     * Can be "null".
     */
    protected Assumption assumption;


    /**
     * Use this constructor only if type != MODE_DF.
     *
     * ass may be "null".
     */
    public Mode(int type, Component component, Assumption ass) {
        assert(type != MODE_DF);

        this.type = type;
        this.component = component;
        this.parent = null;
        this.assumption = ass;
    }

    /**
     * Use this constructor only if type == MODE_DF.
     *
     * ass may be "null".
     */
    public Mode(int type, Component component, Component parent, Assumption ass) {
        assert(type == MODE_DF);

        this.type = type;
        this.component = component;
        this.parent = parent;
        this.assumption = ass;
    }

    public boolean equals(Object o) {
        Mode other = (Mode)o;
        return ((this == other) 
                || ((type == other.type) && (component == other.component) && (parent == other.parent))); 
    }

    public int compareTo(Object o) {
        if (equals(o)) return 0;
        else {
            Mode other = (Mode)o;
            int compareComp = component.compareTo(other.component);
            if (compareComp != 0) return compareComp;
            else {
                if (type < other.type) return -1;
                else if (type > other.type) return +1;
                else {
                    assert(type == MODE_DF);
                    int compareParent = parent.compareTo(other.parent);
                    assert(compareParent != 0);
                    return compareParent;
                }
            }
        }
        
    }

    public String toString() {
        
        StringBuffer res = new StringBuffer();
        if (type != MODE_DF) res.append("mode " + typeAsString(type)
                                        + "(" + component.getName() + ")");
        else res.append("mode " + typeAsString(type) + "(" + parent.getName()  
                        + ", " + component.getName() + ")");
        res.append(": ");
        if (assumption == null) res.append("[no assumption]");
        else res.append((String)assumption.getIdentifier());
        
        return res.toString();
    }

    /*
     * For debugging.
     */
    public String toStringShort() {
        if (type != MODE_DF) {
            return typeAsString(type) + "(" + component.getName() + ")";
        } else {
            return typeAsString(type) + "(" + parent.getName()  
                + ", " + component.getName() + ")";
        }    
    }

    static public String typeAsString(int type) {
        switch(type) {
            case MODE_NAB: 
                return "NAB";
            case MODE_AB:
                return "AB";
            case MODE_IF:
                return "IF";
            case MODE_DF:
                return "DF";
            default: assert(false);
        }
        return null;
    }

    public final int getType() {
        return type;
    }

    public final Component getComponent() {
        return component;
    }

    public final Component getParent() {
        return parent;
    }

    public final Assumption getAssumption() {
        return assumption;
    }
}
