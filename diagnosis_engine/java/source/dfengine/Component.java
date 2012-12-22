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

import utils.*;
import theoremprover.Assumption;

public class Component implements Comparable {

    protected String name;

    // internally used; assigned by DiagnosisProblem
    protected int id = -1;

    protected FailureDepNode fdNode;

    protected Mode mode_nab;
    protected Mode mode_ab;
    protected Mode mode_if;

    // map from Component to Mode
    protected TreeMap dfModes = new TreeMap();

    protected double prob_if = -1.0;


    public Component(String name, int id) {
        this.name = name;
        this.id = id;
    }

    public Component(String name, int id, double prob_if) {
        this(name, id);
        this.prob_if = prob_if;
    }

    public boolean equals(Object o) {
        return (this == o);
    }

    public final FailureDepNode getFDNode() {
        return fdNode;
    }

    public final double getProbIF() {
        return prob_if;
    }

    /*
     * Returns a mode of this component with the passed modeType.
     *
     * If modeType = Mode.MODE_DF, then a mode of the form DF(this, parent) is returned.
     * Otherwise, parent is ignored.
     */
    public final Mode getMode(int modeType, Component parent) {
        
        switch (modeType) {
            case Mode.MODE_AB: return mode_ab;
            case Mode.MODE_NAB: return mode_nab;
            case Mode.MODE_IF: return mode_if;
            case Mode.MODE_DF: {
                Object res = dfModes.get(parent);
                assert(res != null);
                return (Mode)res;
            }
            default: assert(false); return null;
        }
    }
    
    public final Mode getModeAB() {
        return mode_ab;
    }

    public final Mode getModeNAB() {
        return mode_nab;
    }

    public final Mode getModeIF() {
        return mode_if;
    }

    /*
     * Returns a mode of the form DF(this, parent)
     */
    public final Mode getModeDF(Component parent) {
        Object res = dfModes.get(parent);
        assert(res != null);

        return (Mode)res;
    }

    public boolean hasFDGParent(Component parent) {
        return (dfModes.get(parent) != null);
    }

    /*
     * Returns modes DF(this, parent) for all FDG parents of this.
     */
    public final Collection getModesDF() {
        return dfModes.values();
    }

    /*
     * For debugging.
     */
    public String toString() {
        StringBuffer res = new StringBuffer("component \"" + name + "\":" + "\n");
        res.append("ID: " + id);
        res.append("\n");
        res.append(mode_nab.toString());
        res.append("\n");
        res.append(mode_ab.toString());
        res.append("\n");
        res.append(mode_if.toString());
        res.append("\n");

        if (dfModes != null) {
            Iterator it = dfModes.values().iterator();
            while (it.hasNext()) {
                Mode m = (Mode)it.next();
                res.append(m.toString());
                res.append("\n");
            }
            
        }
        
        return res.toString();
    }

    public int compareTo(Object o) {
        Component other = (Component)o;

        assert(id >= 0);
        assert(other.id >= 0);

        if (this.id < other.id) return -1;
        else if (this.id == other.id) return 0;
        else return +1;
    }

    public final String getName() {
        return name;
    }

    public final int getID() {
        return id;
    }

    protected String createAss(String predicate) {
        return predicate + "(" + name + ")";
    }

    /*
     * Initializes the mode variables of this component. 
     * Parameter "assumptions" is a Map from String to Assumption
     */
    public void initFromFDG(Map assumptions, 
                            String assAB, String assNAB, String assIF,
                            String assDF) {
         
        String ass = createAss(assAB);
        Object o = assumptions.get(ass);
        Assumption a = null;
        if (o != null) a = (Assumption)o;
        mode_ab = new Mode(Mode.MODE_AB, this, a);

        ass = createAss(assNAB);
        o = assumptions.get(ass);
        a = null;
        if (o != null) a = (Assumption)o;
        mode_nab = new Mode(Mode.MODE_NAB, this, a);      

        ass = createAss(assIF);
        o = assumptions.get(ass);
        a = null;
        if (o != null) a = (Assumption)o;
        mode_if = new Mode(Mode.MODE_IF, this, a); 

        Iterator itPar = fdNode.getParentsIterator();
        while (itPar.hasNext()) {
            FailureDepNode parent = (FailureDepNode)itPar.next();
            Component parComp = parent.getComponent();
            ass = assDF + "(" + name + ", " + parComp.getName() + ")"; 
            ass = assDF + "(" + parComp.getName() + ", " + name + ")"; 
            o = assumptions.get(ass);
            a = null;
            if (o != null) a = (Assumption)o;
            
            Mode mode = new Mode(Mode.MODE_DF, this, parComp, a);
            dfModes.put(parComp, mode);
        }
    
    } // initFromFDG()

}
