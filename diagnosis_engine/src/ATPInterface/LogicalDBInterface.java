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


package ATPInterface;

import java.util.ArrayList;
import java.util.BitSet;

import theoremprover.IllegalAssumption;

/**
 * This is the interface to a logical DB and its theorem prover.
 *
 * A logical database comprises zero, one or more logical sub-databases which have a
 * unique int identifier. Each logical sub-database consists of a set of logical rules.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 */
public interface LogicalDBInterface {

    /**
     * Add rules, which are passed as a collection of strings, to a logical sub-database.
     *
     * There are 3 logical sub-databases: for SD, OBS, and SDD.
     * The new rules are added to it or, if "replace" is "false", then the new rules replace the old ones.
     */
    int addRules(String subDB, boolean replace, ArrayList newRules) 
        throws LogicParseException;

    /**
     *  The edges are represented by strings like "C1 => C2".
     *
     * Returns the total number of edges in the FDG.
     */
    int addFDGEdges(ArrayList edgeStrings, boolean replace) throws LogicParseException;

    /**
     * Check the consistency of the entire database.
     *
     * Note that the implementor if this interface may cache computed values and re-use
     * them if the database has not changed since the last call.
     *
     * The two string parameters are only used when useFaultModes = true.
     */
    boolean checkConsistency(boolean useFaultModes)
        throws LogicParseException;

    /*
     * Queries is a list of strings representing propositions. For each query q,
     * this method checks separately if "SD |_| OBS |_| {-ab(c) | c in COMP} |_| q"
     * holds (i.e., if q is consistent with all the other sentences).
     *
     * For any query q at index i in queries, the value of bit i in the BitSet result indicates
     * whether q is entailed.
     */
    void performConsistencyChecks(ArrayList queries, boolean useFaultModes, BitSet result)
        throws LogicParseException;

    /**
     * Checks the consistency of the database and return the explanations, if there are any.
     *
     * The method checkConsistency() does not need to be called before.
     *
     * Note that the implementor if this interface may cache computed values and re-use
     * them if the database has not changed since the last call.
     *
     * The two string parameters are only used when useFaultModes = true.
     *
     * @param verboseOutput If true, then all rules (and maybe other data) are printed to stdout.
     */
    ArrayList computeMinDiag(int maxExplSize, int maxNumExpl,
                             boolean useFaultModes, boolean verboseOutput)
        throws LogicParseException, IllegalAssumption;

    /**
     * Computes the diagnosis environments.
     *
     * The DEs are returned as strings. The computed min. diagnoses are
     * returned as strings in minDiags.
     */
    ArrayList computeDEs(int maxExplSize, int maxNumExpl,
                         boolean computeBetaDE, int maxDFChainSize, boolean mergeDEs,
                         boolean discardOrderPerms, ArrayList minDiags)
        throws LogicParseException, IllegalAssumption;
    
    /**
     * Return the total number of rules in this database.
     */
    int getTotalNumRules();

    /**
     * Returns the number of rules of the sub-database whose name ("SD", "OBS", "SDD") is passed, may be 0.
     */
    int getSubDBNumRules(String subDB);
    
    /**
     * Creates a collection of SubDBStat objects stating informations about this DB.
     */
    ArrayList createSubDBStats();

    /**
     * Returns the rules of the sub-database whose name is passed, may be empty.
     */
    ArrayList getSubDBRules(String name);

    /*
     * Returns statistical data on the Failure Dep. Graph.
     */
    public FDGStat createFDGStats();
}
