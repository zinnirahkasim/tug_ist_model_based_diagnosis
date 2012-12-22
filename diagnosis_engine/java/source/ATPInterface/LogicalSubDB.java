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

import java.util.*;

import theoremprover.*;
import hittingsetalg.*;


/**
 * A logical sub-database is part of a logical database, see LogicalDBInterface.
 *
 * A logical sub-database has a unique integer ID and comprises a set of rules.
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */
public class LogicalSubDB {

    // The unique name, e.g. "SD", "OBS", or "SDD".
    protected String name;

    // A container for the rules.
    protected LSentence rules;

    // The number of the rules.
    protected int numRules;

    // A parser for rules.
    protected LogicParser parser;

    // The assumption-based theorem-prover.
    protected ABTheoremProver theoremProver;

    /**
     * Create an empty sub-database with the passed (unique) ID.
     */ 
    public LogicalSubDB(String name) {
        this.name = name;
        rules = new LSentence();
        numRules = 0;
        parser = new LogicParser();
    }


    /**
     * Return the (unique) ID of this sub-database.
     */
    public String getName() {
        return name;
    }


    /**
     * Add new rules (passed as string collection) the the sub-database.
     */
    public void addRules(ArrayList newRules) throws LogicParseException {
        for (int i = 0; i < newRules.size(); ++i) {
  
            String ruleStr = (String)newRules.get(i);

            if (parser.parse(ruleStr)) {
                rules.addRules((LSentence)parser.result());
                ++numRules;
            } else {
                throw new LogicParseException(i + 1);
            }
        }
    }

    /**
     * Return the number of rules in this sub-database.
     */
    public int getNumRules() {
        return numRules;
    }

    /**
     * Return the rules of this sub-database as collection of strings.
     */
    public ArrayList getRulesAsStrings() {
        ArrayList result = new ArrayList();

        Iterator it = rules.rules.iterator();
        while(it.hasNext()) {
            result.add(it.next().toString());
        }

        return result;
    }

    public LSentence getRulesAsLSentence() {
        return rules;
    }

    /**
     * Delete all rules of this sub-database.
     */
    public void clear() {
        rules = new LSentence();
        numRules = 0;
    }


    /**
     * Append the rules of source to this sub-database.
     */
    public void append(LogicalSubDB source) {
        rules.addRules(source.rules);
        numRules += source.numRules;
    }

    /**
     * Check the consistency of this sub-database by invoking the theorem prover.
     * Use this method when no fault modes are used.
     */ 
    public boolean checkConsistency(boolean useFaultModes, String assumptionAB, 
                                    String assumptionNAB) throws LogicParseException {
         
        theoremProver = new ABTheoremProver();
         theoremProver = rules.asABPropositionalSentence(theoremProver);
         if (theoremProver == null) {
             throw new LogicParseException();
         }
         
         if (useFaultModes) {
             ArrayList posAssPrefixes = new ArrayList();
             posAssPrefixes.add(assumptionNAB);
             return (theoremProver.checkConsistency(posAssPrefixes));  
         } else {
             return (theoremProver.checkConsistency());         
         }
    }

    /**
     * Computes the subset-min. diagnoses (if there are any).
     *
     * The result is an ArrayList of an ArrayList. Each element (ie an ArrayList) is
     * a collection of strings which represent the explanations.
     * Thus, if there are only single explanations, then each element of the 
     * returned list is an ArrayList containing only one string.
     *
     * @param verboseOutput If true, then all rules (and maybe other data) are printed to stdout.
     */
    public ArrayList computeMinDiag(int maxExplSize, int maxNumExpl,
                                    boolean useFaultModes,  String assumptionAB, 
                                    String assumptionNAB, boolean verboseOutput) 

        throws IllegalAssumption {

        assert(theoremProver != null);
        
        ArrayList minHS;
        ArrayList result = new ArrayList();
        
        if (useFaultModes) {
            System.out.println("maxExplSize: " + maxExplSize);
            System.out.println("maxNumExpl: " + maxNumExpl);
            System.out.println("ass AB: " + assumptionAB);
            System.out.println("# assumptions: " + theoremProver.getAssumptions().size());
            System.out.println("# rules (altogher): " + rules.rules.size());
            if (verboseOutput) {
                System.out.println("--- rules: --- \n" + rules.toString());
            }
            MinHittingSetsFM hs = new MinHittingSetsFM(false, theoremProver, assumptionAB, assumptionNAB);
            
            hs.compute(maxExplSize, maxNumExpl); 
            minHS = hs.getMinHS();
            System.out.println("#minimal diagnoses: " + minHS.size());

        } else {
            MinHittingSets hs = new MinHittingSets(false, theoremProver);
            hs.compute(maxExplSize, maxNumExpl);
            minHS = hs.getMinHS();
        }
 
        Iterator itMinHS = minHS.iterator();
        while(itMinHS.hasNext()) {
            ArrayList explanation = new ArrayList();

            ArrayList candidate = (ArrayList)itMinHS.next();
            Iterator itCand = candidate.iterator();
            while(itCand.hasNext()) {
                Assumption a = (Assumption)itCand.next();
                explanation.add(a.toString());
            }

            result.add(explanation);
        }
        
        return result;
    }
}
