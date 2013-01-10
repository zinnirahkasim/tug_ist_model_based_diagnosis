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
import theoremprover.*;
import hittingsetalg.*;


public class DiagnosisProblem {

    // map from String (comp. name) to Component
    protected TreeMap components = new TreeMap();

    protected FailureDepGraph fdg;

    // the System Description
    protected LSentence sd;

    // the System Dependencies Description
    protected LSentence sdd;
    
    // the Observations
    protected LSentence obs;

    protected String assAB;
    protected String assNAB;
    protected String assIF;
    protected String assDF;


    public DiagnosisProblem(boolean useProb, String assAB, String assNAB, String assIF, String assDF) {
        fdg = new FailureDepGraph(useProb);
        this.assAB = assAB;
        this.assNAB = assNAB;
        this.assIF = assIF;
        this.assDF = assDF;
    }

    public void clearFDGEdges() {

        // clear the FDG and add components to it

        fdg = new FailureDepGraph(fdg.useProbabilities());
        Iterator itComp = components.values().iterator();
        while (itComp.hasNext()) {
            Component comp = (Component)itComp.next();
            fdg.addNode(comp);
        }
    }

    /**
     * Registers a new component;
     */
    public void addComponent(String compName) {
        
        Component comp = new Component(compName, components.size());
        components.put(compName, comp);
        FailureDepNode fdNode = fdg.addNode(comp);
    }

    public void addComponent(String compName, double prob_if) {
        Component comp = new Component(compName, components.size(), prob_if);
        components.put(compName, comp);
        FailureDepNode fdNode = fdg.addNode(comp);
    }

    /**
     * Adds a failure dependency between 2 components.
     */
    public void addFailureDep(String compFrom, String compTo) {

        Object o1 = components.get(compFrom);
        Object o2 = components.get(compTo);

        // violated if one of the passed components have not been added yet
        assert((o1 != null) && (o2 != null));  
        
        Component c1 = (Component)o1;
        Component c2 = (Component)o2;

        fdg.addEdge(c1, c2);
    }

    public void addFailureDep(String compFrom, String compTo, double prob_df) {

        Object o1 = components.get(compFrom);
        Object o2 = components.get(compTo);

        // violated if one of the passed components have not been added yet
        assert((o1 != null) && (o2 != null));  
        
        Component c1 = (Component)o1;
        Component c2 = (Component)o2;

        fdg.addEdge(c1, c2, prob_df);
    }

    public boolean hasComponent(String compName) {
        return (components.containsKey(compName));
    }

    public void setSD(LSentence sd) {
        this.sd = sd;
    }

    public void setOBS(LSentence obs) {
        this.obs = obs;
    }

    public void setSDD(LSentence sdd) {
        this.sdd = sdd;
    }

    public LSentence getSD() {
        if (sd == null) return new LSentence();
        else return sd;
    }

    public LSentence getOBS() {
        if (obs == null) return new LSentence();
        else return obs;
    }

    public LSentence getSDD() {
        if (sdd == null) return new LSentence();
        else return sdd;
    }

    public FailureDepGraph getFDG() {
        return fdg;
    }

    protected TreeMap getComponents() {
        return components;
    }

    public String getAssAB() {
        return assAB;
    }

    public String getAssNAB() {
        return assNAB;
    }

    public String getAssIF() {
        return assIF;
    }

    public String getAssDF() {
        return assDF;
    }

    /**
     * Computes the minimal hitting sets.
     *
     * See also the documentation of MinHittingSetsFM.
     * Returns an ArrayList of ArrayList of Component.
     * In conflictSets, an ArrayList of ArrayList of SplittedAssumption
     * is returned.
     */
    public ArrayList computeMinHittingSets(int maxHSSize, int maxNumMinHS, 
                                           ArrayList resultingConflictSets) 
        throws IllegalAssumption, ParseError {

        // initializ theorem prover

        LSentence sdUnionObs = new LSentence();
        sdUnionObs.addRules(sd);
        sdUnionObs.addRules(obs);
        
        ABTheoremProver theoremProver = new ABTheoremProver();
        theoremProver = sdUnionObs.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        } 
        
        // compute diagnoses

        MinHittingSetsFM hs = new MinHittingSetsFM(false, theoremProver, assAB, assNAB);
        hs.compute(maxHSSize, maxNumMinHS);
        ArrayList minHS = hs.getMinHS();

        assert(resultingConflictSets.size() == 0);
        ArrayList cs = hs.getConflictsAsAss();
        ArrayList splittedCS = splitAssumptions(cs);
        resultingConflictSets.addAll(splittedCS);

        return convertAssListToCompList(minHS);

    }  // computeMinHittingSets()
  
    /**
     * Converts an ArrayList (AL) of AL of Assumption to an AL of AL of Component.
     *
     * The component names are extracted from the assumption (string) identifiers.
     * Onla NAB assumptions are allowed!
     */
    protected ArrayList convertAssListToCompList(ArrayList assList) throws ParseError {
        
        ArrayList result = new ArrayList(assList.size());
        Iterator itAL = assList.iterator();
        while (itAL.hasNext()) {
            ArrayList expl = (ArrayList)itAL.next();
            ArrayList candidate = new ArrayList(expl.size());
            Iterator itExpl = expl.iterator();

            while (itExpl.hasNext()) {
                Assumption a = (Assumption)itExpl.next();
                String ass = (String)a.getIdentifier();
                if (!ass.matches(assNAB + "\\(" + "[a-zA-Z_0-9]+" + "\\)"))
                    throw new ParseError("[unknown]");

                String compName = ass.substring(assNAB.length() + 1,
                                                ass.length() - 1);
                Object compObj = components.get(compName);
                if (compObj == null) throw new ParseError("[unknown]");
                Component comp = (Component)compObj;
                
                candidate.add(comp);
            }
            
            result.add(candidate);
        }

        return result;

    }

    /*
     * Splits an assumptions: e.g., if ass is "AB(C1)", then
     * this method returns the  ("AB", "C1", "");
     *
     * If ass is something like "DF(C, C_i)", then 
     * the pair ("DF", "C", "C_i") will be returned.
     */
    public SplittedAssumption splitAssumption(Assumption ass) {
        String aStr = (String)ass.getIdentifier();
        String[] sl = aStr.split("[()]");
        assert(sl.length == 2);

        String[] sl2 = sl[1].split(",");
        if (sl2.length == 1) return new SplittedAssumption(sl[0].trim(), null, sl[1].trim());
        else return new SplittedAssumption(sl[0].trim(), sl2[0].trim(), sl2[1].trim());
    }

    /*
     * assList is an ArrayList (AL) of ArrayList of Assumption, the returned ArrayList
     * is a collection of ArrayList's of SplittedAssumption (comp.name, assumption name).
     */
    protected ArrayList splitAssumptions(ArrayList assList) {
        ArrayList result = new ArrayList(assList.size());
        
        Iterator itAL = assList.iterator();
        while (itAL.hasNext()) {
            ArrayList al = (ArrayList)itAL.next();
            ArrayList resItem = new ArrayList(al.size());

            Iterator it = al.iterator();
            while (it.hasNext()) {
                Assumption a = (Assumption)it.next();
                SplittedAssumption splitA = splitAssumption(a);
                resItem.add(splitA);
            }

            result.add(resItem);
        }

        return result;
    }

    /**
     * Computes the diag. env. and returns the result as an ArrayList (AL)
     * of ModeAssignment.
     *
     * minHS: ArrayList of ArrayList of Component
     * conflictSets: ArrayList of ArrayList of Assumption
     */
    public ArrayList computeDEs(ArrayList minHS, boolean computeBetaDE, int maxDFChainSize,
                                ArrayList initialConflictSets)
        throws ParseError {

        DiagnosisEnvironments diagEnvs = new DiagnosisEnvironments(this);
        ArrayList result = diagEnvs.computeDEs(minHS, computeBetaDE,
                                               maxDFChainSize, initialConflictSets);

        return result;
    }

    /*
     * Computed the repair candidates (i.e., the merged DEs) and returns them.
     * See also computeDEs().
     */
    public RepairCandidates computeRepairCandidates(ArrayList minHS, 
                                                    boolean computeBetaDE, int maxDFChainSize,
                                                    boolean discardOrderPerms,
                                                    ArrayList initialConflictSets)
        throws ParseError {

        DiagnosisEnvironments diagEnvs = new DiagnosisEnvironments(this);
        RepairCandidates result 
            = diagEnvs.computeRepairCandidates(minHS, computeBetaDE,
                                               maxDFChainSize, discardOrderPerms,
                                               initialConflictSets);
        
        return result;
    }

    /**
     * Ranks the DEs using probabilities.
     *
     * Returns a list of ObjectPair. Each pair has the form
     * (ModeAssignment, Double), where Double is the probability of the MA.
     * The list is sorted by probabilities in descending order.
     *
     * Note that the probabilities are prior probabilities. If "normalize"
     * is true, then the resulting values will sum up to 1.
     */
    public ArrayList computeDERanking(ArrayList deList, boolean normalize) {
        DiagnosisEnvironments diagEnvs = new DiagnosisEnvironments(this);

        // compute probabilities; store in result arraylist

        double sum = 0.0;

        ArrayList result = new ArrayList(deList.size());
        Iterator itDE = deList.iterator();
        while (itDE.hasNext()) {
            ModeAssignment de = (ModeAssignment)itDE.next();
            double prob = diagEnvs.computeProb(de);
            sum += prob;

            ObjectPair op = new ObjectPair(de, new Double(prob));
            result.add(op);
        }

        // normalize, if desired

        if (normalize && (sum > 0.0)) {
            Iterator itResult = result.iterator();
            while (itResult.hasNext()) {
                ObjectPair op = (ObjectPair)itResult.next();
                Double prob = (Double)op.last;
                double norm_value = prob.doubleValue() / sum; 
                Double newProb = new Double(norm_value);
                op.last = newProb;
            }
        }

        // create comparator for sort algorithm

        Comparator comp = new Comparator() {
                public int compare(Object o1, Object o2) {
                    ObjectPair op1 = (ObjectPair)o1;
                    ObjectPair op2 = (ObjectPair)o2;

                    double d1 = ((Double)op1.last).doubleValue();
                    double d2 = ((Double)op2.last).doubleValue();
                    if (d1 < d2) return +1;
                    else if (d1 > d2) return -1;
                    else return 0;
                }

                public boolean equals(Object o1, Object o2) {
                    return false; // dummy
                }
            };
        
        // sort result
        Collections.sort(result, comp);

        return result;

    }  // computeDERanking() 

}
