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

import theoremprover.*;
import utils.*;



public class DiagnosisEnvironments {

    protected DiagnosisProblem diagProblem;

    protected ABTheoremProver theoremProver;

    // map from String (the ass. name) to Assumption
    protected Map assumptions;

    protected DEGraph deGraph;

    protected ConflictSets conflictSets = new ConflictSets();

    protected RepairCandidates repairCandidates = new RepairCandidates();



    ////////////////////////////////////////////////////////

    protected class ComputationStat {

        int numTPCalls = 0;           // # of actual theorem prover calls
        int numTPCalls_Alpha = 0;     // # of actual theorem prover calls for alpha nodes
        int numTPCalls_Beta = 0;      // # of actual theorem prover calls for beta nodes
        int numConflicting = 0;       // # of avoided TP calls because a conflicting conflict set was found
        int numDFChainTooLong = 0;    // # of nodes which are discarded as their longest DF chain exceeds a given value
        int numDescInconsistent = 0;  // # of nodes marked as "descendants inconsistent" for which no TP call is needed
        int numAlreadyExists = 0;     // # of nodes which are not created since their mode assignment already exists
        int numMinimal = 0;           // # of minimal nodes
        int numMinimal_Alpha = 0;     // # of minimal alpha nodes
        int numMinimal_Beta = 0;      // # of minimal beta nodes
        int numNodes = 0;             // total # of nodes
        int numAlphaNodes = 0;        // total # of alpha nodes
        int numBetaNodes = 0;         // total # of beta nodes
        int numImpliedByRC = 0;       // # of nodes which are not checked for consistency as they are implied by an
                                      // repair candidate
        int numDiscaredOrderPerms = 0;  // # of nodes whose consistency is not checked as there is already a DE
                                        // which comprises the same set of components

        public String toString() {
            String result = "";

            assert((numAlphaNodes + numBetaNodes == numNodes) && (numNodes == deGraph.getNumNodes()));
            result += "number of nodes: " + numNodes + "\n";
            result += "number of ALPHA nodes: " + numAlphaNodes + "\n";
            result += "number of BETA nodes: " + numBetaNodes + "\n";
            result += "\n";

            assert(numTPCalls_Alpha + numTPCalls_Beta == numTPCalls);
            result += "number of TP calls: " + stats.numTPCalls + "\n";
            result += "number of TP calls for ALPHA: " + stats.numTPCalls_Alpha + "\n";
            result += "number of TP calls for BETA: " + stats.numTPCalls_Beta + "\n";
            result += "\n";

            assert(numMinimal_Alpha + numMinimal_Beta == numMinimal);
            result += "number of nodes with a too long DF chain: " + numDFChainTooLong + "\n";

            result += "number of not required TP calls since conflicting: " + numConflicting + "\n";
            result += "number of \"descendant inconsistent\" nodes: " + numDescInconsistent + "\n";
            result += "number of nodes implied by repair candidates: " + numImpliedByRC + "\n";
            result += "number of discarded order permutations: " + numDiscaredOrderPerms + "\n";
            result += "number of already existing nodes: " + numAlreadyExists + "\n";
            result += "\n";

            result += "number of minimal nodes: " + numMinimal + "\n";
            result += "number of minimal ALPHA nodes: " + numMinimal_Alpha + "\n";
            result += "number of minimal BETA nodes: " + numMinimal_Beta + "\n";            
            
            return result;
        }
    }

    ////////////////////////////////////////////////////////

    protected ComputationStat stats = new ComputationStat();


    public DiagnosisEnvironments(DiagnosisProblem diagProblem) {
        this.diagProblem = diagProblem;
    }

    /*
     * Creates the theorem prover containing the entire logical model.
     */
    protected void initTheoremProver() throws ParseError {

        LSentence logModel = new LSentence();
        logModel.addRules(diagProblem.getSD());
        logModel.addRules(diagProblem.getOBS());
        logModel.addRules(diagProblem.getSDD());
        
        theoremProver = new ABTheoremProver();
        theoremProver = logModel.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        } 

    }
    
    /*
     * Iterate through all components, create their modes.
     */
    protected void initComponents() {
        Iterator itC = diagProblem.getComponents().values().iterator();

        while (itC.hasNext()) {
            Component c = (Component)itC.next();
            c.initFromFDG(assumptions, diagProblem.getAssAB(), 
                          diagProblem.getAssNAB(), diagProblem.getAssIF(),
                          diagProblem.getAssDF()); 
        }
    }

    /*
     * Creates a map <String, Assumption>: maps the string identifiers from assumptions
     * to Assumption.
     */
    protected void createAssumptionMap() {

        assumptions = new TreeMap();
        List assList = theoremProver.getAssumptions();

        Iterator itAss = assList.iterator();
        while (itAss.hasNext()) {
            Assumption a = (Assumption)itAss.next();
            assert(a.getIdentifier() instanceof String);
            assumptions.put((String)a.getIdentifier(), a);
        }

    }

    /*
     * Util method for createInitialConflictSets().
     * Sets in ma the mode of compName to that mode which is defined by assName (e.g., "NAB").
     *
     * parentCompName is relevant only if assName denotes the DF mode. 
     */
    protected void setMode(ModeAssignment ma, SplittedAssumption sa) {
        Component c = (Component)diagProblem.getComponents().get(sa.compName);
        assert(c != null);

        if (sa.assName.equals(diagProblem.getAssAB())) {
            ma.setMode(c, Mode.MODE_AB, null);
        } else if (sa.assName.equals(diagProblem.getAssNAB())) {            
            ma.setMode(c, Mode.MODE_NAB, null);
        } else {
            assert(sa.assName.equals(diagProblem.getAssDF()));

            Component cp = (Component)diagProblem.getComponents().get(sa.parentCompName);
            assert(cp != null);
            ma.setMode(c, Mode.MODE_DF, cp);
        }
    }

    protected void createInitialConflictSets(ArrayList reiterConflictSets) {
        
        Iterator itRCS = reiterConflictSets.iterator();
        while (itRCS.hasNext()) {
            ArrayList cs = (ArrayList)itRCS.next();           
            Iterator itCS = cs.iterator();
            ConflictSet newCS = new ConflictSet();

            while (itCS.hasNext()) {
                SplittedAssumption sa = (SplittedAssumption)itCS.next();
                assert((sa.assName.equals(diagProblem.getAssAB()) || sa.assName.equals(diagProblem.getAssNAB()))
                       && (sa.parentCompName == null));

                setMode(newCS, sa);                                
            }

            conflictSets.add(newCS);
        }

    }

    /*
     * Do some initialization before executing the main algorithm.
     */
    protected void initComputation(ArrayList reiterConflictSets) throws ParseError {
            
        // perform initializations

        initTheoremProver();                
        createAssumptionMap();
        initComponents();
        diagProblem.getFDG().computeIndirectDeps();
        diagProblem.getFDG().computeCommonAncestorGraph(true);
        createInitialConflictSets(reiterConflictSets);

        // debug output

        printComponents();
        //System.out.println("\nReiter conflict sets:\n");
        printConflictSets();
        /*System.out.println("\n");
        System.out.println("FAILURE DEPENDENCY GRAPH:\n");
        System.out.println(diagProblem.getFDG().toString());
        System.out.println("\n");*/
    }

    public RepairCandidates computeRepairCandidates(ArrayList diags, boolean computeBetaDE, 
                                                    int maxDFChainSize,
                                                    boolean discardOrderPerms,
                                                    ArrayList reiterConflictSets) 
        throws ParseError {
        
        // initialisations
        initComputation(reiterConflictSets);                

        // compute the DE Graph

        Date startComputationTime = new Date();

        deGraph = new DEGraph();
        if (discardOrderPerms) {
            computeDEGraph(diags, computeBetaDE, maxDFChainSize, false, true);
        } else {
            computeDEGraph(diags, computeBetaDE, maxDFChainSize, true, false);
        }

        Date endComputationTime = new Date();
        long passedTime = endComputationTime.getTime() - startComputationTime.getTime();    

        // debug output
 
        /*
        System.out.println("\nDEGraph:\n");
        System.out.println(deGraph.toStringShort());
        System.out.println("\n\n");

        System.out.println("\nORDER RELATIONS:\n");
        printOrderRelations(resultingNodes);
        System.out.println("\n\n");

        System.out.println("\nREPAIR CANDIDATES:\n");
        System.out.println(repairCandidates.toString());
        System.out.println("\n\n");

        System.out.println("\nAll conflict sets:\n");
        printConflictSets();
        System.out.println("\n");
        
        System.out.println("\nConsistent nodes:\n");
        printConsistentNodes();
        System.out.println("\n\n");
        
        System.out.println("\nInconsistent nodes:\n");
        printInconsistentNodes();
        System.out.println("\n\n");
        */
        
        System.out.println("Computation time [ms]: " + passedTime);
        System.out.println(stats.toString());       

        return repairCandidates;
    }

    /*
     * Returns the DEs as ArrayList of ModeAssignment. The DEs are not merged.
     * reiterConflictSets: ArrayList of ArrayList of Assumption.
     * 
     */
    public ArrayList computeDEs(ArrayList diags,  
                                boolean computeBetaDE, int maxDFChainSize,
                                ArrayList reiterConflictSets) 
        throws ParseError {

        // initialisations
        initComputation(reiterConflictSets);                

        // compute the DE Graph

        Date startComputationTime = new Date();

        deGraph = new DEGraph();
        ArrayList resultingNodes = computeDEGraph(diags, computeBetaDE, maxDFChainSize, false, false);

        Date endComputationTime = new Date();
        long passedTime = endComputationTime.getTime() - startComputationTime.getTime();        

        // compose result

        ArrayList result = new ArrayList();  // list of mode assignments

        Iterator itN = resultingNodes.iterator();
        while (itN.hasNext()) {
            DENode n = (DENode)itN.next();
            result.add(n.getModeAssignment());
        }

        // debug output
 
        /*
          System.out.println("\nDEGraph:\n");
        System.out.println(deGraph.toStringShort());
        System.out.println("\n\n");

        System.out.println("\nORDER RELATIONS:\n");
        printOrderRelations(resultingNodes);
        System.out.println("\n\n");

        System.out.println("\nAll conflict sets:\n");
        printConflictSets();
        System.out.println("\n");
        
        System.out.println("\nConsistent nodes:\n");
        printConsistentNodes();
        System.out.println("\n\n");
        
        System.out.println("\nInconsistent nodes:\n");
        printInconsistentNodes();
        System.out.println("\n\n");
        */

        System.out.println("Computation time [ms]: " + passedTime);
        System.out.println(stats.toString());
        
        return result;

    }  // computeDEs()

 
    // util method for condProb()
    protected final double getModeProb(Component c, Mode m, ModeAssignment ma) {

        double result;

        if (m.getType() == Mode.MODE_IF) return c.getProbIF();
        else {
            assert(m.getType() == Mode.MODE_DF);
            
            Component parent = m.getParent();
            int parentMode = ma.getMode(parent, Mode.MODE_NAB);
            if (parentMode == Mode.MODE_NAB) result = 0.0;
            else {
                Map dfProbMap = c.getFDNode().getProbDF(); 
                result = ((Double)dfProbMap.get(new Integer(parent.getFDNode().getID()))).doubleValue();
            }
        }

        //System.out.println("getModeProb(" + c.getName() + ", " + m.typeAsString(m.getType()) + "), ma = " + ma.toStringShort() + ": " + result);
        return result;
    }

    protected class FMProbResults {
        double prob_nab;
        double sumFMProbs;
    }

    protected FMProbResults computeProb_NAB(Component c, ModeAssignment ma) {
        
        FMProbResults result = new FMProbResults();
        result.prob_nab = (1.0 - c.getProbIF());
        result.sumFMProbs = c.getProbIF();
        
        Map dfProbMap = c.getFDNode().getProbDF(); 
        Collection dfmodes = c.getModesDF();
        Iterator itDFModes = dfmodes.iterator();
        while (itDFModes.hasNext()) {
            Mode dfm = (Mode)itDFModes.next();
            Component parent = dfm.getParent();
            if (ma.getMode(parent, Mode.MODE_NAB) != Mode.MODE_NAB) {
                double edge_prob 
                    = ((Double)dfProbMap.get(new Integer(parent.getFDNode().getID()))).doubleValue();
                
                result.sumFMProbs += edge_prob;
                result.prob_nab *= (1 - edge_prob);
            }
        }

        //System.out.println("computeProb_NAB( " + c.getName() + "), ma = " + ma.toStringShort() + ": " + result.prob_nab + "[sum: " + result.sumFMProbs + "]");
        return result;
    }

    public double computeProb(ModeAssignment ma) {
        
        double result = 1.0;

        // iterate through all components, multiply prob. of each comp. mode

        Map comps = diagProblem.getComponents();
        Iterator itComp = comps.values().iterator();
        while (itComp.hasNext()) {
            Component c = (Component)itComp.next();
            FMProbResults probRes = computeProb_NAB(c, ma);

            Mode m = ma.getMode(c);
            if ((m == null) || (m.getType() == Mode.MODE_NAB)) result *= probRes.prob_nab;
            else {
                double prob_ab = 1 - probRes.prob_nab;
                double mode_prob = getModeProb(c, m, ma);
                result *= (prob_ab * mode_prob / probRes.sumFMProbs);
            }
            
        }

        //System.out.println("total MDE prob. of " + ma.toStringShort() + ": " + result);
        return result;

    }  // computeProb

    protected void printConflictSets() {
        Iterator itCS = conflictSets.iterator();
        while (itCS.hasNext()) {
            ConflictSet cs = (ConflictSet)itCS.next();
            //System.out.println(cs.toStringShort());
        }
    }

    /*
     * Add diagnoses (in which "ab" was substituted by "if") to DEGraph, but only
     * diagnoses d such that min. #if(d) == searchNumIF
     *
     * substDiags contains the diagnoses, while diagsMinNumIF contains Integer's
     * indicating the min. #if of the diagnoses.
     *
     * Returns the new nodes.
     */
    protected void addDiagsToDEGraph(ArrayList substDiags, ArrayList diagsMinNumIF,
                                     int searchNumIF) {
        
        for (int i = 0; i < substDiags.size(); ++i) {
            int diagMinNumIF = ((Integer)diagsMinNumIF.get(i)).intValue();
            
            if (diagMinNumIF == searchNumIF) {
                ModeAssignment substDiag = (ModeAssignment)substDiags.get(i);
                DENode newRootNode = new DENode(DENode.TYPE_ALPHA, null, 0, substDiag, new TreeSet());  
                newRootNode.setMinNumIF(diagMinNumIF);
                    
                deGraph.addRootNode(newRootNode);
                deGraph.getUnexpandedAlphaNodes().add(newRootNode);

                stats.numNodes++;
                stats.numAlphaNodes++;
            
                //System.out.println("add new diagnosis (#if_min = " + searchNumIF + "): " + substDiag.toStringShort());
            
            } else {
                //System.out.println("omit diagnosis with #if_min = " + diagMinNumIF + ": " + ((ModeAssignment)substDiags.get(i)).toStringShort());
            }
        }
    }
    
    protected void generateAllBetaDescendants(DENode n, int maxDFChainSize, ArrayList des,
                                              boolean mergeDEs, boolean discardOrderPerms) {

        assert(!mergeDEs || !discardOrderPerms);  // these options contradict each other
        assert(!n.descInconsistent());

        //System.out.println("generateAllBetaDescendants()");
        
        ArrayList createdNodes = expandNode(n, DENode.TYPE_BETA);
        
       // iterate through new child nodes, add them to DEG
                    
        Iterator itCreatedNodes = createdNodes.iterator();
        while(itCreatedNodes.hasNext()) {
            
            // create and initialize node, but only if maxDFChainSize is not exceeded
            // and if this mode assignment not yet exists.
            
            DENode childNode = (DENode)itCreatedNodes.next();
            //System.out.println("created BETA node: " + childNode.toStringShort());

            if ((childNode.getDistanceToRoot() <= maxDFChainSize) 
                || (childNode.getModeAssignment().computeLongestDFChain() <= maxDFChainSize)) {

                if (deGraph.alreadyExists(childNode.getModeAssignment(), childNode.getRootNode()) == null) {
                
                    // add node to DEG 
                
                    deGraph.addChildNode(n, childNode);
                    stats.numNodes++;
                    stats.numBetaNodes++;
                    
                    // check consistency if there is no repair candidate 
                    // which implies childNode.ma
                    
                    ModeAssignment ma = childNode.getModeAssignment();
                    boolean consistencyCheckNecessary = true;
                    RepairCandidate rc = null;
                    
                    if (mergeDEs || discardOrderPerms) {
                        rc = repairCandidates.findCandidateFor(ma);
                        if (rc != null) 
                        {
                            if (discardOrderPerms) {
                                consistencyCheckNecessary = false;
                                ++stats.numDiscaredOrderPerms;
                            }
                            else if (rc.impliesOrderOf(ma))
                            {
                                consistencyCheckNecessary = false;
                                ++stats.numImpliedByRC;
                            }
                        }
                    }

                    if (consistencyCheckNecessary) {
                        if (checkConsistency(childNode, true)) {
                            childNode.setMinimal(DENode.STATUS_TRUE);
                            ++stats.numMinimal;
                            ++stats.numMinimal_Beta;
                            
                            des.add(childNode); 
                            if (mergeDEs) {
                                if (rc == null) repairCandidates.add(ma);
                                else rc.mergeWith(ma);
                            } else if (discardOrderPerms) {
                                repairCandidates.add(ma);
                            }
                        }
                    }
                    
                    if (childNode.descInconsistent()) {  // descendants are inconsistent                 
                        //System.out.println("DONT expand (BETA): desc. inconsistent: " + childNode.toStringShort()); 
                    } else {
                        
                        // recursive call to generateAllBetaDescendants()
                        generateAllBetaDescendants(childNode, maxDFChainSize, des, mergeDEs, discardOrderPerms);
                    }
                
                } else {
                    //System.out.println("Node already exists: " + childNode.getModeAssignment().toStringShort());
                    ++stats.numAlreadyExists;
                }

            } else {
                ++stats.numDFChainTooLong;
            }
        }
        
    }  // generateAllBetaDescendants()

    /*
     * Expand all nodes n with min. #if(n) == searchNumIF
     * Also performs cons. checks for nodes n with #if(n) == searchNumIF. Those nodes
     * which are consistent are returned in "des".
     * 
     */
    protected void performGraphExpansions(int searchNumIF, boolean computeBetaDE, int maxDFChainSize, 
                                          boolean mergeDEs, boolean discardOrderPerms, 
                                          ArrayList des) {
                                 
        assert(!mergeDEs || !discardOrderPerms);  // these options contradict each other

        //System.out.println("expandDEGraph: searchNumIF = " + searchNumIF);

        // iterate through unexpanded nodes (note: iterator is reset after each expansion
        //   => loop terminates when no more nodes can be expanded

        Iterator itUnexpNodes = deGraph.getUnexpandedAlphaNodes().iterator();
        while (itUnexpNodes.hasNext()) {
            
            DENode n = (DENode)itUnexpNodes.next();
            assert(n.getType() == DENode.TYPE_ALPHA);
            
            // check if n should be expanded in this step

            if (n.getMinNumIF() <= searchNumIF) {  // yes!
                
                // if #if(n) == searchNumIF: check consistency if there is no repair candidate 
                // which implies n.ma

                assert(n.getNumIF() >= searchNumIF);
                if ((n.consistent() == DENode.STATUS_UNKNOWN) && (n.getNumIF() == searchNumIF)) {
                        
                    assert(!n.descInconsistent());

                    ModeAssignment ma = n.getModeAssignment();
                    boolean consistencyCheckNecessary = true;
                    RepairCandidate rc = null;

                    if (mergeDEs || discardOrderPerms) {
                        rc = repairCandidates.findCandidateFor(ma);
                        if (rc != null) 
                        {
                            if (discardOrderPerms) {
                                consistencyCheckNecessary = false;
                                ++stats.numDiscaredOrderPerms;
                            }
                            else if (rc.impliesOrderOf(ma))
                            {
                                consistencyCheckNecessary = false;
                                ++stats.numImpliedByRC;
                            }
                        }
                    }
                        
                    if (consistencyCheckNecessary) {                        
                        if (checkConsistency(n, computeBetaDE)) {
                            n.setMinimal(DENode.STATUS_TRUE);
                            ++stats.numMinimal;
                            ++stats.numMinimal_Alpha;
                            
                            des.add(n);  
                            if (mergeDEs) {
                                if (rc == null) repairCandidates.add(ma);
                                else rc.mergeWith(ma);
                            } else if (discardOrderPerms) {
                                repairCandidates.add(ma);
                            }
                        }
                    }

                    // generate beta nodes for n

                    if (!n.descInconsistent() && computeBetaDE) {
                        generateAllBetaDescendants(n, maxDFChainSize, des, mergeDEs, discardOrderPerms);
                    }
                }
                
                if (n.descInconsistent()) {  // descendants are inconsistent

                    deGraph.getUnexpandedAlphaNodes().remove(n);
                    itUnexpNodes = deGraph.getUnexpandedAlphaNodes().iterator();

                    //System.out.println("DONT expand: desc. inconsistent: " + n.toStringShort());  
                
                } else {  // --- expand ---

                    //System.out.println("EXPAND (ALPHA): " + n.toStringShort());
                    
                    // expand; remove from set of unexpanded nodes
                    ArrayList createdNodes = expandNode(n, DENode.TYPE_ALPHA);
                    deGraph.getUnexpandedAlphaNodes().remove(n);
                    
                    // iterate through new child nodes, add them to DEG
                    
                    Iterator itCreatedNodes = createdNodes.iterator();
                    while(itCreatedNodes.hasNext()) {
                        
                        // create and initialize node, but only if maxDFChainSize is not exceeded
                        // and if this mode assignment not yet exists.
                        
                        DENode childNode = (DENode)itCreatedNodes.next();
                        if ((childNode.getDistanceToRoot() <= maxDFChainSize) 
                            || (childNode.getModeAssignment().computeLongestDFChain() <= maxDFChainSize)) {

                            if (deGraph.alreadyExists(childNode.getModeAssignment(), 
                                                      childNode.getRootNode()) == null) {
                            
                                int childMinNumIF = computeMinNumIF(childNode.getModeAssignment(), maxDFChainSize);
                                childNode.setMinNumIF(childMinNumIF);
                                
                                // add node to DEG and to unexpanded nodes
                                
                                deGraph.addChildNode(n, childNode);
                                deGraph.getUnexpandedAlphaNodes().add(childNode);
                                stats.numNodes++;
                                stats.numAlphaNodes++;
                            } else {
                                //System.out.println("Node already exists: " + childNode.getModeAssignment().toStringShort());
                                ++stats.numAlreadyExists;
                            }
                            
                        } else {
                            ++stats.numDFChainTooLong;
                        }
                        
                    }  // iterate through new child nodes
                    
                    // reset iterator                
                    itUnexpNodes = deGraph.getUnexpandedAlphaNodes().iterator();
                    
                }  // expand n
            
            }  // if (n.getMinNumIF() == searchNumIF)

            else {
                //System.out.println("DONT expand: " + n.getModeAssignment().toStringShort());
            }
            
        }  // iterate through unexpanded nodes
        
    }  // expandDEGraph

    /*
     * In "consideredNodes", this method returns those nodes n in DEGraph for which 
     * (#IF(n) == searchNumIF) holds and whose consistency has not been checked before this method
     * was called.
     *
     * Furthermore, in "des" this method returns those nodes of consideredNodes which are not implied
     * by any repair candidate and which are consistent.
     * 
     */
    protected void computeConsistentDENodes(int searchNumIF, boolean computeBetaDE, int maxDFChainSize,
                                            boolean mergeDEs, boolean discardOrderPerms,
                                            ArrayList consideredNodes, ArrayList des) {

        assert(consideredNodes.size() == 0);
        assert(!mergeDEs || !discardOrderPerms);  // these options contradict each other

         // iterate through all nodes

        Iterator itNodes = deGraph.iterator();
        while (itNodes.hasNext()) {
            DENode n = (DENode)itNodes.next();

            // proceed only if the consistency status is unknown and the #if == searchNumIF

             // even new nodes may be marked as inconsistent if a predecessor node has a parent
            // whose conflict set refutes all descendants
            if ((n.consistent() == DENode.STATUS_UNKNOWN) && (n.getNumIF() == searchNumIF)) {

                assert((n.getType() == DENode.TYPE_ALPHA) && !n.expanded(DENode.TYPE_BETA));

                ModeAssignment ma = n.getModeAssignment();
                boolean consistencyCheckNecessary = true;
                RepairCandidate rc = null;

                if (mergeDEs || discardOrderPerms) {
                    rc = repairCandidates.findCandidateFor(ma);
                    if (rc != null) 
                    {
                        if (discardOrderPerms) {
                            consistencyCheckNecessary = false;
                            ++stats.numDiscaredOrderPerms;
                        }
                        else if (rc.impliesOrderOf(ma)) 
                        {
                            consistencyCheckNecessary = false;
                            ++stats.numImpliedByRC;
                        }
                    }
                }

                if (consistencyCheckNecessary) {

                    if (checkConsistency(n, computeBetaDE)) {
                        n.setMinimal(DENode.STATUS_TRUE);
                        ++stats.numMinimal;
                        ++stats.numMinimal_Alpha;
                        
                        des.add(n);  
                        if (mergeDEs) {
                            if (rc == null) repairCandidates.add(ma);
                            else rc.mergeWith(ma);
                        } else if (discardOrderPerms) {
                            repairCandidates.add(ma);
                        }
                    }
                }

                consideredNodes.add(n);

            }
        }  // iterate through all nodes
    }  // computeConsistentDENodes()

    /*
     * This is the main algorithm. Computes all min. DEs and returns the minimal nodes in the DE Graph.
     */
    protected ArrayList computeDEGraph(ArrayList diags, boolean computeBetaDE,
                                       int maxDFChainSize, boolean mergeDEs, 
                                       boolean discardOrderPerms) {

        assert(!mergeDEs || !discardOrderPerms);  // these options contradict each other

        // ***** preparations *****

        // substitute "ab" in diagnoses with "if", store results in substDiags
        // compute min. #(if) for all diagnoses, store results in diagsMinNumIF

        int maxNumIF = 0;

        ArrayList diagsMinNumIF = new ArrayList(diags.size());
        ArrayList substDiags = new ArrayList(diags.size());

        Iterator itDiags = diags.iterator();
        while (itDiags.hasNext()) {
            ArrayList diagnosis = (ArrayList)itDiags.next();
            if (diagnosis.size() > maxNumIF) maxNumIF = diagnosis.size();
            ModeAssignment substDiag = new ModeAssignment();
            substDiag.setModes(diagnosis, Mode.MODE_IF);
            substDiags.add(substDiag);
            
            int diagMinNumIF = computeMinNumIF(substDiag, maxDFChainSize);
            diagsMinNumIF.add(new Integer(diagMinNumIF));
        }

        // ***** main algorithm: loop until consistent DEs with a minimal #if are found *****
       
        ArrayList des = new ArrayList();
        ArrayList newNodes;

        int searchNumIF = 1; // start with looking for DEs with #if == 1
        do {            
        
            //System.out.println("searchNumIf = " + searchNumIF);
    
            // iterate through diagnoses; for each d with #if(d) == searchNumIF: add d to DEG
            addDiagsToDEGraph(substDiags, diagsMinNumIF, searchNumIF);

            //System.out.println("look for consistent nodes BEFORE expansion");
            ArrayList nodesForBetaExpansion = new ArrayList();
            computeConsistentDENodes(searchNumIF, computeBetaDE, maxDFChainSize, mergeDEs,
                                     discardOrderPerms, nodesForBetaExpansion, des);
            if (computeBetaDE) {
                performBetaExpansions(nodesForBetaExpansion, maxDFChainSize, des, mergeDEs,
                                      discardOrderPerms);
            }
            performGraphExpansions(searchNumIF, computeBetaDE, maxDFChainSize, mergeDEs, 
                                   discardOrderPerms, des);
        
            // can we terminate?
            
            if (des.size() > 0) return des;
            else ++searchNumIF;            

        } while (searchNumIF <= maxNumIF); 

        return des;

    }  // computeDEGraph()

    protected void performBetaExpansions(ArrayList nodesToExpand, int maxDFChainSize, ArrayList des,
                                         boolean mergeDEs, boolean discardOrderPerms) {

        Iterator itNodes = nodesToExpand.iterator();
        while (itNodes.hasNext()) {
            DENode n = (DENode)itNodes.next();
            if (!n.descInconsistent()) {
                generateAllBetaDescendants(n, maxDFChainSize, des, mergeDEs, discardOrderPerms);
            }
        }
    }
    
    protected void printOrderRelations(ArrayList nodes) {

        int n = nodes.size();
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                
                DENode node_i = (DENode)nodes.get(i);
                DENode node_j = (DENode)nodes.get(j);
                
                if (node_i.getNumIF() == node_j.getNumIF()) {
                    ModeAssignment ma_i = node_i.getModeAssignment();
                    ModeAssignment ma_j = node_j.getModeAssignment();

                    if (ma_i.equalComponents(ma_j)) {

                        GraphMatrix tc_i = ma_i.getMaDag().computeTransitiveClosure(false);
                        GraphMatrix tc_j = ma_j.getMaDag().computeTransitiveClosure(false);

                        int order = ma_i.compareOrderTo(ma_j, tc_i, tc_j);
                        if (order == -1) {
                            System.out.println("WEAKER: " + node_i.getID() + " than " + node_j.getID());
                        } else if (order == 1) {
                            System.out.println("STRONGER: " + node_i.getID() + " than " + node_j.getID());
                        } else {
                            assert(order == 0);

                            System.out.println("INDEFINITE: " + node_i.getID() + ", " + node_j.getID());
                        }
                        
                        
                    }
                }
                
            }
        }
    }
    

    /*
     * Performs call to TP, returns "null" if consistent, otherwise the
     * conflict set is returned.
     */
    protected ArrayList callTheoremProver(ModeAssignment ma) {
     
        TreeMap components = diagProblem.getComponents();
        Iterator itC = components.values().iterator();
        ArrayList posAssumptions = new ArrayList();
        ArrayList negAssumptions = new ArrayList();

        // iterate through all comp.; determine pos. and neg. assumptions based on ma

        while (itC.hasNext()) {
            Component c = (Component)itC.next();
            Mode m = ma.getMode(c);
            if (m == null) m = c.getModeNAB();
            
            if (m.getType() == Mode.MODE_NAB) {
                Assumption a = c.getModeNAB().getAssumption();
                if (a != null) posAssumptions.add(a);
                
                a = c.getModeAB().getAssumption();
                if (a != null) negAssumptions.add(a);

                a = c.getModeIF().getAssumption();
                if (a != null) negAssumptions.add(a);

                Collection dfModes = c.getModesDF();
                Iterator itDFModes = dfModes.iterator();
                while (itDFModes.hasNext()) {
                    Mode dfm = (Mode)itDFModes.next();
                    a = dfm.getAssumption();
                    if (a != null) negAssumptions.add(a);
                }

            } else if (m.getType() == Mode.MODE_IF) {

                Assumption a = c.getModeNAB().getAssumption();
                if (a != null) negAssumptions.add(a);

                a = c.getModeAB().getAssumption();
                if (a != null) posAssumptions.add(a);

                a = c.getModeIF().getAssumption();
                if (a != null) posAssumptions.add(a);

                Collection dfModes = c.getModesDF();
                Iterator itDFModes = dfModes.iterator();
                while (itDFModes.hasNext()) {
                    Mode dfm = (Mode)itDFModes.next();
                    a = dfm.getAssumption();
                    if (a != null) negAssumptions.add(a);
                }
            
            } else {  // DF

                assert(m.getType() == Mode.MODE_DF);

                Assumption a = c.getModeNAB().getAssumption();
                if (a != null) negAssumptions.add(a);

                a = c.getModeAB().getAssumption();
                if (a != null) posAssumptions.add(a);

                a = c.getModeIF().getAssumption();
                if (a != null) negAssumptions.add(a);

                Collection dfModes = c.getModesDF();
                Iterator itDFModes = dfModes.iterator();
                while (itDFModes.hasNext()) {
                    Mode dfm = (Mode)itDFModes.next();
                    a = dfm.getAssumption();
                    if (a != null) {
                        if (dfm.getParent() == m.getParent()) posAssumptions.add(a);
                        else negAssumptions.add(a);
                    }
                }
            }
            
        }  // iterate through all components

        // make call to theorem prover
        
        boolean consistent = theoremProver.checkConsistency(posAssumptions, negAssumptions);
        if (consistent) return null;
        else return theoremProver.getConflictSet();

    } // callTheoremProver()

    /*
     * Adds a new conflict set which is defined by a list of Assumption's.
     *
     * Return the converted conflict set.
     */
    protected ConflictSet addConflictSet(ArrayList assumptions) {

        ConflictSet cs = new ConflictSet();

        Iterator itAss = assumptions.iterator();
        while (itAss.hasNext()) {
            Assumption a = (Assumption)itAss.next();
            SplittedAssumption sa = diagProblem.splitAssumption(a);
            setMode(cs, sa);            
        }

        conflictSets.add(cs);
        return cs;
    }

    /*
     * Util method for conflictsWithDescendants().
     *
     * Returns true iff at least one of the components represented by itPossDesc
     * depend in the FDG (directly or indirectly) on c.
     *
     * It is asserted that the modes in itPossDesc are IF-modes. Moreover, such a mode is only
     * considered if it NOT in constCompModes!! 
     */
    protected boolean existsDependentComp(Component c, Iterator itPossDesc, TreeSet constCompModes) {
        boolean result = false;

        while (itPossDesc.hasNext()) {
            Mode depM = (Mode)itPossDesc.next();
            assert(depM.getType() == Mode.MODE_IF);

            if (!constCompModes.contains(depM) 
                && diagProblem.getFDG().hasIndirectDependency(c, depM.getComponent())) {
                
                result = true;
                break;
            }
        }
        
        //System.out.println("existsDependentComp(): " + result);
        return result;
    }

    /*
     * Returns true iff one of the components in itPossAnc is an ancestor of c.
     */
    protected boolean existsAncestor(Component c, Iterator itPossAnc) {
        boolean result = false;

        while (itPossAnc.hasNext()) {
            Mode ancM = (Mode)itPossAnc.next();
            if (diagProblem.getFDG().hasIndirectDependency(ancM.getComponent(), c)) {
                result = true;
                break;
            }
        }
        
        //System.out.println("existsAncestor(): " + result);
        return result;
    }
    
    /*
     * Like existsDependentComp(), but this method checks if c and one of the IF-comp. in itModes
     * have an common ancestor.
     */
    protected boolean existsCommonAncestor(Component c, Iterator itIFModes, TreeSet constCompModes) {
        boolean result = false;
   
        while (itIFModes.hasNext()) {
            Mode ifMode = (Mode)itIFModes.next();
            assert(ifMode.getType() == Mode.MODE_IF);

            if (!constCompModes.contains(ifMode)
                && diagProblem.getFDG().haveCommonAncestor(ifMode.getComponent(), c)) {

                result = true;
                break;
            }
        }
        
        //System.out.println("existsCommonAncestor(): " + result);
        return result;
    }

    /*
     * Returns true if cs proves that all descendants of n must be inconsistent.
     */
    protected boolean conflictsWithDescendants(ConflictSet cs, DENode n, boolean computeBetaDE) {
     
        //System.out.println("check conflictsWithDescendants(): cs = " + cs.toStringShort() + " --- " + n.toStringShort());

        // constMA is those part of n.ma which remains constant in all descendants;
        // in the current implementation: constMA = n.ma
        ModeAssignment constMA = n.getModeAssignment();  
        //System.out.println("Constant MA: " + constMA.toStringShort());

        // iterate through conflict set modes, determine if cs refutes n and its descendants

        boolean result = true;  // result is true per default, try to find reason why not all descendants are refuted

        Iterator itCS = cs.iterator();
        while (itCS.hasNext()) {
            Mode csMode = (Mode)itCS.next();
            //System.out.println("csMode: " + csMode.toStringShort());
            Component csComp = csMode.getComponent();
            
            Mode constMode = constMA.getMode(csComp);
            
            // check if csComp, the component in the conflict set, is in constMA

            if (constMode != null) {  // yes!
                
                //System.out.println("csComp is in constMA, constMode: " + constMode.toStringShort());

                // result is false if the modes are different in cs and constMa 
                // AND if it is not the case that csComp is IF in cs and DF in constMA

                if (! ( (constMode == csMode) 
                        || ((csMode.getType() == Mode.MODE_IF) 
                            && (constMode.getType() == Mode.MODE_DF)) )
                    ) {
                    
                    assert(false);  // because I think this case cannot occur (?)
                    //System.out.println("No conflict: csComp has different modes in cs and constMA");
                    result = false;
                    break;
                }

            } else {  // no: csComp is not in constMA

                //System.out.println("csComp is not in constMA");

                // check if csComp is NAB in both cs and n.ma

                if ((csMode.getType() == Mode.MODE_NAB) 
                    && (n.getModeAssignment().getMode(csComp) == null)) {  // yes!
                    
                    //System.out.println("But csComp is NAB in both cs and n.ma");

                    // ----- determine: is NAB(csComp) constant in all descendants of n??  -----

                    boolean constNAB = true;

                    // if computeBetaDE: is there any FDG-ancestor of csComp in n.ma?
                    if (computeBetaDE && existsAncestor(csComp, n.getModeAssignment().iterator())) {
                        //System.out.println("No conflict: computeBetaDE, and there is an ancestor of csComp");
                        constNAB = false;
                    }

                    // only for ALPHA nodes:

                    if (constNAB && (n.getType() == DENode.TYPE_ALPHA)) {
   
                        // is csComp a FDG-ancestor of any IF-component?
                        if (existsDependentComp(csComp, 
                                                n.getModeAssignment().getModeIterator(Mode.MODE_IF),
                                                n.getConstCompModes())) {
                            
                            //System.out.println("No conflict: there is an IF-component which depends on csComp");
                            constNAB = false;
                        }
                        
                        // if computeBetaDE: have csComp and any IF-component a common ancestor?
                        if (constNAB  && computeBetaDE
                            &&  existsCommonAncestor(csComp, n.getModeAssignment().getModeIterator(Mode.MODE_IF),
                                                     n.getConstCompModes())) {

                            //System.out.println("No conflict: computeBetaDE, and csComp has a common ancestor " + "with an IF-component");
                            constNAB = false;
                        }                                                             
                        
                    }                    

                    //System.out.println("constNAB(" + csComp.getName() + "): " + constNAB);

                    if (!constNAB) {
                        result = false;
                        break;
                    }

                } else {  // no, csComp is IF or DF in cs and/or n.ma (and csComp is not in constMA)
                    //System.out.println("csComp is != NAB either in cs or in n.ma");
                    result = false;
                    break;
                }
                
            }
        }

        //System.out.println("conflictsWithDescendants: RESULT = " + result);
        return result;

    }  // conflictsWithDescendants()

    /*
     * Looks for a conflict set which conflicts with n.ma.
     *
     * Returns null if no such CS is found.
     */
    protected ConflictSet searchConflictForNode(DENode n) {
        
        // first: check if conflict set p.cs of parent p refutes n.ma

        if (n.hasParents()) {
            Iterator itParents = n.getParentsIterator();
            if (itParents.hasNext()) {
                DENode parent = (DENode)itParents.next();
                assert(!itParents.hasNext()); // only 1 parent possible!
                ConflictSet cs = parent.getConflictSet();
                if ((cs != null) && cs.conflictsWith(n.getModeAssignment())) {
                    //System.out.println("parent conflict set refutes child");
                    return cs;
                }
            }
        }

        // if not: search through all conflict sets

        return conflictSets.conflictsWith(n.getModeAssignment());
    }

    /*
     * Returns if the MA of n is consistent.
     *
     * First checks if n.ma is in conflict with a previously computed conflict set.
     * If not, make a call to the TP.
     */
    protected boolean checkConsistency(DENode n, boolean computeBetaDE) {

        assert(!n.descInconsistent() && (n.consistent() == DENode.STATUS_UNKNOWN));

        boolean result;

        if (!n.hasParents()) {  // root node: n.ma is a minimal diagnosis => consistent!
            n.setConsistent(DENode.STATUS_TRUE);            
            //System.out.println("root node must be consistent: " + n.toStringShort());
            result = true;  
        }
        
        else {  // no root node and the descInconsistent flag is not set

            ModeAssignment ma = n.getModeAssignment();
        
            // check if there is a conflict set refuting n.ma
            
            ConflictSet cs = searchConflictForNode(n);
            if (cs != null) {  // CS found
                assert(callTheoremProver(ma) != null);
                
                n.setConsistent(DENode.STATUS_FALSE);
                n.setConflictSet(cs);
                stats.numConflicting++;

                //System.out.println("existing conflict set refutes MA; ma: " + ma.toStringShort() + ";  cs: " + cs.toStringShort());

                // does CS also refute the descendants?

                if (conflictsWithDescendants(cs, n, computeBetaDE)) {
                    deGraph.propagateInconsToDesc(n);
                    ++stats.numDescInconsistent;
                }

                result = false;
            }
        
            else {  // no CS found -> TP call required

                ArrayList newCS = callTheoremProver(ma);
                boolean consistent = (newCS == null);

                stats.numTPCalls++;
                if (n.getType() == DENode.TYPE_ALPHA) stats.numTPCalls_Alpha++;
                else stats.numTPCalls_Beta++; 

                //System.out.println("theorem prover checks consistency: " + ma.toStringShort() + ": " + consistent);

                if (consistent) {
                    n.setConsistent(DENode.STATUS_TRUE);
                    result = true;
                }
                else {

                    // store new conflict set ; determine if the new CS also refultes descendants;                    

                    n.setConsistent(DENode.STATUS_FALSE);
                    cs = addConflictSet(newCS);
                    n.setConflictSet(cs);
                    if (conflictsWithDescendants(cs, n, computeBetaDE)) {
                        deGraph.propagateInconsToDesc(n);
                        ++stats.numDescInconsistent; 
                    }
                    result = false;
                }
            }
        }

        if (((n.getType() == DENode.TYPE_ALPHA) && n.expanded(DENode.TYPE_ALPHA))  
            || ((n.getType() == DENode.TYPE_BETA) && n.expanded(DENode.TYPE_BETA))) {

            n.releaseConstCompModes();
        }
        return result;

    }  // checkConsistency()

    protected ArrayList expandNodeAlpha(DENode n) {

        ArrayList result = new ArrayList();
        TreeSet ccm = null;
        
        // iterate through IF components in n.ma
        
        Iterator itMA = n.getModeAssignment().getModeIterator(Mode.MODE_IF);
        while (itMA.hasNext()) {  
            Mode m = (Mode)itMA.next();
            assert(m.getType() == Mode.MODE_IF);
            Component c = m.getComponent();
 
            if (c.getFDNode().hasParents()) {

                if (ccm == null) {
                    ccm = (TreeSet)n.getConstCompModes().clone();
                }
                
                // determine if this IF is constant 
                if (!ccm.contains(m)) {  // not constant -> proceed
                    
                    // iterate through parents of c (in FDG)
                    
                    Iterator itParents = c.getFDNode().getParentsIterator();
                    while (itParents.hasNext()) {
                        FailureDepNode parFDNode = (FailureDepNode)itParents.next();
                        Component c_i = parFDNode.getComponent();
                        
                        // create mode assignment of new ALPHA node
                        
                        ModeAssignment newMA = (ModeAssignment)n.getModeAssignment().clone();
                        
                        if (newMA.getMode(c_i, Mode.MODE_NAB) == Mode.MODE_NAB) {
                            newMA.setMode(c_i, Mode.MODE_IF, null);
                        }
                        newMA.subst(c, Mode.MODE_DF, c_i);
                        
                        // create new ALPHA node
                        
                        TreeSet newCCM = (TreeSet)ccm.clone();
                        DENode newNode = new DENode(DENode.TYPE_ALPHA, n.getRootNode(), 
                                                    n.getDistanceToRoot() + 1, newMA, newCCM);
                        result.add(newNode);
                    }
                    
                    ccm.add(m);

                }  // if (!ccm.contains(m))
            }  //  if (c.getFDNode().hasParents())

        }  // iterate through IF components in n.ma

        return result;

    }  // expandNodeAlpha

    protected ArrayList expandNodeBeta(DENode n) {
        
        ArrayList result = new ArrayList();
        TreeSet ccm = null;
        
        // iterate through IF/DF components c
        
        Iterator itMA = n.getModeAssignment().iterator();
        while(itMA.hasNext()) {
            
            Mode m = (Mode)itMA.next();
            assert((m.getType() == Mode.MODE_IF) || (m.getType() == Mode.MODE_DF));
            Component c = m.getComponent();
   
            // iterate through FDG children c_i of c
            
            Iterator itChildren = c.getFDNode().getChildrenIterator();
            while (itChildren.hasNext()) {
                FailureDepNode childFDNode = (FailureDepNode)itChildren.next();
                Component c_i = childFDNode.getComponent();
                
                // if FDG child is NAB -> proceed; otherwise: continue with next child

                int modeType_c_i = n.getModeAssignment().getMode(c_i, Mode.MODE_NAB);
                if (modeType_c_i == Mode.MODE_NAB) {

                    if (ccm == null) {
                        if (n.getType() == DENode.TYPE_BETA) {
                            ccm = (TreeSet)n.getConstCompModes().clone();
                        }
                        else {
                            ccm = new TreeSet();
                        }
                    }

                    // check if "df(c, c_i)" is in ccm, if no: proceed

                    Mode mode_df_c_i = c_i.getMode(Mode.MODE_DF, c);
                    if (!ccm.contains(mode_df_c_i)) {
                        
                        // create new mode assignment and new CCM; create new DENode
                        
                        ModeAssignment newMA = (ModeAssignment)n.getModeAssignment().clone();
                        newMA.setMode(c_i, Mode.MODE_DF, c);
                        TreeSet newCCM = (TreeSet)ccm.clone();
                        DENode newNode = new DENode(DENode.TYPE_BETA, n.getRootNode(), 
                                                    n.getDistanceToRoot() + 1, newMA, newCCM);
                        newNode.setMinNumIF(n.getMinNumIF());
                        result.add(newNode);
                        ccm.add(mode_df_c_i);
                    }
  
                }  // FDG child is NAB
                
            }  // iterate through FDG children c_i of c 

        }  // iterate through IF/DF modes in MA for BETA expansion

        return result;

    }  // expandNodeBeta()


    protected ArrayList expandNode(DENode n, int type) {

        ArrayList result;

        if (type == DENode.TYPE_ALPHA) 
        {
            assert((n.getType() == DENode.TYPE_ALPHA) && !n.expanded(DENode.TYPE_ALPHA));
            result = expandNodeAlpha(n);
            n.setExpanded(DENode.TYPE_ALPHA);
        } else {
            assert(!n.expanded(DENode.TYPE_BETA));
            result = expandNodeBeta(n);
            n.setExpanded(DENode.TYPE_BETA);
        }

        // note: const.comp.modes are required by checkConsistency()
        // => they can be released only after checking the consistency
        if ((n.consistent() != DENode.STATUS_UNKNOWN) 
            && (((n.getType() == DENode.TYPE_ALPHA) && n.expanded(DENode.TYPE_ALPHA))
                || ((n.getType() == DENode.TYPE_BETA) && n.expanded(DENode.TYPE_BETA)))) {

            n.releaseConstCompModes();  
        }
        
        return result;
    }

    protected int computeMinNumIF(ModeAssignment ma, int maxDFChainSize) {
        
        LinkedList ifComps = new LinkedList();
        
        Iterator itIF = ma.getModeIterator(Mode.MODE_IF);
        while (itIF.hasNext()) {
            Mode m = (Mode)itIF.next();
            ifComps.add(m.getComponent());
        }

        ArrayList transPiPartitions = diagProblem.getFDG().computeTransPiPartitions(ifComps, maxDFChainSize);
        int minNumIF = transPiPartitions.size();

        /*
          System.out.print("minNumIF of MA " + n.getModeAssignment().toStringShort() + ": " + minNumIF);
          System.out.print(";  PI_r: ");
          for (int i = 0; i < transPiPartitions.size(); ++i) {
          ArrayList part = (ArrayList)transPiPartitions.get(i);
          for (int j = 0; j < part.size(); ++j) {
          Component c = (Component)part.get(j);
          System.out.print(c.getName());
          if (j < part.size() - 1) System.out.print(",");
          }
          if (i < transPiPartitions.size() - 1) System.out.print("; ");
          }
          System.out.println();
        */

        return minNumIF;
    }
    
    protected void printComponents() {
        Iterator itC = diagProblem.getComponents().values().iterator();
        while (itC.hasNext()) {
            Component c = (Component)itC.next();
            System.out.println("\n");
            System.out.println(c.toString());
        }
    }

    protected void printConsistentNodes() {
        Iterator itN = deGraph.iterator();
        while (itN.hasNext()) {
            DENode n = (DENode)itN.next();
            if (n.consistent() == DENode.STATUS_TRUE) {
                System.out.println(n.toStringShort());
            }
        }
    }

    protected void printInconsistentNodes() {
        Iterator itN = deGraph.iterator();
        while (itN.hasNext()) {
            DENode n = (DENode)itN.next();
            if (n.consistent() == DENode.STATUS_FALSE) {
                System.out.println(n.toStringShort());
            }
        }
    }


}
