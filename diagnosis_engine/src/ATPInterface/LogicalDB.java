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


/*
 * LogicalDB: Encapsulates a logical DB including the assumption-based theorem prover.
 *
 * This class implements the LogicalDBInterface. 
 * It is notable that this implementation caches computed values (consistency, 
 * explanations), thus repeated calls to createdExplanations (for example) will
 * return the same results, if the DB has not changed meanwhile. 
 *
 * @author Joerg WEBER
 * @version 1.0, DATE: 18.10.2005
 *
 *
 */

package ATPInterface;

import java.util.*;

import theoremprover.IllegalAssumption;
import dfengine.DiagnosisProblem;
import dfengine.FailureDepGraph;
import dfengine.ModeAssignment;
import dfengine.Component;
import dfengine.RepairCandidates;
import dfengine.RepairCandidate;

public class LogicalDB implements LogicalDBInterface {

    protected boolean changed;

    protected boolean lastUseFaultModes;

    protected boolean lastMergeDEs;

    protected int lastMaxExplSize = -1;

    protected int lastMaxNumExpl = -1;

    protected boolean lastComputeBetaDE;

    protected int lastMaxDFChainSize = -1;

    // cached for later reuse
    protected boolean consistent;

    // cached for later reuse
    protected ArrayList explanations;

    // cached for later reuse
    protected ArrayList diagEnvs;

    protected TreeMap subDBs;

    // the union SD |_| OBS
    protected LogicalSubDB totalDB;

    protected DiagnosisProblem diagProblem;


    /*
     * Use this constructer when no fault modes (ab, -ab) are used.
     */
    public LogicalDB() {
        changed = false;
        consistent = true;
        explanations = new ArrayList();
        diagEnvs = new ArrayList();
        
        diagProblem = new DiagnosisProblem(false, ATPConstants.DEF_AB_ASSUMPTION, ATPConstants.DEF_NAB_ASSUMPTION,
                                           ATPConstants.DEF_IF_ASSUMPTION, ATPConstants.DEF_DF_ASSUMPTION);

        subDBs = new TreeMap();
        subDBs.put(ATPConstants.SUBDB_SD, new LogicalSubDB(ATPConstants.SUBDB_SD));
        subDBs.put(ATPConstants.SUBDB_OBS, new LogicalSubDB(ATPConstants.SUBDB_OBS));
        subDBs.put(ATPConstants.SUBDB_SDD, new LogicalSubDB(ATPConstants.SUBDB_SDD));
    }

    public int addRules(String subDB, boolean replace, ArrayList newRules) 
        throws LogicParseException {
        
        Object o = subDBs.get(subDB);
        assert(o != null);
        LogicalSubDB lsubDB = (LogicalSubDB)o;

        if (replace) lsubDB.clear();
        lsubDB.addRules(newRules);
        changed = true;

        return lsubDB.getNumRules();
    }

    public int addFDGEdges(ArrayList edgeStrings, boolean replace) throws LogicParseException {
        
        if (replace) diagProblem.clearFDGEdges();

        for (int i = 0; i < edgeStrings.size(); ++i) {

            String edgeStr = (String)edgeStrings.get(i);
            int pos = edgeStr.indexOf(ATPConstants.STR_FDG_EDGE);
            
            if (pos < 0) {
                if (edgeStr.length() > 0) {
                    String comp1Str = edgeStr;
                    if (!diagProblem.hasComponent(comp1Str)) diagProblem.addComponent(comp1Str);
                }
            } else {
                String comp1Str = edgeStr.substring(0, pos).trim();
                String comp2Str = edgeStr.substring(pos + ATPConstants.STR_FDG_EDGE.length()).trim();
                
                if ((comp1Str.length() == 0) || (comp2Str.length() == 0)) {
                    throw new LogicParseException(i + 1);
                }
                
                if (!diagProblem.hasComponent(comp1Str)) diagProblem.addComponent(comp1Str);
                if (!diagProblem.hasComponent(comp2Str)) diagProblem.addComponent(comp2Str);
                diagProblem.addFailureDep(comp1Str, comp2Str);
            }
        }

        changed = true;
        return diagProblem.getFDG().getNumEdges();
    }

    public int getNumSubDBs() {
        return subDBs.size();
    }

    /*
     * Checks the consistency of SD |_| OBS |_| {-ab(c) | c in COMP}
     */
    public boolean checkConsistency(boolean useFaultModes) throws LogicParseException {

        if (!changed && (useFaultModes == lastUseFaultModes)) return consistent;  // cache computed values!
        else {
            
            // create total DB

            totalDB = new LogicalSubDB("total");
            totalDB.append((LogicalSubDB)subDBs.get(ATPConstants.SUBDB_SD));
            totalDB.append((LogicalSubDB)subDBs.get(ATPConstants.SUBDB_OBS));

            // perform the theorem proving

            System.out.println("check consistency");

            consistent = totalDB.checkConsistency(useFaultModes, ATPConstants.DEF_AB_ASSUMPTION,
                                                  ATPConstants.DEF_NAB_ASSUMPTION);
            
            if (consistent) {
                explanations = new ArrayList();  // there are no explanations
                diagEnvs = new ArrayList();
            } else {
                explanations = null;  // there are explanations, but they are not computed now!
                diagEnvs = null;
            }
            
            changed = false;  // DB has not changed since last consistency check
            lastUseFaultModes = useFaultModes;
            
            return consistent;
        }
    }

    public void performConsistencyChecks(ArrayList queries, boolean useFaultModes, BitSet result) 
        throws LogicParseException{

        LogicalSubDB queryDB = new LogicalSubDB("QUERY");
        LogicalSubDB sd_obs = new LogicalSubDB("SD_OBS");
        sd_obs.append((LogicalSubDB)subDBs.get(ATPConstants.SUBDB_SD));
        sd_obs.append((LogicalSubDB)subDBs.get(ATPConstants.SUBDB_OBS));

        System.out.println("perform inferences");

        int index = 0;
        Iterator itQuery = queries.iterator();
        while (itQuery.hasNext()) 
        {
            // fill queryDB

            String queryStr = (String)itQuery.next();
            queryDB.clear();
            ArrayList rules = new ArrayList();
            rules.add(queryStr);
            queryDB.addRules(rules);
            
            // create logical DB containing all rules

            LogicalSubDB totalDB = new LogicalSubDB("total");
            totalDB.append(sd_obs);
            totalDB.append(queryDB);
            
            // perform consistency check; set result
            consistent = totalDB.checkConsistency(useFaultModes, ATPConstants.DEF_AB_ASSUMPTION,
                                                  ATPConstants.DEF_NAB_ASSUMPTION);
            assert(index < queries.size());
            result.set(index, consistent);
            assert(result.length() <= index + 1);
            ++index;
            
        }
    
    }  // performConsistencyChecks()

    public ArrayList computeMinDiag(int maxExplSize, int maxNumExpl,
                                    boolean useFaultModes, boolean verboseOutput) 
        throws LogicParseException, IllegalAssumption {

        if (changed || (lastUseFaultModes != useFaultModes)) {
            checkConsistency(useFaultModes);
        }

        if ((explanations == null) || (maxExplSize != lastMaxExplSize) || (maxNumExpl != lastMaxNumExpl)) {
            System.out.println("Compute explanations");            
            explanations = totalDB.computeMinDiag(maxExplSize, maxNumExpl, useFaultModes, 
                                                  ATPConstants.DEF_AB_ASSUMPTION,
                                                  ATPConstants.DEF_NAB_ASSUMPTION,
                                                  verboseOutput);
        }

        lastMaxExplSize = maxExplSize;
        lastMaxNumExpl = maxNumExpl;

        return explanations;
    }

    public ArrayList computeDEs(int maxExplSize, int maxNumExpl, 
                                boolean computeBetaDE, int maxDFChainSize, boolean mergeDEs,
                                boolean discardOrderPerms, ArrayList minDiags) 
        throws LogicParseException, IllegalAssumption {
    
        if (changed || !lastUseFaultModes) {
            checkConsistency(true);
        }

        System.out.println("Compute diagnosis environments");
        
        LogicalSubDB subDB = (LogicalSubDB)subDBs.get(ATPConstants.SUBDB_SD);
        diagProblem.setSD(subDB.getRulesAsLSentence());
        subDB = (LogicalSubDB)subDBs.get(ATPConstants.SUBDB_OBS);
        diagProblem.setOBS(subDB.getRulesAsLSentence());
        subDB = (LogicalSubDB)subDBs.get(ATPConstants.SUBDB_SDD);
        diagProblem.setSDD(subDB.getRulesAsLSentence());
        
        ArrayList minHS;
        ArrayList conflictSets = new ArrayList();
        try {
            
            // minHS is an ArrayList of ArrayList of Component
            minHS = diagProblem.computeMinHittingSets(maxExplSize, maxNumExpl, conflictSets); 
            
            if (!mergeDEs) {
                
                // computation
                
                ArrayList minDEs = diagProblem.computeDEs(minHS, computeBetaDE, 
                                                          maxDFChainSize, conflictSets);
                // compose string result
                
                diagEnvs = new ArrayList(minDEs.size());
                Iterator itDE = minDEs.iterator();
                while (itDE.hasNext()) {
                    ModeAssignment de = (ModeAssignment)itDE.next();
                    diagEnvs.add(de.toStringShort());
                }
            } else {
                
                // computation
                
                RepairCandidates rcs 
                    = diagProblem.computeRepairCandidates(minHS, computeBetaDE,
                                                          maxDFChainSize, discardOrderPerms, conflictSets);
                // compose string result
                
                diagEnvs = new ArrayList(rcs.size());
                Iterator itRC = rcs.iterator();
                while (itRC.hasNext()) {
                    RepairCandidate rc = (RepairCandidate)itRC.next();
                    diagEnvs.add(rc.toString());
                }
                
            }
            
            // compose minDiags
            
            Iterator itMinHS = minHS.iterator();
            while (itMinHS.hasNext()) 
            {
                String diagStr = "";
                
                ArrayList diagnosis = (ArrayList)itMinHS.next();
                Iterator itComps = diagnosis.iterator();
                while (itComps.hasNext()) 
                {
                    Component c = (Component)itComps.next();
                    if (diagStr.length() > 0) diagStr = diagStr + "; ";
                    diagStr = diagStr + ATPConstants.DEF_NAB_ASSUMPTION + "(" + c.getName() + ")";
                }
                
                minDiags.add(diagStr);
            }
            
        } catch (theoremprover.ParseError exc) {
            throw new LogicParseException();
        }            
    
        // store options
        
        lastMaxExplSize = maxExplSize;
        lastMaxNumExpl = maxNumExpl;
        lastComputeBetaDE = computeBetaDE;
        lastMaxDFChainSize = maxDFChainSize;
        lastMergeDEs = mergeDEs;
        
        return diagEnvs;
        
    }  // computeDEs()

    public int getTotalNumRules() {

        int result = 0;

        Iterator itSubDBs = subDBs.values().iterator();
        
        while(itSubDBs.hasNext()) {
            LogicalSubDB sub = (LogicalSubDB)itSubDBs.next();
            result += sub.getNumRules();
        }

        return result;
    }

    public int getSubDBNumRules(String name) {
        Object o = subDBs.get(name);
        assert(o != null);
        LogicalSubDB subDB = (LogicalSubDB)o;
        return subDB.getNumRules();
    }

    public ArrayList createSubDBStats() {
        ArrayList result = new ArrayList();

        Iterator itSubDBs = subDBs.values().iterator();
        
        while(itSubDBs.hasNext()) {
            LogicalSubDB sub = (LogicalSubDB)itSubDBs.next();
            SubDBStat stat = new SubDBStat();
            stat.name = sub.getName();
            stat.numRules = sub.getNumRules();
            result.add(stat);
        }

        return result;
    }
    
    public ArrayList getSubDBRules(String name) {
        Object o = subDBs.get(name);
        assert(o != null);
        LogicalSubDB subDB = (LogicalSubDB)o;
        return subDB.getRulesAsStrings();   
    }

    public FDGStat createFDGStats() {
        FailureDepGraph fdg = diagProblem.getFDG();
        FDGStat result = new FDGStat();
        result.numNodes = fdg.getNumNodes();
        result.numEdges = fdg.getNumEdges();

        return result;
    }
}
