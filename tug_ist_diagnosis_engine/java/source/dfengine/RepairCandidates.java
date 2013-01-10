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

public class RepairCandidates {
    
    // Map from Integer to ArrayList of RepairCandidate
    protected TreeMap items = new TreeMap();

    protected int size = 0;


    protected Collection getCandidatesOfSize(int size) {
        Object o = items.get(new Integer(size));
        if (o == null) return null;
        else return (Collection)o;
    }

    /*
     * If a repair candidate exists which has exactly the same components as ma:
     * return this candidate. Otherwise, return null.
     */
    public RepairCandidate findCandidateFor(ModeAssignment ma) {
        Collection candList = getCandidatesOfSize(ma.size());
        if (candList == null) return null;
        else {
            Iterator itCand = candList.iterator();
            while (itCand.hasNext()) {
                RepairCandidate rc = (RepairCandidate)itCand.next();
                assert(rc.size() == ma.size());
                if (rc.equalComponents(ma)) return rc;
            }
        }

        return null;
    }
    
    public Iterator iterator() {
        return new RepairCandidatesIterator();
    }

    public int size() {
        return size;
    }

    public String toString() {
        StringBuffer result = new StringBuffer();

        result.append("number of repair candidates: " + size);
        result.append("\n********************************\n");
        
        Iterator itCands = iterator();
        while (itCands.hasNext()) {
            RepairCandidate cand = (RepairCandidate)itCands.next();
            result.append(cand.toString());
            result.append("\n-----------------------------------------------\n");
        }

        return result.toString();
    }

    public void add(ModeAssignment ma) {
        
        // precond
        assert(findCandidateFor(ma) == null);

        Collection cands = getCandidatesOfSize(ma.size());
        if (cands == null) {
            cands = new ArrayList();
            items.put(new Integer(ma.size()), cands);
        }
        RepairCandidate rc = new RepairCandidate(ma);
        cands.add(rc);

        ++size;

        // postcond 
        assert((findCandidateFor(ma)).equalComponents(ma));
    }

    
    // ---------------------------------------------------------------

    /*
     * Iterates through all RepairCandidate's.
     */
    class RepairCandidatesIterator implements Iterator {

        Iterator itValues;

        Iterator itColl;

        RepairCandidate nextCand = null;


        public RepairCandidatesIterator() {
            itValues = items.values().iterator();
            moveToNext();
        }

        protected void moveToNext() {
            if ((itColl == null) || (!itColl.hasNext())) {
                if (itValues.hasNext()) {
                    Collection cands = (Collection)itValues.next();
                    itColl = cands.iterator();
                    assert(itColl.hasNext());  // there must be no empty collections here!
                    nextCand = (RepairCandidate)itColl.next();
                } else {
                    nextCand = null;
                }
            } else {
                nextCand = (RepairCandidate)itColl.next();
            }
        }

        public boolean hasNext() {
            return (nextCand != null);
        }

        public Object next() {
            RepairCandidate result = nextCand;
            moveToNext();
            return result;
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }

    }

}
