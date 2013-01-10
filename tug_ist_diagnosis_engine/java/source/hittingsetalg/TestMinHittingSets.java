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



package hittingsetalg;

import java.io.*;      // IO specific classes (File, Stream,..)
import java.util.*;    // Java util

import theoremprover.*;
import utils.SortedIntList;

public class TestMinHittingSets {


    public static void main(String[] args) {

        TestMinHittingSets inst = new TestMinHittingSets();
        inst.run();
    }

    void createConflictSets(ArrayList conflictSets, long seed) {
        SortedIntList cs = new SortedIntList();
        cs.addSorted(1);
        cs.addSorted(2);
        conflictSets.add(cs);

        cs = new SortedIntList();
        cs.addSorted(2);
        cs.addSorted(3);
        conflictSets.add(cs);
        
        cs = new SortedIntList();
        cs.addSorted(1);
        cs.addSorted(3);
        conflictSets.add(cs);
        
        cs = new SortedIntList();
        cs.addSorted(2);
        cs.addSorted(4);
        conflictSets.add(cs);
        
        cs = new SortedIntList();
        cs.addSorted(2);
        conflictSets.add(cs);
    }

    void createConflictSets1(ArrayList conflictSets, long seed) {
        
        int numComponents = 20;
        int minNumConflictSets = 30;
        int maxNumConflictSets = 50;
        int minSizeConflictSet = 2;
        int maxSizeConflictSet = 6;

        Random r = new Random();
        if (seed != 0) r.setSeed(seed);

        int numCS = r.nextInt(maxNumConflictSets - minNumConflictSets) + minNumConflictSets;
        System.out.println("number of CS: " + numCS);
        System.out.println();

        for (int i = 0; i < numCS; ++i) {
            SortedIntList cs = new SortedIntList();
            int csSize = r.nextInt(maxSizeConflictSet - minSizeConflictSet) + minSizeConflictSet;
            
            for (int j = 0; j < csSize; ++j) {
                int c = r.nextInt(numComponents);
                cs.addSorted(c);
            }
            
            conflictSets.add(cs);

            System.out.println("CS: " + cs);
        }       

 
    }

    void run() {
        
        Random r = new Random();
        //long seed = r.nextLong();
        long seed = 6654432597477221005L;
        System.out.println("SEED: " + seed);

        try {
            ArrayList conflictSets = new ArrayList();
            createConflictSets1(conflictSets, seed);        
            
            MinHittingSets hittingSets = new MinHittingSets(false, conflictSets);
            
            Date startComputationTime = new Date();
            
            hittingSets.compute(100, 100);
            
            Date endComputationTime = new Date();
            long passedTime = endComputationTime.getTime() - startComputationTime.getTime();

            Iterator itHS = hittingSets.getMinHSAsIntLists().iterator();
            while(itHS.hasNext()) {
                System.out.println();
                
                SortedIntList hs = (SortedIntList)itHS.next();
                
                Iterator itInt = hs.iterator();
                while(itInt.hasNext()) {
                    Integer n = (Integer)itInt.next();
                    System.out.print(n.intValue() + " ");
                }
                System.out.println();
            }
            
            System.out.println();        

            System.out.println("passed milliseconds: " + passedTime);            

            boolean minimal = hittingSets.checkMinimalityHS();
            if (minimal) System.out.println("MINIMAL!");
            else System.out.println("NOT MINIMAL!");

            boolean hitsAllCS = hittingSets.hitsAllConflictSets();
            if (hitsAllCS) System.out.println("OK, hits all conflict sets");
            else System.out.println("ERROR: does not hit all conflict sets!");

        } catch (AssertionError e) {
            System.out.println("SEED: " + seed);
            throw e;
        }
    }



}
