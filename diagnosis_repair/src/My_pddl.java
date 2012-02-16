import java.io.File;
import pddl4j.Parser;
import pddl4j.Domain;
import pddl4j.Problem;
import pddl4j.PDDLObject;
import pddl4j.ErrorManager;
import pddl4j.ErrorManager.Message;
import java.util.Properties;
import java.util.Iterator;
import pddl4j.RequireKey;
import pddl4j.Source;
import pddl4j.exp.AndExp;
import pddl4j.exp.Exp;
import pddl4j.exp.NotExp;
import pddl4j.exp.Literal;
import pddl4j.exp.AtomicFormula;
import pddl4j.exp.NotAtomicFormula;
import pddl4j.exp.InitEl;
import pddl4j.exp.action.ActionDef;
import pddl4j.exp.action.Action;


public class My_pddl
{

    public static boolean check_goal( java.util.List<AtomicFormula> state, java.util.List<Literal> goal)
    {
   			NotAtomicFormula naf;

        for (Iterator<Literal> j = goal.iterator(); j.hasNext(); )
        {
		      boolean found = false;
	       	Literal l = j.next();
					naf = null;
					if (l instanceof NotAtomicFormula)
					{
						naf = (NotAtomicFormula) l;
						found = true;
					}
        	for (Iterator<AtomicFormula> i = state.iterator(); i.hasNext(); )
          {
          	AtomicFormula h = i.next();
						if (naf!=null) 
						{
							if(h.equals(naf.getExp()))
							{
								System.out.printf("Negative Goal supported" + naf.toString() + "\n");
								return false;
							}
						}                    	
						else
							{
								if(h.equals(l))
								{
									found = true;
								}
							}

         }
       	if (!found) 
				{
					System.out.printf("Goal not supported " + l.toString() +"\n");
					return false;
				}
			}

			return false;
   } // check_goal

public static void main(String[] args) {
     // Checks the command line
     if (args.length != 2) {
         System.err.println("Invalid command line");
     } else {
         // Creates an instance of the java pddl parser
        Properties options = new Properties();
        options.put("source", Source.V3_0);
        options.put(RequireKey.STRIPS, true);
        options.put(RequireKey.TYPING, true);
        options.put(RequireKey.EQUALITY, true);
        options.put(RequireKey.NEGATIVE_PRECONDITIONS, true);
        options.put(RequireKey.DISJUNCTIVE_PRECONDITIONS, true);
        options.put(RequireKey.EXISTENTIAL_PRECONDITIONS, true);
        options.put(RequireKey.UNIVERSAL_PRECONDITIONS, true);
        options.put(RequireKey.CONDITIONAL_EFFECTS, true);
         Parser parser = new Parser(options);
        try{
         Domain domain = parser.parse(new File(args[0]));
         Problem problem = parser.parse(new File(args[1]));
        
         PDDLObject obj = parser.link(domain, problem);
         // Gets the error manager of the pddl parser
         ErrorManager mgr = parser.getErrorManager();
         // If the parser produces errors we print it and stop
         if (mgr.contains(Message.ERROR)) {
             mgr.print(Message.ALL);
         } // else we print the warnings
         else {
             mgr.print(Message.WARNING);
             System.out.println("\nParsing domain \"" + domain.getDomainName()
                         + "\" done successfully ...");
             System.out.println("Parsing problem \"" + problem.getProblemName()
                         + "\" done successfully ...\n");
             System.out.println(problem.getGoal().toConjunctiveNormalForm().toString());
        
        java.util.List<Literal> goal = new java.util.ArrayList<Literal>();
		    java.util.List<AtomicFormula> state = new java.util.ArrayList<AtomicFormula>();

		    Exp ge = problem.getGoal();
        AndExp gae;
        if (ge instanceof AndExp)
		    {
			  gae = (AndExp) ge;	    

		 for(java.util.Iterator<Exp> gi = gae.iterator(); gi.hasNext();)
     {
				Exp gel = gi.next();
				System.out.println(gel.toString() + "\n");
				Literal gl;
				if (gel instanceof Literal)
				{
					gl = (Literal) gel;
					System.out.println("Literal" + gl.toString() + "\n");
					goal.add(gl);
				}
				NotExp ne;
				NotAtomicFormula naf;
				if (gel instanceof NotExp)
				{
					ne = (NotExp) gel;
					System.out.println("NotExp" + ne.getExp().toString() + "\n");
					AtomicFormula af = (AtomicFormula) ne.getExp();

					naf = new NotAtomicFormula(af);
					System.out.println("Literal" + naf.toString() + "\n");
					goal.add(naf);
				}
      }//for
		    
      java.util.List<InitEl> init = problem.getInit();

		  for(java.util.Iterator<InitEl> ii = init.iterator();ii.hasNext();)
		  {
			InitEl ie = ii.next();
			 if (ie instanceof AtomicFormula)
			  {
				 AtomicFormula af = (AtomicFormula) ie;
				 System.out.println("AF" + ie.toString()+"\n");
				 state.add(af);
			  }
      }//for
      check_goal(state,goal);

      java.util.Iterator<ActionDef> ai = domain.actionsIterator();
		    for(; ai.hasNext();)
		    {
          ActionDef aa = ai.next();
			    if (aa instanceof Action)
               {
                Action ca = (Action)aa;
				        System.out.println(ca.toString()+"\n");
               }
		    }//for

		   }//if
       } //else Message
      }catch(Exception e) {}
     }// else length
 }//main
}//class
