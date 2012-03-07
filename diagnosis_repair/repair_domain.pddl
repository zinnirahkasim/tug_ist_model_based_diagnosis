(define (domain repair_domain)
   (:requirements :strips :typing :equality :negative-preconditions :disjunctive-preconditions)
   (:types software hardware)
	 (:constants JB HN LAN - software 
               J L LA - hardware)
   (:predicates
             (component ?c)
             (ab ?c)
             (ok ?c)
             (on ?c - hardware)
             (running ?c - software)
    )

	(:action shutdown
    :parameters (?x - hardware)
		:precondition (on ?x)
		:effect (not (on ?x)))

  (:action power_up
    :parameters (?x - hardware)
		:precondition (not (on ?x))
		:effect (and  (on ?x) (not (ab ?x))))
   
   (:action start_node
    :parameters (?x - software)
		:precondition (and (not (running ?x))
                       (or (and (= ?x JB)(on J)) (not (= ?x JB))) 
                       (or (and (= ?x HN)(on L))  (not (= ?x HN)))
                       (or (and (= ?x LAN)(on LA)) (not (= ?x LAN))) )
		:effect (not (ab ?x)))

   (:action stop_node
    :parameters (?x - software)
		:precondition (running ?x)
		:effect (not (running ?x)))
)

