(define (domain repair_domain)
   (:requirements :strips :equality :typing :negative-preconditions)
   (:types node hardware)
   (:predicates
					   (bad ?c)
						 (good ?c)
						 (running ?c)
             (not_running ?c)
             (not_matched ?c)
             (matched ?c)
             (ok ?c)
             (not_ok ?c)
						 (on ?o)
             (not_on ?o)
    )

   (:action power_up
    :parameters (?c)
                :precondition (and (not_on ?c)(bad ?c))
                :effect (and (on ?c)(good ?c))
    )

   (:action shutdown
    :parameters (?c)
                :precondition (and (on ?c)(bad ?c))
                :effect (and (bad ?c)(not_on ?c))
    )

   (:action stop_node
    :parameters (?c)
                :precondition (and (bad ?c)(running ?c))
                :effect (and (bad ?c)(not_running ?c))
    )

   (:action start_node
    :parameters (?c)
         				:precondition (and (bad ?c)(not_running ?c))
                :effect (good ?c)
    )

   
)

