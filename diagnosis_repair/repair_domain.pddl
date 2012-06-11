(define (domain repair_domain)
   (:requirements :strips :typing :negative-preconditions)
   (:types node hardware)
   (:predicates
						 (component ?c)
             (bad ?c)
						 (good ?c)
						 (running ?c)
             (not_running ?c)
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

     (:action start_node
    :parameters (?c)
         				:precondition (and (bad ?c)(not_running ?c))
                :effect (good ?c)
    )

   (:action stop_node
    :parameters (?c)
                :precondition (and (bad ?c)(running ?c))
                :effect (and (bad ?c)(not_running ?c))
    )
)
