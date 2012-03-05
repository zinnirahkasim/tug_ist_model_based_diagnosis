(define (domain test_repair_domain)
   (:requirements :strips :typing :negative-preconditions)
   (:types node hardware)
   (:predicates
             (component ?c)
             (nab ?c)
             (ab ?c)
             (ok ?o)
             (not_ok ?o)
             (on ?o)
             (not_on ?o)
						 (running ?c)
						 (not_running ?c)
    )

   (:action power_up
    :parameters (?c)
                :precondition (and (not_on ?c)(ab ?c))
                :effect (and  (on ?c) (nab ?c)))

   (:action shutdown
    :parameters (?c)
                :precondition (and (on ?c)(ab ?c))
                :effect (not_on ?c))

   (:action start_node
    :parameters (?c)
                :precondition (and (not_running ?c)(ab ?c))
                :effect (and (running ?c)(nab ?c)))

   (:action stop_node
    :parameters (?c - node)
                :precondition (and (running ?c)(ab ?c))
                :effect (not_running ?c))
)
