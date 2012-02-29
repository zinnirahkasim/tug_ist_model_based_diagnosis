(define (domain repair_domain)
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

    )

   (:action start_node
    :parameters (?c)
		:precondition (ab ?c)
		:effect (and  (nab ?c) (not (ab ?c))))

   (:action stop_node
    :parameters (?c - node)
		:precondition (ab ?c)
		:effect (not (ab ?c))) 

   (:action power_up
    :parameters (?c)
		:precondition (and (not_on ?c)(ab ?c))
		:effect (and  (on ?c) (nab ?c))))

   (:action shutdown
    :parameters (?c)
		:precondition (on ?c)
		:effect (not_on ?c))
)
