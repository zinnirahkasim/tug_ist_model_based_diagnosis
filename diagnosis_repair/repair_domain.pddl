(define (domain repair_domain)
   (:requirements :strips)
   (:predicates
             (component ?c)
             (topic ?t)
             (nab ?c)
             (ab ?c)
             (ok ?t)
             (not_ok ?t)
    )

   (:action restart
    :parameters (?c)
		:precondition (ab ?c)
		:effect (and  (nab ?c) (not (ab ?c))))		
)
