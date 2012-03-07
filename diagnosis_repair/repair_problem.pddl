(define (problem repair_problem)(:domain repair_domain)
 (:requirements :strips :typing :equality :negative-preconditions :disjunctive-preconditions)
 (:objects jaguar laser laser_alignment - hardware
           jaguar_base hokuyo_node laser_alignment_node move_base - software)
 (:init 
      (component jaguar)
      (ab jaguar)
      (not (on jaguar))

      (component laser)
      (ab laser)
      (not (on laser))

      (component laser_alignment)
      (ab laser_alignment)
			(not (on laser_alignment))

			(component jaguar_base)
      (ab jaguar_base)
			(not (running jaguar_base))

      (component hokuyo_node)
      (ab hokuyo_node)
      (not (running hokuyo_node))

      (component laser_alignment_node)
      (ab laser_alignment_node)
      (not (running laser_alignment_node))

      (component move_base)
      (ab move_base)
      (not (running move_base))
 )
(:goal (and (not (ab jaguar))(not (ab jaguar_base))(not (ab laser))(not (ab hokuyo_node))(not (ab laser_alignment))(not (ab laser_alignment_node))))
)

