(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions )
(:types
    dummypoint
    mapgrid
)
(:predicates
    ;(ISADJ ?dir - direction ?x - mapgrid ?y - mapgrid)
    ;up down left right of a field
    (ISADJACENT ?x - mapgrid ?y - mapgrid)
    ;invasive-plants at
    (Invasive-at ?x - mapgrid)
    ;endangered plants at
    (Endangered-at ?x - mapgrid)
    ;player at loc
    (agent-at ?x - mapgrid)
)
(:action move
    :parameters (?from - mapgrid ?to - mapgrid)
    :precondition
    (and
	(agent-at ?from)
	(ISADJACENT ?from ?to)
    )
    :effect
    (and
	(agent-at ?to)
	;(not (agent-at ?from))
    )
)

;(:action spray
;    :parameters (?to - mapgrid)
;   :precondition
;    (and
;        (agent-at ?to)
;	(Invasive-at ?to)
;    )
;    :effect
;    (and
;	(not (Invasive-at ?to))
;	;(not (Endangered-at ?to))
;	(forall (?loc - mapgrid)
;		 (when (ISADJACENT ?to ?loc)
;		     (not (Endangered-at ?loc)
;		)     
;	)
;	
 ;   )
;)

(:action spray
    :parameters (?from - mapgrid ?to - mapgrid)
    :precondition
    (and
        (agent-at ?from)
	(ISADJACENT ?from ?to)
	(Invasive-at ?to)
    )
    :effect
    (and
        (agent-at ?to)
	;(not (agent-at ?from))
	(not (Invasive-at ?to))
	(forall (?loc - mapgrid)
		(when (ISADJACENT ?to ?loc)
			  (and (not (Endangered-at ?loc))
		              (not (Invasive-at ?loc)) )
		)
	)
    )
)
)

