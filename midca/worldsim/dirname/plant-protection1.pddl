(define (domain plant-protection)
(:requirements :strips :negative-preconditions)
(:types
    dummypoint
)
(:predicates
    ;up down left right of a field
    (ISADJACENT ?x ?y)
    ;invasive-plants at
	(Invasive-at ?x)
	;endangered plants at
	(Endangered-at ?x)
	;player at loc
	(agent-at ?x)
)
(:action move
    :parameters (?from ?to)
    :precondition
    (and
		;(not (blocked ?to))
		(agent-at ?from)
		(ISADJACENT ?from ?to)
    )
    :effect
    (and
		(agent-at ?to)
		(not (agent-at ?from))
    )
)

(:action spray
    :parameters (?from ?to)
    :precondition
    (and
        (agent-at ?from)
		(ISADJACENT ?from ?to)
		;(= ?low (- ?from 15))
		(Endangered-at ?to)
    )
    :effect
    (and
        (agent-at ?to)
		(not (agent-at ?from))
		(not (Endangered-at ?to))
    )
)
)

