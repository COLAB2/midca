(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions )
(:types
    dummypoint
    mapgrid
    direction
)
(:predicates
    (ISADJ ?dir - direction ?x - mapgrid ?y - mapgrid)
    (invasive-at ?x - mapgrid)
    (invasive-at ?dir - direction ?x - mapgrid )
    (endangered-at ?dir - direction ?x - mapgrid )
    (endangered-at ?x - mapgrid)
    (agent-at ?x - mapgrid)
)
(:action move
    :parameters (?dir - direction ?from - mapgrid ?to - mapgrid)
    :precondition
    (and
	(agent-at ?from)
	(ISADJ ?dir ?from ?to)
    )
    :effect
    (and
	(agent-at ?to)
	(not (agent-at ?from))
    )
)

(:action spray
    :parameters (?dir - direction ?from - mapgrid ?to - mapgrid)
    :precondition
    (and
        (agent-at ?from)
	(ISADJ ?dir ?from ?to)
	(invasive-at ?to)
    )
    :effect
    (and
        (agent-at ?to)
	(not (invasive-at ?to))
        (not (invasive-at north ?to))
        (not (invasive-at south ?to))
        (not (invasive-at east ?to))
        (not (invasive-at west ?to))
        (not (invasive-at noes ?to))
        (not (invasive-at nows ?to))
        (not (invasive-at soes ?to))
        (not (invasive-at sows ?to))
        (not (endangered-at north ?to))
        (not (endangered-at south ?to))
        (not (endangered-at east ?to))
        (not (endangered-at west ?to))
        (not (endangered-at noes ?to))
        (not (endangered-at nows ?to))
        (not (endangered-at soes ?to))
        (not (endangered-at sows ?to))
	

    )
)
)

