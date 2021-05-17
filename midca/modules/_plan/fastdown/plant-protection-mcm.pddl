(define 
(domain minecraft-beta)
(:requirements :strips :negative-preconditions)
(:types
    dummypoint
    mapgrid
    direction
    directionnorth
    directionsouth
    directioneast
    directionwest
)
(:constants
    east - direction
    west - direction
    south - direction
    north - direction
    noes - direction
    nows - direction
    soes - direction
    sows - direction
)

(:predicates
    (ISADJ ?dir - direction ?x - mapgrid ?y - mapgrid)
    (invasive-at ?x - mapgrid)
    (plant-at ?x - mapgrid)
    ;(plant-at ?dir - direction ?x - mapgrid )
    ;(endangered-at ?dir - direction ?x - mapgrid )
    (agent-at ?x - mapgrid)
)
(:action move
    ;:parameters (?dir - direction ?from - mapgrid ?to - mapgrid)
    :parameters (?dir - direction)
    :precondition
    (and
	(agent-at ?from)
	(ISADJ ?dir ?from ?to)
    )
    :effect
    (and
	(agent-at ?to)
	
    )
)
(:action spray
    :parameters (?to ?north ?south ?east ?west ?noes ?nows ?soes ?sows)
    ;:parameters ()
    :precondition
    (and
        (agent-at ?to)
	(ISADJ north ?to ?north)
	(ISADJ south ?to ?south)
	(ISADJ east ?to ?east)
	(ISADJ west ?to ?west)
	(ISADJ noes ?to ?noes)
	(ISADJ nows ?to ?nows)
	(ISADJ soes ?to ?soes)
	(ISADJ sows ?to ?sows)
    )
    :effect
    (and
        (not (invasive-at ?to))
        (not (invasive-at ?north))
        (not (invasive-at ?south))
        (not (invasive-at ?east))
        (not (invasive-at ?west))
        (not (invasive-at ?nows))
        (not (invasive-at ?noes))
        (not (invasive-at ?sows))
        (not (invasive-at ?soes))
    )
)
)

