(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions )
(:types
    dummypoint
    mapgrid
    direction
    directionnorth
    directionsouth
    directioneast
    directionwest
)
(:predicates
    (ISADJ ?dir - direction ?x - mapgrid ?y - mapgrid)
    (ISADJnor ?dirn - directionnorth ?x - mapgrid ?y - mapgrid)
    (ISADJsou  ?dirs - directionsouth ?x - mapgrid ?y - mapgrid)
    (ISADJeas ?dire - directioneast ?x - mapgrid ?y - mapgrid)
    (ISADJwes ?dirw - directionwest ?x - mapgrid ?y - mapgrid)
    (invasive-at ?x - mapgrid)
    (plant-at ?x - mapgrid)
    ;(plant-at ?dir - direction ?x - mapgrid )
    ;(endangered-at ?dir - direction ?x - mapgrid )
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
	
    )
)
(:action spray
    :parameters (?to - mapgrid ?north - mapgrid ?south - mapgrid ?east - mapgrid ?west - mapgrid ?dir - directionnorth ?dir1 - directionsouth ?dir2 - directioneast ?dir3 - directionwest)
    :precondition
    (and
        (agent-at ?to)
	    (ISADJnor ?dir ?to ?north)
	    (ISADJsou ?dir1 ?to ?south)
	    (ISADJeas ?dir2 ?to ?east)
	    (ISADJwes ?dir3 ?to ?west)
    )
    :effect
    (and
        (not(plant-at ?to))
        (not(invasive-at ?to))
        (not (invasive-at ?north))
        (not (invasive-at ?south))
        (not (invasive-at ?east))
        (not (invasive-at ?west))
        
	    
    )
)
)

