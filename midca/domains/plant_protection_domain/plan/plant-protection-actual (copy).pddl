(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions )
(:types
    dummypoint
    mapgrid
    direction
    human
    agent
    goal
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
    ;(ISADJnoes ?dir - directionnoes ?x - mapgrid ?y - mapgrid)
    ;(ISADJnows ?dir - directionnows ?x - mapgrid ?y - mapgrid)
    ;(ISADJsoes ?dir - directionsoes ?x - mapgrid ?y - mapgrid)
    ;(ISADJsows ?dir - directionsows ?x - mapgrid ?y - mapgrid)
    ;(ISADJnor ?dir - directionnorth ?x - mapgrid ?y - mapgrid)
    ;(ISADJsou ?dir - directionsouth ?x - mapgrid ?y - mapgrid)
    ;(ISADJeas ?dir - directioneast ?x - mapgrid ?y - mapgrid)
    ;(ISADJwes ?dir - directionwest ?x - mapgrid ?y - mapgrid)
    (invasive-at ?x - mapgrid)
    (plant-at ?x - mapgrid)
    (native-at ?x - mapgrid)
    (agent-at ?x - mapgrid)
    (requested ?hum - human ?ag - agent ?g - goal ?x - mapgrid)
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
    :parameters (?to - mapgrid ?no - mapgrid ?so - mapgrid ?ea - mapgrid ?we - mapgrid ?noes - mapgrid  ?nows - mapgrid ?sows - mapgrid ?soes - mapgrid)
    :precondition
    (and
            (agent-at ?to)
	    (ISADJ north ?to ?no)
	(ISADJ south ?to ?so)
	(ISADJ east ?to ?ea)
	(ISADJ west ?to ?we)
	(ISADJ noes ?to ?noes)
	(ISADJ nows ?to ?nows)
	(ISADJ soes ?to ?soes)
	(ISADJ sows ?to ?sows)
            
    )
    :effect
    (and
        (not (plant-at ?to))
        (not (invasive-at ?to))
        (not (invasive-at ?no))
        (not (invasive-at ?so))
        (not (invasive-at ?ea))
        (not (invasive-at ?we))
        (not (invasive-at ?noes))
        (not (invasive-at ?nows))
        (not (invasive-at ?sows))
        (not (invasive-at ?soes))
        (not (native-at ?to))
        (not (native-at ?no))
        (not (native-at ?so))
        (not (native-at ?ea))
        (not (native-at ?we))
        (not (native-at ?noes))
        (not (native-at ?nows))
        (not (native-at ?sows))
        (not (native-at ?soes))
        
	    
    )
)
)

