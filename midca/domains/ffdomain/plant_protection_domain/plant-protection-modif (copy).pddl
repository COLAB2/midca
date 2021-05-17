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
    :parameters (?to - mapgrid ?no - mapgrid ?so - mapgrid ?ea - mapgrid ?we - mapgrid ?noes - mapgrid  ?nows - mapgrid ?sows - mapgrid ?soes - mapgrid )
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
        (not (invasive-at ?to))
        (not (native-at ?to))
        
	    
    )
)
)

