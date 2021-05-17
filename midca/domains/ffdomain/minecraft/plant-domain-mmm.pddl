(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions :typing)
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
    (native-at ?x - mapgrid)
    (plant-at ?x - mapgrid)
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
	    (ISADJ sows ?to ?sows)
	    (ISADJ soes ?to ?soes)
            
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

