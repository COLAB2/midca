(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions :typing :conditional-effects :adl)
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
    (knows ?hum - human)
    (plant-at ?x - mapgrid)
    (native-at ?x - mapgrid)
    (agent-at ?x - mapgrid)
    (requested ?hum - human ?ag - agent ?g - goal ?x - mapgrid)
)
(:action inform
    :parameters(?hum - human ?ag - agent)
    :precondition
    (and
    )
    :effect
    (and 
       (knows ?hum)
    )
)
(:action move
    :parameters (?dir - direction ?from - mapgrid ?to - mapgrid )
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
    :parameters (?to - mapgrid ?so - mapgrid ?no - mapgrid ?ea - mapgrid ?we - mapgrid ?noes - mapgrid ?nows - mapgrid ?soes - mapgrid ?sows - mapgrid)
    :precondition
    (and
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ south ?to ?so)
            (ISADJ north ?to ?no)
            (ISADJ east ?to ?ea)
            (ISADJ west ?to ?we) 
            (ISADJ noes ?to ?noes)
            (ISADJ nows ?to ?nows)
            (ISADJ soes ?to ?soes)
            (ISADJ sows ?to ?sows)
                    
    )
    :effect (and
        (not (invasive-at ?to))
        (not (native-at ?to))
        (not (invasive-at ?so))
        (not (invasive-at ?no))
        (not (invasive-at ?ea))
        (not (invasive-at ?we))
        (not (invasive-at ?soes))
        (not (invasive-at ?sows))
        (not (invasive-at ?noes))
        (not (invasive-at ?nows))
        (not (native-at ?so))
        (not (native-at ?we))
        (not (native-at ?no))
        (not (native-at ?ea))
        (not (native-at ?soes))
        (not (native-at ?sows))
        (not (native-at ?noes))
        (not (native-at ?nows))
        

           
    )
)
)

