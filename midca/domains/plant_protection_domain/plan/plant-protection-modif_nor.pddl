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



(:predicates
    (ISADJ ?dir - direction ?x - mapgrid ?y - mapgrid)
    (invasive-at ?x - mapgrid)
    (knows ?hum - human ?g - goal ?x - mapgrid)
    (plant-at ?x - mapgrid)
    (native-at ?x - mapgrid)
    (agent-at ?x - mapgrid)
    (requested ?hum - human ?ag - agent ?g - goal ?x - mapgrid)
)
(:action inform
    :parameters(?hum - human ?ag - agent ?g - goal ?x - mapgrid)
    :precondition
    (and
    )
    :effect
    (and 
       (knows ?hum ?g ?x)
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

           
    )
)
)

