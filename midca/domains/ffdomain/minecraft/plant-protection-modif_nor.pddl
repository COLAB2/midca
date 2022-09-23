(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions :typing :conditional-effects)
(:types
    dummypoint
    mapgrid
    direction
    ;directionnorth
    ;directionsouth
    ;directioneast
    ;directionwest
    ;directionnoes
    ;directionnows
    ;directionsoes
    ;directionsows
    human
    agent
    goal
)
;(:constants
;    east - direction
;    west - direction
;    south - direction
;    north - direction
;    noes - direction
;    nows - direction
;    soes - direction
;    sows - direction
;)


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
    :parameters (?to - mapgrid ?no - mapgrid ?so - mapgrid ?ea - mapgrid ?we - mapgrid ?noes - mapgrid  ?nows - mapgrid ?sows - mapgrid ?soes - mapgrid)
    :precondition
    (and
            ;(invasive-at ?to)
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
        (not (invasive-at ?to))
        (agent-at ?to)

        
	    
    )
)

(:action spray2
    :parameters (?to - mapgrid ?so - mapgrid ?ea - mapgrid ?no - mapgrid ?noes - mapgrid ?soes - mapgrid )
    :precondition
    (and  
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ north ?to ?no)
	    (ISADJ south ?to ?so)
	    (ISADJ east ?to ?ea)
            (ISADJ noes ?to ?noes)
	    (ISADJ soes ?to ?soes)
            
    )
    :effect
    (and
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))
        
        
	    
    )
)

(:action spray3
    :parameters (?to - mapgrid ?no - mapgrid ?ea - mapgrid ?we - mapgrid ?noes - mapgrid  ?nows - mapgrid )
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ north ?to ?no)
	    
	    (ISADJ east ?to ?ea)
	    (ISADJ west ?to ?we)
            (ISADJ noes ?to ?noes)
	    (ISADJ nows ?to ?nows)
            
    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))
        
        
        
	    
    )
)

(:action spray4
    :parameters (?to - mapgrid ?no - mapgrid ?so - mapgrid  ?we - mapgrid ?nows - mapgrid ?sows - mapgrid )
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ north ?to ?no)
	    (ISADJ south ?to ?so)

	    (ISADJ west ?to ?we)

	    (ISADJ nows ?to ?nows)
	    (ISADJ sows ?to ?sows)


    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))

        (not (native-at ?to))




    )
)

(:action spray5
    :parameters (?to - mapgrid ?so - mapgrid ?ea - mapgrid ?soes - mapgrid )
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)

	    (ISADJ south ?to ?so)
	    (ISADJ east ?to ?ea)

	    (ISADJ soes ?to ?soes)

    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))


        (not (native-at ?to))




    )
)

(:action spray6
    :parameters (?to - mapgrid ?no - mapgrid ?ea - mapgrid ?noes - mapgrid)
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ north ?to ?no)

	        (ISADJ east ?to ?ea)

            (ISADJ noes ?to ?noes)


    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))


        (not (native-at ?to))




    )
)

(:action spray7
    :parameters (?to - mapgrid ?no - mapgrid ?we - mapgrid  ?nows - mapgrid )
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)
            (ISADJ north ?to ?no)

	    (ISADJ west ?to ?we)
	    (ISADJ nows ?to ?nows)


    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))


        (not (native-at ?to))




    )
)

(:action spray8
    :parameters (?to - mapgrid ?so - mapgrid ?we - mapgrid ?sows - mapgrid )
    :precondition
    (and
        
            ;(invasive-at ?to)
            (agent-at ?to)

	    (ISADJ south ?to ?so)

	    (ISADJ west ?to ?we)


	    (ISADJ sows ?to ?sows)


    )
    :effect
    (and
        
        (agent-at ?to)
        (not (plant-at ?to))
        (not (invasive-at ?to))


        (not (native-at ?to))





    )
)




)
