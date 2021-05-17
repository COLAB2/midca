(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions :typing :conditional-effects)
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
    :parameters (?to - mapgrid )
    :precondition
    (and
            ;(invasive-at ?to)
            (agent-at ?to)
            

            
    )
    :effect
    (and
        (agent-at ?to)
        (when (agent-at ?to) (and (not (native-at ?to))(not (invasive-at ?to)) ))
        (when (ISADJ south ?to ?so) (and (not (native-at ?so))(not (invasive-at ?so)) ))
        (when (ISADJ north ?to ?no) (and (not (native-at ?no))(not (invasive-at ?no)) ))
        (when (ISADJ east ?to ?ea) (and (not (native-at ?ea))(not (invasive-at ?ea)) ))
        (when (ISADJ west ?to ?we) (and (not (native-at ?we))(not (invasive-at ?we)) ))
        (when (ISADJ noes ?to ?noes) (and (not (native-at ?noes))(not (invasive-at ?noes)) ))
        (when (ISADJ nows ?to ?nows) (and (not (native-at ?nows))(not (invasive-at ?nows)) ))
        (when (ISADJ soes ?to ?soes) (and (not (native-at ?soes))(not (invasive-at ?soes)) ))
        (when (ISADJ soes ?to ?sows) (and (not (native-at ?sows))(not (invasive-at ?sows)) ))
        
	    
    )
)

(:action spray1
    :parameters (?to - mapgrid ?so - mapgrid ?ea - mapgrid ?we - mapgrid ?sows - mapgrid ?soes - mapgrid )
    :precondition
    (and  
            ;(invasive-at ?to)
            (agent-at ?to)
	    (ISADJ south ?to ?so)
	    (ISADJ east ?to ?ea)
	    (ISADJ west ?to ?we)
	    (ISADJ sows ?to ?sows)
	    (ISADJ soes ?to ?soes)
            
    )
    :effect
    (and
        (agent-at ?to)
        ;(not (plant-at ?to))
        (not (invasive-at ?to))
        (not (invasive-at ?so))
        (not (invasive-at ?ea))
        (not (invasive-at ?we))
        (not (invasive-at ?sows))
        (not (invasive-at ?soes))
        (not (native-at ?to))
        (not (native-at ?so))
        (not (native-at ?ea))
        (not (native-at ?we))
        (not (native-at ?sows))
        (not (native-at ?soes))
        
	    
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
        ;(not (plant-at ?to))
        (not (invasive-at ?to))
        (not (invasive-at ?no))
        (not (invasive-at ?so))
        (not (invasive-at ?ea))
       
        (not (invasive-at ?noes))
        
        (not (invasive-at ?soes))
        (not (native-at ?to))
        (not (native-at ?no))
        (not (native-at ?so))
        (not (native-at ?ea))
        
        (not (native-at ?noes))
        
        (not (native-at ?soes))
        
	    
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
        ;(not (plant-at ?to))
        (not (invasive-at ?to))
        (not (invasive-at ?no))
        
        (not (invasive-at ?ea))
        (not (invasive-at ?we))
        (not (invasive-at ?noes))
        (not (invasive-at ?nows))
        
        (not (native-at ?to))
        (not (native-at ?no))
        
        (not (native-at ?ea))
        (not (native-at ?we))
        (not (native-at ?noes))
        (not (native-at ?nows))
        
        
	    
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
        (not (invasive-at ?to))
        (not (invasive-at ?no))
        (not (invasive-at ?so))
        (not (invasive-at ?we))
        (not (invasive-at ?nows))
        (not (invasive-at ?sows))
        (not (native-at ?to))
        (not (native-at ?no))
        (not (native-at ?so))
        (not (native-at ?we))
        (not (native-at ?nows))
        (not (native-at ?sows))
        
        
	    
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
        ;(not (plant-at ?to))
        (not (invasive-at ?to))

        (not (invasive-at ?so))
        (not (invasive-at ?ea))

        (not (invasive-at ?soes))
        (not (native-at ?to))

        (not (native-at ?so))
        (not (native-at ?ea))

        (not (native-at ?soes))


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
        ;(not (plant-at ?to))
        (not (invasive-at ?to))
        (not (invasive-at ?no))

        (not (invasive-at ?ea))

        (not (invasive-at ?noes))

        (not (native-at ?to))
        (not (native-at ?no))

        (not (native-at ?ea))

        (not (native-at ?noes))



    )
)

(:action spray7
    :parameters (?to - mapgrid ?no - mapgrid ?we - mapgrid  ?nows - mapgrid )
    :precondition
    (and
            
            (agent-at ?to)
            (ISADJ north ?to ?no)

	    (ISADJ west ?to ?we)
	    (ISADJ nows ?to ?nows)


    )
    :effect
    (and
        (agent-at ?to)
        
        (not (invasive-at ?to))
        (not (invasive-at ?no))

        (not (invasive-at ?we))

        (not (invasive-at ?nows))

        (not (native-at ?to))
        (not (native-at ?no))

        (not (native-at ?we))

        (not (native-at ?nows))



    )
)

(:action spray8
    :parameters (?to - mapgrid ?so - mapgrid ?we - mapgrid ?sows - mapgrid )
    :precondition
    (and
            
            (agent-at ?to)

	    (ISADJ south ?to ?so)

	    (ISADJ west ?to ?we)


	    (ISADJ sows ?to ?sows)


    )
    :effect
    (and
        (agent-at ?to)
        
        (not (invasive-at ?to))

        (not (invasive-at ?so))

        (not (invasive-at ?we))

        (not (invasive-at ?sows))

        (not (native-at ?to))

        (not (native-at ?so))

        (not (native-at ?we))

        (not (native-at ?sows))



    )
)





)

