(define (domain minecraft-beta)
	(:requirements :typing :fluents :existential-preconditions)
	(:types
		resource - thing
		material - thing
		tool - thing
		craftgrid
		mapgrid
	)

	(:predicates
		(player-at  ?loc - mapgrid)
		(thing-at-map  ?obj - resource  ?loc - mapgrid)
		(placed-thing-at-map  ?obj - material  ?loc - mapgrid)
		(resource-at-craft  ?res - thing  ?loc - craftgrid)
		(craft-empty  ?loc - craftgrid)
		(connect  ?from - mapgrid  ?to - mapgrid)
		(thing-available  ?obj - thing)
		(tool-in-hand ?tool - tool)
		
	)
	
	
	
	;; ----------------------------------------------------
	(:action move
		:parameters (?from - mapgrid ?to - mapgrid)
		:precondition 
			(and 
				(player-at ?from) 
				(connect ?from ?to)
				
			)
		:effect
			(and
				(player-at ?to)
				(not (player-at ?from))
				
			)
	)

	;; ---------------------------------------------------
	
	
	
	
	(:action get-harvest-wood
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map tree ?target)
				(tool-in-hand ?tool)
				
			)
		:effect
			(and
				(thing-available wood)
				(not (thing-at-map tree ?target))
				
			)
	)


	
)
