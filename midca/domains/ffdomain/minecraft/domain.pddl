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
		(crafting)
	)
	
	(:functions
		(thing-available  ?obj - thing)
		(current-harvest-duration)
		(current-harvest-location)
		(duration-need  ?tool - tool  ?res - resource)
		(location-id  ?loc - mapgrid)
		(tool-id  ?tool - tool)
		(tool-in-hand)
		(tool-max-health ?tool - tool)
		(tool-current-health ?tool - tool)
		(furnace-fuel)
	)

	;; ----------------------------------------------------
	(:action place-on-map
		:parameters (?res - material  ?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(not (placed-thing-at-map ?res ?target))
				(> (thing-available ?res) 0)
				(not (crafting))
			)
		:effect
			(and
				(placed-thing-at-map ?res ?target)
				(decrease (thing-available ?res) 1)
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action move
		:parameters (?from - mapgrid ?to - mapgrid)
		:precondition 
			(and 
				(player-at ?from) 
				(connect ?from ?to)
				(not (crafting))
			)
		:effect
			(and
				(player-at ?to)
				(not (player-at ?from))
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)
			)
	)

	;; ---------------------------------------------------
	(:action change-harvest-loc
		:parameters (?target - mapgrid)
		:precondition
			(and
				(not (= (current-harvest-location) (location-id ?target)))
				(player-at ?target)
				(not (crafting))
			)
		:effect
			(and
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) (location-id ?target))
			)
	)

	;; ----------------------------------------------------
	(:action change-harvest-tool
		:parameters (?tool - tool)
		:precondition
			(and
				(not (= (tool-in-hand) 1))
				(> (thing-available ?tool) 0)
				(not (crafting))
			)
		:effect
			(and 
				(assign (current-harvest-duration) 0)
				(assign (tool-in-hand) (tool-id ?tool))
			)
	)

	;; ----------------------------------------------------
	(:action harvest
		:parameters (?target - mapgrid ?tool - tool ?obj - resource)
		:precondition
			(and
				(player-at ?target)
				(> (thing-available ?tool) 0)
				(> (tool-current-health ?tool) 0)
				(thing-at-map ?obj ?target)
				(= (current-harvest-location) (location-id ?target))
				(= (tool-in-hand) (tool-id ?tool))
				(< (current-harvest-duration) (duration-need ?tool ?obj))
				(not (crafting))
			)
		:effect
			(and
				(increase (current-harvest-duration) 1)
				(decrease (tool-current-health ?tool) 1)
			)
	)

	;; ----------------------------------------------------
	
	;; ----------------------------------------------------
	(:action get-harvest-wood
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map tree ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool tree))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available wood) 1)
				(not (thing-at-map tree ?target))
				(assign (current-harvest-duration) 0)
			)
	)



)
