(define (domain minecraft-beta)
	(:requirements :typing :fluents :existential-preconditions)
	(:types
		resource - thing
		material - thing
		tool - thing
		craftgrid
		mapgrid
		direction
		player
		monster - thing
		weapon - thing

		potion - thing
		helmet - thing
   	    chestplate - thing
	)



	(:predicates
		(player-at  ?loc - mapgrid)

		(in-shelter)


		(thing-at-map  ?obj - resource  ?loc - mapgrid)
		(thing-at ?obj - resource)
		(known-loc ?obj - resource)
		(thing-at-loc ?obj - resource ?loc - mapgrid)
		(placed-thing-at-map  ?obj - material  ?loc - mapgrid)
		(resource-at-craft  ?res - thing  ?loc - craftgrid)
		(craft-empty  ?loc - craftgrid)
		(connect  ?from - mapgrid  ?to - mapgrid)
        (know-where ?res - resource ?loc - mapgrid)
		(crafting)
		(survive)
        (attacking)
		(looking-for ?res - resource)
		(head-armed)
   	    (chest-armed)
   	    (is-attacked)
   	    (is-trapped)
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
		(player-current-health)

		(current-hunger-value)

	)




	;;-------------------------------------------------


	;; ----------------------------------------------------



	;; ----------------------------------------------------
	(:action move
		:parameters (?from - mapgrid ?to - mapgrid)
		:precondition
			(and
				(player-at ?from)
				(connect ?from ?to)
				(not (crafting))
				(> (player-current-health) 20)

			)
		:effect
			(and
				(player-at ?to)
				(not (player-at ?from))
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)
			)
	)
	;;----------------------------------------
(:action event-skeleton-attacked
      	:parameters (?loc1 - mapgrid ?loc - mapgrid )
      	:precondition
      	( and
            (player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at-loc skeleton ?loc)

      	)
      	:effect
      	(and
   			(decrease (player-current-health) 5)
   			(thing-at-map arrow ?loc1)
   			(is-attacked)
      	)
	)



)
