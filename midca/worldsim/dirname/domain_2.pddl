
(define (domain minecraft-beta)
	(:requirements :strips )
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
                (trap-destroyed)
                (searched-left ?obj - resource)
        	(searched-right ?obj - resource)
        	(searched-behind ?obj - resource)
        	(searched-forward ?obj - resource)
        	(looking-right)
        	(looking-left)
        	(looking-forward)
        	(looking-behind)
		(thing-at-map  ?obj - resource  ?loc - mapgrid)
		(thing-at ?obj - resource)
		(known-loc ?obj - resource)
		(thing-at-loc ?obj - resource ?loc - mapgrid)
		(placed-thing-at-map  ?obj - material  ?loc - mapgrid)
		(resource-at-craft  ?res - thing  ?loc - craftgrid)
		(craft-empty  ?loc - craftgrid)
		(connect  ?from - mapgrid  ?to - mapgrid)
		(connect-left  ?from - mapgrid  ?to - mapgrid)
		(connect-right  ?from - mapgrid  ?to - mapgrid)
		(connect-behind  ?from - mapgrid  ?to - mapgrid)
		(connect-forward  ?from - mapgrid  ?to - mapgrid)
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
;;--------------------------------------------------------
	;;------------EVENTS--------------------------------------
	;;--------------------------------------------------------
(:action event-find
	:parameters (?res - resource ?loc - mapgrid ?player_loc - mapgrid)
	:precondition
	(and
		(looking-for ?res)
		(connect ?loc ?player_loc)
	        (thing-at-loc ?res ?loc)
	)
	:effect
	(and
		(know-where ?res ?loc)
		(thing-at-map ?res ?loc)
                (not (looking-for ?res))
	)
)
	)
