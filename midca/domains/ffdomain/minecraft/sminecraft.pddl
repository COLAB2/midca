(define (domain minecraft-beta)
    (:requirements :typing :fluents :existential-preconditions)
    (:types
   	 resource - thing
   	 mapgrid
   	 player
    )

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
   	 food - thing
   	 potion - thing
    )

    (:predicates
   	 (is-trap ?loc - mapgrid)
   	 (looking-for ?res - resource)
   	 (know-where ?res - resource ?loc - mapgrid)
   	 (player-at  ?loc - mapgrid)
   	 (in-shelter)
   	 (look-at ?direction - direction)
   	 (monster-at ?zombie - monster ?loc - mapgrid)
   	 (thing-at-map  ?obj - resource  ?loc - mapgrid)
   	 (thing-at ?obj - resource ?loc - mapgrid)
   	 (known-loc ?obj - resource)
   	 (placed-thing-at-map  ?obj - material  ?loc - mapgrid)
   	 (resource-at-craft  ?res - thing  ?loc - craftgrid)
   	 (craft-empty  ?loc - craftgrid)
   	 (connect  ?from - mapgrid  ?to - mapgrid)
   	 (next-to ?from - direction ?to - direction)
   	 (crafting)
   	 (survive)
   	 (eat ?food - food)
   	 (is-attacked)
   	 (is-trapped)
    )
    (:hidden is-trap thing-at is-attacked is-trapped)

    (:functions
   	 (self) - player
   	 (thing-available  ?obj - thing) - int
   	 (current-harvest-duration) - int
   	 (current-harvest-location) - int
   	 (duration-need  ?tool - tool  ?res - resource) - int
   	 (location-id  ?loc - mapgrid) - int
   	 (tool-id  ?tool - tool) - int
   	 (tool-in-hand) - int
   	 (tool-max-health ?tool - tool) - int
   	 (tool-current-health ?tool - tool) - int
   	 (player-current-health) - int
   	 (furnace-fuel) - int
   	 (current-hunger-value) - int
   	 (food-value ?f -food) - int
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
    ;;-----------------------------------------------------

    (:event fall-in-trap
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            	(player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (is-trap ?loc)
   			 (not (is-trapped))
   			 (neq (player-current-health) 0)
      	)
      	:effect
      	(and
   			(decrease (player-current-health) 5)
   			(thing-at-map arrow ?loc1)
   			(is-trapped)
      	)
	)
    ;;------------------------------------------------------

    (:event skeleton-attacked
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            	(player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at skeleton ?loc)
   			 (not (is-attacked))
   			 (neq (player-current-health) 0)
      	)
      	:effect
      	(and
   			(decrease (player-current-health) 5)
   			(thing-at-map arrow ?loc1)
   			(is-attacked)
      	)
	)
    ;;----------------------------------------------
    (:event monster-explosion
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            	(player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at monster ?loc)
   			 (not (is-attacked))
   			 (neq (player-current-health) 0)
      	)
      	:effect
      	(and
   			(decrease (player-current-health) 5)
   			(thing-at-map monster-remains ?loc1)
   			(is-attacked)
      	)
	)
    ;;----------------------------------------------




)



