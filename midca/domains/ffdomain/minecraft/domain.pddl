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
				(not (= (tool-in-hand) (tool-id ?tool)))
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
	(:action harvest-loose-tool
		:parameters (?tool - tool)
		:precondition
			(and
				(> (thing-available ?tool) 0)
				(= (tool-in-hand) (tool-id ?tool))
				(= (tool-current-health ?tool) 0)
				(not (crafting))
			)
		:effect
			(and
				(decrease (thing-available ?tool) 1)
				(assign (tool-current-health ?tool) (tool-max-health ?tool))
			)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Harvesting
	;;  tree -> wood
	;;	rock -> stone
	;;	coalore -> coal
	;;	ironore -> ironore
	;;	tallgrass -> seeds
	;;	wheatgrass -> wheat
	;;	sandrock -> sandstone
	;;	soil -> sand
	;;	claysoil -> clay
	;;	brown-mushroom -> brown-mushroom
	;;	red-mushroom -> red-mushroom
	;;	skeleton -> bone
	;;	sugarcane -> sugar
	;;	cobweb : shears -> 1 string
	;;	chicken : hand -> egg
	;;	water : fishingrod -> fish
	;;	sheep : shears -> 4 wool
	;;	cow : bucket -> milk

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

	;; ----------------------------------------------------
	(:action get-harvest-stone
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map rock ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool rock))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available stone) 1)
				(not (thing-at-map rock ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-coal
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map coalore ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool coalore))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available coal) 1)
				(not (thing-at-map coalore ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-ironore
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map ironore-rock ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool ironore-rock))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available ironore) 1)
				(not (thing-at-map ironore-rock ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-seeds
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map tallgrass ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool tallgrass))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available seeds) 1)
				(not (thing-at-map tallgrass ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-wheat
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map wheatgrass ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool wheatgrass))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available wheat) 1)
				(not (thing-at-map wheatgrass ?target))
				(thing-at-map grass ?target)
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-sandstone
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map sandrock ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool sandrock))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available sandstone) 1)
				(not (thing-at-map sandrock ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-sand
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map soil ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool soil))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available sand) 1)
				(not (thing-at-map soil ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-clay
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map claysoil ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool claysoil))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available clay) 1)
				(not (thing-at-map claysoil ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-bone
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map skeleton ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool skeleton))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available bone) 1)
				(not (thing-at-map skeleton ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-brown-mushroom
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map brown-mushroom ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool brown-mushroom))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available brown-mushroom) 1)
				(not (thing-at-map brown-mushroom ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-red-mushroom
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map red-mushroom ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool red-mushroom))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available red-mushroom) 1)
				(not (thing-at-map red-mushroom ?target))
				(assign (current-harvest-duration) 0)
			)
	)


	;; ----------------------------------------------------
	(:action get-harvest-sugarcane
		:parameters (?target - mapgrid ?tool - tool)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map sugarcane ?target)
				(= (tool-in-hand) (tool-id ?tool))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need ?tool sugarcane))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available cut-sugarcane) 1)
				(not (thing-at-map sugarcane ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-string
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map cobweb ?target)
				(= (tool-in-hand) (tool-id shears))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need shears cobweb))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available string) 2)
				(not (thing-at-map cobweb ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-egg
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map chicken ?target)
				(= (tool-in-hand) (tool-id hand))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need hand chicken))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available egg) 2)
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-fish
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map water ?target)
				(= (tool-in-hand) (tool-id fishingrod))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need fishingrod water))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available fish) 1)
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-wool
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map sheep ?target)
				(= (tool-in-hand) (tool-id shears))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need shears sheep))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available wool) 4)
				; (not (thing-at-map sheep ?target))
				(assign (current-harvest-duration) 0)
			)
	)

	;; ----------------------------------------------------
	(:action get-harvest-milk
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map cow ?target)
				(= (tool-in-hand) (tool-id bucket))
				(= (current-harvest-location) (location-id ?target))
				(= (current-harvest-duration) (duration-need bucket cow))
				(not (crafting))
			)
		:effect
			(and
				(increase (thing-available milk) 1)
				(assign (current-harvest-duration) 0)
			)
	)

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Miscellaneous
	;;

	;; ----------------------------------------------------
	(:action grow-tallgrass
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map grass ?target)
				(> (thing-available bonemeal) 0)
			)
		:effect
			(and
				(decrease (thing-available bonemeal) 1)
				(not (thing-at-map grass ?target))
				(thing-at-map tallgrass ?target)
			)
	)

	;; ----------------------------------------------------
	 (:action make-farmland
	 	:parameters (?target - mapgrid  ?tool - tool)
	 	:precondition
	 		(and
	 			(player-at ?target)
	 			(thing-at-map grass ?target)
	 			(> (thing-available ?tool) 0)
	 			(= (tool-id ?tool) (tool-id wood-hoe))
			)
	 	:effect
	 		(and
	 			(not (thing-at-map grass ?target))
	 			(thing-at-map farmland ?target)
	 			(decrease (tool-current-health ?tool) 1)
	 		)
	 )

	;; ----------------------------------------------------
	(:action grow-wheatgrass
		:parameters (?target - mapgrid)
		:precondition
			(and
				(player-at ?target)
				(thing-at-map farmland ?target)
				(> (thing-available seeds) 0)
			)
		:effect
			(and
				(decrease (thing-available seeds) 1)
				(not (thing-at-map farmland ?target))
				(thing-at-map wheatgrass ?target)
			)
	)


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Crafting 
	;;

	;; ----------------------------------------------------
	(:action place
		:parameters (?res - material ?loc - craftgrid)
		:precondition
			(and
				(craft-empty ?loc)
				(> (thing-available ?res) 0)
			)
		:effect
			(and
				(crafting) 
				(not (craft-empty ?loc)) 
				(resource-at-craft ?res ?loc) 
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)
				(decrease (thing-available ?res) 1)
			)
	)


	;; ----------------------------------------------------
	(:action craft-wood-plank
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft wood g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0) 	
				(craft-empty g22)
				(not (resource-at-craft wood g22))
				(increase (thing-available plank) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-wood-stick
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft plank g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g22)
				(not (resource-at-craft plank g22))
				(increase (thing-available stick) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-torch
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft stick g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft coal g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(craft-empty g22)
				(not (resource-at-craft stick g12))
				(not (resource-at-craft coal g22))
				(increase (thing-available torch) 4)
				(not (crafting))
			)
	)
	
	;; ----------------------------------------------------
	(:action craft-furnace
		:parameters ()
		:precondition
			(and
				(resource-at-craft stone g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(not (resource-at-craft stone g11))
				(craft-empty g11)
				(increase (thing-available furnace) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-wood-pickaxe
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft stick g12)
				(craft-empty g13)	
				(craft-empty g21)
				(resource-at-craft stick g22)
				(craft-empty g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(resource-at-craft plank g33)
			)
		:effect	
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(not (resource-at-craft stick g12))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(not (resource-at-craft plank g33))
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(increase (thing-available wood-pickaxe) 1)
				(not (crafting))
			)
	)


	;; ---------------------------------------------------
	(:action craft-wood-axe
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft stick g12)
				(craft-empty g13)	
				(resource-at-craft plank g21)
				(resource-at-craft stick g22)
				(craft-empty g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(craft-empty g33)
			)
		:effect
			(and 
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)	
				(not (resource-at-craft stick g12))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft plank g21))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g21)
				(craft-empty g31)
				(craft-empty g32)
				(increase (thing-available wood-axe) 1)
				(not (crafting))
			)
	)

	;; ---------------------------------------------------
	(:action craft-wood-shovel
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft stick g12)
				(craft-empty g13)	
				(craft-empty g21)
				(resource-at-craft stick g22)
				(craft-empty g23)
				(craft-empty g31)
				(resource-at-craft plank g32)
				(craft-empty g33)
			)
		:effect
			(and 
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)	
				(not (resource-at-craft stick g12))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft plank g32))
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g32)
				(increase (thing-available wood-shovel) 1)
				(not (crafting))
			)
	)

	;; ---------------------------------------------------
	(:action craft-wood-hoe
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft stick g12)
				(craft-empty g13)	
				(craft-empty g21)
				(resource-at-craft stick g22)
				(craft-empty g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(craft-empty g33)
			)
		:effect
			(and 
				(assign (current-harvest-duration) 0)
				(assign (current-harvest-location) 0)	
				(not (resource-at-craft stick g12))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(increase (thing-available wood-hoe) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-sandstone
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft sand g21)
				(resource-at-craft sand g22)
				(craft-empty g23)
				(resource-at-craft sand g31)
				(resource-at-craft sand g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft sand g21))
				(not (resource-at-craft sand g22))
				(not (resource-at-craft sand g31))
				(not (resource-at-craft sand g32))
				(increase (thing-available sandstone) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-clayblock
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft clay g21)
				(resource-at-craft clay g22)
				(craft-empty g23)
				(resource-at-craft clay g31)
				(resource-at-craft clay g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft clay g21))
				(not (resource-at-craft clay g22))
				(not (resource-at-craft clay g31))
				(not (resource-at-craft clay g32))
				(increase (thing-available clayblock) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-brick
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft claybrick g21)
				(resource-at-craft claybrick g22)
				(craft-empty g23)
				(resource-at-craft claybrick g31)
				(resource-at-craft claybrick g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft claybrick g21))
				(not (resource-at-craft claybrick g22))
				(not (resource-at-craft claybrick g31))
				(not (resource-at-craft claybrick g32))
				(increase (thing-available brick) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-sugar
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft cut-sugarcane g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g22)
				(not (resource-at-craft cut-sugarcane g22))
				(increase (thing-available sugar) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-paper
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(resource-at-craft cut-sugarcane g31)
				(resource-at-craft cut-sugarcane g32)
				(resource-at-craft cut-sugarcane g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft cut-sugarcane g31))
				(not (resource-at-craft cut-sugarcane g32))
				(not (resource-at-craft cut-sugarcane g33))
				(increase (thing-available paper) 3)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-wool
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft string g21)
				(resource-at-craft string g22)
				(craft-empty g23)
				(resource-at-craft string g31)
				(resource-at-craft string g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft string g21))
				(not (resource-at-craft string g22))
				(not (resource-at-craft string g31))
				(not (resource-at-craft string g32))
				(increase (thing-available wool) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-bed
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft wool g21)
				(resource-at-craft wool g22)
				(resource-at-craft wool g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(resource-at-craft plank g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft wool g21))
				(not (resource-at-craft wool g22))
				(not (resource-at-craft wool g23))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(not (resource-at-craft plank g33))
				(increase (thing-available bed) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-chest
		:parameters ()
		:precondition
			(and
				(resource-at-craft plank g11)
				(resource-at-craft plank g12)
				(resource-at-craft plank g13)
				(resource-at-craft plank g21)
				(craft-empty g22)
				(resource-at-craft plank g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(resource-at-craft plank g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft plank g11))
				(not (resource-at-craft plank g12))
				(not (resource-at-craft plank g13))
				(not (resource-at-craft plank g21))
				(not (resource-at-craft plank g23))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(not (resource-at-craft plank g33))
				(increase (thing-available chest) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-wood-door
		:parameters ()
		:precondition
			(and
				(resource-at-craft plank g11)
				(resource-at-craft plank g12)
				(craft-empty g13)
				(resource-at-craft plank g21)
				(resource-at-craft plank g22)
				(craft-empty g23)
				(resource-at-craft plank g31)
				(resource-at-craft plank g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft plank g11))
				(not (resource-at-craft plank g12))
				(not (resource-at-craft plank g21))
				(not (resource-at-craft plank g22))
				(not (resource-at-craft plank g31))
				(not (resource-at-craft plank g32))
				(increase (thing-available wood-door) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-iron-door
		:parameters ()
		:precondition
			(and
				(resource-at-craft iron g11)
				(resource-at-craft iron g12)
				(craft-empty g13)
				(resource-at-craft iron g21)
				(resource-at-craft iron g22)
				(craft-empty g23)
				(resource-at-craft iron g31)
				(resource-at-craft iron g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft iron g11))
				(not (resource-at-craft iron g12))
				(not (resource-at-craft iron g21))
				(not (resource-at-craft iron g22))
				(not (resource-at-craft iron g31))
				(not (resource-at-craft iron g32))
				(increase (thing-available iron-door) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-ladder
		:parameters ()
		:precondition
			(and
				(resource-at-craft stick g11)
				(craft-empty g12)
				(resource-at-craft stick g13)
				(resource-at-craft stick g21)
				(resource-at-craft stick g22)
				(resource-at-craft stick g23)
				(resource-at-craft stick g31)
				(craft-empty g32)
				(resource-at-craft stick g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g33)
				(not (resource-at-craft stick g11))
				(not (resource-at-craft stick g13))
				(not (resource-at-craft stick g21))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft stick g23))
				(not (resource-at-craft stick g31))
				(not (resource-at-craft stick g33))
				(increase (thing-available ladder) 2)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-fence
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft stick g21)
				(resource-at-craft stick g22)
				(resource-at-craft stick g23)
				(resource-at-craft stick g31)
				(resource-at-craft stick g32)
				(resource-at-craft stick g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft stick g21))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft stick g23))
				(not (resource-at-craft stick g31))
				(not (resource-at-craft stick g32))
				(not (resource-at-craft stick g33))
				(increase (thing-available fence) 2)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-stone-brick
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft stone g21)
				(resource-at-craft stone g22)
				(craft-empty g23)
				(resource-at-craft stone g31)
				(resource-at-craft stone g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(craft-empty g32)
				(not (resource-at-craft stone g21))
				(not (resource-at-craft stone g22))
				(not (resource-at-craft stone g31))
				(not (resource-at-craft stone g32))
				(increase (thing-available stonebrick) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-ironbar
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft iron g21)
				(resource-at-craft iron g22)
				(resource-at-craft iron g23)
				(resource-at-craft iron g31)
				(resource-at-craft iron g32)
				(resource-at-craft iron g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft iron g21))
				(not (resource-at-craft iron g22))
				(not (resource-at-craft iron g23))
				(not (resource-at-craft iron g31))
				(not (resource-at-craft iron g32))
				(not (resource-at-craft iron g33))
				(increase (thing-available ironbar) 16)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-glasspane
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(resource-at-craft glass g21)
				(resource-at-craft glass g22)
				(resource-at-craft glass g23)
				(resource-at-craft glass g31)
				(resource-at-craft glass g32)
				(resource-at-craft glass g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft glass g21))
				(not (resource-at-craft glass g22))
				(not (resource-at-craft glass g23))
				(not (resource-at-craft glass g31))
				(not (resource-at-craft glass g32))
				(not (resource-at-craft glass g33))
				(increase (thing-available glasspane) 16)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-bread
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(resource-at-craft wheat g31)
				(resource-at-craft wheat g32)
				(resource-at-craft wheat g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
				(not (resource-at-craft wheat g31))
				(not (resource-at-craft wheat g32))
				(not (resource-at-craft wheat g33))
				(increase (thing-available bread) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-shears
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft iron g22)
				(craft-empty g23)
				(resource-at-craft iron g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g22)
				(craft-empty g31)
				(not (resource-at-craft iron g22))
				(not (resource-at-craft iron g31))
				(increase (thing-available shears) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-bowl
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft plank g12)
				(craft-empty g13)
				(resource-at-craft plank g21)
				(craft-empty g22)
				(resource-at-craft plank g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(craft-empty g21)
				(craft-empty g23)
				(not (resource-at-craft plank g12))
				(not (resource-at-craft plank g21))
				(not (resource-at-craft plank g23))
				(increase (thing-available bowl) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-bucket
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft iron g12)
				(craft-empty g13)
				(resource-at-craft iron g21)
				(craft-empty g22)
				(resource-at-craft iron g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(craft-empty g21)
				(craft-empty g23)
				(not (resource-at-craft iron g12))
				(not (resource-at-craft iron g21))
				(not (resource-at-craft iron g23))
				(increase (thing-available bucket) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-fishingrod
		:parameters ()
		:precondition
			(and
				(resource-at-craft stick g11)
				(craft-empty g12)
				(resource-at-craft string g13)
				(craft-empty g21)
				(resource-at-craft stick g22)
				(resource-at-craft string g23)
				(craft-empty g31)
				(craft-empty g32)
				(resource-at-craft stick g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g13)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g33)
				(not (resource-at-craft stick g11))
				(not (resource-at-craft string g13))
				(not (resource-at-craft stick g22))
				(not (resource-at-craft string g23))
				(not (resource-at-craft stick g33))
				(increase (thing-available fishingrod) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-bonemeal
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft bone g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g23)
				(craft-empty g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(not (resource-at-craft bone g12))
				(increase (thing-available bonemeal) 1)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-wood-stairs
		:parameters ()
		:precondition
			(and
				(resource-at-craft plank g11)
				(resource-at-craft plank g12)
				(resource-at-craft plank g13)
				(resource-at-craft plank g21)
				(resource-at-craft plank g22)
				(craft-empty g23)
				(resource-at-craft plank g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(not (resource-at-craft plank g11))
				(not (resource-at-craft plank g12))
				(not (resource-at-craft plank g13))
				(not (resource-at-craft plank g21))
				(not (resource-at-craft plank g22))
				(not (resource-at-craft plank g31))
				(increase (thing-available wood-stairs) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-stone-stairs
		:parameters ()
		:precondition
			(and
				(resource-at-craft stone g11)
				(resource-at-craft stone g12)
				(resource-at-craft stone g13)
				(resource-at-craft stone g21)
				(resource-at-craft stone g22)
				(craft-empty g23)
				(resource-at-craft stone g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(not (resource-at-craft stone g11))
				(not (resource-at-craft stone g12))
				(not (resource-at-craft stone g13))
				(not (resource-at-craft stone g21))
				(not (resource-at-craft stone g22))
				(not (resource-at-craft stone g31))
				(increase (thing-available stone-stairs) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-brick-stairs
		:parameters ()
		:precondition
			(and
				(resource-at-craft brick g11)
				(resource-at-craft brick g12)
				(resource-at-craft brick g13)
				(resource-at-craft brick g21)
				(resource-at-craft brick g22)
				(craft-empty g23)
				(resource-at-craft brick g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(not (resource-at-craft brick g11))
				(not (resource-at-craft brick g12))
				(not (resource-at-craft brick g13))
				(not (resource-at-craft brick g21))
				(not (resource-at-craft brick g22))
				(not (resource-at-craft brick g31))
				(increase (thing-available brick-stairs) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action craft-stonebrick-stairs
		:parameters ()
		:precondition
			(and
				(resource-at-craft stonebrick g11)
				(resource-at-craft stonebrick g12)
				(resource-at-craft stonebrick g13)
				(resource-at-craft stonebrick g21)
				(resource-at-craft stonebrick g22)
				(craft-empty g23)
				(resource-at-craft stonebrick g31)
				(craft-empty g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g11)
				(craft-empty g12)
				(craft-empty g13)
				(craft-empty g21)
				(craft-empty g22)
				(craft-empty g31)
				(not (resource-at-craft stonebrick g11))
				(not (resource-at-craft stonebrick g12))
				(not (resource-at-craft stonebrick g13))
				(not (resource-at-craft stonebrick g21))
				(not (resource-at-craft stonebrick g22))
				(not (resource-at-craft stonebrick g31))
				(increase (thing-available stonebrick-stairs) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-mushroomstew-1
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft red-mushroom g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft brown-mushroom g22)
				(craft-empty g23)
				(craft-empty g31)
				(resource-at-craft bowl g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g32)
				(not (resource-at-craft red-mushroom g12))
				(not (resource-at-craft brown-mushroom g22))
				(not (resource-at-craft bowl g32))
				(increase (thing-available mushroomstew) 4)
				(not (crafting))
			)
	)

	;; ----------------------------------------------------
	(:action make-mushroomstew-2
		:parameters ()
		:precondition
			(and
				(craft-empty g11)
				(resource-at-craft brown-mushroom g12)
				(craft-empty g13)
				(craft-empty g21)
				(resource-at-craft red-mushroom g22)
				(craft-empty g23)
				(craft-empty g31)
				(resource-at-craft bowl g32)
				(craft-empty g33)
			)
		:effect
			(and 	
				(assign (current-harvest-location) 0)
				(assign (current-harvest-duration) 0)
				(craft-empty g12)
				(craft-empty g22)
				(craft-empty g32)
				(not (resource-at-craft brown-mushroom g12))
				(not (resource-at-craft red-mushroom g22))
				(not (resource-at-craft bowl g32))
				(increase (thing-available mushroomstew) 4)
				(not (crafting))
			)
	)


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Smelting
	;;

	;; ----------------------------------------------------
	;(:action add-furnace-fuel-coal
	;	:parameters ()
	;	:precondition
	;		(and
	;			(> (thing-available furnace) 0)
	;			(> (thing-available coal) 0)
	;		)
	;	:effect
	;		(and
	;			(decrease (thing-available coal) 1)
	;			(increase (furnace-fuel) 16)
	;		)
	;)

	;; ----------------------------------------------------
	;(:action add-furnace-fuel-wood
	;	:parameters ()
	;	:precondition
	;		(and
	;			(> (thing-available furnace) 0)
	;			(> (thing-available wood) 0)
	;		)
	;	:effect
	;		(and
	;			(decrease (thing-available wood) 1)
	;			(increase (furnace-fuel) 3)
	;		)
	;)

	;; ----------------------------------------------------
	;(:action add-furnace-fuel-plank
	;	:parameters ()
	;	:precondition
	;		(and
	;			(> (thing-available furnace) 0)
	;			(> (thing-available plank) 0)
	;		)
	;	:effect
	;		(and
	;			(decrease (thing-available plank) 1)
	;			(increase (furnace-fuel) 3)
	;		)
	;)

	;; ----------------------------------------------------
	;(:action add-furnace-fuel-stick
	;	:parameters ()
	;	:precondition
	;		(and
	;			(> (thing-available furnace) 0)
	;			(> (thing-available stick) 0)
	;		)
	;	:effect
	;		(and
	;			(decrease (thing-available stick) 1)
	;			(increase (furnace-fuel) 1)
	;		)
	;)

	;; ----------------------------------------------------
	(:action furnace-cook-fish
		:parameters ()
		:precondition
			(and
				(> (thing-available furnace) 0)
				;(> (furnace-fuel) 1)
				(> (thing-available fish) 0)
			)
		:effect
			(and 	
				(decrease (thing-available fish) 1)
				;(decrease (furnace-fuel) 2)
				(increase (thing-available cookedfish) 1)
			)
	)

	;; ----------------------------------------------------
	(:action furnace-fire-brick
		:parameters ()
		:precondition
			(and
				(> (thing-available furnace) 0)
				;(> (furnace-fuel) 1)
				(> (thing-available clay) 0)
			)
		:effect
			(and 	
				(decrease (thing-available clay) 1)
				;(decrease (furnace-fuel) 2)
				(increase (thing-available claybrick) 1)
			)
	)

	;; ----------------------------------------------------
	(:action furnace-make-charcoal
		:parameters ()
		:precondition
			(and
				(> (thing-available furnace) 0)
				;(> (furnace-fuel) 1)
				(> (thing-available wood) 0)
			)
		:effect
			(and 	
				(decrease (thing-available wood) 1)
				;(decrease (furnace-fuel) 2)
				(increase (thing-available coal) 1)
			)
	)

	;; ----------------------------------------------------
	(:action furnace-smelt-iron
		:parameters ()
		:precondition
			(and
				(> (thing-available furnace) 0)
				;(> (furnace-fuel) 1)
				(> (thing-available ironore) 0)
			)
		:effect
			(and 	
				(decrease (thing-available ironore) 1)
				;(decrease (furnace-fuel) 2)
				(increase (thing-available iron) 1)
			)
	)

	;; ----------------------------------------------------
	(:action furnace-make-glass
		:parameters ()
		:precondition
			(and
				(> (thing-available furnace) 0)
				;(> (furnace-fuel) 1)
				(> (thing-available sand) 0)
			)
		:effect
			(and 	
				(decrease (thing-available sand) 1)
				;(decrease (furnace-fuel) 2)
				(increase (thing-available glass) 1)
			)
	)

)
