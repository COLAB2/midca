
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
        (trap-destroyed ?loc - mapgrid)
        (searched-left ?obj - resource)
        (searched-right ?obj - resource)
        (searched-behind ?obj - resource)
        (searched-forward ?obj - resource)
        (looking-right)
        (looking-left)
        (looking-forward)
        (looking-behind)
		(thing-at-map  ?obj - resource  ?loc - mapgrid)
		(thing-at ?obj - resource ?loc - mapgrid)
		(known-loc ?obj - resource ?playerloc - mapgrid )
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
        (attacking ?loc - mapgrid)
		(looking-for ?res - resource ?loc - mapgrid)
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
   ;; (:action event-find
	;;	:parameters (?res - resource ?loc - mapgrid ?player_loc -mapgrid)
	;;	:precondition
	;;		(and
	;;		    (looking-for ?res ?player_loc)
	;;		    (player-at ?player_loc)
	;;			 (connect ?loc ?player_loc)
	;;			 (thing-at-loc ?res ?loc)
	;;		)
	;;	:effect
	;;		(and
	;;			(know-where ?res ?loc)
	;;			(thing-at-map ?res ?loc)
      ;;          (not (looking-for ?res ?player_loc))
		;;	)

	;;)


    ;;--------------------------------------------------------

    (:action event-find-left
		:parameters (?res -resource  ?loc - mapgrid ?player_loc -mapgrid)
		:precondition
			(and
			 (player-at ?player_loc)
			    (looking-left ?player_loc)
				 (connect-left ?player_loc ?loc)
				 (thing-at-loc ?res ?loc)
			)
		:effect
			(and

				(thing-at-map ?res ?loc)
				(known-loc ?res ?player_loc)
				;;(not (looking-left))
			)

	)


    ;;--------------------------------------------------------
(:action event-find-forward
		:parameters (?res -resource  ?loc - mapgrid ?player_loc -mapgrid)
		:precondition
			(and
			    (looking-forward ?player_loc)
			    (player-at ?player_loc)
				 (connect-forward ?player_loc ?loc)
				 (thing-at-loc ?res ?loc)
			)
		:effect
			(and

				(thing-at-map ?res ?loc)
				(known-loc ?res ?player_loc)
				;;(not (looking-forward))
			)

	)
    ;;--------------------------------------------------------

    (:action event-find-right
		:parameters (?res -resource ?loc - mapgrid ?player_loc - mapgrid)
		:precondition
			(and
			 (player-at ?player_loc)
			    (looking-right ?player_loc)
				 (connect-right  ?player_loc ?loc)
				 (thing-at-loc ?res ?loc)
			)
		:effect
			(and

				(thing-at-map ?res ?loc)
				(known-loc ?res ?player_loc)
				;;(not (looking-right))
			)

	)


    ;;--------------------------------------------------------
     (:action event-find-behind
		:parameters (?res -resource ?loc - mapgrid ?player_loc - mapgrid)
		:precondition
			(and
			 (player-at ?player_loc)
			    (looking-behind ?player_loc)
				 (connect-behind ?player_loc ?loc)
				 (thing-at-loc ?res ?loc)
			)
		:effect
			(and

				(thing-at-map ?res ?loc)
				(known-loc ?res ?player_loc)
				(not (looking-behind ?player_loc))
			)

	)


    ;;--------------------------------------------------------


	(:action event-fall-in-trap
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            (player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at-loc arrowtrap ?loc)


   			 (> (player-current-health) 0)
      	)
      	:effect
      	(and
   			(decrease (player-current-health) 1)
   			(thing-at-map arrow ?loc)

   			(is-trapped)
      	)
	)
    ;;------------------------------------------------------

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

   			(decrease (player-current-health) 1)
   			(thing-at-map arrow ?loc)

   			(is-attacked)
      	)
	)
    ;;----------------------------------------------
    (:action event-monster-explosion
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            (player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at-loc monster ?loc)
   			 (not (is-attacked))
   			 (> (player-current-health) 0)

      	)
      	:effect
      	(and
   			(decrease (player-current-health) 5)
   			(thing-at-map monster-remains ?loc1)
   			(is-attacked)
      	)
	)
    ;;----------------------------------------------

    ;;----------------------------------
	(:action event-dead-skeleton
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            (player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at-loc skeleton ?loc)
             (attacking ?loc1)


      	)
      	:effect
      	(and
   			(not (thing-at-loc skeleton ?loc))
   			(not (thing-at-map skeleton ?loc))
   			(not (thing-at-map arrow ?loc))
   			(not(is-attacked))

      	)
	)

	;;---------------------------------------------
	 (:action event-destroy-trap
      	:parameters (?loc1 - mapgrid ?loc - mapgrid)
      	:precondition
      	( and
            (player-at ?loc1)
   			 (connect ?loc1 ?loc)
   			 (thing-at-loc arrowtrap ?loc)
             (trap-destroyed ?loc1)



      	)
      	:effect
      	(and
   			(not (thing-at-loc arrowtrap ?loc))
   			(not (thing-at-map arrowtrap ?loc))
   			(not (thing-at-map arrow ?loc))
   			(not (is-trapped))

      	)
	)

	;; ---------------------------------------------------

	)