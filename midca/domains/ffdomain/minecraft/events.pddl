(define (domain event-minecraft)
    (:types
		resource - thing
		material - thing
		tool - thing
		craftgrid
		mapgrid
		player
	)
    (:event zombie_damage
      :parameters ( ?loc - mapgrid ?zombie_loc - mapgrid )
      :precondition
          (and
               (player_at ?loc)
               (connect ?loc ?zombie_loc))

      :effect
          (and
               (decrease (player-current-health ) 5)
          )
    )

    (:event trap_damage
      :parameters ( ?loc - mapgrid ?trap_loc - mapgrid )
      :precondition
          (and
               (player_at ?loc)
               (connect ?loc ?trap_loc))

      :effect
          (and
               (decrease (player-current-health ) 5)
          )
    )
)