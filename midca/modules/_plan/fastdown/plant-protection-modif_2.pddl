(define (domain minecraft-beta)
(:requirements :strips :negative-preconditions :typing :conditional-effects :adl)
(:types
    dummypoint
    mapgrid
    direction

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
    :effect (and
;(forall (?pos - mapgrid)(when (and (ISADJ east ?to ?ea)(ISADJ north ?ea ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
(when (ISADJ east ?to ?ea) (and (not (native-at ?ea))(not (invasive-at ?ea)) ))
;(forall (?pos - mapgrid)(when (and (ISADJ west ?to ?we)(ISADJ south ?we ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
;(forall (?pos - mapgrid)(when (and (ISADJ east ?to ?ea)(ISADJ nows ?ea ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
;(forall (?pos - mapgrid)(when (and (ISADJ west ?to ?we)(ISADJ north ?we ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
;(forall (?pos - mapgrid)(when (and (ISADJ east ?to ?ea)(ISADJ south ?ea ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
;(forall (?pos - mapgrid)(when (and (ISADJ west ?to ?we)(ISADJ soes ?we ?pos))(and (not (native-at ?pos))(not (invasive-at ?pos)) )))
;(when (ISADJ west ?to ?we) (and (not (native-at ?we))(not (invasive-at ?we)) ))
        (not (invasive-at ?to))
        (not (native-at ?to))


    )
)
)
