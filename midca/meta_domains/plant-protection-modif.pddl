(define (domain meta)
(:requirements :strips :negative-preconditions :typing :conditional-effects)
(:types
    state
    dummypoint
    mapgrid
    direction
    operator
    human
    agent
    goal
)
(:predicates
    (has-explanation ?current-state - state)
    (has-discrepancy ?current-state - state)
    (learning ?op - operator ?current-state - state)
    (has-other-feature ?current-state - state)
    (outdated ?op - operator)
    (caused_discrepancy ?op - operator)
    
)


(:action run-foil
    :parameters (?op - operator ?current-state - state)
    :precondition
    (and
     ;(not (has-explanation ?current-state))	
     (has-discrepancy ?current-state)
     (outdated ?op)      
     (caused_discrepancy ?op ) 
     ;exists data       
    )
    :effect
    (and
        (learning ?op ?current-state)
    )
)


;;; Additional (dummy) learning operator
(:action run-EBL
    :parameters (?op - operator ?current-state - state)
    :precondition
    (and
     (has-other-feature ?current-state)	
     (has-discrepancy ?current-state)
     (outdated ?op)      
     (caused_discrepancy ?op )      
    )
    :effect
    (and
        (learning ?op ?current-state)
    )
)





)
