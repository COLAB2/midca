(define 
(problem learning-problem1)
(:domain meta)
(:objects
native-at - goal
not-invasive-at - goal
spray - operator
move - operator
state-i-plus-1 - state
)
(:init
(caused_discrepancy spray )
(outdated spray)
(has-discrepancy state-i-plus-1)

)
 (:goal
(and
(learning spray state-i-plus-1)
)
))
