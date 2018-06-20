(define
    (problem wood)
    (:domain minecraft)(define
    (problem wood)
    (:domain minecraft)

    (:objects

   	 m0_0 - mapgrid
   	 m0_1 - mapgrid
   	 m0_2  - mapgrid
   	 m0_3 - mapgrid
   	 m0_4  - mapgrid
   	 m1_0 - mapgrid
   	 m1_1  - mapgrid
   	 m1_2 - mapgrid
   	 m1_3 - mapgrid
   	 m1_4 - mapgrid
   	 m2_0 - mapgrid
   	 m2_1 - mapgrid
   	 m2_2 - mapgrid
   	 m2_3 - mapgrid
   	 m2_4 - mapgrid
   	 m3_0 - mapgrid
   	 m3_1 - mapgrid
   	 m3_2  - mapgrid
   	 m3_3 - mapgrid
   	 m3_4  - mapgrid
   	 m4_0  - mapgrid
   	 m4_1 - mapgrid
   	 m4_2 - mapgrid
   	 m4_3  - mapgrid
   	 m4_4 - mapgrid

   	 tree - resource
   	 shelter - resource
   	 skeleton - resource
   	 arrow - resource
   	 monster - resource
   	 monster-remains - resource
    )

    (:init
   	 (player-at m0_0)

   	 (connect m0_0 m0_1)
   	 (connect m0_0 m1_0)
   	 (connect m0_0 m1_1)
   	 (connect m0_1 m0_0)
   	 (connect m0_1 m0_2)
   	 (connect m0_1 m1_0)
   	 (connect m0_1 m1_1)
   	 (connect m0_1 m1_2)
   	 (connect m0_2 m0_1)
   	 (connect m0_2 m0_3)
   	 (connect m0_2 m1_1)
   	 (connect m0_2 m1_2)
   	 (connect m0_2 m1_3)
   	 (connect m0_3 m0_2)
   	 (connect m0_3 m0_4)
   	 (connect m0_3 m1_2)
   	 (connect m0_3 m1_3)
   	 (connect m0_3 m1_4)
   	 (connect m0_4 m0_3)
   	 (connect m0_4 m1_3)
   	 (connect m0_4 m1_4)
   	 (connect m1_0 m0_0)
   	 (connect m1_0 m0_1)
   	 (connect m1_0 m1_1)
   	 (connect m1_0 m2_0)
   	 (connect m1_0 m2_1)
   	 (connect m1_1 m0_0)
   	 (connect m1_1 m0_1)
   	 (connect m1_1 m0_2)
   	 (connect m1_1 m1_0)
   	 (connect m1_1 m1_2)
   	 (connect m1_1 m2_0)
   	 (connect m1_1 m2_1)
   	 (connect m1_1 m2_2)
   	 (connect m1_2 m0_1)
   	 (connect m1_2 m0_2)
   	 (connect m1_2 m0_3)
   	 (connect m1_2 m1_1)
   	 (connect m1_2 m1_3)
   	 (connect m1_2 m2_1)
   	 (connect m1_2 m2_2)
   	 (connect m1_2 m2_3)
   	 (connect m1_3 m0_2)
   	 (connect m1_3 m0_3)
   	 (connect m1_3 m0_4)
   	 (connect m1_3 m1_2)
   	 (connect m1_3 m1_4)
   	 (connect m1_3 m2_2)
   	 (connect m1_3 m2_3)
   	 (connect m1_3 m2_4)
   	 (connect m1_4 m0_3)
   	 (connect m1_4 m0_4)
   	 (connect m1_4 m1_3)
   	 (connect m1_4 m2_3)
   	 (connect m1_4 m2_4)
   	 (connect m2_0 m1_0)
   	 (connect m2_0 m1_1)
   	 (connect m2_0 m2_1)
   	 (connect m2_0 m3_0)
   	 (connect m2_0 m3_1)
   	 (connect m2_1 m1_0)
   	 (connect m2_1 m1_1)
   	 (connect m2_1 m1_2)
   	 (connect m2_1 m2_0)
   	 (connect m2_1 m2_2)
   	 (connect m2_1 m3_0)
   	 (connect m2_1 m3_1)
   	 (connect m2_1 m3_2)
   	 (connect m2_2 m1_1)
   	 (connect m2_2 m1_2)
   	 (connect m2_2 m1_3)
   	 (connect m2_2 m2_1)
   	 (connect m2_2 m2_3)
   	 (connect m2_2 m3_1)
   	 (connect m2_2 m3_2)
   	 (connect m2_2 m3_3)
   	 (connect m2_3 m1_2)
   	 (connect m2_3 m1_3)
   	 (connect m2_3 m1_4)
   	 (connect m2_3 m2_2)
   	 (connect m2_3 m2_4)
   	 (connect m2_3 m3_2)
   	 (connect m2_3 m3_3)
   	 (connect m2_3 m3_4)
   	 (connect m2_4 m1_3)
   	 (connect m2_4 m1_4)
   	 (connect m2_4 m2_3)
   	 (connect m2_4 m3_3)
   	 (connect m2_4 m3_4)
   	 (connect m3_0 m2_0)
   	 (connect m3_0 m2_1)
   	 (connect m3_0 m3_1)
   	 (connect m3_0 m4_0)
   	 (connect m3_0 m4_1)
   	 (connect m3_1 m2_0)
   	 (connect m3_1 m2_1)
   	 (connect m3_1 m2_2)
   	 (connect m3_1 m3_0)
   	 (connect m3_1 m3_2)
   	 (connect m3_1 m4_0)
   	 (connect m3_1 m4_1)
   	 (connect m3_1 m4_2)
   	 (connect m3_2 m2_1)
   	 (connect m3_2 m2_2)
   	 (connect m3_2 m2_3)
   	 (connect m3_2 m3_1)
   	 (connect m3_2 m3_3)
   	 (connect m3_2 m4_1)
   	 (connect m3_2 m4_2)
   	 (connect m3_2 m4_3)
   	 (connect m3_3 m2_2)
   	 (connect m3_3 m2_3)
   	 (connect m3_3 m2_4)
   	 (connect m3_3 m3_2)
   	 (connect m3_3 m3_4)
   	 (connect m3_3 m4_2)
   	 (connect m3_3 m4_3)
   	 (connect m3_3 m4_4)
   	 (connect m3_4 m2_3)
   	 (connect m3_4 m2_4)
   	 (connect m3_4 m3_3)
   	 (connect m3_4 m4_3)
   	 (connect m3_4 m4_4)
   	 (connect m4_0 m3_0)
   	 (connect m4_0 m3_1)
   	 (connect m4_0 m4_1)
   	 (connect m4_1 m3_0)
   	 (connect m4_1 m3_1)
   	 (connect m4_1 m3_2)
   	 (connect m4_1 m4_0)
   	 (connect m4_1 m4_2)
   	 (connect m4_2 m3_1)
   	 (connect m4_2 m3_2)
   	 (connect m4_2 m3_3)
   	 (connect m4_2 m4_1)
   	 (connect m4_2 m4_3)
   	 (connect m4_3 m3_2)
   	 (connect m4_3 m3_3)
   	 (connect m4_3 m3_4)
   	 (connect m4_3 m4_2)
   	 (connect m4_3 m4_4)
   	 (connect m4_4 m3_3)
   	 (connect m4_4 m3_4)
   	 (connect m4_4 m4_3)
   	 (is-trap m3_3)
   	 (thing-at skeleton m2_1)
   	 (thing-at-map tree m2_2)
   	 (thing-at-map shelter m2_3)
    )

    (:values
    ((location-id m0_0) 0)
    ((player-current-health) 20)
    )


    (:goal
   	 (and
   	 ((player-at m1_1))
   	 )
))

    (:objects

   	 m0_0 - mapgrid
   	 m0_1 - mapgrid
   	 m0_2  - mapgrid
   	 m0_3 - mapgrid
   	 m0_4  - mapgrid
   	 m1_0 - mapgrid
   	 m1_1  - mapgrid
   	 m1_2 - mapgrid
   	 m1_3 - mapgrid
   	 m1_4 - mapgrid
   	 m2_0 - mapgrid
   	 m2_1 - mapgrid
   	 m2_2 - mapgrid
   	 m2_3 - mapgrid
   	 m2_4 - mapgrid
   	 m3_0 - mapgrid
   	 m3_1 - mapgrid
   	 m3_2  - mapgrid
   	 m3_3 - mapgrid
   	 m3_4  - mapgrid
   	 m4_0  - mapgrid
   	 m4_1 - mapgrid
   	 m4_2 - mapgrid
   	 m4_3  - mapgrid
   	 m4_4 - mapgrid

   	 tree - resource
   	 shelter - resource
   	 skeleton - resource
   	 arrow - resource
   	 monster - resource
   	 monster-remains - resource
    )

    (:init
   	 (player-at m0_0)

   	 (connect m0_0 m0_1)
   	 (connect m0_0 m1_0)
   	 (connect m0_0 m1_1)
   	 (connect m0_1 m0_0)
   	 (connect m0_1 m0_2)
   	 (connect m0_1 m1_0)
   	 (connect m0_1 m1_1)
   	 (connect m0_1 m1_2)
   	 (connect m0_2 m0_1)
   	 (connect m0_2 m0_3)
   	 (connect m0_2 m1_1)
   	 (connect m0_2 m1_2)
   	 (connect m0_2 m1_3)
   	 (connect m0_3 m0_2)
   	 (connect m0_3 m0_4)
   	 (connect m0_3 m1_2)
   	 (connect m0_3 m1_3)
   	 (connect m0_3 m1_4)
   	 (connect m0_4 m0_3)
   	 (connect m0_4 m1_3)
   	 (connect m0_4 m1_4)
   	 (connect m1_0 m0_0)
   	 (connect m1_0 m0_1)
   	 (connect m1_0 m1_1)
   	 (connect m1_0 m2_0)
   	 (connect m1_0 m2_1)
   	 (connect m1_1 m0_0)
   	 (connect m1_1 m0_1)
   	 (connect m1_1 m0_2)
   	 (connect m1_1 m1_0)
   	 (connect m1_1 m1_2)
   	 (connect m1_1 m2_0)
   	 (connect m1_1 m2_1)
   	 (connect m1_1 m2_2)
   	 (connect m1_2 m0_1)
   	 (connect m1_2 m0_2)
   	 (connect m1_2 m0_3)
   	 (connect m1_2 m1_1)
   	 (connect m1_2 m1_3)
   	 (connect m1_2 m2_1)
   	 (connect m1_2 m2_2)
   	 (connect m1_2 m2_3)
   	 (connect m1_3 m0_2)
   	 (connect m1_3 m0_3)
   	 (connect m1_3 m0_4)
   	 (connect m1_3 m1_2)
   	 (connect m1_3 m1_4)
   	 (connect m1_3 m2_2)
   	 (connect m1_3 m2_3)
   	 (connect m1_3 m2_4)
   	 (connect m1_4 m0_3)
   	 (connect m1_4 m0_4)
   	 (connect m1_4 m1_3)
   	 (connect m1_4 m2_3)
   	 (connect m1_4 m2_4)
   	 (connect m2_0 m1_0)
   	 (connect m2_0 m1_1)
   	 (connect m2_0 m2_1)
   	 (connect m2_0 m3_0)
   	 (connect m2_0 m3_1)
   	 (connect m2_1 m1_0)
   	 (connect m2_1 m1_1)
   	 (connect m2_1 m1_2)
   	 (connect m2_1 m2_0)
   	 (connect m2_1 m2_2)
   	 (connect m2_1 m3_0)
   	 (connect m2_1 m3_1)
   	 (connect m2_1 m3_2)
   	 (connect m2_2 m1_1)
   	 (connect m2_2 m1_2)
   	 (connect m2_2 m1_3)
   	 (connect m2_2 m2_1)
   	 (connect m2_2 m2_3)
   	 (connect m2_2 m3_1)
   	 (connect m2_2 m3_2)
   	 (connect m2_2 m3_3)
   	 (connect m2_3 m1_2)
   	 (connect m2_3 m1_3)
   	 (connect m2_3 m1_4)
   	 (connect m2_3 m2_2)
   	 (connect m2_3 m2_4)
   	 (connect m2_3 m3_2)
   	 (connect m2_3 m3_3)
   	 (connect m2_3 m3_4)
   	 (connect m2_4 m1_3)
   	 (connect m2_4 m1_4)
   	 (connect m2_4 m2_3)
   	 (connect m2_4 m3_3)
   	 (connect m2_4 m3_4)
   	 (connect m3_0 m2_0)
   	 (connect m3_0 m2_1)
   	 (connect m3_0 m3_1)
   	 (connect m3_0 m4_0)
   	 (connect m3_0 m4_1)
   	 (connect m3_1 m2_0)
   	 (connect m3_1 m2_1)
   	 (connect m3_1 m2_2)
   	 (connect m3_1 m3_0)
   	 (connect m3_1 m3_2)
   	 (connect m3_1 m4_0)
   	 (connect m3_1 m4_1)
   	 (connect m3_1 m4_2)
   	 (connect m3_2 m2_1)
   	 (connect m3_2 m2_2)
   	 (connect m3_2 m2_3)
   	 (connect m3_2 m3_1)
   	 (connect m3_2 m3_3)
   	 (connect m3_2 m4_1)
   	 (connect m3_2 m4_2)
   	 (connect m3_2 m4_3)
   	 (connect m3_3 m2_2)
   	 (connect m3_3 m2_3)
   	 (connect m3_3 m2_4)
   	 (connect m3_3 m3_2)
   	 (connect m3_3 m3_4)
   	 (connect m3_3 m4_2)
   	 (connect m3_3 m4_3)
   	 (connect m3_3 m4_4)
   	 (connect m3_4 m2_3)
   	 (connect m3_4 m2_4)
   	 (connect m3_4 m3_3)
   	 (connect m3_4 m4_3)
   	 (connect m3_4 m4_4)
   	 (connect m4_0 m3_0)
   	 (connect m4_0 m3_1)
   	 (connect m4_0 m4_1)
   	 (connect m4_1 m3_0)
   	 (connect m4_1 m3_1)
   	 (connect m4_1 m3_2)
   	 (connect m4_1 m4_0)
   	 (connect m4_1 m4_2)
   	 (connect m4_2 m3_1)
   	 (connect m4_2 m3_2)
   	 (connect m4_2 m3_3)
   	 (connect m4_2 m4_1)
   	 (connect m4_2 m4_3)
   	 (connect m4_3 m3_2)
   	 (connect m4_3 m3_3)
   	 (connect m4_3 m3_4)
   	 (connect m4_3 m4_2)
   	 (connect m4_3 m4_4)
   	 (connect m4_4 m3_3)
   	 (connect m4_4 m3_4)
   	 (connect m4_4 m4_3)
   	 (is-trap m3_3)
   	 (thing-at skeleton m2_1)
   	 (thing-at-map tree m2_2)
   	 (thing-at-map shelter m2_3)
    )

    (:values
    ((location-id m0_0) 0)
    ((player-current-health) 20)
    )


    (:goal
   	 (and
   	 ((player-at m1_1))
   	 )
))
