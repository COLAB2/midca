(define
	(problem wood)
	(:domain minecraft-beta)

	(:objects
		g11 g12 g13 g21 g22 g23 g31 g32 g33 - craftgrid
		m0_0 m0_1 m0_2 m0_3 m0_4 m1_0 m1_1 m1_2 m1_3 m1_4 m2_0 m2_1  - mapgrid
		
		tree
		wood
		bed
			- resource

	

		hand
		wood-pickaxe
		wood-axe
		
			- tool

	)

	(:init
		(player-at m0_0)
		
		(tool-in-hand hand)
		
		
		
		(connect m0_0 m0_1) 
		(connect m0_0 m1_0) 
		(connect m0_0 m1_1) 
		(connect m0_1 m0_0) 
		(connect m0_1 m0_2) 
		(connect m0_1 m1_0) 
		(connect m0_1 m1_1) 

		(thing-at-map tree m0_1) 
		(thing-at-map tree m2_2) 
		
	)

	(:goal
		(thing-available wood) 
		
	)
)

