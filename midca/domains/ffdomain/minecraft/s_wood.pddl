(define
	(problem wood)
	(:domain minecraft-beta)

	(:objects
		g11 g12 g13 g21 g22 g23 g31 g32 g33 - craftgrid
		m0_0 m0_1 m0_2 m0_3 m0_4 m1_0 m1_1 m1_2 m1_3 m1_4 m2_0 m2_1 m2_2 m2_3 m2_4 m3_0 m3_1 m3_2 m3_3 m3_4 m4_0 m4_1 m4_2 m4_3 m4_4 - mapgrid
		water
		chicken
		sheep
		cow
		cobweb
		grass
		tallgrass
		wheatgrass
		tree
		rock
		sandrock
		soil
		farmland
		claysoil
		coalore
		ironore-rock
		brown-mushroom
		red-mushroom
		skeleton
		sugarcane
			- resource

		egg
		fish
		cookedfish
		wool
		string
		seeds
		wheat
		wood
		sand
		clay
		stone
		sandstone
		plank
		stick
		coal
		ironore
		iron
		brown-mushroom
		red-mushroom
		bone
		bonemeal
		wood-door
		wood-stairs
		stone-stairs
		brick-stairs
		stonebrick-stairs
		iron-door
		clayblock
		stonebrick
		glass
		glasspane
		ladder
		fence
		sugar
		paper
		bed
		bread
		mushroomstew
		milk
		ironbar
		claybrick
		brick
		cut-sugarcane
		bowl
			- material

		hand
		furnace
		torch
		chest
		bucket
		fishingrod
		shears
		wood-pickaxe
		wood-axe
		wood-hoe
		wood-shovel
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

