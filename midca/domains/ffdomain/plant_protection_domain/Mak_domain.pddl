(define (domain grid-minecraft)
(:requirements :adl :strips :typing :fluents)
(:types cell entity item)
(:predicates
  (steve_at ?c - cell)
  (visited ?c - cell)
  (diamond_drops_collected)
  (wood_drops_collected)
  (alive)
) ;predicates
(:functions
  (x_cell ?c - cell)
  (z_cell ?c - cell)
  (cell_type ?c - cell)  ;  0=WALKABLE_AND_CLEAR(clear=W) 1=LIQUID(liquid=~) 2=SOLID(solid==) 3=LOG(log=L) 2147483647=UNKNOWN(unknown=?)
  (drop_type ?c - cell)
  (entity_type ?c - cell)
  (wood_in_inventory)
  (wood_goal)
  (diamond_in_inventory)
  (diamond_goal)
  (health)
  (max_health)
) ;functions

(:action move-north
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (z_cell ?start) (+ (z_cell ?end) 1)) (= (x_cell ?start) (x_cell ?end)) (= (cell_type ?end) 0))
  :effect (and  (not (steve_at ?start)) (steve_at ?end))
)

(:action move-south
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (+ (z_cell ?start) 1) (z_cell ?end)) (= (x_cell ?start) (x_cell ?end)) (= (cell_type ?end) 0))
  :effect (and  (not (steve_at ?start)) (steve_at ?end))
)

(:action move-east
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (+ (x_cell ?start) 1) (x_cell ?end)) (= (z_cell ?start) (z_cell ?end)) (= (cell_type ?end) 0))
  :effect (and  (not (steve_at ?start)) (steve_at ?end))
)

(:action move-west
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (x_cell ?start) (+ (x_cell ?end) 1)) (= (z_cell ?start) (z_cell ?end)) (= (cell_type ?end) 0))
  :effect (and  (not (steve_at ?start)) (steve_at ?end))
)

(:action make-bridge
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (z_cell ?start) (+ (z_cell ?end ) 1)) (= (x_cell ?start) (x_cell ?end)) (= (cell_type ?end) 1))
  :effect (and (assign (cell_type ?end) 0))
)

(:action mine-through
  :parameters (?start - cell ?end - cell)
  :precondition (and (steve_at ?start) (= (z_cell ?start) (+ (z_cell ?end) 1)) (= (x_cell ?start) (x_cell ?end)) (= (cell_type ?end) 2))
  :effect (and (assign (cell_type ?end) 0))
)

(:action get_wood_drop
  :parameters ( ?start - cell ?drop_cell - cell )
  :precondition (and (steve_at ?start) (= (z_cell ?start) (z_cell ?drop_cell)) (= (x_cell ?start) (x_cell ?drop_cell)) (= (drop_type ?drop_cell) 3))
  :effect (and (wood_drops_collected) (increase (wood_in_inventory) 1) )
)

(:action get_diamond_drop
  :parameters ( ?start - cell ?drop_cell - cell )
  :precondition (and (steve_at ?start) (= (z_cell ?start) (z_cell ?drop_cell)) (= (x_cell ?start) (x_cell ?drop_cell)) (= (drop_type ?drop_cell) 4))
  :effect (and (diamond_drops_collected) (increase (diamond_in_inventory) 1) )
)

(:event wood_drop_appears
  :parameters ( ?start - cell ?drop_cell - cell )
  :precondition (and (steve_at ?start) (< (wood_in_inventory) (wood_goal)) (< (- (z_cell ?start) (z_cell ?drop_cell)) 3) (= (drop_type ?drop_cell) 3))
  :effect (and (not (wood_drops_collected)) )
)

(:event diamond_drop_appears
  :parameters ( ?start - cell ?drop_cell - cell )
  :precondition (and (steve_at ?start) (< (diamond_in_inventory) (diamond_goal)) (< (- (z_cell ?drop_cell) (z_cell ?start)) 5) (= (drop_type ?drop_cell) 4))
  :effect (and (not (diamond_drops_collected)) )
)

(:event zombie_damage
  :parameters ( ?start - cell ?zombie_cell - cell )
  :precondition (and (alive) (steve_at ?start) (= (entity_type ?zombie_cell) 100) (>= (- (z_cell ?zombie_cell) (z_cell ?start)) -1) (<= (z_cell ?zombie_cell) (z_cell ?start)) (<= (- (x_cell ?start) (x_cell ?zombie_cell)) 1) (<= (- (x_cell ?zombie_cell) (x_cell ?start)) 1) )
  :effect (and (not (alive)) )
)
) ;domain