;;  ---------- Start of domain debug details
;;  action: move-north
;;  action: move-south
;;  action: move-east
;;  action: move-west
;;  action: make-bridge
;;  action: mine-through
;;  action: get_wood_drop
;;  action: get_diamond_drop
;;  event: wood_drop_appears
;;  event: diamond_drop_appears
;;  event: zombie_damage
;;  CellTypes:  0=WALKABLE_AND_CLEAR(clear=W) 1=LIQUID(liquid=~) 2=SOLID(solid==) 3=LOG(log=L) 2147483647=UNKNOWN(unknown=?)
;;  DropTypes:  0=DIRT(dirt=d) 1=COBBLESTONE(cobblestone=c) 3=OAK_WOOD(wood=W) 4=DIAMOND(diamond=D) 8=HARDENED_CLAY(clay=c) 2147483647=UNKNOWN(unknown=?)
;;  EntityTypes:  1=PLAYER([EntityPlayerMP]=X) 100=ZOMBIE([EntityZombie]=Z) 101=SKELETON([Skeleton]=S) 102=CREEPER([Creeper]=C) 2147483647=UNKNOWN([]=?)
;;  ---------- End of domain debug details
(define (problem grid-minecraft-current) 
(:domain grid-minecraft)
(:objects
  ;cells in the problem area
  x1347y45z-17 x1348y45z-17 x1349y45z-17 x1350y45z-17 x1351y45z-17 x1352y45z-17 x1353y45z-17
  x1347y45z-16 x1348y45z-16 x1349y45z-16 x1350y45z-16 x1351y45z-16 x1352y45z-16 x1353y45z-16
  x1347y45z-15 x1348y45z-15 x1349y45z-15 x1350y45z-15 x1351y45z-15 x1352y45z-15 x1353y45z-15
  x1347y45z-14 x1348y45z-14 x1349y45z-14 x1350y45z-14 x1351y45z-14 x1352y45z-14 x1353y45z-14
  x1347y45z-13 x1348y45z-13 x1349y45z-13 x1350y45z-13 x1351y45z-13 x1352y45z-13 x1353y45z-13
  x1347y45z-12 x1348y45z-12 x1349y45z-12 x1350y45z-12 x1351y45z-12 x1352y45z-12 x1353y45z-12
  x1347y45z-11 x1348y45z-11 x1349y45z-11 x1350y45z-11 x1351y45z-11 x1352y45z-11 x1353y45z-11
  ;cells in the observation area with mobs

  ;cells in the observation area with drops
  x1347y45z-19  x1348y45z-19  x1353y45z-19
 - cell
  zombie_4  ;EntityZombie['Zombie'/4, l='Drops10', x=1350.50, y=45.00, z=-15.50]
 - entity
  diamond_197  ;EntityItem['item.item.diamond'/197, l='Drops10', x=1347.55, y=45.13, z=-18.73]
  wood_199  ;EntityItem['item.tile.log.oak'/199, l='Drops10', x=1348.42, y=45.13, z=-18.52]
  wood_210  ;EntityItem['item.tile.log.oak'/210, l='Drops10', x=1353.22, y=45.13, z=-18.33]
  diamond_212  ;EntityItem['item.item.diamond'/212, l='Drops10', x=1350.45, y=45.13, z=-13.88]
 - item
) ;objects
(:init
  (= (x_cell x1347y45z-17) 1347)
  (= (z_cell x1347y45z-17) -17)
  (= (cell_type x1347y45z-17) 0)
  (= (x_cell x1347y45z-16) 1347)
  (= (z_cell x1347y45z-16) -16)
  (= (cell_type x1347y45z-16) 0)
  (= (x_cell x1347y45z-15) 1347)
  (= (z_cell x1347y45z-15) -15)
  (= (cell_type x1347y45z-15) 0)
  (= (x_cell x1347y45z-14) 1347)
  (= (z_cell x1347y45z-14) -14)
  (= (cell_type x1347y45z-14) 0)
  (= (x_cell x1347y45z-13) 1347)
  (= (z_cell x1347y45z-13) -13)
  (= (cell_type x1347y45z-13) 0)
  (= (x_cell x1347y45z-12) 1347)
  (= (z_cell x1347y45z-12) -12)
  (= (cell_type x1347y45z-12) 0)
  (= (x_cell x1347y45z-11) 1347)
  (= (z_cell x1347y45z-11) -11)
  (= (cell_type x1347y45z-11) 0)
  (= (x_cell x1348y45z-17) 1348)
  (= (z_cell x1348y45z-17) -17)
  (= (cell_type x1348y45z-17) 0)
  (= (x_cell x1348y45z-16) 1348)
  (= (z_cell x1348y45z-16) -16)
  (= (cell_type x1348y45z-16) 0)
  (= (x_cell x1348y45z-15) 1348)
  (= (z_cell x1348y45z-15) -15)
  (= (cell_type x1348y45z-15) 0)
  (= (x_cell x1348y45z-14) 1348)
  (= (z_cell x1348y45z-14) -14)
  (= (cell_type x1348y45z-14) 0)
  (= (x_cell x1348y45z-13) 1348)
  (= (z_cell x1348y45z-13) -13)
  (= (cell_type x1348y45z-13) 0)
  (= (x_cell x1348y45z-12) 1348)
  (= (z_cell x1348y45z-12) -12)
  (= (cell_type x1348y45z-12) 0)
  (= (x_cell x1348y45z-11) 1348)
  (= (z_cell x1348y45z-11) -11)
  (= (cell_type x1348y45z-11) 0)
  (= (x_cell x1349y45z-17) 1349)
  (= (z_cell x1349y45z-17) -17)
  (= (cell_type x1349y45z-17) 0)
  (= (x_cell x1349y45z-16) 1349)
  (= (z_cell x1349y45z-16) -16)
  (= (cell_type x1349y45z-16) 2)
  (= (x_cell x1349y45z-15) 1349)
  (= (z_cell x1349y45z-15) -15)
  (= (cell_type x1349y45z-15) 0)
  (= (x_cell x1349y45z-14) 1349)
  (= (z_cell x1349y45z-14) -14)
  (= (cell_type x1349y45z-14) 0)
  (= (x_cell x1349y45z-13) 1349)
  (= (z_cell x1349y45z-13) -13)
  (= (cell_type x1349y45z-13) 0)
  (= (x_cell x1349y45z-12) 1349)
  (= (z_cell x1349y45z-12) -12)
  (= (cell_type x1349y45z-12) 0)
  (= (x_cell x1349y45z-11) 1349)
  (= (z_cell x1349y45z-11) -11)
  (= (cell_type x1349y45z-11) 0)
  (= (x_cell x1350y45z-17) 1350)
  (= (z_cell x1350y45z-17) -17)
  (= (cell_type x1350y45z-17) 2)
  (= (x_cell x1350y45z-16) 1350)
  (= (z_cell x1350y45z-16) -16)
  (= (cell_type x1350y45z-16) 0)
  (= (x_cell x1350y45z-15) 1350)
  (= (z_cell x1350y45z-15) -15)
  (= (cell_type x1350y45z-15) 2)
  (= (x_cell x1350y45z-14) 1350)
  (= (z_cell x1350y45z-14) -14)
  (= (cell_type x1350y45z-14) 0)
  (= (x_cell x1350y45z-13) 1350)
  (= (z_cell x1350y45z-13) -13)
  (= (cell_type x1350y45z-13) 0)
  (= (x_cell x1350y45z-12) 1350)
  (= (z_cell x1350y45z-12) -12)
  (= (cell_type x1350y45z-12) 0)
  (= (x_cell x1350y45z-11) 1350)
  (= (z_cell x1350y45z-11) -11)
  (= (cell_type x1350y45z-11) 0)
  (= (x_cell x1351y45z-17) 1351)
  (= (z_cell x1351y45z-17) -17)
  (= (cell_type x1351y45z-17) 0)
  (= (x_cell x1351y45z-16) 1351)
  (= (z_cell x1351y45z-16) -16)
  (= (cell_type x1351y45z-16) 2)
  (= (x_cell x1351y45z-15) 1351)
  (= (z_cell x1351y45z-15) -15)
  (= (cell_type x1351y45z-15) 0)
  (= (x_cell x1351y45z-14) 1351)
  (= (z_cell x1351y45z-14) -14)
  (= (cell_type x1351y45z-14) 0)
  (= (x_cell x1351y45z-13) 1351)
  (= (z_cell x1351y45z-13) -13)
  (= (cell_type x1351y45z-13) 0)
  (= (x_cell x1351y45z-12) 1351)
  (= (z_cell x1351y45z-12) -12)
  (= (cell_type x1351y45z-12) 0)
  (= (x_cell x1351y45z-11) 1351)
  (= (z_cell x1351y45z-11) -11)
  (= (cell_type x1351y45z-11) 0)
  (= (x_cell x1352y45z-17) 1352)
  (= (z_cell x1352y45z-17) -17)
  (= (cell_type x1352y45z-17) 0)
  (= (x_cell x1352y45z-16) 1352)
  (= (z_cell x1352y45z-16) -16)
  (= (cell_type x1352y45z-16) 0)
  (= (x_cell x1352y45z-15) 1352)
  (= (z_cell x1352y45z-15) -15)
  (= (cell_type x1352y45z-15) 0)
  (= (x_cell x1352y45z-14) 1352)
  (= (z_cell x1352y45z-14) -14)
  (= (cell_type x1352y45z-14) 0)
  (= (x_cell x1352y45z-13) 1352)
  (= (z_cell x1352y45z-13) -13)
  (= (cell_type x1352y45z-13) 0)
  (= (x_cell x1352y45z-12) 1352)
  (= (z_cell x1352y45z-12) -12)
  (= (cell_type x1352y45z-12) 0)
  (= (x_cell x1352y45z-11) 1352)
  (= (z_cell x1352y45z-11) -11)
  (= (cell_type x1352y45z-11) 0)
  (= (x_cell x1353y45z-17) 1353)
  (= (z_cell x1353y45z-17) -17)
  (= (cell_type x1353y45z-17) 0)
  (= (x_cell x1353y45z-16) 1353)
  (= (z_cell x1353y45z-16) -16)
  (= (cell_type x1353y45z-16) 0)
  (= (x_cell x1353y45z-15) 1353)
  (= (z_cell x1353y45z-15) -15)
  (= (cell_type x1353y45z-15) 0)
  (= (x_cell x1353y45z-14) 1353)
  (= (z_cell x1353y45z-14) -14)
  (= (cell_type x1353y45z-14) 0)
  (= (x_cell x1353y45z-13) 1353)
  (= (z_cell x1353y45z-13) -13)
  (= (cell_type x1353y45z-13) 0)
  (= (x_cell x1353y45z-12) 1353)
  (= (z_cell x1353y45z-12) -12)
  (= (cell_type x1353y45z-12) 0)
  (= (x_cell x1353y45z-11) 1353)
  (= (z_cell x1353y45z-11) -11)
  (= (cell_type x1353y45z-11) 0)

  ;special cells
  (= (entity_type x1350y45z-16) 100) ;zombie
  (= (x_cell x1347y45z-19) 1347)
  (= (z_cell x1347y45z-19) -19)
  (= (cell_type x1347y45z-19) 0)
  (= (drop_type x1347y45z-19) 4) ;diamond
  (= (x_cell x1348y45z-19) 1348)
  (= (z_cell x1348y45z-19) -19)
  (= (cell_type x1348y45z-19) 0)
  (= (drop_type x1348y45z-19) 3) ;wood
  (= (x_cell x1353y45z-19) 1353)
  (= (z_cell x1353y45z-19) -19)
  (= (cell_type x1353y45z-19) 0)
  (= (drop_type x1353y45z-19) 3) ;wood
  (= (drop_type x1350y45z-14) 4) ;diamond

  ;character details
  (steve_at x1350y45z-12)
  (= (wood_in_inventory) 0)
  (= (diamond_in_inventory) 0)
  (= (wood_goal) 10)
  (= (diamond_goal) 10)
  (= (health) 20.0)
  (= (max_health) 20.0)
  (alive)
  (wood_drops_collected)
  (not (diamond_drops_collected))
)
(:goal
  (and 
    (steve_at x1353y45z-17)
  (wood_drops_collected)
  (diamond_drops_collected)
  (alive)
  )
)
) ;problem
;;  ---------- Start of problem debug details
;;  observed 1 entities and 4 drops
;;  raw observations:
;;  [EntityZombie] EntityZombie_4=EntityZombie_4x1350y45z-16age:0
;;  [EntityItem] EntityItem_197=EntityItem_197x1347y45z-19age:0
;;  [EntityItem] EntityItem_199=EntityItem_199x1348y45z-19age:0
;;  [EntityItem] EntityItem_210=EntityItem_210x1353y45z-19age:0
;;  [EntityItem] EntityItem_212=EntityItem_212x1350y45z-14age:0
;;         x1340  x1341  x1342  x1343  x1344  x1345  x1346  X1347  X1348  X1349  X1350  X1351  X1352  X1353  x1354  x1355  x1356  x1357  x1358  x1359  x1360  
;;  z-22                                                                                                                                                      
;;  z-21                                                                                                                                                      
;;  z-20                                                                                                                                                      
;;  z-19                                                    D      W                                  W                                                       
;;  z-18                                                                                                                                                      
;;  Z-17                                                    W      W      W      =      W      W      W                                                       
;;  Z-16                                                    W      W      =      WZ     =      W      W                                                       
;;  Z-15                                                    W      W      W      =      W      W      W                                                       
;;  Z-14                                                    W      W      W      WD     W      W      W                                                       
;;  Z-13                                                    W      W      W      W      W      W      W                                                       
;;  Z-12                                                    W      W      W      WX     W      W      W                                                       
;;  Z-11                                                    W      W      W      W      W      W      W                                                       
;;  z-10                                                                                                                                                      
;;  z-9                                                                                                                                                       
;;  z-8                                                                                                                                                       
;;  z-7                                                                                                                                                       
;;  z-6                                                                                                                                                       
;;  z-5                                                                                                                                                       
;;  z-4                                                                                                                                                       
;;  z-3                                                                                                                                                       
;;  z-2                                                                                                                                                       
;;  ---------- End of problem debug details
