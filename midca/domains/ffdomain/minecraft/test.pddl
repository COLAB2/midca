(define
(problem wood)
(:domain minecraft-beta)
(:objects
m0_0 - mapgrid
m0_1 - mapgrid
m0_2 - mapgrid
m0_3 - mapgrid
m0_4 - mapgrid
m0_5 - mapgrid
m0_6 - mapgrid
m0_7 - mapgrid
m0_8 - mapgrid
m0_9 - mapgrid
m1_0 - mapgrid
m1_1 - mapgrid
m1_2 - mapgrid
m1_3 - mapgrid
m1_4 - mapgrid
m1_5 - mapgrid
m1_6 - mapgrid
m1_7 - mapgrid
m1_8 - mapgrid
m1_9 - mapgrid
m2_0 - mapgrid
m2_1 - mapgrid
m2_2 - mapgrid
m2_3 - mapgrid
m2_4 - mapgrid
m2_5 - mapgrid
m2_6 - mapgrid
m2_7 - mapgrid
m2_8 - mapgrid
m2_9 - mapgrid
m3_0 - mapgrid
m3_1 - mapgrid
m3_2 - mapgrid
m3_3 - mapgrid
m3_4 - mapgrid
m3_5 - mapgrid
m3_6 - mapgrid
m3_7 - mapgrid
m3_8 - mapgrid
m3_9 - mapgrid
m4_0 - mapgrid
m4_1 - mapgrid
m4_2 - mapgrid
m4_3 - mapgrid
m4_4 - mapgrid
m4_5 - mapgrid
m4_6 - mapgrid
m4_7 - mapgrid
m4_8 - mapgrid
m4_9 - mapgrid
m5_0 - mapgrid
m5_1 - mapgrid
m5_2 - mapgrid
m5_3 - mapgrid
m5_4 - mapgrid
m5_5 - mapgrid
m5_6 - mapgrid
m5_7 - mapgrid
m5_8 - mapgrid
m5_9 - mapgrid
m6_0 - mapgrid
m6_1 - mapgrid
m6_2 - mapgrid
m6_3 - mapgrid
m6_4 - mapgrid
m6_5 - mapgrid
m6_6 - mapgrid
m6_7 - mapgrid
m6_8 - mapgrid
m6_9 - mapgrid
m7_0 - mapgrid
m7_1 - mapgrid
m7_2 - mapgrid
m7_3 - mapgrid
m7_4 - mapgrid
m7_5 - mapgrid
m7_6 - mapgrid
m7_7 - mapgrid
m7_8 - mapgrid
m7_9 - mapgrid
m8_0 - mapgrid
m8_1 - mapgrid
m8_2 - mapgrid
m8_3 - mapgrid
m8_4 - mapgrid
m8_5 - mapgrid
m8_6 - mapgrid
m8_7 - mapgrid
m8_8 - mapgrid
m8_9 - mapgrid
m9_0 - mapgrid
m9_1 - mapgrid
m9_2 - mapgrid
m9_3 - mapgrid
m9_4 - mapgrid
m9_5 - mapgrid
m9_6 - mapgrid
m9_7 - mapgrid
m9_8 - mapgrid
m9_9 - mapgrid
tree - resource
g11 g12 g13 g21 g22 g23 g31 g32 g33 - craftgrid
shelter - resource
skeleton - resource
arrowtrap - resource
arrow - resource
bone - resource
monster - resource
monster-remains - resource
helmet - helmet
chestplate - chestplate
instant-health-potion - potion
wood - resource
wood-axe - tool
hand - tool
bow - tool
bowl - material)
(:init
(connect m0_0 m1_0)
(connect m0_0 m0_1)
(connect m0_1 m1_1)
(connect m0_1 m0_2)
(connect m0_1 m0_0)
(connect m0_2 m1_2)
(connect m0_2 m0_3)
(connect m0_2 m0_1)
(connect m0_3 m1_3)
(connect m0_3 m0_4)
(connect m0_3 m0_2)
(connect m0_4 m1_4)
(connect m0_4 m0_5)
(connect m0_4 m0_3)
(connect m0_5 m1_5)
(connect m0_5 m0_6)
(connect m0_5 m0_4)
(connect m0_6 m1_6)
(connect m0_6 m0_7)
(connect m0_6 m0_5)
(connect m0_7 m1_7)
(connect m0_7 m0_8)
(connect m0_7 m0_6)
(connect m0_8 m1_8)
(connect m0_8 m0_9)
(connect m0_8 m0_7)
(connect m0_9 m1_9)
(connect m0_9 m0_8)
(connect m1_0 m2_0)
(connect m1_0 m0_0)
(connect m1_0 m1_1)
(connect m1_1 m2_1)
(connect m1_1 m0_1)
(connect m1_1 m1_2)
(connect m1_1 m1_0)
(connect m1_2 m2_2)
(connect m1_2 m0_2)
(connect m1_2 m1_3)
(connect m1_2 m1_1)
(connect m1_3 m2_3)
(connect m1_3 m0_3)
(connect m1_3 m1_4)
(connect m1_3 m1_2)
(connect m1_4 m2_4)
(connect m1_4 m0_4)
(connect m1_4 m1_5)
(connect m1_4 m1_3)
(connect m1_5 m2_5)
(connect m1_5 m0_5)
(connect m1_5 m1_6)
(connect m1_5 m1_4)
(connect m1_6 m2_6)
(connect m1_6 m0_6)
(connect m1_6 m1_7)
(connect m1_6 m1_5)
(connect m1_7 m2_7)
(connect m1_7 m0_7)
(connect m1_7 m1_8)
(connect m1_7 m1_6)
(connect m1_8 m2_8)
(connect m1_8 m0_8)
(connect m1_8 m1_9)
(connect m1_8 m1_7)
(connect m1_9 m2_9)
(connect m1_9 m0_9)
(connect m1_9 m1_8)
(connect m2_0 m3_0)
(connect m2_0 m1_0)
(connect m2_0 m2_1)
(connect m2_1 m3_1)
(connect m2_1 m1_1)
(connect m2_1 m2_2)
(connect m2_1 m2_0)
(connect m2_2 m3_2)
(connect m2_2 m1_2)
(connect m2_2 m2_3)
(connect m2_2 m2_1)
(connect m2_3 m3_3)
(connect m2_3 m1_3)
(connect m2_3 m2_4)
(connect m2_3 m2_2)
(connect m2_4 m3_4)
(connect m2_4 m1_4)
(connect m2_4 m2_5)
(connect m2_4 m2_3)
(connect m2_5 m3_5)
(connect m2_5 m1_5)
(connect m2_5 m2_6)
(connect m2_5 m2_4)
(connect m2_6 m3_6)
(connect m2_6 m1_6)
(connect m2_6 m2_7)
(connect m2_6 m2_5)
(connect m2_7 m3_7)
(connect m2_7 m1_7)
(connect m2_7 m2_8)
(connect m2_7 m2_6)
(connect m2_8 m3_8)
(connect m2_8 m1_8)
(connect m2_8 m2_9)
(connect m2_8 m2_7)
(connect m2_9 m3_9)
(connect m2_9 m1_9)
(connect m2_9 m2_8)
(connect m3_0 m4_0)
(connect m3_0 m2_0)
(connect m3_0 m3_1)
(connect m3_1 m4_1)
(connect m3_1 m2_1)
(connect m3_1 m3_2)
(connect m3_1 m3_0)
(connect m3_2 m4_2)
(connect m3_2 m2_2)
(connect m3_2 m3_3)
(connect m3_2 m3_1)
(connect m3_3 m4_3)
(connect m3_3 m2_3)
(connect m3_3 m3_4)
(connect m3_3 m3_2)
(connect m3_4 m4_4)
(connect m3_4 m2_4)
(connect m3_4 m3_5)
(connect m3_4 m3_3)
(connect m3_5 m4_5)
(connect m3_5 m2_5)
(connect m3_5 m3_6)
(connect m3_5 m3_4)
(connect m3_6 m4_6)
(connect m3_6 m2_6)
(connect m3_6 m3_7)
(connect m3_6 m3_5)
(connect m3_7 m4_7)
(connect m3_7 m2_7)
(connect m3_7 m3_8)
(connect m3_7 m3_6)
(connect m3_8 m4_8)
(connect m3_8 m2_8)
(connect m3_8 m3_9)
(connect m3_8 m3_7)
(connect m3_9 m4_9)
(connect m3_9 m2_9)
(connect m3_9 m3_8)
(connect m4_0 m5_0)
(connect m4_0 m3_0)
(connect m4_0 m4_1)
(connect m4_1 m5_1)
(connect m4_1 m3_1)
(connect m4_1 m4_2)
(connect m4_1 m4_0)
(connect m4_2 m5_2)
(connect m4_2 m3_2)
(connect m4_2 m4_3)
(connect m4_2 m4_1)
(connect m4_3 m5_3)
(connect m4_3 m3_3)
(connect m4_3 m4_4)
(connect m4_3 m4_2)
(connect m4_4 m5_4)
(connect m4_4 m3_4)
(connect m4_4 m4_5)
(connect m4_4 m4_3)
(connect m4_5 m5_5)
(connect m4_5 m3_5)
(connect m4_5 m4_6)
(connect m4_5 m4_4)
(connect m4_6 m5_6)
(connect m4_6 m3_6)
(connect m4_6 m4_7)
(connect m4_6 m4_5)
(connect m4_7 m5_7)
(connect m4_7 m3_7)
(connect m4_7 m4_8)
(connect m4_7 m4_6)
(connect m4_8 m5_8)
(connect m4_8 m3_8)
(connect m4_8 m4_9)
(connect m4_8 m4_7)
(connect m4_9 m5_9)
(connect m4_9 m3_9)
(connect m4_9 m4_8)
(connect m5_0 m6_0)
(connect m5_0 m4_0)
(connect m5_0 m5_1)
(connect m5_1 m6_1)
(connect m5_1 m4_1)
(connect m5_1 m5_2)
(connect m5_1 m5_0)
(connect m5_2 m6_2)
(connect m5_2 m4_2)
(connect m5_2 m5_3)
(connect m5_2 m5_1)
(connect m5_3 m6_3)
(connect m5_3 m4_3)
(connect m5_3 m5_4)
(connect m5_3 m5_2)
(connect m5_4 m6_4)
(connect m5_4 m4_4)
(connect m5_4 m5_5)
(connect m5_4 m5_3)
(connect m5_5 m6_5)
(connect m5_5 m4_5)
(connect m5_5 m5_6)
(connect m5_5 m5_4)
(connect m5_6 m6_6)
(connect m5_6 m4_6)
(connect m5_6 m5_7)
(connect m5_6 m5_5)
(connect m5_7 m6_7)
(connect m5_7 m4_7)
(connect m5_7 m5_8)
(connect m5_7 m5_6)
(connect m5_8 m6_8)
(connect m5_8 m4_8)
(connect m5_8 m5_9)
(connect m5_8 m5_7)
(connect m5_9 m6_9)
(connect m5_9 m4_9)
(connect m5_9 m5_8)
(connect m6_0 m7_0)
(connect m6_0 m5_0)
(connect m6_0 m6_1)
(connect m6_1 m7_1)
(connect m6_1 m5_1)
(connect m6_1 m6_2)
(connect m6_1 m6_0)
(connect m6_2 m7_2)
(connect m6_2 m5_2)
(connect m6_2 m6_3)
(connect m6_2 m6_1)
(connect m6_3 m7_3)
(connect m6_3 m5_3)
(connect m6_3 m6_4)
(connect m6_3 m6_2)
(connect m6_4 m7_4)
(connect m6_4 m5_4)
(connect m6_4 m6_5)
(connect m6_4 m6_3)
(connect m6_5 m7_5)
(connect m6_5 m5_5)
(connect m6_5 m6_6)
(connect m6_5 m6_4)
(connect m6_6 m7_6)
(connect m6_6 m5_6)
(connect m6_6 m6_7)
(connect m6_6 m6_5)
(connect m6_7 m7_7)
(connect m6_7 m5_7)
(connect m6_7 m6_8)
(connect m6_7 m6_6)
(connect m6_8 m7_8)
(connect m6_8 m5_8)
(connect m6_8 m6_9)
(connect m6_8 m6_7)
(connect m6_9 m7_9)
(connect m6_9 m5_9)
(connect m6_9 m6_8)
(connect m7_0 m8_0)
(connect m7_0 m6_0)
(connect m7_0 m7_1)
(connect m7_1 m8_1)
(connect m7_1 m6_1)
(connect m7_1 m7_2)
(connect m7_1 m7_0)
(connect m7_2 m8_2)
(connect m7_2 m6_2)
(connect m7_2 m7_3)
(connect m7_2 m7_1)
(connect m7_3 m8_3)
(connect m7_3 m6_3)
(connect m7_3 m7_4)
(connect m7_3 m7_2)
(connect m7_4 m8_4)
(connect m7_4 m6_4)
(connect m7_4 m7_5)
(connect m7_4 m7_3)
(connect m7_5 m8_5)
(connect m7_5 m6_5)
(connect m7_5 m7_6)
(connect m7_5 m7_4)
(connect m7_6 m8_6)
(connect m7_6 m6_6)
(connect m7_6 m7_7)
(connect m7_6 m7_5)
(connect m7_7 m8_7)
(connect m7_7 m6_7)
(connect m7_7 m7_8)
(connect m7_7 m7_6)
(connect m7_8 m8_8)
(connect m7_8 m6_8)
(connect m7_8 m7_9)
(connect m7_8 m7_7)
(connect m7_9 m8_9)
(connect m7_9 m6_9)
(connect m7_9 m7_8)
(connect m8_0 m9_0)
(connect m8_0 m7_0)
(connect m8_0 m8_1)
(connect m8_1 m9_1)
(connect m8_1 m7_1)
(connect m8_1 m8_2)
(connect m8_1 m8_0)
(connect m8_2 m9_2)
(connect m8_2 m7_2)
(connect m8_2 m8_3)
(connect m8_2 m8_1)
(connect m8_3 m9_3)
(connect m8_3 m7_3)
(connect m8_3 m8_4)
(connect m8_3 m8_2)
(connect m8_4 m9_4)
(connect m8_4 m7_4)
(connect m8_4 m8_5)
(connect m8_4 m8_3)
(connect m8_5 m9_5)
(connect m8_5 m7_5)
(connect m8_5 m8_6)
(connect m8_5 m8_4)
(connect m8_6 m9_6)
(connect m8_6 m7_6)
(connect m8_6 m8_7)
(connect m8_6 m8_5)
(connect m8_7 m9_7)
(connect m8_7 m7_7)
(connect m8_7 m8_8)
(connect m8_7 m8_6)
(connect m8_8 m9_8)
(connect m8_8 m7_8)
(connect m8_8 m8_9)
(connect m8_8 m8_7)
(connect m8_9 m9_9)
(connect m8_9 m7_9)
(connect m8_9 m8_8)
(connect m9_0 m8_0)
(connect m9_0 m9_1)
(connect m9_1 m8_1)
(connect m9_1 m9_2)
(connect m9_1 m9_0)
(connect m9_2 m8_2)
(connect m9_2 m9_3)
(connect m9_2 m9_1)
(connect m9_3 m8_3)
(connect m9_3 m9_4)
(connect m9_3 m9_2)
(connect m9_4 m8_4)
(connect m9_4 m9_5)
(connect m9_4 m9_3)
(connect m9_5 m8_5)
(connect m9_5 m9_6)
(connect m9_5 m9_4)
(connect m9_6 m8_6)
(connect m9_6 m9_7)
(connect m9_6 m9_5)
(connect m9_7 m8_7)
(connect m9_7 m9_8)
(connect m9_7 m9_6)
(connect m9_8 m8_8)
(connect m9_8 m9_9)
(connect m9_8 m9_7)
(connect m9_9 m8_9)
(connect m9_9 m9_8)
(connect-right m0_0 m1_1)
(connect-right m0_0 m0_1)
(connect-behind m0_0 m1_0)
(connect-behind m0_0 m1_1)
(connect-right m0_1 m1_2)
(connect-right m0_1 m0_2)
(connect-left m0_1 m1_0)
(connect-left m0_1 m0_0)
(connect-behind m0_1 m1_0)
(connect-behind m0_1 m1_1)
(connect-behind m0_1 m1_2)
(connect-right m0_2 m1_3)
(connect-right m0_2 m0_3)
(connect-left m0_2 m1_1)
(connect-left m0_2 m0_1)
(connect-behind m0_2 m1_1)
(connect-behind m0_2 m1_2)
(connect-behind m0_2 m1_3)
(connect-right m0_3 m1_4)
(connect-right m0_3 m0_4)
(connect-left m0_3 m1_2)
(connect-left m0_3 m0_2)
(connect-behind m0_3 m1_2)
(connect-behind m0_3 m1_3)
(connect-behind m0_3 m1_4)
(connect-right m0_4 m1_5)
(connect-right m0_4 m0_5)
(connect-left m0_4 m1_3)
(connect-left m0_4 m0_3)
(connect-behind m0_4 m1_3)
(connect-behind m0_4 m1_4)
(connect-behind m0_4 m1_5)
(connect-right m0_5 m1_6)
(connect-right m0_5 m0_6)
(connect-left m0_5 m1_4)
(connect-left m0_5 m0_4)
(connect-behind m0_5 m1_4)
(connect-behind m0_5 m1_5)
(connect-behind m0_5 m1_6)
(connect-right m0_6 m1_7)
(connect-right m0_6 m0_7)
(connect-left m0_6 m1_5)
(connect-left m0_6 m0_5)
(connect-behind m0_6 m1_5)
(connect-behind m0_6 m1_6)
(connect-behind m0_6 m1_7)
(connect-right m0_7 m1_8)
(connect-right m0_7 m0_8)
(connect-left m0_7 m1_6)
(connect-left m0_7 m0_6)
(connect-behind m0_7 m1_6)
(connect-behind m0_7 m1_7)
(connect-behind m0_7 m1_8)
(connect-right m0_8 m1_9)
(connect-right m0_8 m0_9)
(connect-left m0_8 m1_7)
(connect-left m0_8 m0_7)
(connect-behind m0_8 m1_7)
(connect-behind m0_8 m1_8)
(connect-behind m0_8 m1_9)
(connect-left m0_9 m1_8)
(connect-left m0_9 m0_8)
(connect-behind m0_9 m1_8)
(connect-behind m0_9 m1_9)
(connect-right m1_0 m2_1)
(connect-right m1_0 m0_1)
(connect-right m1_0 m1_1)
(connect-behind m1_0 m2_0)
(connect-behind m1_0 m2_1)
(connect-forward m1_0 m0_0)
(connect-forward m1_0 m0_1)
(connect-right m1_1 m2_2)
(connect-right m1_1 m0_2)
(connect-right m1_1 m1_2)
(connect-left m1_1 m2_0)
(connect-left m1_1 m0_0)
(connect-left m1_1 m1_0)
(connect-behind m1_1 m2_0)
(connect-behind m1_1 m2_1)
(connect-behind m1_1 m2_2)
(connect-forward m1_1 m0_0)
(connect-forward m1_1 m0_1)
(connect-forward m1_1 m0_2)
(connect-right m1_2 m2_3)
(connect-right m1_2 m0_3)
(connect-right m1_2 m1_3)
(connect-left m1_2 m2_1)
(connect-left m1_2 m0_1)
(connect-left m1_2 m1_1)
(connect-behind m1_2 m2_1)
(connect-behind m1_2 m2_2)
(connect-behind m1_2 m2_3)
(connect-forward m1_2 m0_1)
(connect-forward m1_2 m0_2)
(connect-forward m1_2 m0_3)
(connect-right m1_3 m2_4)
(connect-right m1_3 m0_4)
(connect-right m1_3 m1_4)
(connect-left m1_3 m2_2)
(connect-left m1_3 m0_2)
(connect-left m1_3 m1_2)
(connect-behind m1_3 m2_2)
(connect-behind m1_3 m2_3)
(connect-behind m1_3 m2_4)
(connect-forward m1_3 m0_2)
(connect-forward m1_3 m0_3)
(connect-forward m1_3 m0_4)
(connect-right m1_4 m2_5)
(connect-right m1_4 m0_5)
(connect-right m1_4 m1_5)
(connect-left m1_4 m2_3)
(connect-left m1_4 m0_3)
(connect-left m1_4 m1_3)
(connect-behind m1_4 m2_3)
(connect-behind m1_4 m2_4)
(connect-behind m1_4 m2_5)
(connect-forward m1_4 m0_3)
(connect-forward m1_4 m0_4)
(connect-forward m1_4 m0_5)
(connect-right m1_5 m2_6)
(connect-right m1_5 m0_6)
(connect-right m1_5 m1_6)
(connect-left m1_5 m2_4)
(connect-left m1_5 m0_4)
(connect-left m1_5 m1_4)
(connect-behind m1_5 m2_4)
(connect-behind m1_5 m2_5)
(connect-behind m1_5 m2_6)
(connect-forward m1_5 m0_4)
(connect-forward m1_5 m0_5)
(connect-forward m1_5 m0_6)
(connect-right m1_6 m2_7)
(connect-right m1_6 m0_7)
(connect-right m1_6 m1_7)
(connect-left m1_6 m2_5)
(connect-left m1_6 m0_5)
(connect-left m1_6 m1_5)
(connect-behind m1_6 m2_5)
(connect-behind m1_6 m2_6)
(connect-behind m1_6 m2_7)
(connect-forward m1_6 m0_5)
(connect-forward m1_6 m0_6)
(connect-forward m1_6 m0_7)
(connect-right m1_7 m2_8)
(connect-right m1_7 m0_8)
(connect-right m1_7 m1_8)
(connect-left m1_7 m2_6)
(connect-left m1_7 m0_6)
(connect-left m1_7 m1_6)
(connect-behind m1_7 m2_6)
(connect-behind m1_7 m2_7)
(connect-behind m1_7 m2_8)
(connect-forward m1_7 m0_6)
(connect-forward m1_7 m0_7)
(connect-forward m1_7 m0_8)
(connect-right m1_8 m2_9)
(connect-right m1_8 m0_9)
(connect-right m1_8 m1_9)
(connect-left m1_8 m2_7)
(connect-left m1_8 m0_7)
(connect-left m1_8 m1_7)
(connect-behind m1_8 m2_7)
(connect-behind m1_8 m2_8)
(connect-behind m1_8 m2_9)
(connect-forward m1_8 m0_7)
(connect-forward m1_8 m0_8)
(connect-forward m1_8 m0_9)
(connect-left m1_9 m2_8)
(connect-left m1_9 m0_8)
(connect-left m1_9 m1_8)
(connect-behind m1_9 m2_8)
(connect-behind m1_9 m2_9)
(connect-forward m1_9 m0_8)
(connect-forward m1_9 m0_9)
(connect-right m2_0 m3_1)
(connect-right m2_0 m1_1)
(connect-right m2_0 m2_1)
(connect-behind m2_0 m3_0)
(connect-behind m2_0 m3_1)
(connect-forward m2_0 m1_0)
(connect-forward m2_0 m1_1)
(connect-right m2_1 m3_2)
(connect-right m2_1 m1_2)
(connect-right m2_1 m2_2)
(connect-left m2_1 m3_0)
(connect-left m2_1 m1_0)
(connect-left m2_1 m2_0)
(connect-behind m2_1 m3_0)
(connect-behind m2_1 m3_1)
(connect-behind m2_1 m3_2)
(connect-forward m2_1 m1_0)
(connect-forward m2_1 m1_1)
(connect-forward m2_1 m1_2)
(connect-right m2_2 m3_3)
(connect-right m2_2 m1_3)
(connect-right m2_2 m2_3)
(connect-left m2_2 m3_1)
(connect-left m2_2 m1_1)
(connect-left m2_2 m2_1)
(connect-behind m2_2 m3_1)
(connect-behind m2_2 m3_2)
(connect-behind m2_2 m3_3)
(connect-forward m2_2 m1_1)
(connect-forward m2_2 m1_2)
(connect-forward m2_2 m1_3)
(connect-right m2_3 m3_4)
(connect-right m2_3 m1_4)
(connect-right m2_3 m2_4)
(connect-left m2_3 m3_2)
(connect-left m2_3 m1_2)
(connect-left m2_3 m2_2)
(connect-behind m2_3 m3_2)
(connect-behind m2_3 m3_3)
(connect-behind m2_3 m3_4)
(connect-forward m2_3 m1_2)
(connect-forward m2_3 m1_3)
(connect-forward m2_3 m1_4)
(connect-right m2_4 m3_5)
(connect-right m2_4 m1_5)
(connect-right m2_4 m2_5)
(connect-left m2_4 m3_3)
(connect-left m2_4 m1_3)
(connect-left m2_4 m2_3)
(connect-behind m2_4 m3_3)
(connect-behind m2_4 m3_4)
(connect-behind m2_4 m3_5)
(connect-forward m2_4 m1_3)
(connect-forward m2_4 m1_4)
(connect-forward m2_4 m1_5)
(connect-right m2_5 m3_6)
(connect-right m2_5 m1_6)
(connect-right m2_5 m2_6)
(connect-left m2_5 m3_4)
(connect-left m2_5 m1_4)
(connect-left m2_5 m2_4)
(connect-behind m2_5 m3_4)
(connect-behind m2_5 m3_5)
(connect-behind m2_5 m3_6)
(connect-forward m2_5 m1_4)
(connect-forward m2_5 m1_5)
(connect-forward m2_5 m1_6)
(connect-right m2_6 m3_7)
(connect-right m2_6 m1_7)
(connect-right m2_6 m2_7)
(connect-left m2_6 m3_5)
(connect-left m2_6 m1_5)
(connect-left m2_6 m2_5)
(connect-behind m2_6 m3_5)
(connect-behind m2_6 m3_6)
(connect-behind m2_6 m3_7)
(connect-forward m2_6 m1_5)
(connect-forward m2_6 m1_6)
(connect-forward m2_6 m1_7)
(connect-right m2_7 m3_8)
(connect-right m2_7 m1_8)
(connect-right m2_7 m2_8)
(connect-left m2_7 m3_6)
(connect-left m2_7 m1_6)
(connect-left m2_7 m2_6)
(connect-behind m2_7 m3_6)
(connect-behind m2_7 m3_7)
(connect-behind m2_7 m3_8)
(connect-forward m2_7 m1_6)
(connect-forward m2_7 m1_7)
(connect-forward m2_7 m1_8)
(connect-right m2_8 m3_9)
(connect-right m2_8 m1_9)
(connect-right m2_8 m2_9)
(connect-left m2_8 m3_7)
(connect-left m2_8 m1_7)
(connect-left m2_8 m2_7)
(connect-behind m2_8 m3_7)
(connect-behind m2_8 m3_8)
(connect-behind m2_8 m3_9)
(connect-forward m2_8 m1_7)
(connect-forward m2_8 m1_8)
(connect-forward m2_8 m1_9)
(connect-left m2_9 m3_8)
(connect-left m2_9 m1_8)
(connect-left m2_9 m2_8)
(connect-behind m2_9 m3_8)
(connect-behind m2_9 m3_9)
(connect-forward m2_9 m1_8)
(connect-forward m2_9 m1_9)
(connect-right m3_0 m4_1)
(connect-right m3_0 m2_1)
(connect-right m3_0 m3_1)
(connect-behind m3_0 m4_0)
(connect-behind m3_0 m4_1)
(connect-forward m3_0 m2_0)
(connect-forward m3_0 m2_1)
(connect-right m3_1 m4_2)
(connect-right m3_1 m2_2)
(connect-right m3_1 m3_2)
(connect-left m3_1 m4_0)
(connect-left m3_1 m2_0)
(connect-left m3_1 m3_0)
(connect-behind m3_1 m4_0)
(connect-behind m3_1 m4_1)
(connect-behind m3_1 m4_2)
(connect-forward m3_1 m2_0)
(connect-forward m3_1 m2_1)
(connect-forward m3_1 m2_2)
(connect-right m3_2 m4_3)
(connect-right m3_2 m2_3)
(connect-right m3_2 m3_3)
(connect-left m3_2 m4_1)
(connect-left m3_2 m2_1)
(connect-left m3_2 m3_1)
(connect-behind m3_2 m4_1)
(connect-behind m3_2 m4_2)
(connect-behind m3_2 m4_3)
(connect-forward m3_2 m2_1)
(connect-forward m3_2 m2_2)
(connect-forward m3_2 m2_3)
(connect-right m3_3 m4_4)
(connect-right m3_3 m2_4)
(connect-right m3_3 m3_4)
(connect-left m3_3 m4_2)
(connect-left m3_3 m2_2)
(connect-left m3_3 m3_2)
(connect-behind m3_3 m4_2)
(connect-behind m3_3 m4_3)
(connect-behind m3_3 m4_4)
(connect-forward m3_3 m2_2)
(connect-forward m3_3 m2_3)
(connect-forward m3_3 m2_4)
(connect-right m3_4 m4_5)
(connect-right m3_4 m2_5)
(connect-right m3_4 m3_5)
(connect-left m3_4 m4_3)
(connect-left m3_4 m2_3)
(connect-left m3_4 m3_3)
(connect-behind m3_4 m4_3)
(connect-behind m3_4 m4_4)
(connect-behind m3_4 m4_5)
(connect-forward m3_4 m2_3)
(connect-forward m3_4 m2_4)
(connect-forward m3_4 m2_5)
(connect-right m3_5 m4_6)
(connect-right m3_5 m2_6)
(connect-right m3_5 m3_6)
(connect-left m3_5 m4_4)
(connect-left m3_5 m2_4)
(connect-left m3_5 m3_4)
(connect-behind m3_5 m4_4)
(connect-behind m3_5 m4_5)
(connect-behind m3_5 m4_6)
(connect-forward m3_5 m2_4)
(connect-forward m3_5 m2_5)
(connect-forward m3_5 m2_6)
(connect-right m3_6 m4_7)
(connect-right m3_6 m2_7)
(connect-right m3_6 m3_7)
(connect-left m3_6 m4_5)
(connect-left m3_6 m2_5)
(connect-left m3_6 m3_5)
(connect-behind m3_6 m4_5)
(connect-behind m3_6 m4_6)
(connect-behind m3_6 m4_7)
(connect-forward m3_6 m2_5)
(connect-forward m3_6 m2_6)
(connect-forward m3_6 m2_7)
(connect-right m3_7 m4_8)
(connect-right m3_7 m2_8)
(connect-right m3_7 m3_8)
(connect-left m3_7 m4_6)
(connect-left m3_7 m2_6)
(connect-left m3_7 m3_6)
(connect-behind m3_7 m4_6)
(connect-behind m3_7 m4_7)
(connect-behind m3_7 m4_8)
(connect-forward m3_7 m2_6)
(connect-forward m3_7 m2_7)
(connect-forward m3_7 m2_8)
(connect-right m3_8 m4_9)
(connect-right m3_8 m2_9)
(connect-right m3_8 m3_9)
(connect-left m3_8 m4_7)
(connect-left m3_8 m2_7)
(connect-left m3_8 m3_7)
(connect-behind m3_8 m4_7)
(connect-behind m3_8 m4_8)
(connect-behind m3_8 m4_9)
(connect-forward m3_8 m2_7)
(connect-forward m3_8 m2_8)
(connect-forward m3_8 m2_9)
(connect-left m3_9 m4_8)
(connect-left m3_9 m2_8)
(connect-left m3_9 m3_8)
(connect-behind m3_9 m4_8)
(connect-behind m3_9 m4_9)
(connect-forward m3_9 m2_8)
(connect-forward m3_9 m2_9)
(connect-right m4_0 m5_1)
(connect-right m4_0 m3_1)
(connect-right m4_0 m4_1)
(connect-behind m4_0 m5_0)
(connect-behind m4_0 m5_1)
(connect-forward m4_0 m3_0)
(connect-forward m4_0 m3_1)
(connect-right m4_1 m5_2)
(connect-right m4_1 m3_2)
(connect-right m4_1 m4_2)
(connect-left m4_1 m5_0)
(connect-left m4_1 m3_0)
(connect-left m4_1 m4_0)
(connect-behind m4_1 m5_0)
(connect-behind m4_1 m5_1)
(connect-behind m4_1 m5_2)
(connect-forward m4_1 m3_0)
(connect-forward m4_1 m3_1)
(connect-forward m4_1 m3_2)
(connect-right m4_2 m5_3)
(connect-right m4_2 m3_3)
(connect-right m4_2 m4_3)
(connect-left m4_2 m5_1)
(connect-left m4_2 m3_1)
(connect-left m4_2 m4_1)
(connect-behind m4_2 m5_1)
(connect-behind m4_2 m5_2)
(connect-behind m4_2 m5_3)
(connect-forward m4_2 m3_1)
(connect-forward m4_2 m3_2)
(connect-forward m4_2 m3_3)
(connect-right m4_3 m5_4)
(connect-right m4_3 m3_4)
(connect-right m4_3 m4_4)
(connect-left m4_3 m5_2)
(connect-left m4_3 m3_2)
(connect-left m4_3 m4_2)
(connect-behind m4_3 m5_2)
(connect-behind m4_3 m5_3)
(connect-behind m4_3 m5_4)
(connect-forward m4_3 m3_2)
(connect-forward m4_3 m3_3)
(connect-forward m4_3 m3_4)
(connect-right m4_4 m5_5)
(connect-right m4_4 m3_5)
(connect-right m4_4 m4_5)
(connect-left m4_4 m5_3)
(connect-left m4_4 m3_3)
(connect-left m4_4 m4_3)
(connect-behind m4_4 m5_3)
(connect-behind m4_4 m5_4)
(connect-behind m4_4 m5_5)
(connect-forward m4_4 m3_3)
(connect-forward m4_4 m3_4)
(connect-forward m4_4 m3_5)
(connect-right m4_5 m5_6)
(connect-right m4_5 m3_6)
(connect-right m4_5 m4_6)
(connect-left m4_5 m5_4)
(connect-left m4_5 m3_4)
(connect-left m4_5 m4_4)
(connect-behind m4_5 m5_4)
(connect-behind m4_5 m5_5)
(connect-behind m4_5 m5_6)
(connect-forward m4_5 m3_4)
(connect-forward m4_5 m3_5)
(connect-forward m4_5 m3_6)
(connect-right m4_6 m5_7)
(connect-right m4_6 m3_7)
(connect-right m4_6 m4_7)
(connect-left m4_6 m5_5)
(connect-left m4_6 m3_5)
(connect-left m4_6 m4_5)
(connect-behind m4_6 m5_5)
(connect-behind m4_6 m5_6)
(connect-behind m4_6 m5_7)
(connect-forward m4_6 m3_5)
(connect-forward m4_6 m3_6)
(connect-forward m4_6 m3_7)
(connect-right m4_7 m5_8)
(connect-right m4_7 m3_8)
(connect-right m4_7 m4_8)
(connect-left m4_7 m5_6)
(connect-left m4_7 m3_6)
(connect-left m4_7 m4_6)
(connect-behind m4_7 m5_6)
(connect-behind m4_7 m5_7)
(connect-behind m4_7 m5_8)
(connect-forward m4_7 m3_6)
(connect-forward m4_7 m3_7)
(connect-forward m4_7 m3_8)
(connect-right m4_8 m5_9)
(connect-right m4_8 m3_9)
(connect-right m4_8 m4_9)
(connect-left m4_8 m5_7)
(connect-left m4_8 m3_7)
(connect-left m4_8 m4_7)
(connect-behind m4_8 m5_7)
(connect-behind m4_8 m5_8)
(connect-behind m4_8 m5_9)
(connect-forward m4_8 m3_7)
(connect-forward m4_8 m3_8)
(connect-forward m4_8 m3_9)
(connect-left m4_9 m5_8)
(connect-left m4_9 m3_8)
(connect-left m4_9 m4_8)
(connect-behind m4_9 m5_8)
(connect-behind m4_9 m5_9)
(connect-forward m4_9 m3_8)
(connect-forward m4_9 m3_9)
(connect-right m5_0 m6_1)
(connect-right m5_0 m4_1)
(connect-right m5_0 m5_1)
(connect-behind m5_0 m6_0)
(connect-behind m5_0 m6_1)
(connect-forward m5_0 m4_0)
(connect-forward m5_0 m4_1)
(connect-right m5_1 m6_2)
(connect-right m5_1 m4_2)
(connect-right m5_1 m5_2)
(connect-left m5_1 m6_0)
(connect-left m5_1 m4_0)
(connect-left m5_1 m5_0)
(connect-behind m5_1 m6_0)
(connect-behind m5_1 m6_1)
(connect-behind m5_1 m6_2)
(connect-forward m5_1 m4_0)
(connect-forward m5_1 m4_1)
(connect-forward m5_1 m4_2)
(connect-right m5_2 m6_3)
(connect-right m5_2 m4_3)
(connect-right m5_2 m5_3)
(connect-left m5_2 m6_1)
(connect-left m5_2 m4_1)
(connect-left m5_2 m5_1)
(connect-behind m5_2 m6_1)
(connect-behind m5_2 m6_2)
(connect-behind m5_2 m6_3)
(connect-forward m5_2 m4_1)
(connect-forward m5_2 m4_2)
(connect-forward m5_2 m4_3)
(connect-right m5_3 m6_4)
(connect-right m5_3 m4_4)
(connect-right m5_3 m5_4)
(connect-left m5_3 m6_2)
(connect-left m5_3 m4_2)
(connect-left m5_3 m5_2)
(connect-behind m5_3 m6_2)
(connect-behind m5_3 m6_3)
(connect-behind m5_3 m6_4)
(connect-forward m5_3 m4_2)
(connect-forward m5_3 m4_3)
(connect-forward m5_3 m4_4)
(connect-right m5_4 m6_5)
(connect-right m5_4 m4_5)
(connect-right m5_4 m5_5)
(connect-left m5_4 m6_3)
(connect-left m5_4 m4_3)
(connect-left m5_4 m5_3)
(connect-behind m5_4 m6_3)
(connect-behind m5_4 m6_4)
(connect-behind m5_4 m6_5)
(connect-forward m5_4 m4_3)
(connect-forward m5_4 m4_4)
(connect-forward m5_4 m4_5)
(connect-right m5_5 m6_6)
(connect-right m5_5 m4_6)
(connect-right m5_5 m5_6)
(connect-left m5_5 m6_4)
(connect-left m5_5 m4_4)
(connect-left m5_5 m5_4)
(connect-behind m5_5 m6_4)
(connect-behind m5_5 m6_5)
(connect-behind m5_5 m6_6)
(connect-forward m5_5 m4_4)
(connect-forward m5_5 m4_5)
(connect-forward m5_5 m4_6)
(connect-right m5_6 m6_7)
(connect-right m5_6 m4_7)
(connect-right m5_6 m5_7)
(connect-left m5_6 m6_5)
(connect-left m5_6 m4_5)
(connect-left m5_6 m5_5)
(connect-behind m5_6 m6_5)
(connect-behind m5_6 m6_6)
(connect-behind m5_6 m6_7)
(connect-forward m5_6 m4_5)
(connect-forward m5_6 m4_6)
(connect-forward m5_6 m4_7)
(connect-right m5_7 m6_8)
(connect-right m5_7 m4_8)
(connect-right m5_7 m5_8)
(connect-left m5_7 m6_6)
(connect-left m5_7 m4_6)
(connect-left m5_7 m5_6)
(connect-behind m5_7 m6_6)
(connect-behind m5_7 m6_7)
(connect-behind m5_7 m6_8)
(connect-forward m5_7 m4_6)
(connect-forward m5_7 m4_7)
(connect-forward m5_7 m4_8)
(connect-right m5_8 m6_9)
(connect-right m5_8 m4_9)
(connect-right m5_8 m5_9)
(connect-left m5_8 m6_7)
(connect-left m5_8 m4_7)
(connect-left m5_8 m5_7)
(connect-behind m5_8 m6_7)
(connect-behind m5_8 m6_8)
(connect-behind m5_8 m6_9)
(connect-forward m5_8 m4_7)
(connect-forward m5_8 m4_8)
(connect-forward m5_8 m4_9)
(connect-left m5_9 m6_8)
(connect-left m5_9 m4_8)
(connect-left m5_9 m5_8)
(connect-behind m5_9 m6_8)
(connect-behind m5_9 m6_9)
(connect-forward m5_9 m4_8)
(connect-forward m5_9 m4_9)
(connect-right m6_0 m7_1)
(connect-right m6_0 m5_1)
(connect-right m6_0 m6_1)
(connect-behind m6_0 m7_0)
(connect-behind m6_0 m7_1)
(connect-forward m6_0 m5_0)
(connect-forward m6_0 m5_1)
(connect-right m6_1 m7_2)
(connect-right m6_1 m5_2)
(connect-right m6_1 m6_2)
(connect-left m6_1 m7_0)
(connect-left m6_1 m5_0)
(connect-left m6_1 m6_0)
(connect-behind m6_1 m7_0)
(connect-behind m6_1 m7_1)
(connect-behind m6_1 m7_2)
(connect-forward m6_1 m5_0)
(connect-forward m6_1 m5_1)
(connect-forward m6_1 m5_2)
(connect-right m6_2 m7_3)
(connect-right m6_2 m5_3)
(connect-right m6_2 m6_3)
(connect-left m6_2 m7_1)
(connect-left m6_2 m5_1)
(connect-left m6_2 m6_1)
(connect-behind m6_2 m7_1)
(connect-behind m6_2 m7_2)
(connect-behind m6_2 m7_3)
(connect-forward m6_2 m5_1)
(connect-forward m6_2 m5_2)
(connect-forward m6_2 m5_3)
(connect-right m6_3 m7_4)
(connect-right m6_3 m5_4)
(connect-right m6_3 m6_4)
(connect-left m6_3 m7_2)
(connect-left m6_3 m5_2)
(connect-left m6_3 m6_2)
(connect-behind m6_3 m7_2)
(connect-behind m6_3 m7_3)
(connect-behind m6_3 m7_4)
(connect-forward m6_3 m5_2)
(connect-forward m6_3 m5_3)
(connect-forward m6_3 m5_4)
(connect-right m6_4 m7_5)
(connect-right m6_4 m5_5)
(connect-right m6_4 m6_5)
(connect-left m6_4 m7_3)
(connect-left m6_4 m5_3)
(connect-left m6_4 m6_3)
(connect-behind m6_4 m7_3)
(connect-behind m6_4 m7_4)
(connect-behind m6_4 m7_5)
(connect-forward m6_4 m5_3)
(connect-forward m6_4 m5_4)
(connect-forward m6_4 m5_5)
(connect-right m6_5 m7_6)
(connect-right m6_5 m5_6)
(connect-right m6_5 m6_6)
(connect-left m6_5 m7_4)
(connect-left m6_5 m5_4)
(connect-left m6_5 m6_4)
(connect-behind m6_5 m7_4)
(connect-behind m6_5 m7_5)
(connect-behind m6_5 m7_6)
(connect-forward m6_5 m5_4)
(connect-forward m6_5 m5_5)
(connect-forward m6_5 m5_6)
(connect-right m6_6 m7_7)
(connect-right m6_6 m5_7)
(connect-right m6_6 m6_7)
(connect-left m6_6 m7_5)
(connect-left m6_6 m5_5)
(connect-left m6_6 m6_5)
(connect-behind m6_6 m7_5)
(connect-behind m6_6 m7_6)
(connect-behind m6_6 m7_7)
(connect-forward m6_6 m5_5)
(connect-forward m6_6 m5_6)
(connect-forward m6_6 m5_7)
(connect-right m6_7 m7_8)
(connect-right m6_7 m5_8)
(connect-right m6_7 m6_8)
(connect-left m6_7 m7_6)
(connect-left m6_7 m5_6)
(connect-left m6_7 m6_6)
(connect-behind m6_7 m7_6)
(connect-behind m6_7 m7_7)
(connect-behind m6_7 m7_8)
(connect-forward m6_7 m5_6)
(connect-forward m6_7 m5_7)
(connect-forward m6_7 m5_8)
(connect-right m6_8 m7_9)
(connect-right m6_8 m5_9)
(connect-right m6_8 m6_9)
(connect-left m6_8 m7_7)
(connect-left m6_8 m5_7)
(connect-left m6_8 m6_7)
(connect-behind m6_8 m7_7)
(connect-behind m6_8 m7_8)
(connect-behind m6_8 m7_9)
(connect-forward m6_8 m5_7)
(connect-forward m6_8 m5_8)
(connect-forward m6_8 m5_9)
(connect-left m6_9 m7_8)
(connect-left m6_9 m5_8)
(connect-left m6_9 m6_8)
(connect-behind m6_9 m7_8)
(connect-behind m6_9 m7_9)
(connect-forward m6_9 m5_8)
(connect-forward m6_9 m5_9)
(connect-right m7_0 m8_1)
(connect-right m7_0 m6_1)
(connect-right m7_0 m7_1)
(connect-behind m7_0 m8_0)
(connect-behind m7_0 m8_1)
(connect-forward m7_0 m6_0)
(connect-forward m7_0 m6_1)
(connect-right m7_1 m8_2)
(connect-right m7_1 m6_2)
(connect-right m7_1 m7_2)
(connect-left m7_1 m8_0)
(connect-left m7_1 m6_0)
(connect-left m7_1 m7_0)
(connect-behind m7_1 m8_0)
(connect-behind m7_1 m8_1)
(connect-behind m7_1 m8_2)
(connect-forward m7_1 m6_0)
(connect-forward m7_1 m6_1)
(connect-forward m7_1 m6_2)
(connect-right m7_2 m8_3)
(connect-right m7_2 m6_3)
(connect-right m7_2 m7_3)
(connect-left m7_2 m8_1)
(connect-left m7_2 m6_1)
(connect-left m7_2 m7_1)
(connect-behind m7_2 m8_1)
(connect-behind m7_2 m8_2)
(connect-behind m7_2 m8_3)
(connect-forward m7_2 m6_1)
(connect-forward m7_2 m6_2)
(connect-forward m7_2 m6_3)
(connect-right m7_3 m8_4)
(connect-right m7_3 m6_4)
(connect-right m7_3 m7_4)
(connect-left m7_3 m8_2)
(connect-left m7_3 m6_2)
(connect-left m7_3 m7_2)
(connect-behind m7_3 m8_2)
(connect-behind m7_3 m8_3)
(connect-behind m7_3 m8_4)
(connect-forward m7_3 m6_2)
(connect-forward m7_3 m6_3)
(connect-forward m7_3 m6_4)
(connect-right m7_4 m8_5)
(connect-right m7_4 m6_5)
(connect-right m7_4 m7_5)
(connect-left m7_4 m8_3)
(connect-left m7_4 m6_3)
(connect-left m7_4 m7_3)
(connect-behind m7_4 m8_3)
(connect-behind m7_4 m8_4)
(connect-behind m7_4 m8_5)
(connect-forward m7_4 m6_3)
(connect-forward m7_4 m6_4)
(connect-forward m7_4 m6_5)
(connect-right m7_5 m8_6)
(connect-right m7_5 m6_6)
(connect-right m7_5 m7_6)
(connect-left m7_5 m8_4)
(connect-left m7_5 m6_4)
(connect-left m7_5 m7_4)
(connect-behind m7_5 m8_4)
(connect-behind m7_5 m8_5)
(connect-behind m7_5 m8_6)
(connect-forward m7_5 m6_4)
(connect-forward m7_5 m6_5)
(connect-forward m7_5 m6_6)
(connect-right m7_6 m8_7)
(connect-right m7_6 m6_7)
(connect-right m7_6 m7_7)
(connect-left m7_6 m8_5)
(connect-left m7_6 m6_5)
(connect-left m7_6 m7_5)
(connect-behind m7_6 m8_5)
(connect-behind m7_6 m8_6)
(connect-behind m7_6 m8_7)
(connect-forward m7_6 m6_5)
(connect-forward m7_6 m6_6)
(connect-forward m7_6 m6_7)
(connect-right m7_7 m8_8)
(connect-right m7_7 m6_8)
(connect-right m7_7 m7_8)
(connect-left m7_7 m8_6)
(connect-left m7_7 m6_6)
(connect-left m7_7 m7_6)
(connect-behind m7_7 m8_6)
(connect-behind m7_7 m8_7)
(connect-behind m7_7 m8_8)
(connect-forward m7_7 m6_6)
(connect-forward m7_7 m6_7)
(connect-forward m7_7 m6_8)
(connect-right m7_8 m8_9)
(connect-right m7_8 m6_9)
(connect-right m7_8 m7_9)
(connect-left m7_8 m8_7)
(connect-left m7_8 m6_7)
(connect-left m7_8 m7_7)
(connect-behind m7_8 m8_7)
(connect-behind m7_8 m8_8)
(connect-behind m7_8 m8_9)
(connect-forward m7_8 m6_7)
(connect-forward m7_8 m6_8)
(connect-forward m7_8 m6_9)
(connect-left m7_9 m8_8)
(connect-left m7_9 m6_8)
(connect-left m7_9 m7_8)
(connect-behind m7_9 m8_8)
(connect-behind m7_9 m8_9)
(connect-forward m7_9 m6_8)
(connect-forward m7_9 m6_9)
(connect-right m8_0 m9_1)
(connect-right m8_0 m7_1)
(connect-right m8_0 m8_1)
(connect-behind m8_0 m9_0)
(connect-behind m8_0 m9_1)
(connect-forward m8_0 m7_0)
(connect-forward m8_0 m7_1)
(connect-right m8_1 m9_2)
(connect-right m8_1 m7_2)
(connect-right m8_1 m8_2)
(connect-left m8_1 m9_0)
(connect-left m8_1 m7_0)
(connect-left m8_1 m8_0)
(connect-behind m8_1 m9_0)
(connect-behind m8_1 m9_1)
(connect-behind m8_1 m9_2)
(connect-forward m8_1 m7_0)
(connect-forward m8_1 m7_1)
(connect-forward m8_1 m7_2)
(connect-right m8_2 m9_3)
(connect-right m8_2 m7_3)
(connect-right m8_2 m8_3)
(connect-left m8_2 m9_1)
(connect-left m8_2 m7_1)
(connect-left m8_2 m8_1)
(connect-behind m8_2 m9_1)
(connect-behind m8_2 m9_2)
(connect-behind m8_2 m9_3)
(connect-forward m8_2 m7_1)
(connect-forward m8_2 m7_2)
(connect-forward m8_2 m7_3)
(connect-right m8_3 m9_4)
(connect-right m8_3 m7_4)
(connect-right m8_3 m8_4)
(connect-left m8_3 m9_2)
(connect-left m8_3 m7_2)
(connect-left m8_3 m8_2)
(connect-behind m8_3 m9_2)
(connect-behind m8_3 m9_3)
(connect-behind m8_3 m9_4)
(connect-forward m8_3 m7_2)
(connect-forward m8_3 m7_3)
(connect-forward m8_3 m7_4)
(connect-right m8_4 m9_5)
(connect-right m8_4 m7_5)
(connect-right m8_4 m8_5)
(connect-left m8_4 m9_3)
(connect-left m8_4 m7_3)
(connect-left m8_4 m8_3)
(connect-behind m8_4 m9_3)
(connect-behind m8_4 m9_4)
(connect-behind m8_4 m9_5)
(connect-forward m8_4 m7_3)
(connect-forward m8_4 m7_4)
(connect-forward m8_4 m7_5)
(connect-right m8_5 m9_6)
(connect-right m8_5 m7_6)
(connect-right m8_5 m8_6)
(connect-left m8_5 m9_4)
(connect-left m8_5 m7_4)
(connect-left m8_5 m8_4)
(connect-behind m8_5 m9_4)
(connect-behind m8_5 m9_5)
(connect-behind m8_5 m9_6)
(connect-forward m8_5 m7_4)
(connect-forward m8_5 m7_5)
(connect-forward m8_5 m7_6)
(connect-right m8_6 m9_7)
(connect-right m8_6 m7_7)
(connect-right m8_6 m8_7)
(connect-left m8_6 m9_5)
(connect-left m8_6 m7_5)
(connect-left m8_6 m8_5)
(connect-behind m8_6 m9_5)
(connect-behind m8_6 m9_6)
(connect-behind m8_6 m9_7)
(connect-forward m8_6 m7_5)
(connect-forward m8_6 m7_6)
(connect-forward m8_6 m7_7)
(connect-right m8_7 m9_8)
(connect-right m8_7 m7_8)
(connect-right m8_7 m8_8)
(connect-left m8_7 m9_6)
(connect-left m8_7 m7_6)
(connect-left m8_7 m8_6)
(connect-behind m8_7 m9_6)
(connect-behind m8_7 m9_7)
(connect-behind m8_7 m9_8)
(connect-forward m8_7 m7_6)
(connect-forward m8_7 m7_7)
(connect-forward m8_7 m7_8)
(connect-right m8_8 m9_9)
(connect-right m8_8 m7_9)
(connect-right m8_8 m8_9)
(connect-left m8_8 m9_7)
(connect-left m8_8 m7_7)
(connect-left m8_8 m8_7)
(connect-behind m8_8 m9_7)
(connect-behind m8_8 m9_8)
(connect-behind m8_8 m9_9)
(connect-forward m8_8 m7_7)
(connect-forward m8_8 m7_8)
(connect-forward m8_8 m7_9)
(connect-left m8_9 m9_8)
(connect-left m8_9 m7_8)
(connect-left m8_9 m8_8)
(connect-behind m8_9 m9_8)
(connect-behind m8_9 m9_9)
(connect-forward m8_9 m7_8)
(connect-forward m8_9 m7_9)
(connect-right m9_0 m8_1)
(connect-right m9_0 m9_1)
(connect-forward m9_0 m8_0)
(connect-forward m9_0 m8_1)
(connect-right m9_1 m8_2)
(connect-right m9_1 m9_2)
(connect-left m9_1 m8_0)
(connect-left m9_1 m9_0)
(connect-forward m9_1 m8_0)
(connect-forward m9_1 m8_1)
(connect-forward m9_1 m8_2)
(connect-right m9_2 m8_3)
(connect-right m9_2 m9_3)
(connect-left m9_2 m8_1)
(connect-left m9_2 m9_1)
(connect-forward m9_2 m8_1)
(connect-forward m9_2 m8_2)
(connect-forward m9_2 m8_3)
(connect-right m9_3 m8_4)
(connect-right m9_3 m9_4)
(connect-left m9_3 m8_2)
(connect-left m9_3 m9_2)
(connect-forward m9_3 m8_2)
(connect-forward m9_3 m8_3)
(connect-forward m9_3 m8_4)
(connect-right m9_4 m8_5)
(connect-right m9_4 m9_5)
(connect-left m9_4 m8_3)
(connect-left m9_4 m9_3)
(connect-forward m9_4 m8_3)
(connect-forward m9_4 m8_4)
(connect-forward m9_4 m8_5)
(connect-right m9_5 m8_6)
(connect-right m9_5 m9_6)
(connect-left m9_5 m8_4)
(connect-left m9_5 m9_4)
(connect-forward m9_5 m8_4)
(connect-forward m9_5 m8_5)
(connect-forward m9_5 m8_6)
(connect-right m9_6 m8_7)
(connect-right m9_6 m9_7)
(connect-left m9_6 m8_5)
(connect-left m9_6 m9_5)
(connect-forward m9_6 m8_5)
(connect-forward m9_6 m8_6)
(connect-forward m9_6 m8_7)
(connect-right m9_7 m8_8)
(connect-right m9_7 m9_8)
(connect-left m9_7 m8_6)
(connect-left m9_7 m9_6)
(connect-forward m9_7 m8_6)
(connect-forward m9_7 m8_7)
(connect-forward m9_7 m8_8)
(connect-right m9_8 m8_9)
(connect-right m9_8 m9_9)
(connect-left m9_8 m8_7)
(connect-left m9_8 m9_7)
(connect-forward m9_8 m8_7)
(connect-forward m9_8 m8_8)
(connect-forward m9_8 m8_9)
(connect-left m9_9 m8_8)
(connect-left m9_9 m9_8)
(connect-forward m9_9 m8_8)
(connect-forward m9_9 m8_9)
 (player-at m0_0)
(= (player-current-health) 20)
 (= (tool-in-hand) 11)
(= (tool-id hand) 0)
(= (tool-id wood-axe) 11)
(= (tool-id bow) 10)
(= (current-harvest-duration) 0)
(= (current-harvest-location) 0)
(= (thing-available wood) 0)
(= (thing-available wood-axe) 1)
(= (thing-available bow) 1)
(= (thing-available instant-health-potion) 2)
(= (thing-available helmet) 1)
(= (thing-available chestplate) 1)
(= (thing-available bone) 0)
(= (duration-need hand tree) 1)
(= (duration-need wood-axe tree) 1)
(= (duration-need wood-axe skeleton) 1000000000)
(= (duration-need bow skeleton) 1000000000)
(= (duration-need wood-axe arrowtrap) 1000000000)
(= (duration-need wood-axe monster) 1000000000)
(= (tool-max-health hand) 1000000000)
(= (tool-max-health wood-axe) 1000000000)
(= (tool-max-health bow) 1000000000)
(= (tool-current-health hand) 1000000000)
(= (tool-current-health wood-axe) 1000000000)
(= (tool-current-health bow) 1000000000)
(= (location-id m0_0) 1)
(= (location-id m0_1) 2)
(= (location-id m0_2) 3)
(= (location-id m0_3) 4)
(= (location-id m0_4) 5)
(= (location-id m0_5) 6)
(= (location-id m0_6) 7)
(= (location-id m0_7) 8)
(= (location-id m0_8) 9)
(= (location-id m0_9) 10)
(= (location-id m1_0) 11)
(= (location-id m1_1) 12)
(= (location-id m1_2) 13)
(= (location-id m1_3) 14)
(= (location-id m1_4) 15)
(= (location-id m1_5) 16)
(= (location-id m1_6) 17)
(= (location-id m1_7) 18)
(= (location-id m1_8) 19)
(= (location-id m1_9) 20)
(= (location-id m2_0) 21)
(= (location-id m2_1) 22)
(= (location-id m2_2) 23)
(= (location-id m2_3) 24)
(= (location-id m2_4) 25)
(= (location-id m2_5) 26)
(= (location-id m2_6) 27)
(= (location-id m2_7) 28)
(= (location-id m2_8) 29)
(= (location-id m2_9) 30)
(= (location-id m3_0) 31)
(= (location-id m3_1) 32)
(= (location-id m3_2) 33)
(= (location-id m3_3) 34)
(= (location-id m3_4) 35)
(= (location-id m3_5) 36)
(= (location-id m3_6) 37)
(= (location-id m3_7) 38)
(= (location-id m3_8) 39)
(= (location-id m3_9) 40)
(= (location-id m4_0) 41)
(= (location-id m4_1) 42)
(= (location-id m4_2) 43)
(= (location-id m4_3) 44)
(= (location-id m4_4) 45)
(= (location-id m4_5) 46)
(= (location-id m4_6) 47)
(= (location-id m4_7) 48)
(= (location-id m4_8) 49)
(= (location-id m4_9) 50)
(= (location-id m5_0) 51)
(= (location-id m5_1) 52)
(= (location-id m5_2) 53)
(= (location-id m5_3) 54)
(= (location-id m5_4) 55)
(= (location-id m5_5) 56)
(= (location-id m5_6) 57)
(= (location-id m5_7) 58)
(= (location-id m5_8) 59)
(= (location-id m5_9) 60)
(= (location-id m6_0) 61)
(= (location-id m6_1) 62)
(= (location-id m6_2) 63)
(= (location-id m6_3) 64)
(= (location-id m6_4) 65)
(= (location-id m6_5) 66)
(= (location-id m6_6) 67)
(= (location-id m6_7) 68)
(= (location-id m6_8) 69)
(= (location-id m6_9) 70)
(= (location-id m7_0) 71)
(= (location-id m7_1) 72)
(= (location-id m7_2) 73)
(= (location-id m7_3) 74)
(= (location-id m7_4) 75)
(= (location-id m7_5) 76)
(= (location-id m7_6) 77)
(= (location-id m7_7) 78)
(= (location-id m7_8) 79)
(= (location-id m7_9) 80)
(= (location-id m8_0) 81)
(= (location-id m8_1) 82)
(= (location-id m8_2) 83)
(= (location-id m8_3) 84)
(= (location-id m8_4) 85)
(= (location-id m8_5) 86)
(= (location-id m8_6) 87)
(= (location-id m8_7) 88)
(= (location-id m8_8) 89)
(= (location-id m8_9) 90)
(= (location-id m9_0) 91)
(= (location-id m9_1) 92)
(= (location-id m9_2) 93)
(= (location-id m9_3) 94)
(= (location-id m9_4) 95)
(= (location-id m9_5) 96)
(= (location-id m9_6) 97)
(= (location-id m9_7) 98)
(= (location-id m9_8) 99)
(= (location-id m9_9) 100)
(thing-at-map tree  m4_6)
(thing-at-map tree  m4_3)
(thing-at-map tree  m7_1)
(thing-at-map tree  m6_1)
(thing-at-map tree  m4_7)
(thing-at skeleton)
(thing-at-loc skeleton  m7_0)
(thing-at arrowtrap)
(thing-at-loc arrowtrap  m6_0)
(thing-at-map shelter m2_0)
)(:goal
(and
(= (thing-available wood) 5)
)))