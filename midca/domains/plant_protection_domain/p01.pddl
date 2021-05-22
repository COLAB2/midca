(define 
(problem wood)
(:domain minecraft-beta)
(:objects
pos0_0 - mapgrid
pos0_1 - mapgrid
pos0_2 - mapgrid
pos0_3 - mapgrid
pos0_4 - mapgrid
pos0_5 - mapgrid
pos1_0 - mapgrid
pos1_1 - mapgrid
pos1_2 - mapgrid
pos1_3 - mapgrid
pos1_4 - mapgrid
pos1_5 - mapgrid
pos2_0 - mapgrid
pos2_1 - mapgrid
pos2_2 - mapgrid
pos2_3 - mapgrid
pos2_4 - mapgrid
pos2_5 - mapgrid
pos3_0 - mapgrid
pos3_1 - mapgrid
pos3_2 - mapgrid
pos3_3 - mapgrid
pos3_4 - mapgrid
pos3_5 - mapgrid
pos4_0 - mapgrid
pos4_1 - mapgrid
pos4_2 - mapgrid
pos4_3 - mapgrid
pos4_4 - mapgrid
pos4_5 - mapgrid
pos5_0 - mapgrid
pos5_1 - mapgrid
pos5_2 - mapgrid
pos5_3 - mapgrid
pos5_4 - mapgrid
pos5_5 - mapgrid
)
(:init
(blocked pos1_4 )
(ISADJACENT pos3_2 pos3_1 )
(ISADJACENT pos1_4 pos2_4 )
(NEXTSPAWN pos2_3 pos2_4 )
(ISADJACENT pos1_3 pos2_3 )
(ISADJACENT pos5_5 pos5_4 )
(NEXTSPAWN pos1_3 pos0_5 )
(ispoint pos5_4 )
(ISADJACENT pos4_3 pos3_3 )
(ISADJACENT pos4_5 pos5_5 )
(ISADJACENT pos0_3 pos0_4 )
(ISADJACENT pos3_1 pos3_2 )
(blocked pos0_4 )
(ISADJACENT pos5_2 pos5_1 )
(ISADJACENT pos4_2 pos4_3 )
(ISADJACENT pos5_5 pos4_5 )
(ISADJACENT pos5_3 pos4_3 )
(ISADJACENT pos2_2 pos2_1 )
(nextsnake pos0_4 pos1_4 )
(ISADJACENT pos0_1 pos1_1 )
(ISADJACENT pos3_1 pos2_1 )
(ISADJACENT pos3_0 pos4_0 )
(NEXTSPAWN pos2_5 pos2_3 )
(ISADJACENT pos3_5 pos2_5 )
(ISADJACENT pos4_0 pos5_0 )
(ISADJACENT pos2_4 pos2_3 )
(ISADJACENT pos4_4 pos4_3 )
(ISADJACENT pos5_1 pos4_1 )
(ISADJACENT pos2_0 pos3_0 )
(ISADJACENT pos2_3 pos3_3 )
(ISADJACENT pos4_1 pos4_2 )
(NEXTSPAWN pos3_2 pos0_1 )
(ISADJACENT pos4_3 pos4_2 )
(NEXTSPAWN pos0_5 pos3_3 )
(ISADJACENT pos2_0 pos1_0 )
(ISADJACENT pos2_1 pos3_1 )
(ISADJACENT pos3_3 pos3_2 )
(ISADJACENT pos1_0 pos0_0 )
(ISADJACENT pos2_1 pos2_0 )
(ispoint pos2_1 )
(ISADJACENT pos5_2 pos5_3 )
(ISADJACENT pos4_5 pos4_4 )
(ISADJACENT pos3_2 pos2_2 )
(ispoint pos0_2 )
(ISADJACENT pos1_3 pos1_4 )
(ISADJACENT pos3_3 pos3_4 )
(ISADJACENT pos1_0 pos2_0 )
(ISADJACENT pos3_2 pos3_3 )
(ISADJACENT pos1_2 pos2_2 )
(ISADJACENT pos4_3 pos4_4 )
(ISADJACENT pos1_0 pos1_1 )
(ISADJACENT pos4_4 pos5_4 )
(ISADJACENT pos3_1 pos4_1 )
(ISADJACENT pos3_0 pos3_1 )
(ISADJACENT pos4_4 pos4_5 )
(ISADJACENT pos0_0 pos1_0 )
(ISADJACENT pos3_5 pos3_4 )
(ISADJACENT pos5_4 pos5_3 )
(ISADJACENT pos1_1 pos1_0 )
(ISADJACENT pos4_1 pos5_1 )
(ISADJACENT pos4_2 pos4_1 )
(ISADJACENT pos4_2 pos3_2 )
(ISADJACENT pos2_2 pos3_2 )
(ISADJACENT pos2_3 pos2_2 )
(NEXTSPAWN pos2_4 pos3_2 )
(ISADJACENT pos5_2 pos4_2 )
(ISADJACENT pos5_3 pos5_4 )
(ISADJACENT pos4_1 pos4_0 )
(ISADJACENT pos4_5 pos3_5 )
(ISADJACENT pos0_1 pos0_2 )
(ISADJACENT pos0_5 pos1_5 )
(ISADJACENT pos3_5 pos4_5 )
(ISADJACENT pos3_2 pos4_2 )
(ISADJACENT pos0_4 pos0_5 )
(ISADJACENT pos2_3 pos1_3 )
(ispoint pos5_0 )
(headsnake pos0_4 )
(ISADJACENT pos5_3 pos5_2 )
(ISADJACENT pos5_0 pos5_1 )
(ISADJACENT pos2_1 pos2_2 )
(ISADJACENT pos2_0 pos2_1 )
(ISADJACENT pos3_1 pos3_0 )
(ISADJACENT pos0_3 pos0_2 )
(ISADJACENT pos4_0 pos3_0 )
(ISADJACENT pos1_1 pos0_1 )
(ISADJACENT pos3_0 pos2_0 )
(ISADJACENT pos2_2 pos2_3 )
(ISADJACENT pos4_3 pos5_3 )
(NEXTSPAWN pos3_4 pos4_5 )
(ISADJACENT pos2_5 pos2_4 )
(ISADJACENT pos2_4 pos3_4 )
(ISADJACENT pos3_3 pos2_3 )
(ISADJACENT pos1_2 pos1_3 )
(ISADJACENT pos0_4 pos1_4 )
(ISADJACENT pos2_4 pos2_5 )
(ISADJACENT pos3_4 pos2_4 )
(ISADJACENT pos5_0 pos4_0 )
(ISADJACENT pos1_4 pos0_4 )
(ISADJACENT pos2_3 pos2_4 )
(ISADJACENT pos0_0 pos0_1 )
(ISADJACENT pos1_5 pos0_5 )
(ISADJACENT pos5_1 pos5_2 )
(ISADJACENT pos3_4 pos3_3 )
(ISADJACENT pos4_2 pos5_2 )
(ISADJACENT pos3_4 pos4_4 )
(ISADJACENT pos4_0 pos4_1 )
(ISADJACENT pos2_5 pos3_5 )
(ISADJACENT pos2_2 pos1_2 )
(ISADJACENT pos4_4 pos3_4 )
(ISADJACENT pos0_3 pos1_3 )
(NEXTSPAWN pos1_2 pos3_0 )
(ISADJACENT pos1_3 pos1_2 )
(ISADJACENT pos5_1 pos5_0 )
(ISADJACENT pos1_5 pos2_5 )
(ISADJACENT pos0_4 pos0_3 )
(NEXTSPAWN pos4_5 pos1_3 )
(ISADJACENT pos1_1 pos2_1 )
(ISADJACENT pos1_2 pos1_1 )
(ISADJACENT pos3_4 pos3_5 )
(NEXTSPAWN pos3_3 pos1_2 )
(spawn pos3_4 )
(ISADJACENT pos0_5 pos0_4 )
(ISADJACENT pos0_2 pos0_1 )
(ISADJACENT pos5_4 pos4_4 )
(NEXTSPAWN pos3_0 pos2_5 )
(ISADJACENT pos2_1 pos1_1 )
(ispoint pos1_0 )
(ISADJACENT pos2_5 pos1_5 )
(ISADJACENT pos0_2 pos1_2 )
(ISADJACENT pos0_2 pos0_3 )
(ISADJACENT pos0_1 pos0_0 )
(ISADJACENT pos4_1 pos3_1 )
(ISADJACENT pos1_4 pos1_3 )
(ISADJACENT pos5_4 pos5_5 )
(ISADJACENT pos1_5 pos1_4 )
(ISADJACENT pos1_4 pos1_5 )
(ISADJACENT pos2_4 pos1_4 )
(ISADJACENT pos1_1 pos1_2 )
(ISADJACENT pos1_2 pos0_2 )
(tailsnake pos1_4 )
(ISADJACENT pos1_3 pos0_3 )
(ISADJACENT pos3_3 pos4_3 )
)
 (:goal
(and
(headsnake pos1_2)
)
))