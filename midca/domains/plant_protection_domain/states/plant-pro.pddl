(define 
(problem wood)
(:domain minecraft-beta)
(:objects
pos0-0 - mapgrid
pos0-1 - mapgrid
pos0-2 - mapgrid
pos0-3 - mapgrid
pos0-4 - mapgrid
pos0-5 - mapgrid
pos1-0 - mapgrid
pos1-1 - mapgrid
pos1-2 - mapgrid
pos1-3 - mapgrid
pos1-4 - mapgrid
pos1-5 - mapgrid
pos2-0 - mapgrid
pos2-1 - mapgrid
pos2-2 - mapgrid
pos2-3 - mapgrid
pos2-4 - mapgrid
pos2-5 - mapgrid
pos3-0 - mapgrid
pos3-1 - mapgrid
pos3-2 - mapgrid
pos3-3 - mapgrid
pos3-4 - mapgrid
pos3-5 - mapgrid
pos4-0 - mapgrid
pos4-1 - mapgrid
pos4-2 - mapgrid
pos4-3 - mapgrid
pos4-4 - mapgrid
pos4-5 - mapgrid
pos5-0 - mapgrid
pos5-1 - mapgrid
pos5-2 - mapgrid
pos5-3 - mapgrid
pos5-4 - mapgrid
pos5-5 - mapgrid
)
(:init
(ISADJACENT pos4-0 pos5-0 )
(ISADJACENT pos0-4 pos0-5 )
(ISADJACENT pos1-3 pos2-3 )
(ISADJACENT pos2-2 pos2-1 )
(ISADJACENT pos1-0 pos1-1 )
(ISADJACENT pos3-4 pos3-5 )
(ISADJACENT pos2-4 pos2-5 )
(ISADJACENT pos2-1 pos2-2 )
(ISADJACENT pos3-0 pos4-0 )
(ISADJACENT pos2-2 pos1-2 )
(ISADJACENT pos3-0 pos3-1 )
(ISADJACENT pos5-0 pos5-1 )
(ISADJACENT pos3-1 pos3-0 )
(ISADJACENT pos5-3 pos4-3 )
(ISADJACENT pos1-4 pos0-4 )
(ISADJACENT pos3-5 pos2-5 )
(ISADJACENT pos1-1 pos1-2 )
(ISADJACENT pos1-1 pos0-1 )
(ISADJACENT pos1-3 pos0-3 )
(ISADJACENT pos2-3 pos2-4 )
(ISADJACENT pos2-5 pos3-5 )
(ISADJACENT pos1-4 pos2-4 )
(ISADJACENT pos3-5 pos3-4 )
(ISADJACENT pos5-2 pos5-3 )
(ISADJACENT pos5-2 pos4-2 )
(ISADJACENT pos5-3 pos5-2 )
(ISADJACENT pos4-0 pos4-1 )
(ISADJACENT pos4-4 pos3-4 )
(ISADJACENT pos4-5 pos3-5 )
(ISADJACENT pos4-1 pos3-1 )
(ISADJACENT pos4-5 pos4-4 )
(ISADJACENT pos0-1 pos1-1 )
(ISADJACENT pos3-2 pos4-2 )
(ISADJACENT pos2-1 pos2-0 )
(ISADJACENT pos4-1 pos4-2 )
(ISADJACENT pos2-5 pos2-4 )
(ISADJACENT pos2-5 pos1-5 )
(ISADJACENT pos3-3 pos3-2 )
(ISADJACENT pos1-5 pos1-4 )
(ISADJACENT pos1-3 pos1-4 )
(ISADJACENT pos3-1 pos3-2 )
(ISADJACENT pos5-3 pos5-4 )
(ISADJACENT pos2-2 pos3-2 )
(ISADJACENT pos4-5 pos5-5 )
(ISADJACENT pos1-0 pos0-0 )
(ISADJACENT pos3-3 pos2-3 )
(ISADJACENT pos0-4 pos1-4 )
(ISADJACENT pos2-4 pos3-4 )
(ISADJACENT pos4-3 pos3-3 )
(ISADJACENT pos3-1 pos2-1 )
(ISADJACENT pos0-0 pos0-1 )
(ISADJACENT pos4-4 pos4-5 )
(ISADJACENT pos5-4 pos4-4 )
(Invasive-at pos5-1 )
(ISADJACENT pos1-2 pos2-2 )
(ISADJACENT pos3-5 pos4-5 )
(ISADJACENT pos1-4 pos1-5 )
(ISADJACENT pos0-1 pos0-0 )
(ISADJACENT pos1-2 pos0-2 )
(agent-at pos0-1 )
(ISADJACENT pos1-2 pos1-1 )
(ISADJACENT pos1-1 pos1-0 )
(ISADJACENT pos0-1 pos0-2 )
(ISADJACENT pos4-2 pos5-2 )
(ISADJACENT pos5-1 pos4-1 )
(ISADJACENT pos5-0 pos4-0 )
(ISADJACENT pos5-1 pos5-0 )
(ISADJACENT pos4-3 pos5-3 )
(ISADJACENT pos5-5 pos4-5 )
(ISADJACENT pos3-2 pos2-2 )
(ISADJACENT pos0-2 pos1-2 )
(ISADJACENT pos2-1 pos1-1 )
(ISADJACENT pos4-1 pos4-0 )
(ISADJACENT pos5-4 pos5-3 )
(ISADJACENT pos1-5 pos0-5 )
(ISADJACENT pos1-1 pos2-1 )
(ISADJACENT pos2-0 pos3-0 )
(ISADJACENT pos0-4 pos0-3 )
(ISADJACENT pos4-4 pos4-3 )
(ISADJACENT pos5-5 pos5-4 )
(ISADJACENT pos1-0 pos2-0 )
(ISADJACENT pos2-4 pos1-4 )
(Invasive-at pos2-2 )
(ISADJACENT pos4-2 pos4-1 )
(ISADJACENT pos2-0 pos1-0 )
(ISADJACENT pos5-2 pos5-1 )
(ISADJACENT pos4-2 pos4-3 )
(ISADJACENT pos4-3 pos4-2 )
(ISADJACENT pos1-3 pos1-2 )
(ISADJACENT pos0-2 pos0-3 )
(ISADJACENT pos2-3 pos2-2 )
(ISADJACENT pos3-3 pos3-4 )
(ISADJACENT pos1-2 pos1-3 )
(ISADJACENT pos4-0 pos3-0 )
(ISADJACENT pos0-2 pos0-1 )
(ISADJACENT pos3-4 pos3-3 )
(ISADJACENT pos3-4 pos4-4 )
(ISADJACENT pos3-2 pos3-3 )
(ISADJACENT pos2-1 pos3-1 )
(ISADJACENT pos1-4 pos1-3 )
(ISADJACENT pos5-4 pos5-5 )
(ISADJACENT pos3-4 pos2-4 )
(ISADJACENT pos3-0 pos2-0 )
(ISADJACENT pos2-4 pos2-3 )
(ISADJACENT pos3-1 pos4-1 )
(ISADJACENT pos0-3 pos0-4 )
(ISADJACENT pos2-0 pos2-1 )
(ISADJACENT pos0-0 pos1-0 )
(ISADJACENT pos2-3 pos1-3 )
(ISADJACENT pos2-3 pos3-3 )
(ISADJACENT pos3-2 pos3-1 )
(Endangered-at pos2-1 )
(ISADJACENT pos0-3 pos1-3 )
(ISADJACENT pos4-3 pos4-4 )
(ISADJACENT pos0-3 pos0-2 )
(ISADJACENT pos4-4 pos5-4 )
(ISADJACENT pos5-1 pos5-2 )
(ISADJACENT pos2-2 pos2-3 )
(ISADJACENT pos0-5 pos0-4 )
(ISADJACENT pos1-5 pos2-5 )
(ISADJACENT pos4-1 pos5-1 )
(ISADJACENT pos3-3 pos4-3 )
(ISADJACENT pos0-5 pos1-5 )
(ISADJACENT pos4-2 pos3-2 )
)
 (:goal
(and
(not(Invasive-at pos2-2))
)
))