#!/bin/bash

python run.py --withoutagents  &
python midca_structured_search_grace1.py > terminal_output/grace.txt &
python midca_structured_search_franklin.py > terminal_output/franklin.txt &
python midca_structured_search_remus.py > terminal_output/remus.txt
