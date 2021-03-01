## Traveling Salesman Problem in OpenAMASE

Quickstart

1. Run the OpenAMASE taskallocator.sh example


    cd OpenAMASE/run/linux/Examples
    sh TaskAllocator.sh

2. Once AMASE opens, go to the menu and select File > Open Scenario


    Navigate to 'example scenarios' folder and choose 'Example4_Assigning Task.xml'


3. Run the MIDCA TSP Demo Script


    cd midca/
    source .env/bin/activate
    python midca/examples/tsp_amase_demo.py

You should notice that AMASE now shows two uavs and two new paths.

4. Click the 'Play' button (right arrow symbol) to run the agents. You'll see output from the MIDCA example that shows it's sending and receiving information to AMASE.

