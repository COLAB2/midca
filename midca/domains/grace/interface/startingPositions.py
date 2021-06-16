from simulator import Simulator
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
np.random.seed(555)

start_pos = [(4.361675414742551382e+00, 1.458277069766090328e+01),
             (1.545820006278236569e+01, 6.457247090829543623e+00),
             (6.295868290928718913e-01, 7.231587833833630796e+00),
             (1.125840547944832792e+01, 4.275635236417141272e-01),
             (1.547922926485634232e+00, 4.900255414078060312e+00),
             (1.510704267728113237e+01, 1.783478215173388648e+01),
             (2.204755346558018303e-01, 6.173268062869025741e+00),
             (4.958469349586305697e+00, 4.716507947006558510e+00),
             (1.479491107030286301e+01, 1.459564048874953457e+01),
             (3.171471926407209985e+00, 1.005076470181938575e+01),
             (1.234584097168078287e+01, 6.461087057506860631e+00),
             (1.067132595610898349e+01, 1.024013704378120160e+01),
             (2.627585809888457469e+00, 1.260149797724160514e+01),
             (4.046709532503120599e+00, 1.067246776102524386e+01),
             (1.408113055270808722e+01, 1.423915807887335561e+01),
             (1.414334725656964054e+01, 1.920331742981397838e+01),
             (8.702805949851414979e+00, 1.927219991502269991e+01),
             (1.902459420257870448e+01, 1.459444585671132621e+01),
             (1.071787563199973192e+01, 6.014617171762742132e+00),
             (7.813659480510352751e+00, 3.109500212981364253e+00),
            (2.0, 2.0), (2.0, 6.0), (2.0, 10.0), (2.0, 14.0), (2.0, 18.0),
            (6.0, 2.0), (6.0, 6.0), (6.0, 10.0), (6.0, 14.0), (6.0, 18.0),
            (10.0, 2.0), (10.0, 6.0), (10.0, 10.0), (10.0, 14.0), (10.0, 18.0),
            (14.0, 2.0), (14.0, 6.0), (14.0, 10.0), (14.0, 14.0), (14.0, 18.0),
            (18.0, 2.0), (18.0, 6.0), (18.0, 10.0), (18.0, 14.0), (18.0, 18.0),
            (3.75, 3.75), (3.75, 7.75), (3.75, 11.75), (3.75, 15.75), (3.75, 19.75),
            (7.75, 3.75), (7.75, 7.75), (7.75, 11.75), (7.75, 15.75), (7.75, 19.75),
            (11.75, 3.75), (11.75, 7.75), (11.75, 11.75), (11.75, 15.75), (11.75, 19.75),
            (15.75, 3.75), (15.75, 7.75), (15.75, 11.75), (15.75, 15.75), (15.75, 19.75),
            (19.75, 3.75), (19.75, 7.75), (19.75, 11.75), (19.75, 15.75), (19.75, 19.75),
            (0.25, 0.25), (0.25, 4.25), (0.25, 8.25), (0.25, 12.25), (0.25, 16.25),
            (4.25, 0.25), (4.25, 4.25), (4.25, 8.25), (4.25, 12.25), (4.25, 16.25),
            (8.25, 0.25), (8.25, 4.25), (8.25, 8.25), (8.25, 12.25), (8.25, 16.25),
            (12.25, 0.25), (12.25, 4.25), (12.25, 8.25), (12.25, 12.25), (12.25, 16.25),
            (16.25, 0.25), (16.25, 4.25), (16.25, 8.25), (16.25, 12.25), (16.25, 16.25),
            (17.25, 18.25), (19.25, 3.25), (15.25, 8.25), (9.25, 10.25), (2.25, 1.25)
            ]

positions = [((each[0]/4)*80 + -141, (each[1]/4)*80 + -324) for each in start_pos]


if __name__ == "__main__":
    """
    # print positions
    simulator_coordinates = []
    simulator = Simulator(start_x=-141, start_y=76, side=80, size_m=5, size_n = 5)
    for each in positions:
        x,y = simulator.sim_to_grid(each[0],each[1])
        simulator_coordinates.append([x,y])

    first_agent_coordinates = simulator_coordinates[:]
    second_agent_coordinates = simulator_coordinates[:]
    third_agent_coordinates = simulator_coordinates[:]
    np.random.shuffle(first_agent_coordinates)
    np.random.shuffle(second_agent_coordinates)
    np.random.shuffle(third_agent_coordinates)

    percentage_goals_achieved = [
    ]

    flow_locations = [[[0,2], [2, 0]]]*100

    location_data = pd.DataFrame(
    {'grace': first_agent_coordinates,
     'franklin': second_agent_coordinates,
     'remus': third_agent_coordinates,
     'flow_locations': flow_locations
    })

    print (location_data.head(5))
    location_data.to_csv("data.csv",index=False)
    """
    results = pd.read_csv('data.csv')
    results["baseline"] = (results["baseline"]/25) * 100
    results["total_goals_delegated_agent"] = (results["total_goals_delegated_agent"]/25)*100
    print (results.head())
    results.plot(kind='bar')
    plt.xlabel("Number of Experiments")
    plt.ylabel("Percentage of goals achieved")
    plt.show()





