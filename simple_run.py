from examples.predicateworld import UserGoalsMidca
MIDCA_ROOT = "/Users/swordofmorning/Documents/_programming/repos/MIDCA/"

myMidca = UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/defstate.sim")

myMidca.init()
myMidca.run()
