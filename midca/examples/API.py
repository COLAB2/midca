import json


def is_json(myjson):
  '''
  :param myjson: a json stucture or an invalid json structure
  :return: whether it is a json or not.
  :testcase:
  function call : is_json( { "location": {"x":9,"y":8} } )
  output: true

  function call: is_json([{ "location": {"x":9,"y":8} }])
  output: false
  '''
  try:
    json_object = json.loads(myjson)
  except ValueError, e:
    return False
  return True

def create_coordinate(predicate,number):
    '''
    :param predicate:
                String to append states
    :param number:
                an integer to create a state in the format
                   COORDINATE(number)
    :return:
            append the state to the predicate and return
    :testcase:
    function call: create_coordinate("",1)
    output: "COORDINATE(1)"
    '''
    predicate = predicate + "COORDINATE" + "(" + str(number) + ")\n"
    return predicate

def create_entity(predicate, states):
    '''
    :param predicate:
                String to append states
    :param states:
                a list of json entities
    :return:
            append the state to the predicate and return
    :testcase:
    function call: create_entity("",[{"hiker":{"x":3,"y":4}})
    output: "hiker(3,4)"
    '''
    for each in states:
        for key, value in each.iteritems():
            #predicate = create_coordinate(predicate,value["x"])
            #predicate = create_coordinate(predicate, value["y"])
            predicate = predicate + "atlocation(" + str(key) + "," + " " + str(value["x"]) + "," + " " + str(value["y"]) + ")" + "\n";

    return predicate


def create_searchlocation(predicate, states):
    '''
    :param predicate:
                String to append states
    :param states:
                a list of json entities
    :return:
            append the state to the predicate and return
    :testcase:
    function call: create_entity("",[{"x":3,"y":4}])
    output: "atsearcharea(RPA,3,4)"
    '''
    for value in states:
            #predicate = create_coordinate(predicate,value["x"])
            #predicate = create_coordinate(predicate, value["y"])
            predicate = predicate + "atsearcharea(RPA," + " " + str(value["x"]) + "," + " " + str(value["y"]) + ")" + "\n";

    return predicate



def json_predicateargument():
    '''
    Reads the json file and converts into predicate argument format
    '''
    with open('/home/sampath/Documents/rpa.json') as json_data:
        states = json.load(json_data)
        predicate = ""
        for each in range(0,10):
            predicate = create_coordinate(predicate,each)

        if 'knownEntities' in states:
            predicate = create_entity(predicate, states['knownEntities'])

        if 'location' in states:
            location = { "RPA" : states['location'] }
            predicate = create_entity(predicate, [ { "RPA" : states['location'] } ] )

        if 'searchArea' in states:
            predicate = create_searchlocation(predicate, states['searchArea'])
	
        return predicate


print (json_predicateargument())
