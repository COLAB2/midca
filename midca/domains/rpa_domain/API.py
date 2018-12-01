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


def create_coordinate(number):
    '''

    :param number:
                an integer to create a state in the format
                   COORDINATE(number)
    :return:
            append the state to the predicate and return
    :testcase:
    function call: create_coordinate(1)
    output: "COORDINATE(1)"
    '''

    state = ""
    state = state + "COORDINATE" + "(" + str(number) + ")\n"
    return state


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
            # predicate = create_coordinate(predicate,value["x"])
            # predicate = create_coordinate(predicate, value["y"])
            predicate = predicate + "atlocation(" + str(key) + "," + " " + str(value["x"]) + "," + " " + str(
                value["y"]) + ")" + "\n";

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
        # predicate = create_coordinate(predicate,value["x"])
        # predicate = create_coordinate(predicate, value["y"])
        predicate = predicate + "atsearcharea(RPA," + " " + str(value["x"]) + "," + " " + str(value["y"]) + ")" + "\n";

    return predicate


def size(items):
    '''
    :param items: contains the json {
      "X":10,
      "Y":10,
      "type":"size"
   }
    :return:
    the predicate of the coordinates
    '''
    states = ""
    x = items['X']
    y = items['Y']
    for i in range(0, x):
        states = states + create_coordinate(i)
    for i in range(0,y):
        states = states + create_coordinate(i)
    return states

def tile(items):
    '''
    :param items: json object {
            "X":9,
            "Y":9,
            "type":"tile"
         },
    :return:
            9,9
    '''
    return str(items['X']) + "," + str(items['Y'])

def base(items):
    '''
    :param items: json object {
         "location":{
            "X":9,
            "Y":9,
            "type":"tile"
         },
         "name":"base0",
         "type":"BASE"
      }
    :return: returns the predicate argument format atlocation(base0,x,y)
    '''
    state = ""
    location = ""
    if items['location']:
            function = items['location']['type'].lower()
            location = location + eval(function)(items['location'])
    if items['name']:
            state = state + "ENTITY(" + items['name'] + ")" + "\n"
            state = state + "atlocation" + "(" + items['name'] + "," + location + ')' + "\n"
    return state

def storm(items):
    '''
    :param items: json object {
         "location":{
            "X":9,
            "Y":9,
            "type":"tile"
         },
         "name":"storm",
         "type":"STORM"
      }
    :return: returns the predicate argument format atlocation(storm,x,y)
    '''
    state = ""
    location = ""
    if items['location']:
            function = items['location']['type'].lower()
            location = location + eval(function)(items['location'])
    if items['name']:
            state = state + "ENTITY(" + items['name'] + ")" + "\n"
            state = state + "atlocation" + "(" + items['name'] + "," + location + ')' + "\n"
    return state



def hiker(items):
    '''
    :param items: json object {
         "location":{
            "X":9,
            "Y":9,
            "type":"tile"
         },
         "name":"hiker0",
         "type":"HIKER"
      }
    :return: returns the predicate argument format atlocation(hiker0,x,y)
    '''
    state = ""
    location = ""
    if items['location']:
            function = items['location']['type'].lower()
            location = location + eval(function)(items['location'])
    if items['name']:
            state = state + "ENTITY(" + items['name'] + ")" + "\n"
            state = state + "atlocation" + "(" + items['name'] + "," + location + ')' + "\n"
    return state

def json_predicateargument(json_data):
    '''
    Reads the json file and converts into predicate argument format
    '''
    states = eval(json_data)
    midca_states = ""

    # an rpa has a name
    rpa_name = states['name']
    midca_states = midca_states + "ENTITY(" + rpa_name + ")" + "\n"


    if 'gridSize' in states:
        # this is the function call with type information
        function = states['gridSize']['type'].lower()
        midca_states = midca_states + eval(function)(states['gridSize'])

    # an rpa has a location too
    location_rpa = ""
    function = states['location']['type'].lower()
    location_rpa = location_rpa + eval(function)(states['location'])
    midca_states = midca_states + "atlocation(" + rpa_name + "," \
                                                + location_rpa + ")" + "\n"


    if 'knownEntities' in states:
        # iterate through all the entities
        for each_entity in states['knownEntities']:
            # execute based on type information
            function = each_entity['type'].lower()
            midca_states = midca_states + eval(function)(each_entity)

    if 'stormEffects' in states:
        # iterate through all the storms
        for each_storm in states['stormEffects']:
            # every storm has a name, outer effect and inner effect
            name = each_storm['name']
            # remove outer braces : "[lightning]" to "lightning
            outer_effect = each_storm['outerEffect'].replace("[", "")
            outer_effect = outer_effect.replace("]", "")
            # remove braces
            inner_effect = each_storm['innerEffect'].replace("[", "")
            inner_effect = inner_effect.replace("]", "")

            # storm is an entity
            midca_states = midca_states + "ENTITY(" + name + ")" + "\n"
            # outer_effects and inner_effects are EFFECTS
            midca_states = midca_states + "EFFECT(" + outer_effect + ")" + "\n"
            midca_states = midca_states + "EFFECT(" + inner_effect + ")" + "\n"
            # outer band tiles
            for each_tile in each_storm['outerBandTiles']:
                location = ""
                function = each_tile['type'].lower()
                location = location + eval(function)(each_tile)
                midca_states = midca_states + "is_affected(" + name + "," \
                                                             + outer_effect + ","\
                                                             + location + ")" + "\n"
            # inner band tiles
            for each_tile in each_storm['innerBandTiles']:
                location = ""
                function = each_tile['type'].lower()
                location = location + eval(function)(each_tile)
                midca_states = midca_states + "is_affected(" + name + "," \
                                                             + inner_effect + ","\
                                                             + location + ")" + "\n"

    if 'searchArea' in states:
        for each_tile in states['searchArea']:
            location = ""
            function = each_tile['type'].lower()
            location = location + eval(function)(each_tile)
            midca_states = midca_states + "atsearcharea(" + rpa_name + "," \
                                                          + location + ")" + "\n"
    return midca_states


#print (json_predicateargument())
