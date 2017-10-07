import copy
from midca.domains.blocksworld.block import Block

class Scene:
    blocks = []         # contains all blocks

    def __init__(self, blocks):
        self.blocks = blocks

    def draw(self):
        print self.layers_to_s(self._makelayers())
    
    def __str__(self):
        return self.layers_to_s(self._makelayers())

    def _makelayers(self):
        blockscopy = copy.deepcopy(self.blocks)

        layers = []         # each new layer will be appended to layers
        currentlayer = []   # layer currently being prepared

        # find table
        for i in range(len(blockscopy)):
            if blockscopy[i].type == Block.TABLE:
                block = blockscopy.pop(i)
                currentlayer.append(block)
                layers.append(currentlayer)
                break

        # while there are more blocks above the current layer
        while len(blockscopy) > 0:
            nextlayer = []

            #print len(currentlayer), len(blockscopy)
            for i in range(len(currentlayer)):
                for j in range(len(blockscopy)):
                    if blockscopy[j].on == currentlayer[i]:
                        nextlayer.append(blockscopy[j])

            for i in range(len(nextlayer)):
                blockscopy.remove(nextlayer[i])

            currentlayer = nextlayer
            layers.append(currentlayer)
            if not currentlayer:
                break

        return layers

    # layers - as made by _makelayers
    def layers_to_s(self, layers):
        assert len(layers[0]) == 1 and layers[0][0].type == Block.TABLE

        width = len(layers[1])

        asciisquares = []
        asciisquares.append([("--------------", layers[0][0]) for tv in range(width + 1)])

        for i in range(1, len(layers), 1):
            asciisquares.append([("", None) for tv in range(width)])
            asciisquares.append([("", None) for tv in range(width)])
            asciisquares.append([("", None) for tv in range(width)])

            for j in range(width):
                written = False
                for k in range(len(layers[i])):
                    if layers[i][k].on == asciisquares[-4][j][1]:
                        written = True
                        if layers[i][k].type == Block.TRIANGLE:
                        	if layers[i][k].onfire:
                        		asciisquares[-1][j] = ("          / \\  ", layers[i][k])
                        		asciisquares[-2][j] = ("         / * \\ ", layers[i][k])
                        		asciisquares[-3][j] = ("        / *" + str(layers[i][k].id) + "*\\", layers[i][k])
                        	else:
                        		asciisquares[-1][j] = ("          / \\  ", layers[i][k])
                        		asciisquares[-2][j] = ("         /   \\ ", layers[i][k])
                        		asciisquares[-3][j] = ("        /  " + str(layers[i][k].id) + " \\", layers[i][k])
                        elif layers[i][k].type == Block.SQUARE:
                            if layers[i][k].onfire and layers[i][k].hasmortar:
                                asciisquares[-1][j] = ("        +++++++", layers[i][k])
                                asciisquares[-2][j] = ("       | *" + str(layers[i][k].id) + "* |", layers[i][k])
                                asciisquares[-3][j] = ("        -------", layers[i][k])
                            elif layers[i][k].onfire:
                                asciisquares[-1][j] = ("        -------", layers[i][k])
                                asciisquares[-2][j] = ("       | *" + str(layers[i][k].id) + "* |", layers[i][k])
                                asciisquares[-3][j] = ("        -------", layers[i][k])
                            elif layers[i][k].hasmortar:
                                asciisquares[-1][j] = ("        +++++++", layers[i][k])
                                asciisquares[-2][j] = ("       |  " + str(layers[i][k].id) + "  |", layers[i][k])
                                asciisquares[-3][j] = ("        -------", layers[i][k])        
                            else:
                                asciisquares[-1][j] = ("        -------", layers[i][k])
                                asciisquares[-2][j] = ("       |  " + str(layers[i][k].id) + "  |", layers[i][k])
                                asciisquares[-3][j] = ("        -------", layers[i][k])
                        layers[i].pop(k)
                        break
                if not written:
                    asciisquares[-1][j] = ("              ", None)
                    asciisquares[-2][j] = ("              ", None)
                    asciisquares[-3][j] = ("              ", None)

        toprint = ""
        for i in range(len(asciisquares)-1, -1, -1):
            for segment in asciisquares[i]:
                toprint += segment[0]
            toprint += "\n"

        return toprint
    

    