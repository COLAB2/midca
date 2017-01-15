import sys
#import pydot
import random
from frame import Frame
from frame import Role

# given input, creates an xpnet XP
class Parser:
    def __init__(self):
        self.text = None

    # make graph of interconnected frames
    def makeframegraph(self, text):
        frames = {} # key is frame name
        lines = text.split("\n")

        # make frames
        centerflag = True
        for l in lines:
            if l[0:14] == "For the frame ":
                framename = l[14:-1]
                if not framename in frames:
                    frames[framename] = Frame(framename, isstate=False, iscenter=centerflag)
                    centerflag = False

            if l[0:20] == "For the state frame ":
                framename = l[20:-1]
                if not framename in frames:
                    frames[framename] = Frame(framename, isstate=True, iscenter=centerflag)
                    centerflag = False

        # establish frame relations
        curframe = None
        for l_index in range(len(lines)):
            l = lines[l_index]
            if len(l) > 14 and l[0:14] == "For the frame ":
                curframe = l[14:-1]
            if len(l) > 20 and l[0:20] == "For the state frame ":
                curframe = l[20:-1]
            words = l.split()
            if len(words) >= 8 and words[0] == "The" and words[2] == "facet":
                facettype = words[1]
                roletype = words[5]

                # obtain the frame names constituing the facet's value
                facetvalues = []
                if words[8][0] != "(":
                    facetvalues = [words[8][:-1]]
                else:
                    for w in words[8:]:
                        w = w.replace("(","")
                        w = w.replace(").","")
                        facetvalues.append(w)
                if words[-1][-2:] != ").":
                    i = 1
                    while len(lines) > 38 and lines[l_index + i][0:38] == "                                      ":
                        addwords = lines[l_index + i].split()
                        for w in addwords:
                            w = w.replace("(","")
                            w = w.replace(").","")
                            facetvalues.append(w)
                        i += 1

                if not roletype in frames[curframe].roles:
                    frames[curframe].roles[roletype] = Role()
                if facettype == "VALUE":
                    for framename in facetvalues:
                        if not frames[framename] in frames[curframe].roles[roletype].facetvalue:
                            frames[curframe].roles[roletype].facetvalue.append(frames[framename])
                elif facettype == "RELATION":
                    for framename in facetvalues:
                        if not frames[framename] in frames[curframe].roles[roletype].facetrelation:
                            frames[curframe].roles[roletype].facetrelation.append(frames[framename])
                else:
                    print "Unrecognized facet type!"
                    print "    " + facettype
                    sys.exit(1)
        return frames

    def displayframesascii(self, frames):
        print "All frame names:"
        for k in frames.keys():
            print "    " + k

        for k in frames.keys():
            print "For frame: " + k
            for j in frames[k].roles.keys():
                print "    " + j + ":    "
                print "        facetvalues:"
                for i in frames[k].roles[j].facetvalue:
                    print "            " + i.name
                print "        facetrelations:"
                for i in frames[k].roles[j].facetrelation:
                    print "            " + i.name

    # Make simple undirected graph, no labels on edges, save in file frame_graph_simple.png
#    def displayframesgraphsimple(self, frames, filename='frame_graph_simple.png'):
#        graph = pydot.Dot(graph_type='graph')
#        for k in frames.keys():
#            for j in frames[k].roles.keys():
#                for i in frames[k].roles[j].facetvalue:
#                    graph.add_edge(pydot.Edge(k,i.name))
#                for i in frames[k].roles[j].facetrelation:
#                    graph.add_edge(pydot.Edge(k,i.name))
#        graph.write_png(filename)
#
#    # Make graph, save to file frame_graph_color.png, colored to make reading easier, directed, labeled edges
#    def displayframesgraph(self, frames, filename="frame_graph_color.png"):
#        graph = pydot.Dot(graph_type='digraph')
#        nodes = {}
#        nodecolors = {}
#
#        # Make and add nodes
#        for k in frames.keys():
#            color = self._giverandomlightcolor()
#            node = pydot.Node(k, style="filled", fillcolor=color)
#            if not frames[k].isstate:
#                node = pydot.Node(k, style="filled", shape='box', fillcolor=color)
#            nodecolors[k] = color
#            nodes[k] = node
#            graph.add_node(node)
#
#        # Make and add edges
#        for k in frames.keys():
#            for j in frames[k].roles.keys():
#                for i in frames[k].roles[j].facetvalue:
#                    graph.add_edge(pydot.Edge(nodes[k], nodes[i.name], label=j.replace(":"," ").replace("-"," "), fontcolor=nodecolors[k], fontsize="10.0", color=nodecolors[k]))
#                for i in frames[k].roles[j].facetrelation:
#                    graph.add_edge(pydot.Edge(nodes[k], nodes[i.name], label=j.replace(":"," ").replace("-"," "), fontcolor=nodecolors[k], fontsize="10.0", color=nodecolors[k]))
#
#        # Save
#        graph.write_png(filename)
#
#    def _giverandomlightcolor(self):
#        color = ""
#        for i in range(3):
#            color += hex(random.randint(40,200))[2:]
#        return "#" + color
#
#if __name__ == "__main__":
#    # get text
#    f = open("../output.txt", "r")
#    text = f.read()
#    f.close()
#
#    # parse text
#    p = Parser()
#    frames = p.makeframegraph(text)
#
#    # display
#    p.displayframesgraph(frames)
#
#
