from midca import base
import os,inspect

#thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#MIDCA_ROOT = thisDir + "/../"
#DATA_FILENAME = MIDCA_ROOT + "metamodules/sas_plan"
#original_file = "/home/zahid/Downloads//foil-python-master/src/trimlogic/test/FamilyTreeTestCase.py"
#learning_file = "/home/zahid/Downloads//foil-python-master/src/trimlogic/test/FamilyTreeTestCase_1.py"

class MRSimpleMonitor(base.BaseModule):

    

    def run(self, cycle, verbose = 2):
        print("*********************IN META MONITOR*******************")
        self.verbose = verbose
        self.mem.set(self.mem.TRACE_SEGMENT, [self.mem.trace.get_current_phase(),self.get_last_phase()])
        
        if self.verbose >= 1: print("    Retrieved last segment of trace") #+str(self.mem.get(self.mem.TRACE_SEGMENT)))
        if self.verbose >= 1: print("  "+str(self.mem.get(self.mem.TRACE_SEGMENT)))

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.mem.trace.get_data(self.mem.trace.get_current_cycle(), self.mem.trace.get_current_phase())



class MRSimpleMonitor_plant:
    def __init__(self):
        self.native = nat
        self.invasive = inv
        pass
    def init(self, world, mem):
        self.world = world
        self.mem = mem

    def run(self, cycle, verbose=2):
        #print("*********************IN META MONITOR*******************")
        self.verbose = verbose
        self.mem.set(self.mem.TRACE_SEGMENT, [self.mem.trace.get_current_phase(), self.get_last_phase()])
        #print(self.mem.get(self.mem.TRACE_SEGMENT))
        if (self.mem.DISCREPANCY is True ):

            nat = []
            inv = []
            f = open(DATA_FILENAME, 'a')
            for x in self.mem.WORLD_CHANGES:
                #print('hi',str(x))
                if(str(x)[:5] == 'invas'):
                    inv.append(int(str(x)[15]))
                    inv.append(int(str(x)[17]))
                elif(str(x)[:5] == 'nativ'):
                    nat.append(int(str(x)[13]))
                    nat.append(int(str(x)[15]))
                f.write(str(x))
            f.write("\n")


            spr=[]
            x = str(self.mem.get(self.mem.ACTIONS)[-1][0])
            if(x[:6] == 'spray('):
                spr.append(x[9:12])
            else:
                spr.append(x[10:13])
            #print(nat,inv,spr)
            f.write(x)

            f.write("\n")
            #f.write(str(self.mem.get(self.mem.ACTIONS)[-1]))
            f.write(str(self.mem.get(self.mem.STATES)[-2]))
            f.write("\n")
            f.close()
            nat = [nat[i:i + 2] for i in range(0, len(nat), 2)]

            inv = [inv[i:i + 2] for i in range(0, len(inv), 2)]
            nat_t2 = [item for item in self.native if item not in nat]

            inv_t2 = [item for item in self.invasive if item not in inv]
            #print(spr[0][0],spr[0][2])
            try:
                for x,y in inv_t2:
                    if x == int(spr[0][0]) and y == int(spr[0][2]):
                        inv_t2.remove([x,y])
            except:
                pass

            inv1,nat1,spr1,pos,spr2=[],[],[],[],[]
            a=''
            for i in nat:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t2'
                pos.append(a)
            for j in inv:
                a = 'p.p' + str(j[0]) + '_' + str(j[1]) + ',' + 't.t2'
                pos.append(a)
            for i in self.invasive:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t1'
                inv1.append(a)
            for i in self.native:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t1'
                nat1.append(a)
            for i in nat_t2:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t2'
                nat1.append(a)
            for i in inv_t2:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t2'
                inv1.append(a)
            '''for i in self.native:
                a = 'p.p' + str(i[0]) + '_' + str(i[1]) + ',' + 't.t1'
                nat1.append(a)'''
            for i in spr:
                a = 'p.p' + str(i[0]) + '_' + str(i[2]) + ',' + 't.t1'
                spr1.append(a)
                a = 'p.p' + str(i[0]) + '_' + str(i[2]) + ',' + 't.t2'
                spr2.append(a)

            #adding current state to the learning file
            inputfile = open(original_file, 'r').readlines()
            write_file = open(learning_file, 'w')
            for line in inputfile:
                #print(line)
                if 'map(self.plant.add_rule,' in line:
                    line1= '    map(self.plant.add_rule,['
                    write_file.write(line1)

                    for i in range(len(nat1)):
                        #print(str(i))
                        write_file.write('(')
                        write_file.write(str(nat1[i]))
                        write_file.write(')')
                        write_file.write(',')
                    for i in range(len(inv1)):
                        #print(str(i))
                        write_file.write('(')
                        write_file.write(str(inv1[i]))
                        write_file.write(')')
                        if(int(i)+1 != len(inv1)):
                            write_file.write(',')
                    write_file.write('])')
                    write_file.write('\n')
                elif 'positive_tuples' in line[:20]:
                    line1 = '    positive_tuples=['
                    write_file.write(line1)
                    for i in range(len(pos)):
                        #print(str(i))
                        write_file.write('[')
                        write_file.write(str(pos[i]))
                        write_file.write(']')
                        if (int(i) + 1 != len(pos)):
                            write_file.write(',')
                    write_file.write(']')
                    write_file.write('\n')
                elif 'negative_tuples' in line[:20]:
                    line1 = '    negative_tuples=['
                    write_file.write(line1)
                    for i in range(len(nat1)):
                        #print(str(i))
                        write_file.write('[')
                        write_file.write(str(nat1[i]))
                        write_file.write(']')
                        write_file.write(',')
                    for i in range(len(inv1)):
                        #print(str(i))
                        write_file.write('[')
                        write_file.write(str(inv1[i]))
                        write_file.write(']')
                        write_file.write(',')
                    for i in range(len(spr2)):
                        write_file.write('[')
                        write_file.write(str(spr2[i]))
                        write_file.write(']')
                    write_file.write(']')
                    write_file.write('\n')
                elif 'map(self.spray.add_rule,' in line:
                    line1= '    map(self.spray.add_rule,['
                    write_file.write(line1)

                    for i in range(len(spr1)):
                        #print(str(i))
                        write_file.write('(')
                        write_file.write(str(spr1[i]))
                        write_file.write(')')
                        if(int(i)+1 != len(spr1)):
                            write_file.write(',')
                    write_file.write('])')
                    write_file.write('\n')
                else:
                    write_file.write(line)
            write_file.close()
            #self.mem.LOG.append(self.mem.WORLD_CHANGES)
            #self.mem.LOG.append(str(self.mem.get(self.mem.ACTIONS)[-1]))
            #self.mem.LOG.append(self.mem.get(self.mem.STATES)[-2])
            #self.mem.INVASIVE_DEAD=0
            self.mem.WORLD_CHANGES=[]

            #for x in self.mem.LOG:
            #    print(x)

        if self.verbose >= 1: print(
            "    Retrieved last segment of trace")  # +str(self.mem.get(self.mem.TRACE_SEGMENT)))
        if self.verbose >= 1: print()
        "  " + str(self.mem.get(self.mem.TRACE_SEGMENT))

    def get_last_phase(self):
        """ Return a small part of the trace """
        return self.mem.trace.get_data(self.mem.trace.get_current_cycle(), self.mem.trace.get_current_phase())
