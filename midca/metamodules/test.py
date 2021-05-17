input_file1 = "/home/zahid/Downloads/midca-update-to-python36/midca/domains/ffdomain/minecraft/plant-pro_modif_test.pddl"

with open(input_file1, 'r+') as f:
    for line in f:
        if line.startswith("    :precondition"):
            print(f.next())
            # Or use next(f, '') to return <empty string> instead of raising a
            # StopIteration if the last line is also a match.
            break


'''inputfile = open(input_file1,'r').readlines()
write_file = open(input_file1,'w')
for line in inputfile:
    write_file.write(line)
    if ':precondition' in line:
        if ''
        for i in range(b):
            native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
            invasive_plant = "(invasive-at pos%d-%d)" %(inv[i][0],inv[i][1])
            request_1 = "(requested human1 agent1 native-at pos%d-%d )" %(nat[i][0],nat[i][1])
            request_2 = "(requested human1 agent1 not-invasive-at pos%d-%d )" %(inv[i][0],inv[i][1])
            write_file.write(native_plant + "\n")
            write_file.write(invasive_plant + "\n")
            write_file.write(request_1 + "\n")
            write_file.write(request_2 + "\n")
write_file.close()
for line in input_file:
    print('hi')
    write_file.write(line)
    if '(and' in line:
        for i in range(b):
            native_plant = "(native-at pos%d-%d)" %(nat[i][0],nat[i][1])
            invasive_plant = "(invasive-at pos%d-%d)" %(inv[i][0],inv[i][1])
            write_file.write(native_plant + "\n")
            write_file.write(invasive_plant + "\n")
write_file.close()'''