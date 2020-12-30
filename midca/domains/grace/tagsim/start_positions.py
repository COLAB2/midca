row = 5
col = 5

def initial_points(row, col, scale=4, filename = "initial_positions"):

    for i in range(row):
        for j in range(col):
            x1 = i*scale
            x2 = (i+1)*scale
            y1 = j*scale
            y2 = (j+1)*scale
            print("(" + str(float((x1+x2)/2)) +", "
                   + str(float((y1+y2)/2)) +")" + ","),
        print("")

def main():
    initial_points(5,5)

if __name__ == "__main__":
    main()