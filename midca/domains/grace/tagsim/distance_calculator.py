from scipy.spatial import distance


def calculate_distance(points):

    hotspot = (6,14)
    for a in points:
        dist = distance.euclidean(a, hotspot)
        print (dist)

def main():
    points =  [
        (2.0, 2.0), (2.0, 6.0), (2.0, 10.0), (2.0, 14.0), (2.0, 18.0),
        (6.0, 2.0), (6.0, 6.0), (6.0, 10.0), (6.0, 14.0), (6.0, 18.0),
        (10.0, 2.0), (10.0, 6.0), (10.0, 10.0), (10.0, 14.0), (10.0, 18.0),
        (14.0, 2.0), (14.0, 6.0), (14.0, 10.0), (14.0, 14.0), (14.0, 18.0),
        (18.0, 2.0), (18.0, 6.0), (18.0, 10.0), (18.0, 14.0), (18.0, 18.0)
             ]
    calculate_distance(points)

if __name__ == "__main__":
    main()