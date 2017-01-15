import numpy as np

def homography_floor_to_img(img_points, floor_points):
    """
    Return a 3x3 matrix that is the homography from points in the floor to
    points in the image. The inputs are two arrays of lenght 4 containing
    4 points in the image and 4 points in the floor correspondent to the
    points in the image.

    """
    img_p1 = img_points[0]
    img_p2 = img_points[1]
    img_p3 = img_points[2]
    img_p4 = img_points[3]
    u1 = img_p1[0]; v1 = img_p1[1]
    u2 = img_p2[0]; v2 = img_p2[1]
    u3 = img_p3[0]; v3 = img_p3[1]
    u4 = img_p4[0]; v4 = img_p4[1]
    
    
    floor_p1 = floor_points[0];
    floor_p2 = floor_points[1];
    
    floor_p3 = floor_points[2];
    floor_p4 = floor_points[3];
    x1 = floor_p1[0]; y1 = floor_p1[1]
    x2 = floor_p2[0]; y2 = floor_p2[1]
    x3 = floor_p3[0]; y3 = floor_p3[1]

    x4 = floor_p4[0]; y4 = floor_p4[1]
    A = np.array([[x1, y1, 1, 0,0,0, -u1*x1, -u1*y1],
        [0,0,0,x1,y1,1,-v1*x1,-v1*y1],
        [x2,y2,1,0,0,0,-u2*x2,-u2*y2],
        [0,0,0,x2,y2,1,-v2*x2,-v2*y2],
        [x3,y3,1,0,0,0,-u3*x3,-u3*y3],
        [0,0,0,x3,y3,1,-v3*x3,-v3*y3],
        [x4,y4,1,0,0,0,-u4*x4,-u4*y4],
        [0,0,0,x4,y4,1,-v4*x4,-v4*y4]])
    b = np.array([[u1],[v1],[u2],[v2],[u3],[v3],[u4],[v4]])
    h = np.dot(np.linalg.inv(A),b)
    one = np.array([[1]])    
    h = np.concatenate((h,one),axis=0)
    return np.reshape(h,(3,3))


def homography_img_to_floor(img_points, floor_points):
    """
    Return a 3x3 matrix that is the homography from points in the image to
    points in the floor. The inputs are two arrays of lenght 4 containing
    4 points in the image and 4 points in the floor correspondent to the
    points in the image.

    """
    H = homography_floor_to_img(img_points, floor_points)
    Q = np.linalg.inv(H)
    return Q

def apply_homography(H, point):
    """
    Apply the homography to a given point and return the resulting point.

    """
    xbar = np.array([[point[0]], [point[1]], [1]])
    ubar = np.dot(H,xbar).T[0]
    u = np.int(ubar[0]/ubar[2])
    v = np.int(ubar[1]/ubar[2])
    return [u,v]

def pixel_to_floor(H,point):
    u = point[0]
    v = point[1]    
    Q = np.linalg.inv(H)
    q11 = Q[0,0]
    q12 = Q[0,1]
    q13 = Q[0,2]
    q21 = Q[1,0]
    q22 = Q[1,1]
    q23 = Q[1,2]
    q31 = Q[2,0]
    q32 = Q[2,1]
    q33 = Q[2,2]
    x = (q11*u + q12*v + q13)/(q31*u+q32*v+q33)
    y = (q21*u+q22*v+q23)/(q31*u+q32*v+q33)
    return [x,y]
