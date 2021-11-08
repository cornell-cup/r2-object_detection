import numpy as np

def coordinate_transformation(x,y,z):
    # The default coordinate values ((x,y,z) coordinate in camera)
    default_coordinate = np.array([[x], [y], [z]])

    # The three translation parameters between two coordinate origins
    # Notice that we put the origin of the camera in the frame of arm coordinate here
    diff_x=0
    diff_y=0
    diff_z=0.24
    diff_coordinates = np.array([[diff_x], [diff_y], [diff_z]])

    # The three rotational angles between two coordinate systems, should be in degree
    theta_x=115*np.pi/180
    theta_y=0*np.pi/180
    theta_z=0*np.pi/180
    angles = np.array([theta_x, theta_y, theta_z])

    # A scale factor (if needed)
    meu = 1

    # Rotational Matrix
    R = np.dot(np.dot([[np.cos(theta_z), np.sin(theta_z), 0], [-np.sin(theta_z), np.cos(theta_z), 0], [0, 0, 1]],
               [[np.cos(theta_y), 0, -np.sin(theta_y)], [0, 1, 0], [np.sin(theta_y), 0, np.cos(theta_y)]]),
               [[1, 0, 0], [0, np.cos(theta_x), np.sin(theta_x)], [0, -np.sin(theta_x), np.cos(theta_x)]])

    # Transformaion formula
    transformed_coordinate = np.add(diff_coordinates, meu*np.dot(R,default_coordinate))

    return(transformed_coordinate)

if __name__ == '__main__':
    print(coordinate_transformation(-0.05,-0.13,0.704))
