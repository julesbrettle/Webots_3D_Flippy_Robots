"""
make_flippys.py
Jonathan Zerez
Spring, 2021

This script contains functions that generate Webots world file (.wbt) code that
specifies a swarm of flippy robots, each with a unique ID. It allows the user to
specify the size, location, and orientation of the swarm.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def create_layout(dims, spacing,
                  offset=np.array([0,0,0]),
                  x_noise=(0, 0.005),
                  y_noise=(0, 0.01),
                  z_noise=(0, 0.005),
                  rot_noise=(0, 10*np.pi/180)):
    """
    Creates the flippy layout. Adds random noise for position and orientation.
    Only the heading of the flippys are changed. All noise is modeled as normal
    distributions. All length units are meters, all angular units are radians.

    In Webots space, X and Z comprise the ground plane and Y is vertical.

    Parameters:
        dims: (3,) np.array
            The dimensions of the swarm in terms of number of flippys.
            If dims = np.array([3,4,5]), the function will return out a swarm of flippys that is 3 flippys wide (x direction), 4 flippys tall (y direction) and 5 flippys long (z direction)
        spacing: (3,) np.array
            The initial spacing in x, y, and z between flippys in the swarm.
            Specified in meters.
        offset: (3,) np.array
            starting location for the first flippy in x,y, and z.
            Specified in meters.
        x_noise: (2,) tuple
            A tuple containing parameters for the noise of the x location of
            flippys. The first element is the mean noise and the second element
            is the standard devation of noise (in meters)
        y_noise: (2,) tuple
            A tuple containing parameters for the noise of the y location of
            flippys. The first element is the mean noise and the second element
            is the standard devation of noise (in meters)
        z_noise: (2,) tuple
            A tuple containing parameters for the noise of the z location of
            flippys. The first element is the mean noise and the second element
            is the standard devation of noise (in meters)
        rot_noise: (2,0) tuple
            A tuple containing parameters for the noise of the orientation of
            flippys. The first element is the mean noise and the second element
            is the standard devation of noise (in radians)

    Returns:
        locs: (n,3) np.array
            An array of the locations of flippys in [x,y,z] coordinates.
            Specified in meters. These coordinates include random noise
        rots: (n,4) np.array
            An array containing the rotation vector and the rotation magnitude
            in radians for each of the flippys. Each row of the array contains
            a vector [u,v,w,t], where u,v,w form a normalized rotation vector
            and t is the rotation magnitude in radians.

    """

    # Check argument sizes
    assert (dims.shape==(3,)), "`dims` must be a (3,) np array"
    assert (spacing.shape==(3,)), "`spacing` must be a a (3,) np array"

    # unpack values
    x_dim, y_dim, z_dim = dims
    x_space, y_space, z_space = spacing
    x_start, y_start, z_start = offset

    # create x,y and z ranges for the flippy coordinates
    xs = np.arange(x_dim) * x_space + x_start
    ys = np.arange(y_dim) * y_space + y_start
    zs = np.arange(z_dim) * z_space + z_start

    # mesh the x,y and z ranges to get every combination of them
    x_mesh, y_mesh, z_mesh = np.meshgrid(xs, ys, zs)

    # initialize the result arrays
    locs = np.empty((x_mesh.size, 3))
    rots = np.empty((x_mesh.size, 4))

    # loop over every coordinate (x,y,z) within the swarm, while keeping track of index i
    for i, (x,y,z) in enumerate(zip(x_mesh.ravel(), y_mesh.ravel(), z_mesh.ravel())):
        # Take base location and then add noise
        locs[i] = np.array([x,y,z]) + make_noise(x_noise, y_noise, z_noise)

        # Rotate the flippy so that its spheres are almost flat with the ground
        r1 = R.from_euler('z', -30, degrees=True)
        # Rotate the flippy some random amount to change its heading
        r2 = R.from_euler('y', np.random.normal(*rot_noise))
        # Combine the rotations and express them as a rotation vector
        r = (r2 * r1).as_rotvec()
        # In scipy, the length of the rotation vector is the mangitude of the rotaiton
        rads = np.linalg.norm(r)
        # Nomralize the rotation vector for Webots
        r /= rads
        rot_vec = np.append(r, rads)
        rots[i] = rot_vec

    return locs, rots

def make_flippys(locs, rots,
                default_state=0,
                start_index=1,
                base_proto_path='./protos/flippy_template.proto',
                write_to_wbt=False
                ):
    """
    This function takes in arrays for the location and rotations for a swarm of
    flippys and creates Webots readable text according the desired robots. Each
    flippy is initilized with a unique zeropadded ID number from 000-999

    Parameters:
        locs: (n,3) np.array
            An array of the locations of flippys in [x,y,z] coordinates.
            Specified in meters. These coordinates include random noise
        rots: (n,4) np.array
            An array containing the rotation vector and the rotation magnitude
            in radians for each of the flippys. Each row of the array contains
            a vector [u,v,w,t], where u,v,w form a normalized rotation vector
            and t is the rotation magnitude in radians.
        defualt_state: int
            An integer with the value of 0, 1, or 2. It sets the state of the
            flippy. 0 implies stationary, 1 implies that the first node is
            moving, and 2 implies that the second node is moving.
        start_index: int
            An integer that represents the starting index number for the flippys
        base_proto_path: str
            The path to the base proto file. This file is used as a template for
            creating Webots readable text
        write_to_wbt: bool
            If false, the function will create a .txt file which must be pasted
            into the desired Webots world. If true, the function will generate a
            new .wbt file directly.

    Returns:
        N/A

    """
    # Ensure we won't exceed index limit
    # (The physics plugin written in C++ does not support string ID's over 3 characters long)
    if len(locs) + start_index > 999:
        raise Exception("The number of maximum flippy's exceeded. Max flippy index number is 999")
    # Ensure the specified state is one of the allowable ones
    if default_state != 0 and default_state != 1 and default_state != 2:
        raise Exception("Invalid Default State. Must be 0, 1, 2")

    base_proto = open(base_proto_path, 'r').read()

    if write_to_wbt:
        print('Writing directly to: ./worlds/flippy_sim_autogenerated.wbt')
        print('Template wbt file: ./script_inputs/fimppy_sim_template.wbt')
        output_file_template = open('./script_inputs/flippy_sim_template.wbt', 'r').read()
        output_file = open('./worlds/flippy_sim_autogenerated.wbt', 'w')
        output_file.write(output_file_template)
    else:
        print('Writing to intermediate .txt file: /script_outputs/flippys.txt')
        print('No template wbt file used. You must copy the contents of the intermediate .txt folder to your desired .wbt file.')
        output_file = open('./script_outputs/flippys.txt', 'w')

    # Iterate over the locations and rotations, while keeping track of index i
    for i, (loc, rot) in enumerate(zip(locs, rots)):
        num = i + start_index

        # Create zero padded ID string based on number.
        # The ID string MUST BE exactly 3 characters long, which is why zero padding is imporant.
        if num < 10:
            num_str = '00' + str(num)
        elif num < 100:
            num_str = '0' + str(num)
        else:
            num_str = str(num)

        # casts the location and rotation into a string and removes the hard
        # brackets on either end of the array
        pos = str(loc)[1:-1]
        rot = str(rot)[1:-1]

        # Create new file temp_proto and write to it by replacing placeholder text
        temp_proto = base_proto.replace('###', num_str)
        temp_proto = temp_proto.replace('translation', 'translation ' + pos, 1)
        temp_proto = temp_proto.replace('rotation', 'rotation ' + rot, 1)
        temp_proto = temp_proto.replace('controllerArg#', str(default_state))

        # Write to output file
        output_file.write(temp_proto)

    # close the file
    output_file.close()

def make_noise(x_params, y_params, z_params):
    """
    Takes in noise parameters and draws random samples from normal distributions
    that the parameters represent.

    Parameters:
        x_params: (2,) tuple
            A tuple containing parameters for the noise in the x dimension. The
            first element is the mean of the noise and the second element is the
            standard devation of the noise
        x_params: (2,) tuple
            A tuple containing parameters for the noise in the y dimension. The
            first element is the mean of the noise and the second element is the
            standard devation of the noise
        x_params: (2,) tuple
            A tuple containing parameters for the noise in the z dimension. The
            first element is the mean of the noise and the second element is the
            standard devation of the noise
    Returns:
        _: (3,) np.array
            An array containing the noise in each dimension (x,y,z)
    """
    # Draw random samples.
    x_noise = np.random.normal(*x_params)
    y_noise = np.random.normal(*y_params)
    z_noise = np.random.normal(*z_params)

    # return as array
    return np.array([x_noise, y_noise, z_noise])


if __name__=="__main__":
    # Create a swarm of flippys 4 wide, 1 tall, and 2 long
    dims = np.array([1, 1, 2])
    # The average spacing between the flippys will be 0.15m, 0.03m and 0.2m
    spacing = np.array((0.25, 0.1, 0.3))
    # The first flippy will be located at:
    # offset=np.array([-0.15, 0.3, -0.5])
    offset=np.array([-0.15, 0.08, -0.4])


    locs, rots = create_layout(dims, spacing, offset=offset)
    make_flippys(locs, rots, write_to_wbt=True, start_index=0)
