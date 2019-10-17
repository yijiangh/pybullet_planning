def read_pcd_file(path):
    """
    Reads a *.pcd pointcloud file
    :param path: path to the *.pcd pointcloud file
    :return: list of points
    """
    with open(path) as f:
        data = f.readline().split()
        num_points = 0
        while data[0] != 'DATA':
            if data[0] == 'POINTS':
                num_points = int(data[1])
            data = f.readline().split()
            continue
        return [tuple(map(float, f.readline().split())) for _ in range(num_points)]
