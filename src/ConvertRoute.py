import numpy as np
import csv

def save_route(drone_position, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X', 'Y', 'Z', 'ALFA', 'BETA', 'GAMA'])
        writer.writerows(drone_position)


if __name__ == '__main__':
    output_filename = 'rota_office_4.csv'
    coord = np.array([0, 0, 0])
    base_vec = np.array([1, 0])

    points = []

    directions_vec = []

    list_position = []

    directions = []
    directions_vectors = [np.array(d[:2]) for d in directions_vec]

    for vec in directions_vectors:
        cosTheta = np.dot(vec, base_vec) / np.linalg.norm(vec) / np.linalg.norm(base_vec)
        theta = -np.arccos(cosTheta) if vec[1] < base_vec[1] else np.arccos(cosTheta)
        directions.append([0, 0, theta])

    list_position = [np.concatenate((point + coord, vector)) for point, vector in zip(points, directions)]
    save_route(list_position, output_filename)
