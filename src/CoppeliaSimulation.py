from config import parse_settings_file
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any

import os
import sys
import csv
import numpy as np
import cv2 as cv


def init_coppelia() -> tuple[RemoteAPIClient, Any]:
    """
    Initialize the ZMQ remote control of the CoppeliaSim and starts the simulation
    :rtype: object
    :return: tuple with Remoteclient and the sim handle.
    """
    client_init: RemoteAPIClient = RemoteAPIClient()
    sim_init = client_init.getObject('sim')
    client_init.setStepping(True)
    sim_init.startSimulation()
    return client_init, sim_init


def init_control(handle_names: list, sim) -> dict:
    """
    Method used to get the objects used in the quadcopter control.
    :param sim:
    :param handle_names: List of the objects to get handle
    :return: Nothing
    """
    handles = {}
    for handle_name in handle_names:
        if handle_name not in handles:
            handles[handle_name] = sim.getObject(handle_name)
    return handles


def set_quadcopter_pos_ori(position,
                           orientation,
                           quad_target_handle: int,
                           quad_base_handle: int,
                           sim: Any,
                           client: Any,
                           fast_set: bool = False):
    """
    This method is used to move the quadcopter in the CoppeliaSim scene to the position pos.
    :param sim:
    :param quad_base_handle: The handle to get the quadcopter current position
    :param quad_target_handle:  The handle to the target of the quadcopter. This handle is used to position give the
    position that the quadcopter must be after control.
    :param orientation: The object orientation
    :param pos: The new position of the quadcopter is moved to.
    :return: A boolean indicating if the quadcopter reach the target position.
    """

    MIN_DIFF_POS = 0.05
    MIN_DIFF_ORI = 0.02

    if fast_set:
        MIN_DIFF_POS = 0.2
        MIN_DIFF_ORI = 0.1

    last_pos = np.array(sim.getObjectPosition(quad_base_handle, sim.handle_world))
    last_ori = np.array(sim.getObjectOrientation(quad_base_handle, sim.handle_world))

    client.step()

    sim.setObjectPosition(quad_target_handle, sim.handle_world, position.tolist())
    sim.setObjectOrientation(quad_target_handle, sim.handle_world, orientation.tolist())

    while True:
        curr_pos = np.array(sim.getObjectPosition(quad_base_handle, sim.handle_world))
        curr_ori = np.array(sim.getObjectOrientation(quad_base_handle, sim.handle_world))

        diff_pos = curr_pos - last_pos
        diff_ori = curr_ori - last_ori

        norm_diff_pos = np.linalg.norm(diff_pos)
        norm_diff_ori = np.linalg.norm(diff_ori)

        diff_pos_final = position - curr_pos
        diff_ori_final = orientation - curr_ori

        norm_diff_pos_final = np.linalg.norm(diff_pos_final)
        norm_diff_ori_final = np.linalg.norm(diff_ori_final)

        if (norm_diff_pos < MIN_DIFF_POS and (norm_diff_ori < MIN_DIFF_ORI or np.abs(norm_diff_ori - 2 * np.pi) < MIN_DIFF_ORI)) and (
                norm_diff_pos_final < MIN_DIFF_POS and (norm_diff_ori_final < MIN_DIFF_ORI or np.abs(norm_diff_ori_final - 2 * np.pi) < MIN_DIFF_ORI)):
            break

        last_pos = curr_pos
        last_ori = curr_ori

        client.step()


def quadcopter_control(position: list,
                       orientation: list,
                       quad_target_handle: int,
                       quad_base_handle: int,
                       sim: Any,
                       client: Any):
    """
    This method is used to move the quadcopter in the CoppeliaSim scene to the position pos.
    :param sim:
    :param time_to_stabilize:
    :param quad_base_handle: The handle to get the quadcopter current position
    :param quad_target_handle:  The handle to the target of the quadcopter. This handle is used to position give the
    position that the quadcopter must be after control.
    :param orientation: The object orientation
    :param pos: The new position of the quadcopter is moved to.
    :return: A boolean indicating if the quadcopter reach the target position.
    """
    MIN_VAR_POS = 0.5
    MIN_VAR_ORI = 0.2618
    stabilized_pos = False
    stabilized_ori = False
    next_pos = None
    next_ori = None

    while True:
        curr_pos = next_pos if stabilized_pos else np.array(sim.getObjectPosition(quad_base_handle, sim.handle_world))
        curr_ori = next_ori if stabilized_ori else np.array(sim.getObjectOrientation(quad_base_handle, sim.handle_world))

        diff_pos = position - curr_pos
        diff_ori = orientation - curr_ori
        norm_diff_pos = np.linalg.norm(diff_pos)
        norm_diff_ori = np.linalg.norm(diff_ori)

        if not stabilized_pos and norm_diff_pos < MIN_VAR_POS:
            stabilized_pos = True
            is_pos_ori_stabilized = stabilized_pos and stabilized_ori
            set_quadcopter_pos_ori(position, curr_ori, quad_target_handle,
                                   quad_base_handle, sim, client, not is_pos_ori_stabilized)
            next_pos = curr_pos = position

        if not stabilized_ori and (norm_diff_ori < MIN_VAR_ORI or np.abs(norm_diff_ori - 2 * np.pi) < MIN_VAR_ORI):
            stabilized_ori = True
            is_pos_ori_stabilized = stabilized_pos and stabilized_ori
            set_quadcopter_pos_ori(curr_pos, orientation, quad_target_handle,
                                   quad_base_handle, sim, client, not is_pos_ori_stabilized)
            next_ori = curr_ori = orientation

        if stabilized_pos and stabilized_ori:
            break

        if not stabilized_pos:
            next_pos = curr_pos + diff_pos / norm_diff_pos * MIN_VAR_POS

        if not stabilized_ori:
            next_ori = curr_ori + diff_ori / norm_diff_ori * MIN_VAR_ORI

        set_quadcopter_pos_ori(next_pos, next_ori, quad_target_handle, quad_base_handle, sim, client, True)


def get_image(idx: int, path: str, file_name: str, extension: str, vision_handle: int, sim: Any):
    """
    Method used to get the image from vision sensor on coppeliaSim and save the image in a file.
    The vision handle must be previously loaded.
    :param path:
    :param vision_handle: Vison sensor handle to CoppeliaSim vision sensor.
    :param file_name: File name to saved image
    :param sequence: Parameter not used yet
    :return: Nothing
    """
    img, resolution = sim.getVisionSensorImg(vision_handle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    img = cv.flip(cv.cvtColor(img, cv.COLOR_BGR2RGB), 0)
    filename = file_name + str(idx) + extension
    cv.imwrite(path + filename, img)


def save_reconstruction_images(reconstruction_file_name: str,
                               position_file_name: str,
                               extension_img: str,
                               path: str,
                               vision_sensor_name: str,
                               quadcopter_name: str,
                               quadcopter_base: str,
                               client: Any,
                               sim: Any):
    """
    Method used to save reconstruction images
    :param sim:
    :param client:
    :param quadcopter_base:
    :param quadcopter_name:
    :param path:
    :param extension_img:
    :param vision_sensor_name:
    :param reconstruction_file_name:
    :param position_file_name:
    :return:
    """
    handles = init_control(
        [quadcopter_name, vision_sensor_name, quadcopter_base], sim)
    count_image = 0
    position = []
    orientation = []

    if os.path.isfile(position_file_name):
        with open(position_file_name, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            header = next(reader)
            for row in reader:
                if float(row[2]) < 0.16:
                    row[2] = '0.16'

                position.append(np.array([float(v) for v in row[:3]]))
                orientation.append(np.array([float(v) for v in row[3:]]))
    else:
        print('Position csv file not found. Adjust the config.yaml file.')
        quit(-1)

    if not os.path.exists(path):
        os.makedirs(path)

    photo_count = len(position)
    for pos, orient in zip(position, orientation):        
        quadcopter_control(pos,
                           orient,
                           handles[quadcopter_name],
                           handles[quadcopter_base],
                           sim,
                           client)
        get_image(idx=count_image,
                  path=path,
                  file_name=reconstruction_file_name,
                  vision_handle=handles[vision_sensor_name],
                  sim=sim,
                  extension=extension_img)
        count_image += 1
        print(f'\x1b[1K\r[{count_image}/{photo_count}] capture photo', end=' ')

    print('')
    print("Adjust the sequence to the maximum of {} images".format(str(count_image)))
    sim.stopSimulation()


if __name__ == '__main__':
    settings = parse_settings_file('config.yaml')
    reconstruction_file_name = 'reconstruct_image_'
    client, sim = init_coppelia()

    n = len(sys.argv)
    if n > 2:
        folder = sys.argv[1]
        file_name = sys.argv[2]

        settings['path'] = folder
        settings['positions file name'] = file_name

    try:
        save_reconstruction_images(reconstruction_file_name=reconstruction_file_name,
                               path=settings['path'],
                               position_file_name=settings['positions file name'],
                               extension_img=settings['extension'],
                               vision_sensor_name=settings['vision sensor names'][0],
                               quadcopter_name=settings['quadcopter name'],
                               quadcopter_base=settings['quadcopter base'],
                               client=client,
                               sim=sim)
    except Exception as err:
        sim.stopSimulation()
        print(err)
