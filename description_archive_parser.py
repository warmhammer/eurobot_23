#!/usr/bin/python3

from multiprocessing.connection import answer_challenge
from signal import raise_signal
import xml.etree.ElementTree as ET
import sys
import zipfile

def urdf_archive_unzipping(src_dir):
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        namelist = zip_ref.namelist()

        if 'dolly_description/meshes/' not in namelist or \
           'dolly_description/urdf/dolly_description.urdf' not in namelist:
           raise ValueError('No meshes or urdf in zip')

        for name in namelist:
            meshes_dir = 'dolly_description/meshes/'

            if name[:len(meshes_dir)] == meshes_dir:
                zip_ref.extract(name, src_dir)

        zip_ref.extract('dolly_description/urdf/dolly_description.urdf', src_dir)

def urdf_changing(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    # print(f'root{root.attrib}')

    for link in root.findall('link'):
        attrib = link.attrib
        if attrib['name'] == 'base_footprint':
            for inertial in link.findall('inertial'):
                link.remove(inertial)

    tree.write(filename)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise ValueError('No path to description zip')

    zip_path = sys.argv[1]
    zip_name = 'dolly_description.zip'

    if len(zip_name) > len(zip_path) or zip_path[-len(zip_name):] != zip_name:
        raise ValueError("zip path should end with 'dolly_description.zip'")

    src_dir = 'catkin_ws/src/'
    urdf_archive_unzipping(src_dir)

    urdf_path = src_dir + 'dolly_description/urdf/dolly_description.urdf'
    urdf_changing(urdf_path)