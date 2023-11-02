# Copyright 2015 Yale University - Grablab
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Modified to work with Python 3 by Sebastian Castro, 2020

from calendar import c
from genericpath import isdir, isfile
import os
from re import L
import sys
import json
import urllib
import shutil
from urllib.request import Request, urlopen
import trimesh
import pybullet as p

import numpy as np

from glob import glob

import xml.etree.ElementTree as ET
from xml.dom import minidom

# Define an output folder
output_directory = os.path.join("models", "ycb")

# Define file path of weight file
weight_file = "object_weights.yaml"

# Define a list of objects to download from
# http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/
objects_to_download = "all"
# objects_to_download = ["001_chips_can",
#                        "002_master_chef_can",
#                        "003_cracker_box",
#                        "004_sugar_box"]

# You can edit this list to only download certain kinds of files.
# 'berkeley_rgbd' contains all of the depth maps and images from the Carmines.
# 'berkeley_rgb_highres' contains all of the high-res images from the Canon cameras.
# 'berkeley_processed' contains all of the segmented point clouds and textured meshes.
# 'google_16k' contains google meshes with 16k vertices.
# 'google_64k' contains google meshes with 64k vertices.
# 'google_512k' contains google meshes with 512k vertices.
# See the website for more details.
# files_to_download = ["berkeley_rgbd", "berkeley_rgb_highres", "berkeley_processed", "google_16k", "google_64k", "google_512k"]
files_to_download = ["berkeley_processed", "google_16k"]


folders_blacklist = ["023_wine_glass"]
# Extract all files from the downloaded .tgz, and remove .tgz files.
# If false, will just download all .tgz files to output_directory
extract = True

# Convert all extracted to mujoco compatible collision accurate files, including mujoco xml files
convert = True

base_url = "http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/"
objects_url = "https://ycb-benchmarks.s3.amazonaws.com/data/objects.json"

if not os.path.exists(output_directory):
    os.makedirs(output_directory)


def load_weights():
    import yaml

    with open(weight_file, "r") as f:
        weights = yaml.safe_load(f)

    weights = dict((k, v / 1000 if v != 0 else 50 / 1000) for k, v in weights.items())
    return weights


def fetch_objects(url):
    """Fetches the object information before download"""
    response = urlopen(url)
    html = response.read()
    objects = json.loads(html)
    return objects["objects"]


def download_file(url, filename):
    """Downloads files from a given URL"""
    u = urlopen(url)
    f = open(filename, "wb")
    file_size = int(u.getheader("Content-Length"))
    print(f"Downloading: {filename} ({file_size/1000000.0} MB)")

    file_size_dl = 0
    block_sz = 65536
    while True:
        buffer = u.read(block_sz)
        if not buffer:
            break

        file_size_dl += len(buffer)
        f.write(buffer)
        status = r"%10d  [%3.2f%%]" % (
            file_size_dl / 1000000.0,
            file_size_dl * 100.0 / file_size,
        )
        status = status + chr(8) * (len(status) + 1)
        print(status)
    f.close()


def tgz_url(object, type):
    """Get the TGZ file URL for a particular object and dataset type"""
    if type in ["berkeley_rgbd", "berkeley_rgb_highres"]:
        return f"{base_url}berkeley/{object}/{object}_{type}.tgz"
    elif type in ["berkeley_processed"]:
        return f"{base_url}berkeley/{object}/{object}_berkeley_meshes.tgz"
    else:
        return f"{base_url}google/{object}_{type}.tgz"


def extract_tgz(filename, dir):
    """Extract a TGZ file"""
    tar_command = f"tar -xzf {filename} -C {dir}"
    os.system(tar_command)
    os.remove(filename)


def check_url(url):
    """Check the validity of a URL"""
    try:
        request = Request(url)
        request.get_method = lambda: "HEAD"
        response = urlopen(request)
        return True
    except Exception as e:
        return False


def pb_vhacd(file_path):
    folder_path = os.path.dirname(file_path)

    p.connect(p.DIRECT)
    vhacd_out_path = os.path.join(folder_path, "textured_vhacd.obj")
    log_path = os.path.join(folder_path, "vhacd_log.txt")
    p.vhacd(file_path, vhacd_out_path, log_path)
    p.disconnect()

    submesh_files = []
    submesh_proportion = []
    mesh = trimesh.load(vhacd_out_path)

    try:
        mesh_volume = mesh.volume
    except RuntimeError as e:
        mesh_volume = 0

    submeshes = mesh.split()
    for i, submesh in enumerate(submeshes):
        submesh_out_path = os.path.join(folder_path, f"textured_collision_{i}.obj")
        submesh.export(submesh_out_path)
        submesh_files.append(submesh_out_path)
        try:
            submesh_volume = submesh.volume
        except RuntimeError as e:
            submesh_volume = 0
        submesh_proportion.append(submesh_volume / mesh_volume)

    return submesh_files, submesh_proportion


def adjust_xml_file(xml_file, obj_name, collision_files, collision_proportions, weight):
    dirname = os.path.abspath(os.path.dirname(xml_file))
    outfile = os.path.join(os.path.dirname(xml_file), f"{obj_name}.xml")
    print(f"processing {xml_file}")
    print(f"to {outfile}")

    tree = ET.parse(xml_file)
    root = tree.getroot()

    for node in root.iter():
        node.text = ""
        node.tail = ""

        if node.tag == "mujoco":
            node.set("model", obj_name)

        if node.tag == "texture":
            node.set("name", f"{obj_name}_tex")

        if "texture" in node.attrib:
            node.set("texture", f"{obj_name}_tex")

        if node.tag == "material":
            node.set("name", f"{obj_name}_mat")

        if "material" in node.attrib:
            node.set("material", f"{obj_name}_mat")

        if node.tag == "geom":
            if node.get("mesh") == "textured":
                node.set("mesh", obj_name)

        if node.tag == "asset":
            for i, collision_file in enumerate(collision_files):
                filename = collision_file.split("/")[-1]
                mesh_node = ET.SubElement(node, "mesh")
                mesh_node.set("file", os.path.join(dirname, filename))
                mesh_node.set("name", f"{obj_name}_collision_{i}")

        if node.tag == "body":
            for child in node.getchildren():
                if "material" not in child.attrib:
                    node.remove(child)
                    break

            if np.abs(np.sum(collision_proportions) - 1.0) > 0.1:
                collision_proportions = [1 / len(collision_proportions)] * len(
                    collision_proportions
                )
            for i, (collision_file, collision_proportion) in enumerate(
                zip(collision_files, collision_proportions)
            ):
                geom_node = ET.SubElement(node, "geom")
                geom_node.set("mesh", f"{obj_name}_collision_{i}")
                geom_node.set("conaffinity", "1")
                geom_node.set("condim", "4")
                geom_node.set("friction", "0.1 0.1 0.1")
                geom_node.set("mass", f"{weight*collision_proportion}")
                geom_node.set("rgba", "1 1 1 1")
                geom_node.set("solimp", "0.99 0.99 0.01")
                geom_node.set("solref", ".001 0.1")
                geom_node.set("type", "mesh")
                geom_node.set("group", "3")
            joint_node = ET.SubElement(node, "joint")
            joint_node.set("damping", "0.001")
            joint_node.set("name", f"{obj_name}:joint")
            joint_node.set("type", "free")

        if "file" in node.attrib:
            filename = node.get("file")
            if not filename.startswith("/"):
                node.set("file", os.path.join(dirname, filename))

        if "name" in node.attrib:
            prev_name = node.get("name")
            new_name = prev_name.replace("textured", obj_name)
            node.set("name", new_name)

    # for node in root.iter():
    #     if node.tag == "mesh":
    #         node.set("scale", "0.5 0.5 0.5")

    rough_string = ET.tostring(root, "utf-8")
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")
    with open(outfile, "w") as f:
        f.write(pretty_xml)


def convert_to_mj(directory):
    print(directory)
    # Exactly two folder levels down since the third one usually contains the generated texture
    obj_folders = glob(os.path.join(directory, "*"))
    obj_folders.sort()

    obj_weights = load_weights()

    for obj_folder in obj_folders:
        obj_id = obj_folder.split("/")[-1]
        if obj_id in folders_blacklist:
            continue
        _obj_folder = None
        if os.path.isfile(os.path.join(obj_folder, "google_16k", "textured.obj")):
            _obj_folder = os.path.join(obj_folder, "google_16k")
        elif os.path.isfile(os.path.join(obj_folder, "tsdf", "textured.obj")):
            _obj_folder = os.path.join(obj_folder, "tsdf")
        else:
            print(f"ignoring {obj_folder}")
            continue

        weight = obj_weights[obj_id] if obj_id in obj_weights else 0.05

        print(f"@ Converting {_obj_folder}")

        tar_command = f"obj2mjcf --verbose --overwrite --save-mtl --save-mjcf --obj-dir {_obj_folder}"
        os.system(tar_command)

        submesh_files, submesh_proportion = pb_vhacd(
            os.path.join(_obj_folder, "textured", "textured.obj")
        )

        # collision_files = glob(os.path.join(_obj_folder, "textured", "textured_collision_*.obj"))
        # collision_files.sort()

        obj_name = _obj_folder.split("/")[-2][4:]

        xml_file = os.path.join(_obj_folder, "textured/textured.xml")
        adjust_xml_file(xml_file, obj_name, submesh_files, submesh_proportion, weight)

        create_urdf(_obj_folder, weight)

        best_quality_path = os.path.join(obj_folder, "textured")
        cur_quality_path = os.path.join(_obj_folder, "textured")

        if os.path.isdir(best_quality_path):
            shutil.rmtree(best_quality_path)
        elif os.path.isfile(best_quality_path):
            os.remove(best_quality_path)
        shutil.copytree(cur_quality_path, best_quality_path)


def create_urdf(_obj_folder, weight):
    print(f"@ URDFING {_obj_folder}")

    obj_name = _obj_folder.split("/")[-2][4:]

    dirname = os.path.abspath(_obj_folder)

    textured_folder = os.path.join(dirname, "textured")
    collision_file = os.path.join(textured_folder, "textured_vhacd.obj")
    outfile = os.path.join(textured_folder, f"{obj_name}.urdf")
    textured_mat = glob(os.path.join(textured_folder, "*.mtl"))[
        0
    ]  # There should be exactly one

    full_str = f"""<robot name="{obj_name}.urdf">
        <link name="{obj_name}">
            <inertial>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <mass value="{weight}" />
            <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="1.0"  iyz="0.0"  izz="1.0" />
            </inertial>
            <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="textured.obj" scale="1 1 1"/>
            </geometry>
            <material name="texture">
                <texture filename="{textured_mat}"/>
            </material>
            </visual>
            <collision>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <geometry>
                    <mesh filename="{collision_file}" scale="1 1 1"/>
                </geometry>
            </collision>
        </link>
    </robot>"""

    with open(outfile, "w") as f:
        f.write(full_str)


def copy_best_quality(directory):
    obj_folders = glob(os.path.join(directory, "*/"))
    for obj_folder in obj_folders:
        g16k_path = os.path.join(obj_folder, f"google_16k/textured")
        tsdf_path = os.path.join(obj_folder, f"tsdf/textured")
        best_quality_path = os.path.join(obj_folder, f"textured")

        if os.path.isdir(best_quality_path):
            shutil.rmtree(best_quality_path)
        elif os.path.isfile(best_quality_path):
            os.remove(best_quality_path)

        if os.path.isdir(g16k_path):
            print(f"g16k: {obj_folder}")
            shutil.copytree(g16k_path, best_quality_path)
        elif os.path.isdir(tsdf_path):
            print(f"tsdf: {obj_folder}")
            shutil.copytree(tsdf_path, best_quality_path)
        else:
            print(f"Nothing: {obj_folder}")


if __name__ == "__main__":

    # Grab all the object information
    objects = fetch_objects(objects_url)

    # Download each object for all objects and types specified
    for object in objects:
        if objects_to_download == "all" or object in objects_to_download:
            for file_type in files_to_download:
                url = tgz_url(object, file_type)
                if not check_url(url):
                    continue
                filename = f"{output_directory}/{object}_{file_type}.tgz"
                download_file(url, filename)
                if extract:
                    extract_tgz(filename, output_directory)

    if convert:
        convert_to_mj(output_directory)
