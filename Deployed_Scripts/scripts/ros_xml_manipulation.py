#! /usr/bin/env python

from __future__ import print_function

import rospy
import sys
from geometry_msgs.msg import Pose
from xml.etree import ElementTree as ModuleElementTree
from xml.etree.ElementTree import Element, SubElement, tostring, ElementTree
from xml.dom import minidom




##  Returns a more human readable version of the generated xml
def prettify_xml(elem):
    #  If the input is a string, then we can immediatly generate the xml
    if isinstance(elem, basestring):
        parsed = minidom.parseString(elem)
        return parsed.toprettyxml()

    default_str = ModuleElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(default_str)
    return reparsed.toprettyxml()



##  This method will trim the xml version element
def trim_xml_version_element(elem):
    unwanted_str = "<?xml version=\"1.0\" ?>"
    trim_len = len(unwanted_str)

    untrimmed_xml = prettify_xml(elem)
    trimmed_xml = untrimmed_xml[trim_len:]

    return trimmed_xml




##  Method generates xml for an object
def generate_xml_from_data(name, obj_type, pose):

    #  If the object type were None, then this would be a container
    has_type = True
    entry = 'object'

    if obj_type == None:
       has_type = False
       entry = 'container'

    tag_object = Element(entry, {'name':name})

    if has_type:
        tag_type = SubElement(tag_object, 'type')
        tag_type.text = str(obj_type)

    #  Generate xml from position data
    generate_xml_from_pos(pose, tag_object)

    return tag_object




##  This method will construct xml for a given Pose message
def generate_xml_from_pos(pose, tag_main_element):
    xp = pose.position.x
    yp = pose.position.y
    zp = pose.position.z

    xo = pose.orientation.x
    yo = pose.orientation.y
    zo = pose.orientation.z
    wo = pose.orientation.w

    #  Generate main tags <position> and <orientation>
    tag_position = SubElement(tag_main_element, 'position')
    tag_orientation = SubElement(tag_main_element, 'orientation')
    
    tag_xpos = SubElement(tag_position, 'x_pos')
    tag_ypos = SubElement(tag_position, 'y_pos')
    tag_zpos = SubElement(tag_position, 'z_pos')

    tag_xorient = SubElement(tag_orientation, 'x_orient')
    tag_yorient = SubElement(tag_orientation, 'y_orient')
    tag_zorient = SubElement(tag_orientation, 'z_orient')
    tag_worient = SubElement(tag_orientation, 'w_orient')


    #  Populate the tags with the relevant values
    tag_xpos.text = str(xp)
    tag_ypos.text = str(yp)
    tag_zpos.text = str(zp)

    tag_xorient.text = str(xo)
    tag_yorient.text = str(yo)
    tag_zorient.text = str(zo)
    tag_worient.text = str(wo)






##  This method will write the generated xml snippets into an existing file containing containers or objects
def append_to_xml_file(file=None, name=None, obj_type=None, pose=None):
    with open(file, mode='r+') as xml_file:
        #  Generate new tag
        raw_insertion_elem = generate_xml_from_data(name, obj_type, pose)
        
        try:
            #  Get root of the file, allows us to insert inbetween the root tag
            root = ModuleElementTree.parse(file).getroot()  # Get all file contents
            trimmed_insertion_elem = ModuleElementTree.fromstring(trim_xml_version_element(raw_insertion_elem))

            root.append(trimmed_insertion_elem)  # Insert at the bottom of the file
            final_str = ModuleElementTree.tostring(root)
            xml_file.write(final_str)

            

        except ModuleElementTree.ParseError as error:  # File is empty
            root = Element('root')
            # first_insert = Element.fromstring(raw_insertion_elem)
            root.append(raw_insertion_elem)
            xml_file.write(prettify_xml(root))






# container_position = Pose()
# container_position.position.x = 1
# container_position.position.y = 2
# container_position.position.z = 3
# container_position.orientation.x = 4
# container_position.orientation.y = 5
# container_position.orientation.z = 6
# container_position.orientation.w = 7



# append_to_xml_file(file='testxml.xml', name="Orange Ball", obj_type="Ball", pose=container_position)
