#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Zhenyu Ren
# E-mail     : rzy.1996@qq.com
# Description: 
# Date       : 20/05/2019 2:45 PM
# File Name  :

from lxml.etree import Element
from xml.dom.minidom import parseString
import lxml.etree as etree

import xml.dom.minidom


def create_xml_std(filename='None'):
    # 在内存中创建一个空的文档
    doc = xml.dom.minidom.Document()
    # 创建一个根节点Managers对象
    root = doc.createElement('group_state')
    # 设置根节点的属性
    root.setAttribute('group', 'arm')
    root.setAttribute('name', 'test1')
    # 将根节点添加到文档对象中
    doc.appendChild(root)

    joint_pose = [0, 1, 2, 3, 4, 5, 6]

    for i in joint_pose:
        node_manager = doc.createElement('joint')
        node_manager.setAttribute('name', 'joint' + str(i+1))
        node_manager.setAttribute('value', str(joint_pose[i]))
        # 最后将Manager添加到根节点Managers中
        root.appendChild(node_manager)

    # 开始写xml文档
    fp = open(filename + '.xml', 'a+')
    doc.writexml(fp, indent='\t', addindent='\t', newl='\n', encoding="utf-8", )


def create_group_state(group_state, group, joint_states, fp):
    node_root = Element('group_state')
    node_root.set('name', group_state)
    node_root.set('group', group)

    for i in range(7):
        node_joint = Element('joint')
        node_joint.set('name', 'joint' + str(i+1))
        node_joint.set('value', '%.3f' % float(joint_states[i]))

        node_root.append(node_joint)

    print(etree.tostring(node_root, pretty_print=True))

    tree = etree.ElementTree(node_root)
    tree.write(fp, pretty_print=True, xml_declaration=False, encoding='utf-8')


if __name__ == "__main__":
    fp = open('group_state' + '.xml', 'a+')
    fp.write('\n')
    joint_states = [0, 1, 2, 3, 4, 5, 6]
    group_state = 'test'
    group = 'arm'
    create_group_state(group_state, group, joint_states, fp)
    fp.close()

