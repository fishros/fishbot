# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time
import rclpy
import yaml

from tf2_msgs.srv import FrameGraph


class RosTfTreeDotcodeGenerator(object):

    def __init__(self, initial_listen_duration=1):
        """
        :param initial_listen_duration: how many secs to listen to tf initially.
        """
        self.last_drawargs = None
        self.dotcode = None
        self.firstcall = True
        self.listen_duration = initial_listen_duration
        self.rank = None
        self.rankdir = None
        self.ranksep = None
        self.graph = None
        self.dotcode_factory = None

    def generate_dotcode(self,
                         dotcode_factory,
                         tf2_frame_srv,
                         timer=rclpy.clock.Clock(),
                         yaml_parser=yaml,
                         rank='same',   # None, same, min, max, source, sink
                         ranksep=0.2,   # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         force_refresh=False):
        """
        :param force_refresh: if False, may return same dotcode as last time
        """
        if self.firstcall is True:
            self.firstcall = False
            force_refresh = True

        drawing_args = {
            'dotcode_factory': dotcode_factory,
            'rank': rank,
            'rankdir': rankdir,
            'ranksep': ranksep}

        selection_changed = False
        if self.last_drawargs != drawing_args:
            selection_changed = True
            self.last_drawargs = drawing_args

            self.dotcode_factory = dotcode_factory
            self.rank = rank
            self.rankdir = rankdir
            self.ranksep = ranksep

        # generate new dotcode
        if force_refresh or self.dotcode is None or selection_changed:
            if self.listen_duration > 0:
                time.sleep(self.listen_duration)
            # no need to listen more once we've listened for 1 sec?
            self.listen_duration = 0

            yaml_data = tf2_frame_srv.call(FrameGraph.Request()).frame_yaml
            data = yaml_parser.safe_load(yaml_data)
            self.graph = self.generate(data, timer.now().nanoseconds / rclpy.time.CONVERSION_CONSTANT)
            self.dotcode = self.dotcode_factory.create_dot(self.graph)

        return self.dotcode

    def generate(self, data, timestamp):
        graph = self.dotcode_factory.get_graph(rank=self.rank,
                                               rankdir=self.rankdir,
                                               ranksep=self.ranksep)

        if data is None or len(data) == 0:
            self.dotcode_factory.add_node_to_graph(graph, 'No tf data received')
            return graph

        for frame_dict in data:
            tf_frame_values = data[frame_dict]
            if not tf_frame_values['parent'] in data:
                root = tf_frame_values['parent']
            self.dotcode_factory.add_node_to_graph(graph,
                                                   str(tf_frame_values['parent']),
                                                   shape='ellipse')
            self.dotcode_factory.add_node_to_graph(
                graph, frame_dict, shape='ellipse')

            edge_label = '"Broadcaster: %s\\n' % str(tf_frame_values['broadcaster'])
            edge_label += 'Average rate: %s\\n' % str(tf_frame_values['rate'])
            edge_label += 'Buffer length: %s\\n' % str(tf_frame_values['buffer_length'])
            edge_label += 'Most recent transform: %s\\n' % str(tf_frame_values['most_recent_transform'])
            edge_label += 'Oldest transform: %s"' % str(tf_frame_values['oldest_transform'])
            self.dotcode_factory.add_edge_to_graph(graph,
                                                   str(tf_frame_values['parent']),
                                                   frame_dict,
                                                   label=edge_label)

        # create legend before root node
        legend_label = '"Recorded at time: %s"' % str(timestamp)
        self.dotcode_factory.add_node_to_graph(graph, legend_label)
        self.dotcode_factory.add_edge_to_graph(graph,
                                               legend_label,
                                               root,
                                               style='invis')

        # dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
        # dot += '"Recorded at time: '+str(rospy.Time.now().to_sec())+'"[ shape=plaintext ] ;\n'
        # dot += '}->"'+root+'"[style=invis];\n}'
        return graph
