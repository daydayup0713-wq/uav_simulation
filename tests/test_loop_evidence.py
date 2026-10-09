from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def test_loop_constraints_exclude_sequential_edges_and_camera_markers():
    from uav_lab_experiments.loop_evidence import loop_edge_count
    sequential=Marker(type=Marker.LINE_LIST,ns='CameraPoseVisualization',points=[Point(),Point()])
    sequential.color.b=1.
    loop=Marker(type=Marker.LINE_STRIP,ns='CameraPoseVisualization',points=[Point(),Point()])
    loop.color.r=1.
    graph=MarkerArray(markers=[sequential,loop])
    assert loop_edge_count('vins_fusion', graph)==1
    assert loop_edge_count('lio_sam', graph)==0
    sam=Marker(type=Marker.LINE_LIST,ns='loop_edges',points=[Point() for _ in range(4)])
    assert loop_edge_count('lio_sam',MarkerArray(markers=[sam]))==2
    assert loop_edge_count('vins_fusion',MarkerArray(markers=[sam]))==0
