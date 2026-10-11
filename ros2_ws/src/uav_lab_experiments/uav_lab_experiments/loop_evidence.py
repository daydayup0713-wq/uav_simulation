"""Count confirmed loop edges, excluding ordinary sequential graph edges."""
def loop_edge_count(backend, message):
    count=0
    for marker in message.markers:
        if marker.action!=marker.ADD:
            continue
        if backend=='lio_sam' and marker.ns=='loop_edges' and marker.type==marker.LINE_LIST:
            count+=len(marker.points)//2
        elif backend=='vins_fusion' and marker.ns=='CameraPoseVisualization' and marker.type==marker.LINE_STRIP:
            if marker.color.r>.9 and marker.color.g<.1 and marker.color.b<.1 and len(marker.points)==2:
                count+=1
    return count
