from Motor_Control_Classes import *

def path_finding_algo(bboxes, scores, labels):
    if len(bboxes) == 0:
        move_to_different_areas()
    else:
        for i in range(len(bboxes)):
            bbox = bboxes[i]
            for j in range(len(bbox)):
                x0, y0, x1, y1 = bbox[j][0], bbox[j][1], bbox[j][2], bbox[j][3]
                
    return
        
 def move_to_different_areas():
    pass   