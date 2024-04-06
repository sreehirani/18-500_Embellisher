from Motor_Control_Classes import *

wheels = MecWheels(0.5) 
pick_up = PickUpMechanism()

def path_finding_algo(bboxes, scores, labels):
    pick_up_activated = False
    # no objects detected
    if len(bboxes) == 0:
        # all the objects are picked up
        if pick_up_activated:
            pick_up.deactivate()
            pick_up_activated = False
        # note: MecWheel.rotate should have angle as a parameter
        wheels.rotate(45)
    else:
        print(f"{len(bboxes)} number of objects to be collected")
        pick_up_activated = True
        pick_up.activate()
        time.sleep(2)
        wheels.move_forward()
    return
