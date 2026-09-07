from threading import Thread

import cv2
from djitellopy import Tello

tello = Tello(host='192.168.0.154')
tello.connect()
keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()



# def videoRecorder():
#     height, width, _ = frame_read.frame.shape
#     cur_time = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
#     video = cv2.VideoWriter(f'drones/recordings/{cur_time}.avi', cv2.VideoWriter_fourcc(*'XVID'), 15, (width, height))
#
#     while keepRecording:
#         video.write(frame_read.frame)
#         time.sleep(1 / 10)
#
#     video.release()


def display():
    global keepRecording
    while keepRecording:
        img = frame_read.frame
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert from BGR to RGB
        cv2.imshow("drone", img_rgb)

        key = cv2.waitKey(1) & 0xff
        if key == 27:  # ESC
            keepRecording = False
            break
        elif key == ord('w'):
            tello.move_forward(30)
        elif key == ord('s'):
            tello.move_back(30)
        elif key == ord('a'):
            tello.move_left(30)
        elif key == ord('d'):
            tello.move_right(30)
        elif key == ord('e'):
            tello.rotate_clockwise(30)
        elif key == ord('q'):
            tello.rotate_counter_clockwise(30)
        elif key == ord('r'):
            tello.move_up(30)
        elif key == ord('f'):
            tello.move_down(30)
        elif key == ord('j'):
            tello.send_command_without_return('[TELLO] DIY_start')
        elif key == ord('k'):
            tello.send_command_without_return('[TELLO] DIY_stop')
        elif key == ord('o'):
            tello.land()
        elif key == ord('p'):
            tello.takeoff()
        elif key == ord('m'):
            tello.enable_mission_pads()
            tello.set_mission_pad_detection_direction(0)
        elif key == ord('n'):
            tello.go_xyz_speed_mid(10, 0, 80, 20, 1)
            tello.go_xyz_speed_mid(10, 0, 31, 10, 1)
        # if key is space bar, stop all motors immediately
        elif key == 32:
            tello.emergency()
    tello.land()


# recorder = Thread(target=videoRecorder)
# recorder.start()

tello.takeoff()
tello.send_keepalive()

displayer = Thread(target=display)
displayer.start()


# recorder.join()
displayer.join()
cv2.destroyAllWindows()
