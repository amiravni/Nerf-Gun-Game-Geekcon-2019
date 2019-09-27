import sys
import os
import time
import numpy as np
import serial  ### pip install serial, pip install pyserial
import struct
import logging
# import dlib
import copy

from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time

# Threads
# from thread import start_new_thread
from threading import Lock
import threading

# from matplotlib import pyplot as plt

# from ovrsdk import *  ### pip install python-ovrsdk
# from Quaternion import Quat
import binascii
import cv2  #### DONOT: pip install opencv-python DO: sudo apt-get install python-opencv

# import face_recognition  # https://github.com/ageitgey/face_recognition

NUMOFSCREENS = 2
MAX_FACES = 10
serialExist = True

wanted_dir = "./examples/wanted/"
boring_dir = "./examples/boring/"
camera_L = 1
camera_R = 0
ArduinoCOM = '/dev/ttyACM0'
factor_LR = 0.6
factor_UD = 0.1
factor_line = 0.2
DRAW_RIGHT_EYE = True
cy_sub = 540
cx_sub = 960
dx_sub = 150
dy_sub = 100
tmp_dy_sub = 80
tmp_dx_sub = 0

left_camera_image = -1
right_camera_image = -1
face_locations_global = []
face_names_global = []
face_interest_global = []
face_landmarks_list_global = []
wanted_list_global = []
tracking_frame_global = []
wanted_sz = 100

grab_data = True
track_data = False
dx = 0
dy = 0
frame_lock = Lock()

cap = cv2.VideoCapture(0)
key = 0
dec = 0
ra = 0
killFlag = False

lock = threading.Lock()

fov_h = 61  # DEG
fov_v = 34.4

error_in_deg_v = 0
error_in_deg_h = 0

mouseX = mouseY = 0
update_tracker = False

lk_params = dict(winSize = (15, 15),
                 maxLevel = 4,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
minimun_pts_track = 10
tarcking_window_height = 10
tracking_window_width = 10
bias_h = 0
bias_v = 0
p_bias_h = 0
p_bias_v = 0
e_h = 0
e_v = 0
p_e_h = 0
p_e_v = 0

track_center = np.zeros(2)

def draw_circle(event, x, y, flags, param):
    global gray
    global mouseX, mouseY
    global error_in_deg_v, error_in_deg_h
    global update_tracker
    global start_tracking,track_x,track_y, bias_h,bias_v , width , height , p_bias_h ,p_bias_v,track_center,e_h,e_v , p_e_h , p_e_v

    if event == cv2.EVENT_LBUTTONDBLCLK:
        update_tracker = True


        mouseX, mouseY = x, y
        start_tracking = True
        track_x = x
        track_y = y
        p_bias_h = bias_h
        p_bias_v = bias_v
        p_e_h = e_h
        p_e_v = e_v
        bias_h += width / 2 - x
        bias_v += height / 2 - y
        if track_center[0]:
            e_h = width / 2 - track_center[0]
            e_v = height / 2 - track_center[1]
        print(p_e_h,p_e_v,p_bias_h,p_bias_v,bias_h,bias_v,x, y)

def camera_thread_better():
    global key, ra, dec, killFlag, error_in_deg_h, error_in_deg_v
    global gray
    global mouseX, mouseY
    global update_tracker
    global start_tracking,track_x,track_y , bias_h,bias_v , width , height, p_bias_h ,p_bias_v,track_center , p_e_h , p_e_v

    start_tracking = False
    is_tracking = False
    track_x, track_y = (0, 0)
    tracking_corners = None

    lk_params = dict(winSize=(15, 15),
                     maxLevel=4,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    minimun_pts_track = 10

    cam = cv2.VideoCapture(0)

    cv2.namedWindow('viewer', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("viewer", draw_circle)

    old_gray = None

    while True:
        ret, frame = cam.read()

        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape



            if is_tracking:
                new_points, status, error = cv2.calcOpticalFlowPyrLK(old_gray, gray, tracking_corners, None,
                                                                     **lk_params)
                tracking_corners = new_points

                if tracking_corners.shape[0] < minimun_pts_track:
                    for i in range(-5, 5):
                        for j in range(-5, 5):
                            tracking_corners.append([track_x + i, track_y + j])
                    tracking_corners = np.array(tracking_corners).reshape(len(tracking_corners), 1, 2).astype('float32')

                track_center = tracking_corners[:, 0, :].mean(axis=0).astype('int32')
                #print(track_center)
                # x, y = new_points.ravel()
                cv2.circle(frame, (track_center[0], track_center[1]), 5, (0, 255, 0), -1)

                #x = track_center[0]
                #y = track_center[1]


                pix_to_deg_v = height / fov_v
                pix_to_deg_h = width / fov_h

                error_x = (width / 2 + p_bias_h - p_e_h) - track_center[0]
                error_y = (height / 2 + p_bias_v - p_e_v) - track_center[1]

                error_in_deg_v = error_y / pix_to_deg_v
                error_in_deg_h = error_x / pix_to_deg_h

                #print(error_x,error_y)

            if start_tracking:
                tracking_corners = []
                for i in range(-5, 5):
                    for j in range(-5, 5):
                        tracking_corners.append([track_x + i, track_y + j])
                tracking_corners = np.array(tracking_corners).reshape(len(tracking_corners), 1, 2).astype('float32')
                # print(tracking_corners)
                start_tracking = False
                is_tracking = True

            cv2.line(frame, (width / 2, height / 2 - 10), (width / 2, height / 2 + 10), (0, 255, 0), 3)
            cv2.line(frame, (width / 2 - 10, height / 2), (width / 2 + 10, height / 2), (0, 255, 0), 3)

            old_gray = gray.copy()

            cv2.imshow("viewer", frame)
            cv2.waitKey(1)



def camera_thread():
    global key, ra, dec, killFlag, error_in_deg_h, error_in_deg_v
    global gray
    global mouseX, mouseY
    global update_tracker

    dx = dy = 25

    # mouseX, mouseY = -1, -1
    # cap = cv2.VideoCapture(0)
    #
    # ret, frame = cap.read()
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.namedWindow('frame')
    # cv2.imshow('frame', gray)
    # cv2.setMouseCallback('frame', draw_circle)
    # while (True):
    #     # Capture frame-by-frame
    #     ret, frame = cap.read()
    #
    #     # Our operations on the frame come here
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     height, width = gray.shape
    #
    #     pix_to_deg_v = height / fov_v
    #     pix_to_deg_h = width / fov_h
    #
    #     # Display the resulting frame
    #     cv2.line(gray, (width / 4, height / 4 - 10), (width / 4, height / 4 + 10), (0, 255, 0), 3)
    #     cv2.line(gray, (width / 4 - 10, height / 4), (width / 4 + 10, height / 4), (0, 255, 0), 3)
    #
    #     cv2.line(gray, (3 * width / 4, 3 * height / 4 - 10), (3 * width / 4, 3 * height / 4 + 10), (0, 255, 0), 3)
    #     cv2.line(gray, (3 * width / 4 - 10, 3 * height / 4), (3 * width / 4 + 10, 3 * height / 4), (0, 255, 0), 3)
    #
    #     cv2.line(gray, (width / 4, 3 * height / 4 - 10), (width / 4, 3 * height / 4 + 10), (0, 255, 0), 3)
    #     cv2.line(gray, (width / 4 - 10, 3 * height / 4), (width / 4 + 10, 3 * height / 4), (0, 255, 0), 3)
    #
    #     cv2.line(gray, (3 * width / 4, height / 4 - 10), (3 * width / 4, height / 4 + 10), (0, 255, 0), 3)
    #     cv2.line(gray, (3 * width / 4 - 10, height / 4), (3 * width / 4 + 10, height / 4), (0, 255, 0), 3)
    #
    #     if mouseX > -1 and mouseY > -1:
    #         cv2.circle(gray, (mouseX, mouseY), 10, (0, 0, 0), thickness=3, lineType=8, shift=0)
    #
    #     cv2.circle(gray, (width / 2, height / 2), 10, (22, 222, 22), thickness=3, lineType=8, shift=0)
    #
    #     error_x = width / 2 - mouseX
    #     error_y = height / 2 - mouseY
    #
    #     error_in_deg_v = error_y / pix_to_deg_v
    #     error_in_deg_h = error_x / pix_to_deg_h
    #
    #     print (error_in_deg_h, error_in_deg_v)
    #     cv2.imshow('frame', gray)
    #
    #     # print(cv2.waitKey(1))
    #
    #     temp = 0
    #     lock.acquire()
    #     try:
    #         temp = ra
    #     finally:
    #         lock.release()
    #
    #     key = cv2.waitKey(1)
    #     if key & 0xFF == ord('q'):
    #         print("breaking")
    #         break
    #     if key & 0xFF == ord('w'):
    #         temp = temp + 5
    #         print("ra(temp): {}".format(temp))
    #     if key & 0xFF == ord('s'):
    #         temp = temp - 5
    #         print("ra(temp): {}".format(temp))
    #
    #     lock.acquire()
    #     try:
    #         ra = temp
    #     finally:
    #         lock.release()
    #
    # # When everything done, release the capture
    # cap.release()
    # cv2.destroyAllWindows()
    # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the

    # initialize a dictionary that maps strings to their corresponding
    # OpenCV object tracker implementations
    OPENCV_OBJECT_TRACKERS = {
        # "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        # "mosse": cv2.TrackerMOSSE_create
    }

    # grab the appropriate object tracker using our dictionary of
    # OpenCV object tracker objects
    # tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
    tracker = OPENCV_OBJECT_TRACKERS['medianflow']()

    # initialize the bounding box coordinates of the object we are going
    # to track
    initBB = None

    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)

    # initialize the FPS throughput estimator
    fps = None

    # loop over frames from the video stream
    while True:
        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape

        # check to see if we have reached the end of the stream
        if frame is None:
            break

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        # frame = imutils.resize(frame, width=500)
        (H, W) = frame.shape[:2]

        # check to see if we are currently tracking an object
        if initBB is not None:
            # grab the new bounding box coordinates of the object
            (success, box) = tracker.update(frame)

            # check to see if the tracking was a success
            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + dx, y + dy),
                              (0, 255, 0), 2)
                error_x = width / 2 - x
                error_y = height / 2 - y

                error_in_deg_v = error_y / pix_to_deg_v
                error_in_deg_h = error_x / pix_to_deg_h

                # print (error_in_deg_h, error_in_deg_v)

            # update the FPS counter
            fps.update()
            fps.stop()

            # initialize the set of information we'll be displaying on
            # the frame
            info = [
                ("Tracker", 'medianflow'),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())),
            ]

            # loop over the info tuples and draw them on our frame
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # check to see if we are currently tracking an object
        pix_to_deg_v = height / fov_v
        pix_to_deg_h = width / fov_h

        # Display the resulting frame
        cv2.line(gray, (width / 4, height / 4 - 10), (width / 4, height / 4 + 10), (0, 255, 0), 3)
        cv2.line(gray, (width / 4 - 10, height / 4), (width / 4 + 10, height / 4), (0, 255, 0), 3)

        cv2.line(gray, (3 * width / 4, 3 * height / 4 - 10), (3 * width / 4, 3 * height / 4 + 10), (0, 255, 0), 3)
        cv2.line(gray, (3 * width / 4 - 10, 3 * height / 4), (3 * width / 4 + 10, 3 * height / 4), (0, 255, 0), 3)

        cv2.line(gray, (width / 4, 3 * height / 4 - 10), (width / 4, 3 * height / 4 + 10), (0, 255, 0), 3)
        cv2.line(gray, (width / 4 - 10, 3 * height / 4), (width / 4 + 10, 3 * height / 4), (0, 255, 0), 3)

        cv2.line(gray, (3 * width / 4, height / 4 - 10), (3 * width / 4, height / 4 + 10), (0, 255, 0), 3)
        cv2.line(gray, (3 * width / 4 - 10, height / 4), (3 * width / 4 + 10, height / 4), (0, 255, 0), 3)

        if update_tracker and mouseX > -1 and mouseY > -1:
            update_tracker = False
            #frame = vs.read()
            cv2.circle(frame, (mouseX, mouseY), 10, (0, 0, 0), thickness=3, lineType=8, shift=0)
            cv2.rectangle(frame, (mouseX - dx, mouseY - dy), (mouseX + dx, mouseY + dy), (0, 0, 255), 2)
            initBB = (mouseX - dx, mouseY - dy, mouseX + dx, mouseY + dy)
            # print (initBB)
            tracker = OPENCV_OBJECT_TRACKERS['medianflow']()
            tracker.init(frame, initBB)
            fps = FPS().start()

        cv2.circle(frame, (width / 2, height / 2), 10, (22, 222, 22), thickness=3, lineType=8, shift=0)

        # error_x = width / 2 - mouseX
        # error_y = height / 2 - mouseY

        # error_in_deg_v = error_y / pix_to_deg_v
        # error_in_deg_h = error_x / pix_to_deg_h

        # print (error_in_deg_h, error_in_deg_v)
        # show the output frame
        cv2.imshow("Frame", frame)
        cv2.setMouseCallback("Frame", draw_circle)
        key = cv2.waitKey(1) & 0xFF

        # if the 's' key is selected, we are going to "select" a bounding
        # box to track
        if key == ord("s"):
            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            initBB = cv2.selectROI("Frame", frame, fromCenter=False,
                                   showCrosshair=True)
            # print (initBB)

            # start OpenCV object tracker using the supplied bounding box
            # coordinates, then start the FPS throughput estimator as well
            tracker.init(frame, initBB)
            fps = FPS().start()
            # if the `q` key was pressed, break from the loop

        elif key == ord("q"):
            break

    # if we are using a webcam, release the pointer
    vs.stop()

    # # otherwise, release the file pointer
    # else:
    #     vs.release()

    # close all windows
    cv2.destroyAllWindows()
    killFlag = True


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


def oculus_handle():
    global key, ra, dec, error_in_deg_h, error_in_deg_v
    yaw_lastStep = 0
    pitch_lastStep = 0
    roll_lastStep = 0
    yaw_steps = 0
    pitch_steps = 0

    bufferLength = 2
    yawVec = np.zeros(bufferLength)
    pitchVec = np.zeros(bufferLength) + 127

    raiseFlag = True
    local_ra = 0
    while killFlag is False:

        # ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
        # pose = ss.Predicted.Pose
        # q = Quat([pose.Orientation.w, pose.Orientation.x, pose.Orientation.y,
        #           pose.Orientation.z])  # q.ra --> Pitch , q.dec --> Yaw , q.roll --> Roll
        # print q.ra, q.dec, q.roll

        # this part is true only for "pitch" of -90 to 90 degrees (The half dome infront of a person )
        #tic = time.time()
        lock.acquire()
        dec = error_in_deg_h * 1.0
        if dec > 45:
            decLim = 45.0
            raiseFlag = False
        elif dec < -45:
            raiseFlag = True
            decLim = -45.0
        else:
            decLim = dec

        steps_per_rev = 127
        yaw_newStep = ((decLim / 180) * steps_per_rev)
        yaw_lastStep = yaw_steps
        yaw_steps = int(round(yaw_newStep))

        yawVec[:-1]=yawVec[1:]
        yawVec[-1] = yaw_steps
        yaw_steps = np.mean(yawVec)

        # lock.acquire()
        try:
            #error_in_deg_h = error_in_deg_h % 360
            # local_ra = ra = error_in_deg_h*1.0
            local_ra = ra = error_in_deg_v * 1.0

            if local_ra > 23 and local_ra < 180:
                raLim = 23.0
            elif local_ra > 180 and local_ra < 338:
                raLim = 338.0
            else:
                raLim = local_ra
            #
            # if local_ra < -23:
            #     ra = local_ra = -23
            #     raiseFlag = True
            # if 23 < local_ra < 180:
            #     raLim = 23.0
            # elif 180 < local_ra < 338:
            #
            #     raLim = 338.0
            # else:
            #     raLim = local_ra
        finally:
            lock.release()

        if raLim <= 90 or raLim >= 270:
            raLim = np.mod(raLim + 180, 360)

        pitch_newStep = (((raLim) / 180) * steps_per_rev)
        pitch_lastStep = pitch_steps
        pitch_steps = int(round(pitch_newStep))

        pitchVec[:-1]=pitchVec[1:]
        pitchVec[-1] = pitch_steps
        pitch_steps = np.mean(pitchVec)

        #print(time.time() - tic)
        # print ra,raLim,pitch_steps
        #if yaw_steps != yaw_lastStep or pitch_steps != pitch_lastStep:
        #    print(dec, local_ra, yaw_steps, pitch_steps, raLim)
        # ser.write(struct.pack(2*'B',yaw_steps + 128,pitch_steps + 128))
        if serialExist:
            ser.write(struct.pack('BBB', yaw_steps + 128, pitch_steps, 10))
            #ser.write(struct.pack('BBB', 135, 135, 10))

            # ser.write(struct.pack('BBB', yaw_steps + 128, 128, 10))
            # ser.write(struct.pack('BBB', error_in_deg_h+128, error_in_deg_v, 10))
            #print (error_in_deg_h,yaw_steps)
        # ser.write(struct.pack('BBB', 64, pitch_steps, 10))
        time.sleep(0.01)
        if (serialExist):
            recv = ser.read(1024).lstrip().rstrip()
            if len(recv) > 0:
                print(recv)

        # dec = dec + 1s
        # if raiseFlag:
        #     ra = ra + 5
        # else:
        #     ra = ra - 5
        #time.sleep(0.01)


if __name__ == '__main__':
    # serialExist = False
    if serialExist:
        ser = serial.Serial(ArduinoCOM, baudrate=115200)
        ser.timeout = 0.002
        print("serial ==> " + str(ser.is_open))
        time.sleep(1)

    # start_new_thread(face_recognition_handle,())
    # start_new_thread(cameras_handle,())
    # start_new_thread(oculus_handle, ())
    # start_new_thread(track_handle,())
    logging.info("Main    : before creating thread")
    x = threading.Thread(target=oculus_handle, args=[])
    #y = threading.Thread(target=camera_thread, args=[])
    y = threading.Thread(target=camera_thread_better, args=[])

    camera_thread
    logging.info("Main    : before running thread")
    x.start()
    y.start()
    logging.info("Main    : wait for the thread to finish")
    x.join()
    logging.info("Main    : all done")
    # while True:
    #     time.sleep(0.01)
