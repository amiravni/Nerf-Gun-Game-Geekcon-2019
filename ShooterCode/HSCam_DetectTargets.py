from ximea import xiapi
import cv2
import time
import imutils
import threading

import numpy as np
import socket
import struct
import numpy as np
import time


class Camera:
    """
    Class that tracks the number of occurrences ("counts") of an
    arbitrary event and returns the frequency in occurrences
    (counts) per second. The caller must increment the count.
    """

    def __init__(self):
        # create instance for first connected camera
        self.cam = xiapi.Camera()

        # start communication
        print('Opening first camera...')
        self.cam.open_device()

        # cam.set_height(100)
        # cam.set_width(100)

        # self.cam.set_limit_bandwidth(49 * 8)
        # settings
        self.cam.set_exposure(1000)

        self.cam.set_imgdataformat('XI_MONO8')
        self.cam.set_acq_timing_mode('XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT')
        self.cam.set_downsampling("XI_DWN_2x2")
        self.cam.set_downsampling_type("XI_SKIPPING")
        self.cam.set_framerate(100)
        # cam.get_binning_horizontal_mode()

        # create instance of Image to store image data and metadata
        self.img = xiapi.Image()
        # start data acquisition
        print('Starting data acquisition...')
        self.cam.start_acquisition()
        tic = time.time()
        self.frame_idx = 0
        self.last_frame = None
        self.prev_frame = None
        self.stopped = False
        self.read_lock = threading.Lock()
        self.start()

    def start(self):
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        tic = time.time()
        while not self.stopped:
            self.cam.get_image(self.img)

            with self.read_lock:

                # create numpy array with data from camera. Dimensions of the array are
                # determined by imgdataformat
                self.prev_frame = self.last_frame
                self.last_frame = self.img.get_image_data_numpy()
            self.frame_idx += 1
            if self.frame_idx % 500 == 0:
                toc = time.time()
                print("FPS is : ", 500.0 / (toc - tic))
                tic = toc

    def getframe(self):
        while self.last_frame is None:
            time.sleep(0.1)
        with self.read_lock:
            frame = self.last_frame.copy()
            return frame

    def getdiff(self):
        while self.last_frame is None or self.prev_frame is None:
            time.sleep(0.1)
        # with self.read_lock:
        diff = self.last_frame.astype('float32') - self.prev_frame.astype('float32')
        return diff

    def stop(self):
        self.stopped = True
        self.thread.join()

    def __exit__(self, exec_type, exc_value, traceback):
        print('Stopping acquisition...')
        self.cam.stop_acquisition()
        # stop communication
        self.cam.close_device()


class TargetDetector:
    """
    Class that tracks the number of occurrences ("counts") of an
    arbitrary event and returns the frequency in occurrences
    (counts) per second. The caller must increment the count.
    """

    def __init__(self, camera, target_poly, detector_id):
        # create instance for first connected camera
        self.cam = camera
        self.target_poly = target_poly
        self.detector_id = detector_id
        self.stopped = False
        self.last_detected_time = 0
        self.read_lock = threading.Lock()
        self.start()

    def start(self):
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):

        while not self.stopped:
            ret = self.is_target_activated()
            if ret:
                with self.read_lock:
                    self.last_detected_time = time.time()
            time.sleep(0.01)

    def is_target_activated(self):
        mask = np.zeros(self.cam.last_frame.shape)
        mask2 = cv2.fillConvexPoly(mask, self.target_poly, (1, 1, 1), 1).astype(np.bool)

        target_diff = self.cam.getdiff() * mask2
        thresh = np.sum(np.abs(target_diff.astype('float32')) > 50)
        if thresh > 5:
            print('target #{}, 1st detection: {}, {}'.format(self.detector_id, thresh,
                                                             np.max(np.abs(target_diff.astype('float32')))))
            frame_i = self.cam.frame_idx
            # while self.cam.frame_idx - frame_i < 1:
            #     time.sleep(0.005)
            #
            # target_diff = self.cam.getdiff() * mask2
            # thresh = np.sum(np.abs(target_diff.astype('float32')) > 30)
            # if thresh > 5:
            #     print('target #{}, 2nd detection: {}, {}'.format(self.detector_id, thresh,
            #                                                      np.max(np.abs(target_diff.astype('float32')))))
            return True

        return False

    def get_time_since_last_event(self):
        with self.read_lock:
            return time.time() - self.last_detected_time

    def stop(self):
        self.stopped = True
        self.thread.join()


class LedControl:

    def __init__(self):
        # create instance for first connected camera

        self.UDP_IP = "192.168.2.101"
        self.UDP_PORT = 5555
        self.NumBoxes = 32

        # Every 5 Bytes are command to one box
        # 0 - Box num (0--> 59)
        # 1 - Opcode ( 0 - On, 1 - Off, 10 to 255 - animation*times)
        #    2-4 - r,g,b
        self.sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_DGRAM)  # UDP

    def target_map(self, target):
        target_vec = [1, 2, 0, 3, 6, 5, 7, 4, 11, 8, 10, 9, 13, 12, 14, 15, 17, 18, 16, 19, 20, 23, 21, 22, 25, 24, 26,
                      27, 28, 30, 29, 31]
        return target_vec[target]

    def send_single(self, target, op, r, g, b):
        print(target)
        vec = [target, op, r, g, b]
        # print(vec)
        MESSAGE = struct.pack('B' * 5, *vec)

        self.sock.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))
        # time.sleep(0.01)
        # self.sock.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))
        # self.sock.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))

    def reset(self):
        vec = []
        # print(vec)
        for i in range(self.NumBoxes):
            vec.append(i)
            vec.append(0)
            vec.append(0)
            vec.append(255)
            vec.append(0)

        print(vec)
        MESSAGE = struct.pack('B' * 5 * self.NumBoxes, *vec)

        self.sock.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))


led_control = LedControl()
ignore_vec = []  # 12,13,14,15,24, 25,26,27]
for i in range(led_control.NumBoxes):
    if i in ignore_vec:
        led_control.send_single(i, 1, 0, 0, 0)
    else:
        led_control.send_single(i, 0, 0, 255, 0)
    time.sleep(0.05)

last_frame = None
camera = Camera()
# box_world_pts = np.array([[0.055,    0,      0],
#                          [0.114,    0.045,  0],
#                          [0.055,    0.09,   0],
#                          [0.114,    0.13,   0]])
box_world_pts = np.array([[0, 0, 0],
                          [1.96, 0, 0],
                          [1.96, 1.48, 0],
                          [0, 1.15, 0]], dtype='float32')
dx = 0.114
dy = 0.09
target_ver = np.array([[0, 0, 0],
                       [dy, 0, 0],
                       [dy, dx, 0],
                       [0.0, dx, 0]])
target_hor = np.array([[0, 0, 0],
                       [dx, 0, 0],
                       [dx, dy, 0],
                       [0.0, dy, 0]])
target_group_ver = []
for i in range(2):
    for j in range(2):
        target_group_ver.append(target_ver + np.array([dy, 0, 0]) * i + np.array([0, dx, 0]) * j)

target_group_hor = []
for i in range(2):
    for j in range(2):
        target_group_hor.append(target_hor + np.array([dx, 0, 0]) * i + np.array([0, dy, 0]) * j)
target_list = []
# offsets = [[1.72, 1.16, -0.06], [1.71, 0.19, -0.03] ,[1.32,0.92,-0.04],[1.05,0.03, -0.07], [0.67,0.85, -0.06], [0.0, 0.33,-0.00]]
offsets = [[1.72, 1.16, -0.06], [1.71, 0.19, -0.03], [1.32, 0.92, -0.04], [1.14, 0.57, -0.04], [1.07, 0.04, -0.07],
           [0.7, 0.87, -0.06], [0.65, 0.40, -0.04], [0.02, 0.34, -0.00]]
ver_masks = [0, 0, 0, 1, 0, 0, 1, 0]

for offset, ver_mask in zip(offsets, ver_masks):
    if ver_mask == 0:
        for tar in target_group_hor:
            target_list.append(tar + np.array(offset))
    else:
        for tar in target_group_ver:
            target_list.append(tar + np.array(offset))

print(target_list)


# box2_world_pts = np.array([[0,    0,      0],
#
# box1_world_pts = np.array([[0,    0,      0],
#                          [0.114,    0,  0],
#                          [0.114,    0.09,   0],
#                          [0.0,    0.09,   0]])
# box2_world_pts = np.array([[0,    0,      0],
#                          [0.114,    0,  0],
#                          [0.114,    0.09,   0],
#                          [0.0,    0.09,   0]])+np.array([0.116,0,0])
# box3_world_pts = np.array([[0,    0,      0],
#                          [0.114,    0,  0],
#                          [0.114,    0.09,   0],
#                          [0.0,    0.09,   0]])+np.array([0.0,0.09,0])
# box4_world_pts = np.array([[0,    0,      0],
#                          [0.114,    0,  0],
#                          [0.114,    0.09,   0],
#                          [0.0,    0.09,   0]])+np.array([0.116,0.09,0])

def draw_circle(event, x, y, flags, param):
    global mouseX, mouseY, mouse_clicked
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # cv2.circle(img,(x,y),100,(255,0,0),-1)
        mouseX, mouseY = x, y
        mouse_clicked = True


def draw_axis(img, R, t, K, dist):
    # unit is mm
    rotV, _ = cv2.Rodrigues(R)
    points = np.float32([[0.10, 0, 0], [0, 0.10, 0], [0, 0, 0.10], [0, 0, 0]]).reshape(-1, 3)
    axisPoints, _ = cv2.projectPoints(points, R, t, K, dist)
    # print(axisPoints)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255, 0, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0, 255, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0, 0, 255), 3)
    return img


mtx = np.load("mtx.npy")
dist = np.load("dist.npy")
pts = []
# pts = [[138, 173], [788, 187],[902, 653], [134, 593]]
pts = [[180, 148],[839, 164],[952, 640],[173, 571]]
global mouseX, mouseY, mouse_clicked
cv2.namedWindow('image')
cv2.setMouseCallback('image', draw_circle)
mouse_clicked = False
axis_exist = False
frame_idx = 0
try:
    print('Starting video. Press CTRL+C to exit.')
    t0 = time.time()
    while True:
        target_detected = []
        data = camera.getframe()
        data = cv2.GaussianBlur(data, (5, 5), 0)
        rgb_data = cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)

        # # print(data.shape)
        if last_frame is None:
            last_frame = data
            # pts = [[198, 107], [847,  88], [888, 579], [138, 468]]
        # pts = [[184, 46],[728, 57],[767, 608],[108, 610]]
        # pts = [[213, 136],[841, 111],[884, 580],[148, 474]]
        if len(pts) < 4 and mouse_clicked:
            print(mouseX, mouseY)
            mouse_clicked = False
            pts.append([mouseX, mouseY])

        if not axis_exist and len(pts) == 4:
            pts = np.array(pts)
            temp_pts = pts.reshape((4, 1, 2))
            print(temp_pts)
            ret, rvec, tvec = cv2.solvePnP(box_world_pts, temp_pts.astype('float32'), mtx, dist, flags=cv2.SOLVEPNP_P3P)
            axis_exist = True
            box_poly_list = []
            detectors_list = []
            for target in target_list:
                box1_poly, _ = cv2.projectPoints(target.reshape(-1, 3), rvec, tvec, mtx, dist)
                detectors_list.append(TargetDetector(camera, box1_poly.astype('int32')[:, 0, :], len(detectors_list)))
                box_poly_list.append(box1_poly.astype('int32')[:, 0, :])

        for point in pts:
            #print(point)
            cv2.circle(rgb_data, (point[0], point[1]), 10, (12, 200, 82), -1)

        if axis_exist:
            rgb_data = draw_axis(rgb_data, rvec, tvec, mtx, dist)
            #
            #
            #
            # diff = data.astype('float32') - last_frame.astype('float32')
            # mask = np.zeros((data.shape))
            detector_idx = 0
            for detector in detectors_list:
                if detector.get_time_since_last_event() < 0.1:
                    print("diff was detected in target #{}, {}".format(detector_idx,
                                                                       detector.get_time_since_last_event()))
                    target_detected.append(detector.detector_id)
                detector_idx += 1
            # mask_total = mask.astype(np.bool)
            target_idx = 0
            for box_poly in box_poly_list:
                cv2.polylines(rgb_data, [box_poly.astype(np.int32)], True, (0, 255, 0), thickness=3)
                if target_idx in target_detected:
                    print("fill poly for target {}".format(target_idx))

                    rgb_data = cv2.fillConvexPoly(rgb_data, box_poly, (255, 0, 0), 1)
                    led_control.send_single(led_control.target_map(target_idx), 0, 255, 0, 0)
                target_idx += 1

        cv2.imshow('image', rgb_data)
        # cv2.imwrite('/media/gal/DATA/tmp/img{}.jpg'.format(frame_idx),rgb_data)
        frame_idx += 1

        # cv2.waitKey(1)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            print("breaking")
            break
        if key & 0xFF == ord('r'):
            print("reseting")
            led_control.reset()
            print(type(target_detected))
        # if len(target_deteqcted) > 0:
        #     time.sleep(2)
        # last_frame = data
        #
        # # frame_idx += 1
        # toc = time.time()
        # if toc - tic > 5:
        #     print("FPS is : ", float(frame_idx) / (toc - tic))
        #     tic = toc
        #     frame_idx = 0

except KeyboardInterrupt:
    cv2.destroyAllWindows()

finally:
    cv2.destroyAllWindows()
    # # stop data acquisition
    # print('Stopping acquisition...')
    # cam.stop_acquisition()
    #
    # # stop communication
    # cam.close_device()
    camera.stop()
    try:
        for detector in detectors_list:
            detector.stop()
    except Exception as err:
        print("[Exception] - in finnaly: {}".format(err))

print('Done.')
