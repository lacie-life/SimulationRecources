import airsim
from pathlib import Path
import numpy as np
import time
import rospy
from rospy import Time
import rosbag
from sensor_msgs.msg import Imu, Image, CompressedImage
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

import pprint
import tempfile
import os
import time
import cv2

pp = pprint.PrettyPrinter(indent=4)


_compress = False
# saveDir = str(Path.home())+ '/Datasets/Airsim/'
client = airsim.VehicleClient()
client.confirmConnection()

rospy.init_node('writer', anonymous=True)
bridge = CvBridge()
bag = rosbag.Bag('H:/Dataset/RosBag/static4.bag', 'w')

ctime = Time.now()
# ctime_sec = ctime.to_sec()

count_img = 0
count_IMU = 0
interval_IMU = 0.005
interval_img = 0.25

def getBinocularImg(count = 0, time=Time.now(), need_compress = False):
    responds = client.simGetImages([
        airsim.ImageRequest("RGB_Left", airsim.ImageType.Scene, compress=need_compress),
        airsim.ImageRequest("RGB_Right", airsim.ImageType.Scene, compress=need_compress)
    ])
    # get numpy array
    imgL1d = np.frombuffer(responds[0].image_data_uint8, dtype=np.uint8)
    imgL_rgb = imgL1d.reshape(responds[0].height, responds[0].width, 3)
    cv2.imwrite("Left/" + str(count_img) + ".png", imgL_rgb)

    imgR1d = np.frombuffer(responds[1].image_data_uint8, dtype=np.uint8)
    imgR_rgb = imgR1d.reshape(responds[1].height, responds[1].width, 3)
    cv2.imwrite( "Right/" + str(count_img) + ".png", imgR_rgb)

    msg_L = bridge.cv2_to_imgmsg(cvim=imgL_rgb, encoding="bgr8")
    msg_L.header.seq = count
    msg_L.header.stamp.set(time.secs, time.nsecs)

    msg_R = bridge.cv2_to_imgmsg(cvim=imgR_rgb, encoding="bgr8")
    msg_R.header.seq = count
    msg_R.header.stamp.set(time.secs, time.nsecs)

    return msg_L, msg_R

def getGroundTruthImg(count = 0, time=Time.now(), need_compress = False):
    responds = client.simGetImages([
        airsim.ImageRequest("RGB_Left", airsim.ImageType.DepthPlanar, pixels_as_float=True, compress=need_compress),
        airsim.ImageRequest("RGB_Left", airsim.ImageType.Segmentation, compress=need_compress)
    ])
    # get numpy array
    img_depth = airsim.list_to_2d_float_array(responds[0].image_data_float, responds[0].width, responds[0].height)

    img_buf = np.frombuffer(responds[1].image_data_uint8, dtype=np.uint8)
    imgR_seg = img_buf.reshape(responds[1].height, responds[1].width, 3)

    msg_depth = bridge.cv2_to_imgmsg(cvim=img_depth, encoding="passthrough")
    msg_depth.header.seq = count
    msg_depth.header.stamp.set(time.secs, time.nsecs)

    msg_seg = bridge.cv2_to_imgmsg(cvim=imgR_seg, encoding="bgr8")
    msg_seg.header.seq = count
    msg_seg.header.stamp.set(time.secs, time.nsecs)

    return msg_depth, msg_seg

if __name__ == "__main__":
    # waitForTakeOff()
    is_flying = airsim.wait_key('c')
    responses = client.simGetImages([
        airsim.ImageRequest("RGB_Left", airsim.ImageType.Scene),
        airsim.ImageRequest("RGB_Right", airsim.ImageType.Scene)])

    tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")

    x = 0
    while (x < 20):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.15), airsim.to_quaternion(0, 0, x*0.0174532925)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 20):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.15), airsim.to_quaternion(0, 0, x*(-1)*0.0174532925)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 20):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.75), airsim.to_quaternion(0, 0, x*0.0174532925)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 20):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.75), airsim.to_quaternion(0, 0, x*(-1)*0.0174532925)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 10):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -2.5), airsim.to_quaternion(x*0.0174532925, 0, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 10):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -2.5), airsim.to_quaternion(x*(-1)*0.0174532925, 0, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.15), airsim.to_quaternion(0, x*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 0, -3.15), airsim.to_quaternion(0, x*(-1)*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 1, -2.65), airsim.to_quaternion(0, x*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, 1, -2.65), airsim.to_quaternion(0, x*(-1)*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, -1, -2.65), airsim.to_quaternion(0, x*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    x = 0
    while (x < 30):
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(-1, -1, -2.65), airsim.to_quaternion(0, x*(-1)*0.0174532925, 0)), True)
        img_Time = Time.from_sec(ctime.to_sec() + count_img * interval_img)
        msg_L, msg_R = getBinocularImg(count=count_img, time=img_Time, need_compress=False)

        for i, response in enumerate(responses):
            if response.pixels_as_float:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
                airsim.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), airsim.get_pfm_array(response))
            else:
                print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
                airsim.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(count_img) + "_" + str(i) + '.png')), response.image_data_uint8)

        bag.write(topic='/cam0/image_raw', msg=msg_L, t=img_Time)
        bag.write(topic='/cam1/image_raw', msg=msg_R, t=img_Time)
        print('- Get Image: ', count_img)
        count_img += 1
        x = x + 1
        time.sleep(0.25)

    bag.close()
    client.simPause(False)
    