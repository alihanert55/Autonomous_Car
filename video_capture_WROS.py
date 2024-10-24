#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

# Global değişkenler
save_directory = "/home/smart/Desktop/"  # Kaydedilecek dizin
video_filename = "captured_video.avi"  # Kaydedilecek video dosya adı
fps = 30  # Video FPS (Frame Per Second)

# CvBridge nesnesi oluştur
bridge = CvBridge()

# Video writer nesnesi oluştur
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
is_recording = False

# Görüntü mesajlarını alma işlemi
def image_callback(msg):
    global out, is_recording

    try:
        # ROS Image mesajını OpenCV formatına dönüştürmek için CvBridge kullanılır
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        if is_recording:
            # Video dosyası açık değilse, aç
            if out is None:
                file_path = os.path.join(save_directory, video_filename)
                out = cv2.VideoWriter(file_path, fourcc, fps, (cv_image.shape[1], cv_image.shape[0]))

            # Görüntüyü video dosyasına yaz
            out.write(cv_image)

    except CvBridgeError as e:
        rospy.logerr(e)

# Main fonksiyon
def main():
    global is_recording
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, image_callback)

    # Kayıt başlatma komutu
    start_recording()

    rospy.spin()

# Kayıt başlatma fonksiyonu
def start_recording():
    global is_recording, out
    is_recording = True
    rospy.loginfo("Recording started.")

# Kayıt durdurma fonksiyonu
def stop_recording():
    global is_recording, out
    is_recording = False
    if out is not None:
        out.release()
        rospy.loginfo("Recording stopped.")

if __name__ == '__main__':
    main()