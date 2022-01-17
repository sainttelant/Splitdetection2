
import cv2

import os

 

#标定图像保存路径

photo_path = "/home/nvidia/camcalib/image"

#创建路径

def CreateFolder(path):

    #去除首位空格

    del_path_space = path.strip()

    #去除尾部'\'

    del_path_tail = del_path_space.rstrip('\\')

    #判读输入路径是否已存在

    isexists = os.path.exists(del_path_tail)

    if not isexists:

        os.makedirs(del_path_tail)

        return True

    else:

        return False

#获取不同角度的标定图像
image_width = 2560#2560#1920
image_height = 1440#1440#1080
rtsp_latency = 0
#cap = cv2.VideoCapture("rtsp://admin:Ucit-2119@10.203.204.192:554/Streaming/Channels/101")

#uri = "rtsp://admin:Ucit-2119@10.203.204.246:554/h264/ch1/sub/av_stream"
#uri = "rtsp://admin:Ucit-2119@10.8.5.27:554/h264/ch1/main/av_stream"
uri = "rtsp://admin:Ucit2020@10.8.5.27:554/h264/ch1/main/av_stream"
gst_str = ("rtspsrc location={} latency={} ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! videoconvert ! appsink sync=false").format(uri, rtsp_latency, image_width, image_height)
cap = cv2.VideoCapture(gst_str)
def gain_photo(photo_path):

    # 检查输入路径是否存在——不存在就创建

    #CreateFolder(photo_path)

    #开启摄像头

    ok,frame = cap.read() 

    #显示窗口名称

    #photo_window = 'calibration'

    cv2.namedWindow("cameracalibration",0)
    cv2.resizeWindow("cameracalibration", 1280, 720)
    #保存的标定图像名称以数量命名

    photo_num = 0
    
    #print("width = %d\n"%cap.get(cv2.CV_CAP_PROP_FRAME_WIDTH))
    #print("height = %d\n"%cap.get(cv2.CV_CAP_PROP_FRAME_HEIGHT))
    while True:

        ok,frame = cap.read()                                #读一帧的图像

        if not ok:

            break

        else:

            cv2.imshow("cameracalibration",frame)

            key = cv2.waitKey(10)

            #按键盘‘A’保存图像

            if key & 0xFF == ord('a'):

                photo_num += 1

                photo_name = photo_path + '/' + str(photo_num) + '.bmp'

                cv2.imwrite(photo_name,frame)

 

                print('create photo is :',photo_name)

            #按键盘‘Q’中断采集

            if key & 0xFF == ord('q'):

                break

 

if __name__ == '__main__':

    gain_photo(photo_path)
