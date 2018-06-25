#!/usr/bin/env python
import qi
import rospy
from std_msgs.msg import Empty
from PIL import Image

video_service = 0

def picture_callback(data):
    resolution = 10    # VGA
    colorSpace = 11    # RGB

    videoClient   = video_service.subscribe("pepper_imitation_picture_node", resolution, colorSpace, 5)
    naoImage      = video_service.getImageRemote(videoClient)
    video_service.unsubscribe(videoClient)

    imageWidth  = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    image_string = str(bytearray(array))
    im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)
    im.show()

def main():
    global video_service
    rospy.init_node("pepper_imitation_picture_node", anonymous=True)
    rospy.Subscriber("pepper_imitation/cmd_take_picture", Empty, picture_callback)
    session = qi.Session()
    session.connect("tcp://" + rospy.get_param("host") + ":" + str(rospy.get_param("port")))
    video_service = session.service("ALVideoDevice");
    rospy.spin()

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
