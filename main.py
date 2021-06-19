import sensor, image, time, math , pyb

enable_lens_corr = False # turn on for straighter lines...
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.

# The AprilTags library outputs the pose information for tags. This is the x/y/z translation and
# x/y/z rotation. The x/y/z rotation is in radians and can be converted to degrees. As for
# translation the units are dimensionless and you must apply a conversion function.

# f_x is the x focal length of the camera. It should be equal to the lens focal length in mm
# divided by the x sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# f_y is the y focal length of the camera. It should be equal to the lens focal length in mm
# divided by the y sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# c_x is the image x center position in pixels.
# c_y is the image y center position in pixels.

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

car_stop_1=0
car_stop_2=0
line_detection=1
april_tag_detection=0
initial_flag=1

while(line_detection):


   clock.tick()
   img = sensor.snapshot()
   if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

   # `merge_distance` controls the merging of nearby lines. At 0 (the default), no
   # merging is done. At 1, any line 1 pixel away from another is merged... and so
   # on as you increase this value. You may wish to merge lines as line segment
   # detection produces a lot of line segment results.

   # `max_theta_diff` controls the maximum amount of rotation difference between
   # any two lines about to be merged. The default setting allows for 15 degrees.


   uart = pyb.UART(3,9600,timeout_char=1000)
   uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)


   for l in img.find_line_segments(merge_distance = 5, max_theta_diff = 5):
     img.draw_line(l.line(), color = (255, 0, 0))
     #print(l)
     if(l.y2()==0 and initial_flag==1 ):
        initial_x2=l.x2()
        initial_flag=0
        uart.write("/mission/run 0 0 \n")


     elif(l.y1()>2 and l.y1()<10 and l.length()>25 and l.length()<40 and l.theta()>80 and l.theta()<100):
        print("car stop!")
        print(l)
        car_stop_1=1

        time.sleep(2.5)

        uart.write("/stop/run \n")

        april_tag_detection=1
        line_detection=0

        uart.write("/task/run 1 1 \n")
        uart.write("/goStraight/run 35\n")


     if(l.y2()==0 and car_stop_1==0):
        print(l)
        diff=(l.x2()-initial_x2)/900
        if(diff < 0):
          factor=str(-0.75+diff)
        else :
          factor=str(0.75-diff)
        print(factor)

        command="/turn/run 35 " + factor +"\n"

        uart.write(command)
        time.sleep(0.5)

        uart.write("/goStraight/run 35\n")

while(april_tag_detection):
   clock.tick()
   img = sensor.snapshot()


   uart = pyb.UART(3,9600,timeout_char=1000)
   uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

   for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
      img.draw_rectangle(tag.rect(), color = (255, 0, 0))
      img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))

      print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
            degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
      # Translation units are unknown. Rotation units are in degrees.

      if(degrees(tag.y_rotation())-180<0 and car_stop_2==0):
        distance=math.sqrt(pow((tag.z_translation())*(-7.025),2)+pow(6,2)-2*((tag.z_translation())*(-7.025))*(6)*(math.cos(tag.y_rotation())))-20

        turn_angles=degrees(math.asin(6*(math.sin(tag.y_rotation()))/distance))
        factor= str((0.0149)*(turn_angles) - 0.9)
        print("factor=%s" %factor)
        command="/turn/run 100 " + factor +"\n"

        uart.write(command)
        time.sleep(1.1)


        if(distance>22):
            distance_1=42
            speed=str(13.95*(math.exp(0.0628*(distance_1))))
            command = "/goStraight/run " + speed + " \n"
            uart.write(command)
            time.sleep(2)

            #uart.write("/stop/run \n")

            distance_2=distance-12
            speed=str(13.95*(math.exp(0.0628*(distance_2))))
            command = "/goStraight/run " + speed + " \n"
            uart.write(command)
            time.sleep(2)


        else:
            speed=str(13.95*(math.exp(0.0628*(distance+22))))
            print("distance=%s" %distance)

            command = "/goStraight/run " + speed + " \n"
            uart.write(command)
            time.sleep(2)

        uart.write("/stop/run \n")

        turn_angles_2=degrees(tag.y_rotation())+turn_angles

        print("turn angles_2=%f"%turn_angles_2)

        factor= str( (-0.0149)*(turn_angles_2) + 0.93)
        command="/turn/run 100 " + factor +"\n"

        uart.write(command)
        time.sleep(1.4)

        uart.write("/stop/run \n")
        april_tag_detection=0
        car_stop_2=1
        uart.write("/task/run 3 3 \n")
        uart.write("/task4/run 0 0 \n")

      print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
   #print(clock.fps())
