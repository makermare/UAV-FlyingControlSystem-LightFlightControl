# Find Lines Example     By: Linlin's Space
#
# This example shows off how to find lines in the image. For each line object
# found in the image a line object is returned which includes the line's rotation.

# Note: Line detection is done by using the Hough Transform:
# http://en.wikipedia.org/wiki/Hough_transform
# Please read about it above for more information on what `theta` and `rho` are.

# find_lines() finds infinite length lines. Use find_line_segments() to find non-infinite lines.

enable_lens_corr = False # turn on for straighter lines...

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.B64X32)
sensor.skip_frames(time = 2000)
clock = time.clock()

# All line objects have a `theta()` method to get their rotation angle in degrees.
# You can filter lines based on their rotation angle.

min_degree = 0
max_degree = 179

a1=0;
b1=0;
c1=0;
a2=0;
b2=0;
c2=0;
last_x=0;
last_y=0;
# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.
i=0;
while(True):
    clock.tick()
    img = sensor.snapshot()
    if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

    # `threshold` controls how many lines in the image are found. Only lines with
    # edge difference magnitude sums greater than `threshold` are detected...

    # More about `threshold` - each pixel in the image contributes a magnitude value
    # to a line. The sum of all contributions is the magintude for that line. Then
    # when lines are merged their magnitudes are added togheter. Note that `threshold`
    # filters out lines with low magnitudes before merging. To see the magnitude of
    # un-merged lines set `theta_margin` and `rho_margin` to 0...

    # `theta_margin` and `rho_margin` control merging similar lines. If two lines
    # theta and rho value differences are less than the margins then they are merged.

    for l in img.find_lines(threshold = 1000, theta_margin = 50, rho_margin = 50):
        i=i+1;
        if(min_degree <= l.theta()) and (l.theta() <= max_degree):
            if(i==1):
                #img.draw_line(l.line(), color = (255, 0, 0))
                a1=l[3]-l[1];
                b1=l[0]-l[2];
                c1=l[2]*l[1]-l[0]*l[3];
            elif(i==2):

                a2=l[3]-l[1];
                b2=l[0]-l[2];
                c2=l[2]*l[1]-l[0]*l[3];
            else:
                img.draw_line(l.line(), color = (255, 0, 0));
    #if(a1/a2!=b1/b2):
    if(i>=2):
        if((a1*b2-a2*b1)!=0 and (a1*b2-a2*b1)!=0):
            img.draw_cross(int((b1*c2-b2*c1)/(a1*b2-a2*b1)),int((c1*a2-c2*a1)/(a1*b2-a2*b1)),10,color=[255,0,0]);
            last_x=int((b1*c2-b2*c1)/(a1*b2-a2*b1));
            last_y=int((c1*a2-c2*a1)/(a1*b2-a2*b1));
            print(last_x,last_y)
        else:
            img.draw_cross(last_x,last_y,5,color=[255,0,0]);
    else:
        img.draw_cross(last_x,last_y,5,color=[255,0,0]);
    i=0;

   # print("FPS %f" % clock.fps())

# About negative rho values:
#
# A [theta+0:-rho] tuple is the same as [theta+180:+rho].
