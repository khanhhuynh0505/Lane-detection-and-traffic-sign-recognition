#!/usr/bin/python3
#for virtual env compatibility /home/htqkhanh/catkin_ws/src/autobot/.venv/bin/python3

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
# import pdb
#from edge_detection import *
import matplotlib.pyplot as plt

# Global variables
prev_leftx = None
prev_lefty = None
prev_rightx = None
prev_righty = None
prev_left_fit = []
prev_right_fit = []

prev_leftx2 = None
prev_lefty2 = None
prev_rightx2 = None
prev_righty2 = None
prev_left_fit2 = []
prev_right_fit2 = []
msg_desired_center = Float64()
cx = 0

############################################################################################################################################################
class Lane:
  def __init__(self, orig_frame):
    self.orig_frame = orig_frame  
    self.lane_line_markings = None

    #image after perspective transformation
    self.warped_frame = None
    self.transformation_matrix = None
    self.inv_transformation_matrix = None

    self.orig_image_size = self.orig_frame.shape[::-1][1:]

    width = self.orig_image_size[0]
    height = self.orig_image_size[1]
    self.width = width
    self.height = height
    
    # coordinates region of interest
    self.roi_points = np.float32([
      (int(0.38*width),int(0.6*height)), # Top-left 
      (0.03*width, height-1), # Bottom-left            
      (int(0.965*width),height-1), # Bottom-right 
      (int(0.62*width),int(0.6*height)) # Top-right 
    ])
        
    # The desired corner locations  of the region of interest after perspective transformation.
    self.padding = int(0.25 * width) # padding from side of the image in pixels
    self.desired_roi_points = np.float32([
      [self.padding, 0], # Top-left corner
      [self.padding, self.orig_image_size[1]], # Bottom-left corner         
      [self.orig_image_size[
        0]-self.padding, self.orig_image_size[1]], # Bottom-right corner
      [self.orig_image_size[0]-self.padding, 0] # Top-right corner
    ]) 
    # Histogram shows the white pixel peaks
    self.histogram = None
        
    # Sliding window parameters
    self.no_of_windows = 10
    self.margin = int((1/12) * width)  # Window width is +/- margin
    self.minpix = int((1/24) * width)  # Min no. of pixels to recenter window
        
    # Best fit polynomial lines for left line and right line of the lane
    self.left_fit = None
    self.right_fit = None
    self.left_lane_inds = None
    self.right_lane_inds = None
    self.ploty = None
    self.left_fitx = None
    self.right_fitx = None
    self.leftx = None
    self.rightx = None
    self.lefty = None
    self.righty = None
        
    # Radii of curvature and offset
    # self.left_curvem = None
    # self.right_curvem = None
    # self.center_offset = None
      
  def calculate_histogram(self,frame=None,plot=True):
    # Calculate the image histogram to find peaks in white pixel count
    if frame is None:
      frame = self.warped_frame
            
    # Generate the histogram
    self.histogram = np.sum(frame[int(
              frame.shape[0]/2):,:], axis=0)

    if plot == True:
      figure, (ax1, ax2) = plt.subplots(2,1) # 2 row, 1 columns
      figure.set_size_inches(10, 5)
      ax1.imshow(frame, cmap='gray')
      ax1.set_title("Warped Binary Frame")
      ax2.plot(self.histogram)
      ax2.set_title("Histogram Peaks")
      plt.show()
            
    return self.histogram
   
  def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
    # Use the lane line from the previous sliding window to get the parameters for the polynomial line for filling in the lane line
    # margin is a sliding window parameter
    margin = self.margin

    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame.         
    nonzero = self.warped_frame.nonzero()  
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
        
    # Store left and right lane pixel indices
    left_lane_inds = ((nonzerox > (left_fit[0]*(
      nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (
      nonzerox < (left_fit[0]*(
      nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
    right_lane_inds = ((nonzerox > (right_fit[0]*(
      nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (
      nonzerox < (right_fit[0]*(
      nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))           
    self.left_lane_inds = left_lane_inds
    self.right_lane_inds = right_lane_inds

    # Get the left and right lane line pixel locations  
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]  

    global prev_leftx2
    global prev_lefty2 
    global prev_rightx2
    global prev_righty2
    global prev_left_fit2
    global prev_right_fit2
        
    # Make sure we have nonzero pixels      
    if len(leftx)==0 or len(lefty)==0 or len(rightx)==0 or len(righty)==0:
      leftx = prev_leftx2
      lefty = prev_lefty2
      rightx = prev_rightx2
      righty = prev_righty2

    self.leftx = leftx
    self.rightx = rightx
    self.lefty = lefty
    self.righty = righty
        
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2) 

    # Add the latest polynomial coefficients        
    prev_left_fit2.append(left_fit)
    prev_right_fit2.append(right_fit)

    # Calculate the moving average  
    if len(prev_left_fit2) > 10:
      prev_left_fit2.pop(0)
      prev_right_fit2.pop(0)
      left_fit = sum(prev_left_fit2) / len(prev_left_fit2)
      right_fit = sum(prev_right_fit2) / len(prev_right_fit2)

    self.left_fit = left_fit
    self.right_fit = right_fit
        
    prev_leftx2 = leftx
    prev_lefty2 = lefty 
    prev_rightx2 = rightx
    prev_righty2 = righty
        
    # Create the x and y values to plot on the image
    ploty = np.linspace(
      0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0]) 
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    self.ploty = ploty
    self.left_fitx = left_fitx
    self.right_fitx = right_fitx
        
    if plot==True:
        
      # Generate images to draw on
      out_img = np.dstack((self.warped_frame, self.warped_frame, (
                           self.warped_frame)))*255
      window_img = np.zeros_like(out_img)
            
      # Add color to the left and right line pixels
      out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
      out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
                                                                     0, 0, 255]
      # Create a polygon to show the search window area, and recast 
      # the x and y points into a usable format for cv2.fillPoly()
      margin = self.margin
      left_line_window1 = np.array([np.transpose(np.vstack([
                                    left_fitx-margin, ploty]))])
      left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([
                                    left_fitx+margin, ploty])))])
      left_line_pts = np.hstack((left_line_window1, left_line_window2))
      right_line_window1 = np.array([np.transpose(np.vstack([
                                     right_fitx-margin, ploty]))])
      right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([
                                     right_fitx+margin, ploty])))])
      right_line_pts = np.hstack((right_line_window1, right_line_window2))
            
      # Draw the lane onto the warped blank image
      cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
      cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
      result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
      
      # Plot the figures 
      figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(self.warped_frame, cmap='gray')
      ax3.imshow(result)
      ax3.plot(left_fitx, ploty, color='yellow')
      ax3.plot(right_fitx, ploty, color='yellow')
      ax1.set_title("Original Frame")  
      ax2.set_title("Warped Frame")
      ax3.set_title("Warped Frame With Search Window")
      plt.show()
            
  def get_lane_line_indices_sliding_windows(self, plot=False):
    # Sliding window width is +/- margin
    margin = self.margin

    frame_sliding_window = self.warped_frame.copy()

    # Set the height of the sliding windows
    window_height = np.int32(self.warped_frame.shape[0]/self.no_of_windows)       

    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame. 
    nonzero = self.warped_frame.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1]) 
        
    # Store the pixel indices for the left and right lane lines
    left_lane_inds = []
    right_lane_inds = []
        
    # Current positions for pixel indices for each window,
    # which we will continue to update
    leftx_base, rightx_base = self.histogram_peak()
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Go through one window at a time
    no_of_windows = self.no_of_windows
        
    for window in range(no_of_windows):
      
      # Identify window boundaries in x and y (and right and left)
      win_y_low = self.warped_frame.shape[0] - (window + 1) * window_height
      win_y_high = self.warped_frame.shape[0] - window * window_height
      win_xleft_low = leftx_current - margin
      win_xleft_high = leftx_current + margin
      win_xright_low = rightx_current - margin
      win_xright_high = rightx_current + margin
      cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
        win_xleft_high,win_y_high), (255,255,255), 2)
      cv2.rectangle(frame_sliding_window,(win_xright_low,win_y_low),(
        win_xright_high,win_y_high), (255,255,255), 2)

      # Identify the nonzero pixels in x and y within the window
      good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (
                           nonzerox < win_xleft_high)).nonzero()[0]
      good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (
                            nonzerox < win_xright_high)).nonzero()[0]
                                                        
      # Append these indices to the lists
      left_lane_inds.append(good_left_inds)
      right_lane_inds.append(good_right_inds)
        
      # If you found > minpix pixels, recenter next window on mean position
      minpix = self.minpix
      if len(good_left_inds) > minpix:
        leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
      if len(good_right_inds) > minpix:        
        rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
                    
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract the pixel coordinates for the left and right lane lines
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds] 
    righty = nonzeroy[right_lane_inds]

    # Fit a second order polynomial curve to the pixel coordinates for
    # the left and right lane lines
    left_fit = None
    right_fit = None
        
    global prev_leftx
    global prev_lefty 
    global prev_rightx
    global prev_righty
    global prev_left_fit
    global prev_right_fit

    # Make sure we have nonzero pixels      
    if len(leftx)==0 or len(lefty)==0 or len(rightx)==0 or len(righty)==0:
      leftx = prev_leftx
      lefty = prev_lefty
      rightx = prev_rightx
      righty = prev_righty
        
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2) 

    # Add the latest polynomial coefficients        
    prev_left_fit.append(left_fit)
    prev_right_fit.append(right_fit)

    # Calculate the moving average  
    if len(prev_left_fit) > 10:
      prev_left_fit.pop(0)
      prev_right_fit.pop(0)
      left_fit = sum(prev_left_fit) / len(prev_left_fit)
      right_fit = sum(prev_right_fit) / len(prev_right_fit)

    self.left_fit = left_fit
    self.right_fit = right_fit
        
    prev_leftx = leftx
    prev_lefty = lefty 
    prev_rightx = rightx
    prev_righty = righty

    # test
    ploty = np.linspace(
        0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    cx = int(round(np.mean([left_fitx[239], right_fitx[239]], axis=0)))
    msg_desired_center.data = cx
    #print("Cx: ",cx)

    if plot==True:
        
      # Create the x and y values to plot on the image  
      ploty = np.linspace(
        0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
      left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
      right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

      # Generate an image to visualize the result
      out_img = np.dstack((
        frame_sliding_window, frame_sliding_window, (
        frame_sliding_window))) * 255
            
      # Add color to the left line pixels and right line pixels
      out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
      out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
        0, 0, 255]
                
      # Plot the figure with the sliding windows
      figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(frame_sliding_window, cmap='gray')
      ax3.imshow(out_img)
      ax3.plot(left_fitx, ploty, color='yellow')
      ax3.plot(right_fitx, ploty, color='yellow')
      ax1.set_title("Original Frame")  
      ax2.set_title("Warped Frame with Sliding Windows")
      ax3.set_title("Detected Lane Lines with Sliding Windows")
      plt.show()        
            
    return self.left_fit, self.right_fit

  def get_line_markings(self, frame=None):
    if frame is None:
      frame = self.orig_frame
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # #cv2.imwrite('/home/mastersgray.jpg', gray) 
    # # filter image to remove noise or smoothing the image
    gray = cv2.medianBlur(gray,9)
    # # gray = self.filter.update(gray)

    # # create a binary thresholded image on grayscale image between white and yellow
    # #thresh = cv2.inRange(gray, 190, 255)
    self.lane_line_markings = cv2.inRange(gray, 190, 255)
    # cv2.imshow("markings",self.lane_line_markings)
    # cv2.waitKey(0)
    return self.lane_line_markings
        
  def histogram_peak(self):
    """
    Get the left and right peak of the histogram
    Return the x coordinate of the left histogram peak and the right histogram peak.
    """
    midpoint = np.int32(self.histogram.shape[0]/2)
    leftx_base = np.argmax(self.histogram[:midpoint])
    rightx_base = np.argmax(self.histogram[midpoint:]) + midpoint

    #print("Left Base ", leftx_base, " Right Base ",rightx_base)

    # (x coordinate of left peak, x coordinate of right peak)
    return leftx_base, rightx_base
        
  def overlay_lane_lines(self, plot=False):
    # Generate an image to draw the lane lines on 
    warp_zero = np.zeros_like(self.warped_frame).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
        
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([
                         self.left_fitx, self.ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([
                          self.right_fitx, self.ploty])))])
    pts = np.hstack((pts_left, pts_right))
        
    # Draw lane on the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
    # cv2.polylines(color_warp, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=10)
    # cv2.polylines(color_warp, np.int_([pts_right]), isClosed=False, color=(0, 0, 255), thickness=10)
    # Warp the blank back to original image space using inverse perspective 
    # matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (
                                  self.orig_frame.shape[
                                  1], self.orig_frame.shape[0]))
    
    # Combine the result with the original image
    result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)
        
    if plot==True:
     
      # Plot the figures 
      figure, (ax1, ax2) = plt.subplots(2,1) # 2 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
      ax1.set_title("Original Frame")  
      ax2.set_title("Original Frame With Lane Overlay")
      plt.show()   

    return result           
    
  def perspective_transform(self, frame=None, plot=False):
    if frame is None:
      frame = self.lane_line_markings
            
    # Calculate the transformation matrix
    self.transformation_matrix = cv2.getPerspectiveTransform(
      self.roi_points, self.desired_roi_points)

    # Calculate the inverse transformation matrix           
    self.inv_transformation_matrix = cv2.getPerspectiveTransform(
      self.desired_roi_points, self.roi_points)

    # Perform the transform using the transformation matrix
    self.warped_frame = cv2.warpPerspective(
      frame, self.transformation_matrix, self.orig_image_size, flags=(
     cv2.INTER_LINEAR)) 
    
    
    # Convert image to binary (already thresholded)
    (thresh, binary_warped) = cv2.threshold(
      self.warped_frame, 127, 255, cv2.THRESH_BINARY)           
    self.warped_frame = binary_warped
    
    # Display the perspective transformed (i.e. warped) frame
    if plot == True:
      warped_copy = self.warped_frame.copy()
      warped_plot = cv2.polylines(warped_copy, np.int32([
                    self.desired_roi_points]), True, (147,20,255), 3)

      # Display the image
      while(1):
        cv2.imshow('Warped Image', warped_plot)
        # Press any key to stop
        if cv2.waitKey(0):
          break

      cv2.destroyAllWindows()   
            
    return self.warped_frame        
    
  def plot_roi(self, frame=None, plot=False):
    """
    Plot the region of interest on an image.
    :param: frame The current image frame
    :param: plot Plot the roi image if True
    """
    if plot == False:
      return
            
    if frame is None:
      frame = self.orig_frame.copy()

    # Overlay trapezoid on the frame
    this_image = cv2.polylines(frame, np.int32([
      self.roi_points]), True, (147,20,255), 3)

    # Display the image
    while(1):
      cv2.imshow('ROI Image', this_image)
            
      # Press any key to stop
      if cv2.waitKey(0):
        break

    cv2.destroyAllWindows()

############################################################################################################################################################
############################################################################################################################################################




class lane_following:
    def __init__(self):
        self.image_lane_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.callback, queue_size = 1, buff_size=2**24)
        self.detect_midlane_pub = rospy.Publisher('/camera/midlane_detected/compressed', CompressedImage, queue_size = 1)
        self.detect_mask_lane_pub = rospy.Publisher('/camera/mask_lane_detected/compressed', CompressedImage, queue_size = 1)
        self.pub_lane = rospy.Publisher('/detect/lane', Float64, queue_size = 1)
        self.cvBridge = CvBridge()
        self.prev_time = 0
        self.left_lane_fit_bef = 0
        self.right_lane_fit_bef = 0
        self.log_status = None

    def callback(self, image_msg):
        
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Create a Lane object
        lane_obj = Lane(orig_frame=img )

        # Perform thresholding to isolate lane lines
        lane_line_markings = lane_obj.get_line_markings()

        # Plot the region of interest on the image
        lane_obj.plot_roi(plot=False)

        # Perform the perspective transform to generate a bird's eye view
        # If Plot == True, show image with new region of interest
        warped_frame = lane_obj.perspective_transform(plot=True)

        # Generate the image histogram to serve as a starting point
        # for finding lane line pixels
        histogram = lane_obj.calculate_histogram(plot=False)  
        
        # Find lane line pixels using the sliding window method 
        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
        plot=False)

        # Fill in the lane line
        lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
        
        # Overlay lines on the original frame
        frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)

        # Calculate lane line curvature (left and right lane lines)
        # lane_obj.calculate_curvature(print_to_terminal=False)

        # Calculate center offset                                                                 
        # lane_obj.calculate_car_position(print_to_terminal=False)
        
        # Display curvature and center offset on image
        # frame_with_lane_lines2 = lane_obj.display_curvature_offset(
            # frame=frame_with_lane_lines, plot=False)
        
 
        # write lane center point pixel x to publish
        
        # msg_desired_center = Float64()
        # msg_desired_center.data = 160
        img2 = frame_with_lane_lines.copy()
        img3 = warped_frame.copy()
        # print("CXX ", msg_desired_center.data)
        img2 = cv2.circle(img2, (msg_desired_center.data, 239), radius=5, color=(0, 0, 255), thickness=-1)
        img3 = cv2.circle(img3, (msg_desired_center.data, 239), radius=5, color=(255, 255, 255), thickness=-1)
        #publish 
        self.pub_lane.publish(msg_desired_center)
        self.detect_mask_lane_pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(img3, "jpg"))
        self.detect_midlane_pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(img2, "jpg"))


def main():
    rospy.init_node('lane_detection')
    rospy.loginfo('Lane detection is running')
    lane_following()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass