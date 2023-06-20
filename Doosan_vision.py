import cv2
import numpy as np
import imutils
import os



def detectRectangles(img, binary_treshold = 150, minimum_line_width = 300, minimum_line_height = 300):
    """
    The function detect_rectangles returns the rectangles found in the image

    :param img: numpy.ndarray - The image containing the rectangles
    :param binary_treshold: int - The value determining whether the pixel is black or white
    :param minimum_line_width: int - The smallest width that the rectangle may have in pixels
    :param minimum_line_height: int - The smallest height that the rectangle may have in pixels
    :param show_image: Bool - When "True" the image with the rectangles will be shown. When "False" no image will be shown

    :returns: (List[2] center, List[2] w_h, int rotation, numpy.ndarray DrawRect)
    """
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE )[0]
    
    DrawRect = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)
    center = [0, 0]
    width_height = [0, 0]
    rotation = 0
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)

        if len(approx) == 4:

            center, width_height, rotation = cv2.minAreaRect(contour)


            if width_height[0]  > minimum_line_width and width_height[1] > minimum_line_height:
                cv2.circle(DrawRect, (int(center[0]), int(center[1])), 5, (0, 0, 255), 15)
                DrawRect = cv2.drawContours(DrawRect, [contour], -1, (0,255,0), 6)
                break
        center = [0, 0]
        width_height = [0, 0]
        rotation = 0
            

    if rotation >= 45:
        rotation = rotation - 90

    return center,width_height,rotation,DrawRect


def nothing(x):
	pass


def ImageMasking(img, low_h = 0  ,high_h= 255 ,low_s = 0  ,high_s= 255 ,low_v = 150,high_v= 255):
    """
    The function Imagemasking makes a mask with the given HSV tresholds

    :param img: numpy.ndarray - The image
    :param low_h: int - Lower Hue treshold
    :param high_h: int - Higher Hue treshold
    :param low_s: int - Lower Saturation treshold
    :param high_s: int - Lower Saturation treshold
    :param low_v: int - Lower Value treshold
    :param high_v: int - Higher Value treshold

    :returns: ndarray Mask
    """
    mask = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    # low_h = int(cv2.getTrackbarPos('low H','controls'))
    # high_h = int(cv2.getTrackbarPos('high H','controls'))
    # low_s = int(cv2.getTrackbarPos('low s','controls'))
    # high_s = int(cv2.getTrackbarPos('high s','controls'))
    # low_v = int(cv2.getTrackbarPos('low v','controls'))
    # high_v = int(cv2.getTrackbarPos('high v','controls'))

    lowerrange = np.array([low_h,low_s,low_v])       
    upperrange = np.array([high_h,high_s,high_v]) 
    mask = cv2.inRange(mask, lowerrange, upperrange)
    return mask

def TemplateMatching(masked_img, templates, template_path, template_size = (565,565),min_val = 0.50):

    max_val= 0
    template_type = "No plate found"
    template_angle = 0

    for template_name in templates:
        template = cv2.imread(os.path.join(template_path, template_name))
        template = cv2.resize(template, template_size)

        masked_template = ImageMasking(template)
    

        for angle in range(0,360,90):
            rotated_template = imutils.rotate(masked_template.copy(), angle)
            match_depth_map = cv2.matchTemplate(masked_img, rotated_template,cv2.TM_CCOEFF_NORMED)
            min, max, min_loc, max_loc = cv2.minMaxLoc(match_depth_map)

            if max > max_val:
                max_val = max
                if max > min_val:
                    template_type = template_name
                    template_angle = angle

    return max_val, template_type, template_angle

def DopeImgShow(img,waitkey,name="img"):
    cv2.imshow(name, cv2.resize(img,(int(img.shape[1]/1),int(img.shape[0]/1)),interpolation = cv2.INTER_AREA))
    cv2.waitKey(waitkey)


def CalculateProbeLocations(center, width_height, rotation, Panel_Width_heigth = [300,300]):
    """
    The function CalculateProbeLocations uses the values: center, widht_heigth(Pixels), rotation and Rectangle_width_heigth(mm) of a found rectangle
    to calculate probe points around the Rectangle

    :param img: List[2] - center
    :param img: List[2] - width_height
    :param img: float - rotation
    :param img: List[2] - Panel_width_height

    :returns: (list[4] ProbePointsDoosan )
    """
    
    # Calculate the probe distance and width, based on width of panel
    Probe_distance = width_height[0]/2+width_height[0]/15
    Probe_offset =  width_height[0]/6

    # Convert degrees to radians
    rotation_rad = np.radians(rotation)
    c, s = np.cos(rotation_rad), np.sin(rotation_rad)

    # Create a z-axis rotation matrix 
    rot_z = np.matrix([[c, -s,  0],
                       [s,  c,  0],
                       [0,  0,  1]])

    # Set the locations of the probe locations from the centerpoint
    Probepoints = []
    Probepoints.append(np.matrix([-Probe_offset,    -Probe_distance,    0]))
    Probepoints.append(np.matrix([Probe_offset,     -Probe_distance,    0]))
    Probepoints.append(np.matrix([Probe_distance,   -Probe_offset,      0]))
    Probepoints.append(np.matrix([Probe_distance,    Probe_offset,      0]))

    # Create vectors for the centerpoints from the camera and the Doosan
    ProbepointsDoosan = []
    CenterpointDoosan = np.matrix([-499.8   ,   -28.1    ,      0]).reshape(3,1)
    CenterpointCamera = np.matrix([center[0],   center[1],      0]).reshape(3,1)
    
    # Convert the location of the probe point from camera space to Doosan space
    for i in range(len(Probepoints)):
        Probepoints[i] = rot_z*Probepoints[i].reshape(3,1) + CenterpointCamera
        PointFromCenter = CenterpointCamera - Probepoints[i]      
        Doosx = PointFromCenter.item(1)
        Doosy = PointFromCenter.item(0)
        
        DoosanPropePointPixels = np.matrix([Doosx,Doosy,0]).reshape(3,1)
        DoosanPropePoint = DoosanPropePointPixels*Panel_Width_heigth[0]/width_height[0]+CenterpointDoosan

        ProbepointsDoosan.append(DoosanPropePoint)

    return ProbepointsDoosan , Probepoints




#------------------------------
#-----main program cycle-------
#------------------------------
if __name__ == "__main__":
    vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)



    mtx = np.array([[7.62513178e+03, 0.00000000e+00, 9.59529181e+02],
                    [0.00000000e+00, 7.39190921e+03, 4.85476599e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    dist = np.array([[ 3.48070677e+00, -2.84798589e+01,  4.02883711e-01, -3.08511140e-03,
                         7.01393997e+03]])

    RawInputCamera = vid.read()[1]
    h,  w = RawInputCamera.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1)


    #List of templates
    template_path =  os.path.join(os.getcwd() , 'Example images' , 'templates')
    templates = os.listdir(template_path)

while(1):
    w_h=[0,0]
    while(w_h==[0,0]):
        RawInputCamera = vid.read()[1]

        # undistort
        RawInputCamera = cv2.undistort(RawInputCamera, mtx, dist, None, newcameramtx)

        RawCroppedCamera = RawInputCamera[0:850, 450:1400]

        MaskCroppedCamera = ImageMasking(RawCroppedCamera)

        center,w_h,rotation,DrawRect = detectRectangles(MaskCroppedCamera)

        ProbePointsDoosan,Probepoints = CalculateProbeLocations(center, w_h, rotation)

        for i in Probepoints:
            Probes = cv2.circle(DrawRect, (int(i[0]), int(i[1])), 5, (0, 0, 255), 15)
        DopeImgShow(Probes,1, "probes")

    cv2.destroyWindow('probes')
    
    MaskStraightenedCamera = imutils.rotate(MaskCroppedCamera, rotation)

    max_val, template, angle = TemplateMatching(MaskStraightenedCamera, templates, template_path)
    print(">-----------------------<")
    
    if(template=="tempA0.png"):
        PanelType = 0
    else:
        PanelType = 1

    print(
    "AlignmentProbeList =  [\n"
        "posx(" + str(float(ProbePointsDoosan[3][0])) + "," + str(float(ProbePointsDoosan[3][1])) + ", 65.0, 0, 180.0, 0.0), \n"
        "posx(" + str(float(ProbePointsDoosan[2][0])) + "," + str(float(ProbePointsDoosan[2][1])) + ", 65.0, 0, 180.0, 0.0), \n"
        "posx(" + str(float(ProbePointsDoosan[1][0])) + "," + str(float(ProbePointsDoosan[1][1])) + ", 65.0, 0, 180.0, 0.0), \n"
        "posx(" + str(float(ProbePointsDoosan[0][0])) + "," + str(float(ProbePointsDoosan[0][1])) + ", 65.0, 0, 180.0, 0.0)  \n"
    "]\n"
    "\n"
    "PanelType = "+str(PanelType)
    )
    print("Yaw=",rotation)

    DopeImgShow(Probes,0, "probes")
