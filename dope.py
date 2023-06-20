"""
Doosan Object Positioning Erudition
-----------------------------------
"""
from DRCF import *
import numpy as np
import powerup.motion as motion
# import cv2
# import imutils


def CreateRadialPropePoints(Centerpoint, Radius, Points, IncludeCenter=True):
    """
    Creates a radial pattern of probe points around a center for Z axis probing.
    The center can be included in the output array using the include center parameter.

    :param Centerpoint: posx: float[6] - Center point of planar probing array
    :param Radius: float - Radius of probe points around center
    :param Points: float - Number of points to generate around center
    :Param IncludeCenter: bool - Include center in return array

    :returns: (array: PointArray - Array containing the probe points)
    """
    Step = 2 * np.pi / Points
    if IncludeCenter == True:
        PointArray = [Centerpoint]
    else:
        PointArray = []
    for x in range(Points):
        PointArray.append(posx(Centerpoint[0] + np.cos(x * Step) * Radius, Centerpoint[1] + np.sin(x * Step) * Radius,
                           Centerpoint[2], Centerpoint[3], Centerpoint[4], Centerpoint[5]))
    return PointArray


def ProbePoint(ToolAxis, Displacement=200.0, ForceThreshold=20.0, MoveAcceleration=400.0, MoveVelocity=100.0,
               ProbeAcceleration=8.0, ProbeVelocity=15.0, ReturnProbetoStart=True):
    """
    Moves the toolhead [Displacement] mm's in X,Y,Z cardinal directions relative to the base and returns the final location if the threshold was reached.
    If returnprobe is set to true the probe will return to its starting location after probing.

    :param ToolAxis: string - The movement direction ["X","Y","Z"]
    :param Displacement: float - The distance we move befpre we consider the probing failed
    :param ForceThreshold: float - The maximum measured Force before move is considered complete
    :param MoveAcceleration: float - The acceleration of the return movement
    :param MoveVelocity: float - The velocity of the return movement
    :param ProbeAcceleration: float - The acceleration of the probing movement
    :param ProbeVelocity: float - The velocity of the probing movement
    :param ReturnProbetoStart: Boolean - [True-> return movement enabled, False-> return movement disabled]

    :returns: (posx: float[6] Tool position-> Force threshold reached, None -> Final position reached)
    """
    # Make Toolaxis lowercase to remove case sensitivity
    ToolAxis = ToolAxis.lower()

    # Record current position
    StartPos = get_current_posx(ref=DR_BASE)[0]

    # Create the movement offset and specify force measurement axis
    if ToolAxis == 'x':
        ToolAxis = DR_AXIS_X
        ProbePos = add_pose(StartPos, posx(Displacement, 0, 0, 0, 0, 0))

    elif ToolAxis == 'y':
        ToolAxis = DR_AXIS_Y
        ProbePos = add_pose(StartPos, posx(0, Displacement, 0, 0, 0, 0))

    elif ToolAxis == 'z':
        ToolAxis = DR_AXIS_Z
        ProbePos = add_pose(StartPos, posx(0, 0, Displacement, 0, 0, 0))

    else:
        raise Exception("ToolAxis must be either x, y or z")

    # Move in desired direction
    amovel(ProbePos, ProbeVelocity, ProbeAcceleration)

    while check_force_condition(axis=ToolAxis, max=ForceThreshold, ref=DR_BASE):
        if check_motion() == 0:
            raise Exception("Probing reached endpoint without triggering force sensor threshold.")
    stop(DR_QSTOP)

    # Record position after probe reaches desired point
    ProbePos = get_current_posx(ref=DR_BASE)[0]

    # Return probe to starting location after measurement if [ReturnProbetoStart] is True
    if ReturnProbetoStart:
        movel(StartPos, MoveVelocity, MoveAcceleration)

    return ProbePos


def ProbeYaw(ProbePointList, DisplacementX=-200, DisplacementY=200, ForceThreshold=20.0, MoveAcceleration=400.0,
             MoveVelocity=100.0, ProbeVelocity=15.0, ProbeAcceleration=8.0, Z_off=100.0):
    """
    Probes panel sides 4 times and returns Coordinates of probed points.

    :param ProbePointList: list - Probe starting points
    :param DisplacementX: float - The distance we offset in X direction
    :param DisplacementY: float - The distance we offset in Y direction
    :param ForceThreshold: float - The maximum measured Force before move is considered complete
    :param MoveAcceleration: float - The acceleration of the return movement
    :param MoveVelocity: float - The velocity of the return movement
    :param ProbeAcceleration: float - The acceleration of the probing movement
    :param ProbeVelocity: float - The velocity of the probing movement
    :param z_off: float - Approach of probe starting points offset in Z direction

    :returns: (ProbedLocations - posx: float[6] list)
    """

    if len(ProbePointList) < 4:
        raise Exception("ProbePlane ProbePointList must contain 4 or more posx: float[6] values")

    ProbedLocations = []

    for i in ProbePointList[0:2]:
        movel(add_pose(i, posx(0, 0, Z_off, 0, 0, 0)), v=MoveVelocity, a=MoveAcceleration)
        movel(i, v=MoveVelocity, a=MoveAcceleration)
        ProbedLocations.append(
            ProbePoint('y', ForceThreshold=ForceThreshold, Displacement=DisplacementY, ProbeVelocity=ProbeVelocity,
                       ProbeAcceleration=ProbeAcceleration))
        movel(add_pose(i, posx(0, 0, Z_off, 0, 0, 0)), v=MoveVelocity, a=MoveAcceleration)

    for i in ProbePointList[2:]:
        movel(add_pose(i, posx(0, 0, Z_off, 0, 0, 0)), v=MoveVelocity, a=MoveAcceleration)
        movel(i, v=MoveVelocity, a=MoveAcceleration)
        ProbedLocations.append(
            ProbePoint('x', ForceThreshold=ForceThreshold, Displacement=DisplacementX, ProbeVelocity=ProbeVelocity,
                       ProbeAcceleration=ProbeAcceleration))
        movel(add_pose(i, posx(0, 0, Z_off, 0, 0, 0)), v=MoveVelocity, a=MoveAcceleration)

    return ProbedLocations


def ProbePlane(ProbePointList, ToolAxis="z", Displacement=-200, ForceThreshold=20, MoveAcceleration=50.0,
               MoveVelocity=100, ProbeAcceleration=8.0, ProbeVelocity=15.0):
    """
    Measures the probe height at each probe point and returns the measured points in an array.
    At least three points which aren't in line with one another must be given to determine a plane otherwise the results will not be accurate.

    :param ProbePointList: list[>2] - Array containing posx: float[6] positions to determine plane allignment minimum
    :param ToolAxis: string - The movement direction
    :param Displacement: float - The distance we offset in the given ToolAxis
    :param ForceThreshold: float - The maximum measured Force before move is considered complete
    :param MoveAcceleration: float - The acceleration of the return and movement
    :param MoveVelocity: float - The velocity of the return and movement
    :param ProbeAcceleration: float - The acceleration of the probing movement
    :param ProbeVelocity: float - The velocity of the probing movement

    :returns: (float Roll, float Pitch, posx: float[6] FinalProbeLocationList)-> Force threshold reached, None -> Final position reached
    """

    # Check array length for number of points
    if len(ProbePointList) < 3:
        raise Exception("ProbePlane ProbePointList must contain 3 or more posx: float[6] values")

    ProbedLocations = []

    # Move between probe points and measure the height there of
    for i in ProbePointList:
        movel(i, MoveVelocity, MoveAcceleration)
        ProbedLocations.append(
            ProbePoint(ToolAxis=ToolAxis, Displacement=Displacement, ForceThreshold=ForceThreshold,
                       MoveAcceleration=MoveAcceleration, MoveVelocity=MoveVelocity,
                       ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeVelocity))

    return ProbedLocations


def CompYaw(ProbePointList):
    """
    Calculates a yaw angle of the first measured side and returns the result in degrees.

    :param ProbedLocations: list - List containing probed locations

    :returns: (Yaw: float Origin yaw)
    """

    posx1 = posx(-502.8, -178.4, 56.6, 123.2, -174.1, -87.1)
    posx2 = posx(-561.5, -173.0, 58.0, 129.3, -174.2, -80.3)
    x1, y1 = ProbePointList[0][0], ProbePointList[0][1]
    x2, y2 = ProbePointList[1][0], ProbePointList[1][1]

    Yaw = r2d(atan2((y1 - y2), (x1 - x2)))

    return Yaw


def CompOriginXY(ProbePointList, ProbeRadius):
    """
    Calculates and returns origin of 2 intersection lines based on their probed locations.

    :param ProbedLocations: list - List containing probed locations
    :param ProbeRadius: float - Radius of measurement probe in mm

    :returns: ((OriginX, OriginY): float(2) - X and Y coordinates of the origin point)
    """

    if len(ProbePointList) == 4:
        x1, y1 = ProbePointList[0][0], ProbePointList[0][1]
        x2, y2 = ProbePointList[1][0], ProbePointList[1][1]
        x3, y3 = ProbePointList[2][0], ProbePointList[2][1]
        x4, y4 = ProbePointList[3][0], ProbePointList[3][1]

        OriginX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / (
                (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)) - ProbeRadius
        OriginY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / (
                (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)) + ProbeRadius

        return OriginX, OriginY
    else:
        raise Exception("CompOriginXY ProbePointList must contain 4 posx: float[6] values")


def CompPlane(ProbedLocations):
    """
    Calculates and returns the Roll and Pitch of the measured plane using a least squares estimation.

    :param ProbedLocations: list - List containing probed locations

    :returns: ((Roll, Pitch): float(2) - Roll and pitch of the measure plane in degrees)
    """

    # Calculate least squares estimation
    a = []
    b = []
    for i in ProbedLocations:
        a.append([i[0], i[1], 1])
        b.append(i[2])

    # ax + by + c = z
    b = np.matrix(b).T
    a = np.matrix(a)
    abc = (a.T * a).I * a.T * b

    # Calculate Roll Pitch
    Roll = -float(r2d(atan2(abc[1], 1)))
    Pitch = float(r2d(atan2(abc[0], 1)))

    return Roll, Pitch


def CalibrateByProbing(PlaneProbingLocations, YawProbingLocations, ProbeRadius, ProbeLength, DisplacementX=-200,
                       DisplacementY=200, DisplacemenZ=-200, ForceThreshold=20, MoveAcceleration=200, MoveVelocity=300,
                       ProbeAcceleration=8.0, ProbeVelocity=15.0):
    # For clamex probe radius = 4.49/2
    """
    Performs planar and yaw probing using default settings on a flat horizontal object with straight edges.
    Computes the origin and probes its location to acquire a set height.
    returns A coordinate frame ID if the operation was successful.
    returns -1 if the operation failed.

    :param PlaneProbingLocations: list - List containing coordinates for planar probing
    :param YawProbingLocations: list[4] - List containing 4 coordinates coordinates for yaw probing
    :param ProbeRadius: float - Radius of measurement probe in mm
    :param ProbeLength: float - Length of measurement probe in mm
    :param DisplacementX: float - The distance we offset in X direction
    :param DisplacementY: float - The distance we offset in Y direction
    :param DisplacementY: float - The distance we offset in Z direction
    :param ForceThreshold: float - The maximum measured Force before move is considered complete
    :param MoveAcceleration: float - The acceleration of the return movement
    :param MoveVelocity: float - The velocity of the return movement
    :param ProbeAcceleration: float - The acceleration of the probing movement
    :param ProbeVelocity: float - The velocity of the probing movement

    :returns: SettingSucess: int - [Successful coordinate setting: Set coordinate ID (101 - 200) / Failed coordinate
    setting : -1]
    """
    # Perform planar probing
    ProbedPlaneLocs = ProbePlane(PlaneProbingLocations, Displacement=DisplacemenZ, ForceThreshold=ForceThreshold,
                                 MoveAcceleration=MoveAcceleration, MoveVelocity=MoveVelocity,
                                 ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeVelocity)

    # Compute the planar roll and pitch
    Roll, Pitch = CompPlane(ProbedPlaneLocs)

    # Perform Yaw probing
    ProbedYawLocs = ProbeYaw(YawProbingLocations, DisplacementX=DisplacementX, DisplacementY=DisplacementY,
                             ForceThreshold=ForceThreshold, MoveAcceleration=MoveAcceleration,
                             MoveVelocity=MoveVelocity, ProbeAcceleration=ProbeAcceleration,
                             ProbeVelocity=ProbeVelocity, Z_off=-DisplacemenZ / 4)

    # Compute yaw
    Yaw = CompYaw(ProbedYawLocs)

    # Compute origin coordinates
    OriginX, OriginY = CompOriginXY(ProbedYawLocs, ProbeRadius)

    # Move probe above origin position
    movel(posx(OriginX, OriginY, -DisplacemenZ / 2, 0.0, 180.0, 0.0), MoveVelocity, MoveAcceleration)

    # probe origin Z height and compensate for probe height knowing the probe is vertical
    OriginZ = subtract_pose(
        ProbePoint("Z", Displacement=-200, MoveAcceleration=MoveAcceleration, MoveVelocity=MoveVelocity,
                   ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeVelocity, ForceThreshold=ForceThreshold)
        , posx(0, 0, ProbeLength, 0, 0, 0))

    # Convert RPY
    UCSEuler = motion.rpy2eul(Roll, Pitch, Yaw)

    PanelOrigin = posx(OriginX, OriginY, OriginZ[2], UCSEuler[0], UCSEuler[1], UCSEuler[2])
    PanelUCS = set_user_cart_coord(PanelOrigin, ref=DR_BASE)
    return PanelUCS


def ProbeCoordinateOffset(Pointlist, OffsetX=0, OffsetY=0):
    """
       Returns the coordinate(s) offset in X and Y directions by the specified probe offset

    :param Pointlist: list or posx: float[6] - Containing coordinates to be offset. Return type is either list or posx: float[6]
    :param OffsetX: float - The distance we offset in X direction
    :param OffsetY: float - The distance we offset in Y direction

    :returns: (posx: float[6] CorrectedPointlist-> CorrectedPointlist type is posx, list: CorrectedPointlist of type posx: float[6]-> CorrectedPointlist type is list of type posx: float[6])
    """

    if isinstance(Pointlist, type(posx())):
        CorrectedPointlist = trans(Pointlist, posx(OffsetX, OffsetY, 0, 0, 0, 0), DR_TOOL, DR_BASE)
    else:
        CorrectedPointlist = []
        for i in Pointlist:
            CorrectedPointlist.append(trans(i, posx(OffsetX, OffsetY, 0, 0, 0, 0), DR_TOOL, DR_BASE))
    return CorrectedPointlist


class Probe:
    """
    A class containing compiled methods for easier probing and setting of probe offsets and data.

    :Methods: ProbeCoordinateOffset(self,Pointlist)
    :Methods: ProbePoint(self, ToolAxis, Displacement=200, ReturnProbetoStart=True)
    :Methods: ProbeYaw(self, ProbePointList, DisplacementX=-200, DisplacementY=200)
    :Methods: ProbePlane(self, ProbePointList, ToolAxis="z", Displacement=-200)
    :Methods: CalibrateByProbing(self, PlaneProbingLocations, YawProbingLocations, DisplacementX=-200, DisplacementY=200,DisplacementZ=None)


    """

    def __init__(self, ProbeRadius, ProbeLength, ForceThreshold=20, MoveAcceleration=50.0,
                 MoveVelocity=100, ProbeAcceleration=8.0, ProbeVelocity=15.0, ClearanceZ=100.0, OffsetX=0.0,
                 OffsetY=0.0):
        """
        Returns the coordinate(s) offset in X and Y directions by the specified probe offset

        :param ProbeRadius: float - Radius of probe
        :param ProbeLength: float - Length of probe (Z height offset)
        :param ForceThreshold: float - The maximum measured Force before move is considered complete
        :param MoveAcceleration: float - The acceleration of the return movement
        :param MoveVelocity: float - The velocity of the return movement
        :param ProbeAcceleration: float - The acceleration of the probing movement
        :param ProbeVelocity: float - The velocity of the probing movement
        :param ClearanceZ: float - Height to move above material when attempting to manouver
        :param OffsetX: float - Probe X direction offset
        :param OffsetY: float - Probe Y direction offset

        :returns: (posx: float[6] CorrectedPointlist-> CorrectedPointlist type is posx, list: CorrectedPointlist of type posx: float[6]-> CorrectedPointlist type is list of type posx: float[6])
        """
        self.ProbeRadius = ProbeRadius
        self.ProbeLength = ProbeLength
        self.ForceThreshold = ForceThreshold
        self.MoveAcceleration = MoveAcceleration
        self.MoveVelocity = MoveVelocity
        self.ProbeAcceleration = ProbeAcceleration
        self.ProbeVelocity = ProbeVelocity
        self.Z_off = ClearanceZ + ProbeLength
        self.OffsetX = OffsetX
        self.OffsetY = OffsetY

    def ProbeCoordinateOffset(self, Pointlist):
        """
        Returns the coordinate(s) offset in X and Y directions by the specified probe offset

        :param Pointlist: list or posx: float[6] - Containing coordinates to be offset. Return type is either list or posx: float[6]

        :returns: (posx: float[6] CorrectedPointlist-> CorrectedPointlist type is posx, list: CorrectedPointlist of type posx: float[6]-> CorrectedPointlist type is list of type posx: float[6])
        """

        return ProbeCoordinateOffset(Pointlist, OffsetX=self.OffsetX, OffsetY=self.OffsetY)

    def ProbePoint(self, ToolAxis, Displacement=200, ReturnProbetoStart=True):
        """
        Moves the toolhead [Displacement] mm's in X,Y,Z cardinal directions relative to the base and returns the final location if the threshold was reached.
        If returnprobe is set to true the probe will return to its starting location after probing.

        :param ToolAxis: string - The movement direction ["X","Y","Z"]
        :param Displacement: float - The distance we move befpre we consider the probing failed
        :param ReturnProbetoStart: Boolean - [True-> return movement enabled, False-> return movement disabled]

        :returns: (posx: float[6] Tool position-> Force threshold reached, None -> Final position reached)
        """
        return ProbePoint(ToolAxis, Displacement=Displacement, ForceThreshold=self.ForceThreshold,
                          MoveAcceleration=self.MoveAcceleration, MoveVelocity=self.MoveVelocity,
                          ProbeAcceleration=self.ProbeAcceleration, ProbeVelocity=self.ProbeVelocity,
                          ReturnProbetoStart=ReturnProbetoStart)

    def ProbeYaw(self, ProbePointList, DisplacementX=-200, DisplacementY=200):
        """
        Probes panel sides 4 times and returns Coordinates of probed points.

        :param ProbePointList: list - Probe starting points
        :param DisplacementX: float - The distance we offset in X direction
        :param DisplacementY: float - The distance we offset in Y direction

        :returns: (ProbedLocations - posx: float[6] list)
        """
        return ProbeYaw(ProbePointList, DisplacementX=-DisplacementX, DisplacementY=DisplacementY,
                        ForceThreshold=self.ForceThreshold, MoveAcceleration=self.MoveAcceleration,
                        MoveVelocity=self.MoveVelocity, ProbeAcceleration=self.ProbeAcceleration,
                        ProbeVelocity=self.ProbeVelocity, Z_off=self.Z_off)

    def ProbePlane(self, ProbePointList, ToolAxis="z", Displacement=-200):
        """
        Measures the probe height at each probe point and returns the measured points in an array.
        At least three points which aren't in line with one another must be given to determine a plane otherwise the results will not be accurate.

        :param ProbePointList: list[>2] - Array containing posx: float[6] positions to determine plane allignment minimum
        :param ToolAxis: string - The movement direction
        :param Displacement: float - The distance we offset

        :returns: (float Roll, float Pitch, posx: float[6] FinalProbeLocationList)-> Force threshold reached, None -> Final position reached
        """

        return ProbePlane(ProbePointList, ToolAxis="z", Displacement=-200, ForceThreshold=self.ForceThreshold,
                          MoveAcceleration=self.MoveAcceleration, MoveVelocity=self.MoveVelocity,
                          ProbeAcceleration=self.ProbeAcceleration, ProbeVelocity=self.ProbeVelocity)

    def CalibrateByProbing(self, PlaneProbingLocations, YawProbingLocations, DisplacementX=-200, DisplacementY=200,
                           DisplacementZ=None):
        """
        Performs planar and yaw probing using default settings on a flat horizontal object with straight edges.
        Computes the origin and probes its location to acquire a set height.
        returns A coordinate frame ID if the operation was successful.
        returns -1 if the operation failed.

        :param PlaneProbingLocations: list - List containing coordinates for planar probing
        :param YawProbingLocations: list[4] - List containing 4 coordinates coordinates for yaw probing
        :param ProbeRadius: float - Radius of measurement probe in mm
        :param ProbeLength: float - Length of measurement probe in mm
        :param DisplacementX: float - The distance we offset in X direction
        :param DisplacementY: float - The distance we offset in Y direction
        :param DisplacementY: float - The distance we offset in Z direction

        :returns: SettingSucess: int - [Successful coordinate setting: Set coordinate ID (101 - 200) / Failed coordinate
        setting : -1]
        """

        if DisplacementZ == None:
            DisplacementZ = self.Z_off

        return CalibrateByProbing(PlaneProbingLocations, YawProbingLocations, ProbeRadius=self.ProbeRadius,
                                  ProbeLength=self.ProbeLength, DisplacementX=DisplacementX,
                                  DisplacementY=DisplacementY, DisplacemenZ=DisplacementZ,
                                  ForceThreshold=self.ForceThreshold, MoveAcceleration=self.MoveAcceleration,
                                  MoveVelocity=self.MoveVelocity, ProbeAcceleration=self.ProbeAcceleration,
                                  ProbeVelocity=self.ProbeVelocity)


def PlaceProfileInSlotSide(SlotLocs, PanelUCS, ApproachDirection, ApproachOffset,
                           ClamexRadius=30, MoveVelocity=100, MoveAcceleration=400.0,
                           TresholdTorque=2,ProbeLength = 35,ForceThreshold = 25,
                           ProbeAcceleration=10.0, ProbeVelocity=30.0):
    """
    Approaches slots and places clamex in side panel slots whilst checking the torque.

    :param SlotLocs: posx: float[6] - position to place Clamex Profiles
    :param PanelUCS: int - Coordinate system set by the user corresponding to the Panel
    :param ApproachDirection: int - An identifier needed for the correct approach
    :param ApproachOffset: float - The offset distance approaching the slot
    :param ClamexRadius: float - The radius of the clamex profile
    :param MoveVelocity: float - The velocity of the return and movement
    :param MoveAcceleration: float - The acceleration of the return and movement
    :param TresholdTorque: float - The maximum torque allowed while placing Clamex
    :param ProbeLength: float - The length of the probe in mm
    """

    ApproachDirection = ApproachDirection.lower()
    if ApproachDirection == "x" and ApproachOffset > 0:
        Approachrot = -135
        MountingROT = -90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(-30, 0, 70, 0, 0, 0))
        ApproachOffset = -abs(-ApproachOffset - ClamexRadius)
        PoseForIkin = coord_transform(
            add_pose(SlotLocs,posx(ApproachOffset, 0, abs(ApproachOffset), 0, 0,Approachrot)),
            PanelUCS, DR_BASE)
        PosjApproach = ikin(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(ApproachOffset, 0, 0, 0, 0, Approachrot))
        FinalApproachx = add_pose(SlotLocs, posx(-ClamexRadius, 0, 0, 0, 0, Approachrot))
        ExitOffset = posx(ApproachOffset, 0, 0, 0, 0, 0)
    elif ApproachDirection == "x" and ApproachOffset <= 0:
        Approachrot = -180
        MountingROT = 90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(30, 0, 70, 0, 0, 0))
        ApproachOffset = abs(ApproachOffset - ClamexRadius)
        PoseForIkin = coord_transform(
            add_pose(SlotLocs, posx(ApproachOffset, 0, abs(ApproachOffset), 0, 0, Approachrot)),
            PanelUCS, DR_BASE)
        PosjApproach = ikin(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(ApproachOffset, 0, 0, 0, 0, Approachrot))
        FinalApproachx = add_pose(SlotLocs, posx(ClamexRadius, 0, 0, 0, 0, Approachrot))
        ExitOffset = posx(ApproachOffset, 0, 0, 0, 0, 0)
    elif ApproachDirection == "y" and ApproachOffset > 0:
        Approachrot = 45
        MountingROT = 90
        ArmSolspace = 3
        ProbeLoc = add_pose(SlotLocs, posx(0, 30, 70, 0, 0, 0))
        ApproachOffset =-ApproachOffset- ClamexRadius
        PoseForIkin = coord_transform(add_pose(
            SlotLocs, posx(0, ApproachOffset, abs(ApproachOffset)
                           , 0, 0, Approachrot)), PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(0, ApproachOffset, 0, 0, 0, Approachrot))
        FinalApproachx = add_pose(SlotLocs, posx(0, -ClamexRadius, 0, 0, 0, Approachrot))
        ExitOffset = posx(0, ApproachOffset, 0, 0, 0, 0)
    elif ApproachDirection == "y" and ApproachOffset <= 0:
        Approachrot = 45
        MountingROT = -90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(0, -30, 70, 0, 0, 0))
        ApproachOffset = abs(ApproachOffset -ClamexRadius)
        PoseForIkin = coord_transform(add_pose(
            SlotLocs, posx(0, ApproachOffset, abs(ApproachOffset)
                           , 0, 0, Approachrot)), PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(0, ApproachOffset, 0, 0, 0, Approachrot))
        FinalApproachx = add_pose(SlotLocs, posx(0, ClamexRadius, 0, 0, 0, Approachrot))
        ExitOffset = posx(0, ApproachOffset, 0, 0, 0, 0)
    else:
        raise Exception("OHOH, coordinate not found whoops")
    # approach
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    # Allign for probing and probe
    movel(ProbeLoc, ref=PanelUCS, v=ProbeVelocity, a=ProbeAcceleration)
    ProbedSlot = ProbePoint('z', Displacement=-150,ForceThreshold = ForceThreshold)
    # Calculate final approach
    ProbedSlot = coord_transform(ProbedSlot, DR_BASE, PanelUCS)
    ProbedSlot = subtract_pose(posx(0, 0, ProbeLength, 0, 0, 0), ProbedSlot)
    FinalApproachx = posx(FinalApproachx[0], FinalApproachx[1], FinalApproachx[2] + ProbedSlot[2],
                          FinalApproachx[3], FinalApproachx[4], FinalApproachx[5])
    # Enter profile holes
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

    movel(ApproachPosx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movel(FinalApproachx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    # place profile
    posej = get_current_posj()
    posej[5] = posej[5] + MountingROT
    amovej(posej, v=50, a=50)
    CheckPlacingTorque(TresholdTorque=TresholdTorque)
    movel(add_pose(ExitOffset, get_current_posx(ref=PanelUCS)[0]),
          ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

def PlaceProfileInSlotTop(SlotLocs, PanelUCS, ApproachDirection,
                          WristOrientation, ApproachOffset, ClamexRadius=30,
                          MoveVelocity=100, MoveAcceleration=400.0, TresholdTorque=2,
                          ProbeLength = 35,ForceThreshold=25,
                          ProbeAcceleration=10.0, ProbeVelocity=30.0):
    """
    Approaches slots and places clamex in top panel slots whilst checking the torque.

    :param SlotLocs: posx: float[6] - position to place Clamex Profiles
    :param PanelUCS: int - Coordinate system set by the user corresponding to the Panel
    :param ApproachDirection: int - An identifier needed for the correct approach
    :param WristOrientation: int - An identifier needed for wrist position
    :param ApproachOffset: float - The offset distance approaching the slot
    :param ClamexRadius: float - The radius of the clamex profile
    :param MoveVelocity: float - The velocity of the return and movement
    :param MoveAcceleration: float - The acceleration of the return and movement
    :param TresholdTorque: float - The maximum torque allowed while placing Clamex
    :param ProbeLength: float - The length of the probe in mm
    """
    ApproachDirection = ApproachDirection.lower()
    if ApproachDirection == "z" and ApproachOffset > 0:
        Approachrot = 135
        MountingROT = 90
        if WristOrientation == 0: ArmSolspace, WristAngle, ProbeOffsets = 7, -90, posx(0, -30, 70, 0, 0, 0)
        if WristOrientation == 1: ArmSolspace, WristAngle, ProbeOffsets = 7, 180, posx(30, 0, 70, 0, 0, 0)
        if WristOrientation == 2: ArmSolspace, WristAngle, ProbeOffsets = 6, 90, posx(0, 30, 70, 0, 0, 0)
        if WristOrientation == 3: ArmSolspace, WristAngle, ProbeOffsets = 6, 0, posx(-30, 30, 70, 0, 0, 0)
        ProbeSlot = coord_transform(add_pose(SlotLocs,ProbeOffsets),PanelUCS, DR_BASE)
        PoseForIkin = coord_transform(add_pose(SlotLocs,
                                               posx(0, 0, ApproachOffset * 2 + ClamexRadius,
                                                    WristAngle, 90, Approachrot)),
                                      PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin,
                                  sol_space=ArmSolspace, ref=DR_BASE,ref_pos_opt=1)[0]
        ApproachPosx = add_pose(SlotLocs, posx(0, 0, ApproachOffset/2, WristAngle, 90, Approachrot))
        FinalApproachx = add_pose(SlotLocs, posx(0, 0, ClamexRadius, WristAngle, 90, Approachrot))
        ExitOffset = posx(0, 0, ApproachOffset + ClamexRadius, 0, 0, 0)
    elif ApproachDirection == "z" and ApproachOffset <= 0:
        Approachrot = 180
        MountingROT = -90
        ArmSolspace = 6

        PosjApproach = ikin(add_pose(SlotLocs, posx(0, 0, ApproachOffset * 2 - ClamexRadius, -90, 90, 0)),
                             sol_space=ArmSolspace, ref=PanelUCS)[0]

        ApproachPosx = add_pose(SlotLocs, posx(0, 0, ApproachOffset - ClamexRadius, -90, 90, 0))

        FinalApproachx = add_pose(SlotLocs, posx(0, 0, -ClamexRadius, -90, 90, 0))
        ExitOffset = posx(0, 0, ApproachOffset - ClamexRadius, 0, 0, 0)
    else:
        raise Exception("OHOH, coordinate not found whoops")
    # approach
    # movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

    movel(ProbeSlot, v=MoveVelocity, a=MoveAcceleration)
    ProbedSlot = ProbePoint('z',Displacement = -150,ForceThreshold = ForceThreshold,
                            ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeVelocity)
    ProbedSlot = coord_transform(ProbedSlot,DR_BASE ,PanelUCS )
    ProbedSlot = subtract_pose(posx(0, 0, ProbeLength, 0, 0, 0), ProbedSlot)
    FinalApproachx = posx(FinalApproachx[0],FinalApproachx[1],FinalApproachx[2]+ProbedSlot[2],
                          FinalApproachx[3],FinalApproachx[4],FinalApproachx[5])

    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    movel(ApproachPosx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movel(FinalApproachx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)

    posej = get_current_posj()
    posej[5] = posej[5] + MountingROT
    amovej(posej, v=20, a=20)
    CheckPlacingTorque(TresholdTorque = TresholdTorque)
    movel(add_pose(ExitOffset, get_current_posx(ref=PanelUCS)[0]),
          ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

def RemoveProfileFromSlotTop(SlotLocs, PanelUCS, ApproachDirection, WristOrientation,
                             ApproachOffset, ClamexRadius=30,
                             MoveVelocity=100, MoveAcceleration=400.0, TresholdTorque=2,
                             ProbeLength=35,ForceThreshold=25,
                             ProbeAcceleration=10.0, ProbeVelocity=30.0):
    """
    Approaches slots and removes clamex in top panel slots whilst checking the torque.

    :param SlotLocs: posx: float[6] - position to place Clamex Profiles
    :param PanelUCS: int - Coordinate system set by the user corresponding to the Panel
    :param ApproachDirection: int - An identifier needed for the correct approach
    :param WristOrientation: int - An identifier needed for wrist position
    :param ApproachOffset: float - The offset distance approaching the slot
    :param ClamexRadius: float - The radius of the clamex profile
    :param MoveVelocity: float - The velocity of the return and movement
    :param MoveAcceleration: float - The acceleration of the return and movement
    :param TresholdTorque: float - The maximum torque allowed while placing Clamex
    :param ProbeLength: float - The length of the probe in mm
    """
    ApproachDirection = ApproachDirection.lower()
    if ApproachDirection == "z" and ApproachOffset > 0:
        Approachrot = 135
        MountingROT = 90
        if WristOrientation == 0: ArmSolspace, WristAngle, ProbeOffsets = 7, -90, posx(0, -30, 70, 0, 0, 0)
        if WristOrientation == 1: ArmSolspace, WristAngle, ProbeOffsets = 7, 180, posx(30, 0, 70, 0, 0, 0)
        if WristOrientation == 2: ArmSolspace, WristAngle, ProbeOffsets = 6, 90, posx(0, 30, 70, 0, 0, 0)
        if WristOrientation == 3: ArmSolspace, WristAngle, ProbeOffsets = 6, 0, posx(-30, 30, 70, 0, 0, 0)
        ProbeLoc = add_pose(SlotLocs, ProbeOffsets)
        PoseForIkin = coord_transform(add_pose(SlotLocs,
                                               posx(0, 0, ApproachOffset * 2 + ClamexRadius,
                                                    WristAngle, 90, Approachrot + MountingROT)),
                                      PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin,
                                 sol_space=ArmSolspace, ref=DR_BASE, ref_pos_opt=1)[0]
        ApproachPosx = add_pose(SlotLocs,
                                posx(0, 0, ApproachOffset/2 , WristAngle, 90, Approachrot + MountingROT))
        FinalApproachx = add_pose(SlotLocs, posx(0, 0, ClamexRadius, WristAngle, 90, Approachrot + MountingROT))
        ExitOffset = posx(0, 0, ApproachOffset + ClamexRadius, 0, 0, 0)
    elif ApproachDirection == "z" and ApproachOffset <= 0:
        Approachrot = 180
        MountingROT = -90
        ArmSolspace = 6

        PosjApproach = ikin(add_pose(SlotLocs, posx(0, 0, ApproachOffset * 2 - ClamexRadius, -90, 90, 0)),
                            sol_space=ArmSolspace, ref=PanelUCS)[0]

        ApproachPosx = add_pose(SlotLocs, posx(0, 0, ApproachOffset - ClamexRadius, -90, 90, 0))

        FinalApproachx = add_pose(SlotLocs, posx(0, 0, -ClamexRadius, -90, 90, 0))
        ExitOffset = posx(0, 0, ApproachOffset - ClamexRadius, 0, 0, 0)
    else:
        raise Exception("OHOH, coordinate not found whoops")
    # approach
    # movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    # Allign for probing and probe
    movel(ProbeLoc, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    ProbedSlot = ProbePoint('z', Displacement=-150,ForceThreshold = ForceThreshold,
                            ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeVelocity)
    # Calculate final approach
    ProbedSlot = coord_transform(ProbedSlot, DR_BASE, PanelUCS)
    ProbedSlot = subtract_pose(posx(0, 0, ProbeLength, 0, 0, 0), ProbedSlot)
    FinalApproachx = posx(FinalApproachx[0], FinalApproachx[1], FinalApproachx[2] + ProbedSlot[2],
                          FinalApproachx[3], FinalApproachx[4], FinalApproachx[5])
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    movel(ApproachPosx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movel(FinalApproachx, ref=PanelUCS, v=20, a=20)
    # placing
    posej = get_current_posj()
    posej[5] = posej[5] + MountingROT
    amovej(posej, v=20, a=20)
    CheckPlacingTorque(TresholdTorque=TresholdTorque)
    movel(add_pose(ExitOffset, get_current_posx(ref=PanelUCS)[0]),
          ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

def RemoveProfileFromSlotSide(SlotLocs, PanelUCS, ApproachDirection, ApproachOffset,
                           ClamexRadius=30, MoveVelocity=100, MoveAcceleration=400.0, TresholdTorque=2,
                          ProbeLength = 35,ForceThreshold=25,ProbeAcceleration=10.0, ProbeVelocity=30.0):
    """
    Approaches slots and removes clamex in side panel slots whilst checking the torque.

    :param SlotLocs: posx: float[6] - position to place Clamex Profiles
    :param PanelUCS: int - Coordinate system set by the user corresponding to the Panel
    :param ApproachDirection: int - An identifier needed for the correct approach
    :param ApproachOffset: float - The offset distance approaching the slot
    :param ClamexRadius: float - The radius of the clamex profile
    :param MoveVelocity: float - The velocity of the return and movement
    :param MoveAcceleration: float - The acceleration of the return and movement
    :param TresholdTorque: float - The maximum torque allowed while placing Clamex
    :param ProbeLength: float - The length of the probe in mm
    """

    ApproachDirection = ApproachDirection.lower()
    if ApproachDirection == "x" and ApproachOffset > 0:
        Approachrot = -135
        MountingROT = -90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(-30, 0, 70, 0, 0, 0))
        ApproachOffset = -abs(-ApproachOffset - ClamexRadius)
        PoseForIkin = coord_transform(
            add_pose(SlotLocs,posx(ApproachOffset, 0, abs(ApproachOffset), 0, 0,Approachrot+MountingROT)),
            PanelUCS, DR_BASE)
        PosjApproach = ikin(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(ApproachOffset, 0, 0, 0, 0, Approachrot+MountingROT))
        FinalApproachx = add_pose(SlotLocs, posx(-ClamexRadius, 0, 0, 0, 0, Approachrot+MountingROT))
        ExitOffset = posx(ApproachOffset, 0, 0, 0, 0, 0)
    elif ApproachDirection == "x" and ApproachOffset <= 0:
        Approachrot = -180
        MountingROT = 90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(30, 0, 70, 0, 0, 0))
        ApproachOffset = abs(ApproachOffset - ClamexRadius)
        PoseForIkin = coord_transform(
            add_pose(SlotLocs, posx(ApproachOffset, 0, abs(ApproachOffset), 0, 0, Approachrot+MountingROT)),
            PanelUCS, DR_BASE)
        PosjApproach = ikin(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(ApproachOffset, 0, 0, 0, 0, Approachrot+MountingROT))
        FinalApproachx = add_pose(SlotLocs, posx(ClamexRadius, 0, 0, 0, 0, Approachrot+MountingROT))
        ExitOffset = posx(ApproachOffset, 0, 0, 0, 0, 0)
    elif ApproachDirection == "y" and ApproachOffset > 0:
        Approachrot = 45
        MountingROT = 90
        ArmSolspace = 3
        ProbeLoc = add_pose(SlotLocs,posx(0,30,70,0,0,0))
        ApproachOffset =-ApproachOffset- ClamexRadius
        PoseForIkin = coord_transform(add_pose(
            SlotLocs, posx(0, ApproachOffset, abs(ApproachOffset)
                           , 0, 0, Approachrot)), PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(0, ApproachOffset, 0, 0, 0, Approachrot+MountingROT))
        FinalApproachx = add_pose(SlotLocs, posx(0, -ClamexRadius, 0, 0, 0, Approachrot+MountingROT))
        ExitOffset = posx(0, ApproachOffset, 0, 0, 0, 0)
    elif ApproachDirection == "y" and ApproachOffset <= 0:
        Approachrot = 45
        MountingROT = -90
        ArmSolspace = 6
        ProbeLoc = add_pose(SlotLocs, posx(0, -30, 70, 0, 0, 0))
        ApproachOffset = abs(ApproachOffset -ClamexRadius)
        PoseForIkin = coord_transform(add_pose(
            SlotLocs, posx(0, ApproachOffset, abs(ApproachOffset)
                           , 0, 0, Approachrot)), PanelUCS, DR_BASE)
        PosjApproach = ikin_norm(PoseForIkin, sol_space=ArmSolspace, ref=DR_BASE)[0]
        ApproachPosx = add_pose(SlotLocs, posx(0, ApproachOffset, 0, 0, 0, Approachrot+MountingROT))
        FinalApproachx = add_pose(SlotLocs, posx(0, ClamexRadius, 0, 0, 0, Approachrot+MountingROT))
        ExitOffset = posx(0, ApproachOffset, 0, 0, 0, 0)
    else:
        raise Exception("OHOH, coordinate not found whoops")
    # approach
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    #Allign for probing and probe
    movel(ProbeLoc, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    ProbedSlot = ProbePoint('z', Displacement=-150,ForceThreshold = ForceThreshold,
                            ProbeAcceleration=ProbeAcceleration, ProbeVelocity=ProbeAcceleration)
    # Calculate final approach
    ProbedSlot = coord_transform(ProbedSlot, DR_BASE, PanelUCS)
    ProbedSlot = subtract_pose(posx(0,0,ProbeLength,0,0,0),ProbedSlot)
    FinalApproachx = posx(FinalApproachx[0], FinalApproachx[1], FinalApproachx[2]+ProbedSlot[2],
                          FinalApproachx[3], FinalApproachx[4], FinalApproachx[5])
    # Enter profile holes
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)
    movel(ApproachPosx, ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movel(FinalApproachx, ref=PanelUCS, v=10, a=20)
    # Remove profile
    posej = get_current_posj()
    posej[5] = posej[5] + MountingROT
    amovej(posej, v=20, a=20)
    CheckPlacingTorque(TresholdTorque=TresholdTorque)
    movel(add_pose(ExitOffset, get_current_posx(ref=PanelUCS)[0]),
          ref=PanelUCS, v=MoveVelocity, a=MoveAcceleration)
    movej(PosjApproach, v=MoveVelocity, a=MoveAcceleration)

def CheckPlacingTorque(TresholdTorque):
    """This function is for sensing J6 torque during placing"""
    StartingPosj = get_current_posj()
    while check_force_condition(DR_AXIS_C, max=TresholdTorque, ref=DR_TOOL):
        print(check_motion())
        if check_motion() == 0:
            return
    stop(DR_QSTOP)
    movej(StartingPosj, v=10, a=5)


def detectRectangles(img, binary_treshold = 123, minimum_line_width = 100, minimum_line_height = 100):
    """
    The function detect_rectangles returns the rectangles found in the image

    :param img: numpy.ndarray - The image containing the rectangles
    :param binary_treshold: int - The value determining whether the pixel is black or white
    :param minimum_line_width: int - The smallest width that the rectangle may have in pixels
    :param minimum_line_height: int - The smallest height that the rectangle may have in pixels
    :param show_image: Bool - When "True" the image with the rectangles will be shown. When "False" no image will be shown

    :returns: ()
    """
    
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE )[0]
    
    DrawRect = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)
    center=[0,0]
    w_h=[0,0]
    r=0
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
        if len(approx) == 4:
            center, w_h, r = cv2.minAreaRect(contour)
            if w_h[0]  > minimum_line_width and w_h[1] > minimum_line_height:
                cv2.circle(DrawRect, (int(center[0]), int(center[1])), 5, (0, 0, 255), 15)
                DrawRect = cv2.drawContours(DrawRect, [contour], -1, (0,255,0), 6)

    if r >= 45:
        r = r - 90
    
    return center,w_h,int(r),DrawRect


def ImageMasking(img, binary_treshold=123, low_h = 0  ,high_h= 100 ,low_s = 0  ,high_s= 255 ,low_v = 170,high_v= 255):
    mask = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    lowerrange = np.array([low_h,low_s,low_v])       
    upperrange = np.array([high_h,high_s,high_v]) 
    mask = cv2.inRange(mask, lowerrange, upperrange)
    return mask


def CalculateProbeLocations(center, width_height, rotation, Panel_Width_heigth = [300,300]):
    Probe_distance = width_height[0]/2+width_height[0]/8
    Probe_offset =  width_height[0]/6

    rotation_rad = np.radians(rotation)
    c, s = np.cos(rotation_rad), np.sin(rotation_rad)

    rot_z = np.matrix([[c,-s,0],
                      [s,c,0],
                      [0,0,1]])

    Probepoints = []

    Probepoints.append(np.matrix([-Probe_offset,-Probe_distance,0]))
    Probepoints.append(np.matrix([Probe_offset,-Probe_distance,0]))
    Probepoints.append(np.matrix([Probe_distance,-Probe_offset,0]))
    Probepoints.append(np.matrix([Probe_distance,Probe_offset,0]))

    ProbepointsDoosan = []
    CenterpointDoosan = np.matrix([-504.3,-32.1,0]).reshape(3,1)
    CenterpointCamera = np.matrix([center[0],center[1],0]).reshape(3,1)
    
    for i in range(len(Probepoints)):
        Probepoints[i] = rot_z*Probepoints[i].reshape(3,1) + CenterpointCamera
        PointFromCenter = CenterpointCamera - Probepoints[i]      
        Doosx = PointFromCenter.item(1)
        Doosy = PointFromCenter.item(0)

        DoosanPropePointPixels = np.matrix([Doosx,Doosy,0]).reshape(3,1)
        DoosanPropePoint = DoosanPropePointPixels*Panel_Width_heigth[0]/width_height[0]+CenterpointDoosan

        ProbepointsDoosan.append(DoosanPropePoint)

    
    return ProbepointsDoosan

def PlaceClamexInSlots(SlotLocs,PanelType,PanelUCS,ApproachOffset = 200,
                       MoveAcceleration=200, MoveVelocity=100, ProbeLength = 35):
    if PanelType == 0:
        for i in range(SlotLocs):
            PlaceProfileInSlotTop(SlotLocs[i],WristOrientation=i, PanelUCS = PanelUCS,
                             ApproachDirection = 'z',ApproachOffset = ApproachOffset,
                             MoveAcceleration=MoveAcceleration, MoveVelocity=MoveVelocity)
    if PanelType == 1:
        for i in range(SlotLocs):
            if i  == 0 :
                ApproachDirection = 'y'
                ApproachOffset = -ApproachOffset
            if i == 1 :
                ApproachDirection = 'x'
                ApproachOffset = ApproachOffset
            if i  == 2 :
                ApproachDirection = 'y'
                ApproachOffset = ApproachOffset
            if i == 3 :
                ApproachDirection = 'x'
                ApproachOffset = -ApproachOffset
            PlaceProfileInSlotSide(SlotLocs = SlotLocs[i], PanelUCS = PanelUCS,
                                   ApproachDirection = ApproachDirection,
                                   ApproachOffset = ApproachOffset, ProbeLength = ProbeLength)