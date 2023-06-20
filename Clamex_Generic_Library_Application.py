from DRCF import *
import dope as dp


AlignmentProbeList =  [
posx(-615.9528825020984,-161.922673289894, 65.0, 0, 180.0, 0.0),
posx(-525.005434781121,-203.4986489614291, 65.0, 0, 180.0, 0.0),
posx(-365.977326710106,-144.2528825020984, 65.0, 0, 180.0, 0.0),
posx(-324.40135103857085,-53.30543478112105, 65.0, 0, 180.0, 0.0)
]

PanelType = 0
Yaw= 24.567171096801758


set_accj(75)
set_velj(100)
ProbeRadius = 4.49/2
ProbeLength = 35

start_posj = posj(3.3, -33.9, -98.6, -0.5, -46.7, 2.8)
PitchRollProbingLocations= dp.CreateRadialPropePoints(
    Centerpoint = posx(-504.3, -32.1, 110, 27.9, 180.0, 0.0),Radius = 100,Points =3,IncludeCenter= False)

move_home()
movej(start_posj)
PanelUCS = dp.CalibrateByProbing(PlaneProbingLocations = PitchRollProbingLocations,
                                 YawProbingLocations = AlignmentProbeList,
                                 ProbeRadius = ProbeRadius,ProbeLength = ProbeLength,ForceThreshold=25,
                                 MoveAcceleration=1000, MoveVelocity=800,
                                 ProbeAcceleration=10.0, ProbeVelocity=30.0)
def OffsetCompensation(Yaw):
    return -0.5*(Yaw/10) -3.5
ButterOffset = OffsetCompensation(Yaw)
SlotLocsTOP = [posx(148, ButterOffset, 0, 0, 180, 0),
                posx(2, -150, 0, 0, 180, 0),
                posx(148, -300+4.5, 0, 0, 180, 0),
                posx(302, -150-3.5, 50, 0, 180, 0)]#watch out SlotLocsTOP[3] has a Z of 50 for testing
SlotLocsSIDE = [posx(150, 5.0, -7.5, 0, 180, 0),
                posx(0, -150, 0, 0, 180, 0),
                posx(150, -304.5, -2.0, 0, 180, 0),
                posx(300, -150, 0, 0, 180, 0)]
if PanelType == 0:
    dp.RemoveProfileFromSlotTop(SlotLocsTOP[0],WristOrientation=0, PanelUCS = PanelUCS,
                             ApproachDirection = 'z',ApproachOffset = 100,
                             MoveAcceleration=600, MoveVelocity=600,
                                TresholdTorque=5,ForceThreshold = 25,
                                ProbeAcceleration=10.0, ProbeVelocity=30.0)
    dp.PlaceProfileInSlotTop(SlotLocsTOP[2],WristOrientation=2, PanelUCS = PanelUCS,
                             ApproachDirection = 'z',ApproachOffset = 100,
                             MoveAcceleration=600, MoveVelocity=600,
                             TresholdTorque=5,ForceThreshold = 25,
                             ProbeAcceleration=10.0, ProbeVelocity=30.0)

elif PanelType == 1:
    dp.RemoveProfileFromSlotSide(SlotLocsSIDE[0], PanelUCS=PanelUCS, ApproachDirection='y',
                                 ApproachOffset=-25,MoveAcceleration = 200,
                                 MoveVelocity=200, ProbeLength=35,
                               TresholdTorque=5, ForceThreshold=25,
                                 ProbeAcceleration=10.0, ProbeVelocity=30.0)
    dp.PlaceProfileInSlotSide(SlotLocsSIDE[2], PanelUCS=PanelUCS, ApproachDirection='y',
                              ApproachOffset=25,MoveAcceleration = 200,
                                 MoveVelocity=50, ProbeLength=35,
                              TresholdTorque=5, ForceThreshold=25,
                              ProbeAcceleration=10.0, ProbeVelocity=30.0)
move_home()