# ======================================================================================
#      Based on the original dynwalk.py, but modified and working using the class
#      ContactSelect to handle the contacts in the dynamic sot
#      - There is no task for the arms (and they are kept backwards "by default")
# ======================================================================================


import sys
sys.path.append('.')

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.oscar import SolverMotionReduced, FclBoxMeshCollision
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph import plug

from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime
from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToRPY

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, AddContactHelper
from dynamic_graph.sot.oscar.zmp_estimator import ZmpEstimator
from dynamic_graph.sot.oscar.meta_task_passing_point import *

from numpy import *

#from contact_handler import AddContactPointsHelper, prune
from contact_handler_new import AddContactPointsHelperNew


initialConfig.clear()
initialConfig['hrp10small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0, -0.5236, 0, 0, 0, 0, 0.2618, 0.1745, 0, -0.5236, 0, 0, 0, 0 )
initialConfig['hrp14small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745,  -0.5236, 0, 0, 0, 0, 0.2618, 0.1745,  -0.5236, 0, 0, 0, 0 )


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName  = 'hrp14small'
robotDim   = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)

robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,verbose=False)
dt=5e-3


# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    robot.increment(dt)
    attime.run(robot.control.time)

    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    # robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    # if dyn.com.time >0:
    #     robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])

    updateFeetCollision()
    contactSelect.update(pg)


@loopInThread
def loop():
    inc()
runner=loop()

@optionalparentheses
def go():
    if runner.isQuit:
        globals()['runner']=loop()
    runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()

# --- shortcuts -------------------------------------------------
@optionalparentheses
def q():
    if 'dyn' in globals(): print dyn.ffposition.__repr__()
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def t(): print robot.state.time-1
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay
@optionalparentheses
def n(): inc(); qdot()
@optionalparentheses
def n5(): 
    for loopIdx in range(5): inc()
@optionalparentheses
def n10():
    for loopIdx in range(10): inc()


# ------------------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------
# ------------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()
dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# ------------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH ---------------------------------------------------------
# ------------------------------------------------------------------------------

# sot = SolverDynReduced('sot')
# contact = AddContactHelper(sot)
sot = SolverMotionReduced('sot')
contact = AddContactPointsHelperNew(sot)

sot.setSize(robotDim-6)
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.solution, robot.control)
plug(sot.acceleration,robot.acceleration)


#--- ZMP ----------------
zmp = ZmpEstimator('zmp')
zmp.declare(sot,dyn)


# ------------------------------------------------------------------------------
# --- PG ----------------------------------------------------------------------
# ------------------------------------------------------------------------------

from dynamic_graph.sot.pattern_generator import *

pg = PatternGenerator('pg')
pg.setVrmlDir(modelDir+'/')
pg.setVrml(modelName[robotName])
pg.setXmlSpec(specificitiesPath)
pg.setXmlRank(jointRankPath)
pg.buildModel()

# Standard initialization
pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(dyn.position,pg.position)
plug(dyn.com,pg.com)
pg.motorcontrol.value = robotDim*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

# --- REFERENCES ---------
# Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                  ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess, comRef.selec)

# --- HERDT PG AND START ---------

# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')

# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
pg.parseCmd(':setfeetconstraint XY 0.09 0.06')
# Start the robot with a speed of 0.1 m/0.8 s.
pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')
# You can now modifiy the speed of the robot using set pg.velocitydes [3]( x, y, yaw)
pg.velocitydes.value =(0.1,0.0,0.0)


# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ------------------------------------------------------------------------------

# ---- WAIST TASK ---------
taskWaist=MetaTaskDyn6d('waist',dyn,'waist','waist')
''' Imposed errordot =0: is there a better alternative ??? '''
taskWaist.featureDes.velocity.value=(0,0,0)
taskWaist.feature.errordot.value=(0,0,0)


# Build the reference waist pos homo-matrix from PG.
waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 5000

# --- TASK COM ---------
taskCom = MetaTaskDynCom(dyn,dt)
taskCom.feature.selec.value = "011"
taskCom.gain.setConstant(5000)
plug(comRef.ref, taskCom.featureDes.errorIN)

# --- Testing tasks for the feet ---
taskPPrf = MetaTaskPassingPoint('PPrf',dyn,'rf','right-ankle')
taskPPrf.feature.frame('current')
taskPPrf.task.dt.value = dt

taskPPlf = MetaTaskPassingPoint('PPlf',dyn,'lf','left-ankle')
taskPPlf.feature.frame('current')
taskPPlf.task.dt.value = dt

# ---- CONTACTS ---------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),
                     (-0.105,-0.105,-0.105,-0.105))
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),
                     (-0.105,-0.105,-0.105,-0.105))

''' Imposed erordot = 0 : is there a better alternative ??? '''
contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
contactLF.feature.errordot.value=(0,0,0,0,0,0)
contactRF.featureDes.velocity.value=(0,0,0,0,0,0)
contactRF.feature.errordot.value=(0,0,0,0,0,0)


# --- Angular position and velocity limits ---------
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = dt

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqup = (1000,)*robotDim
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])


# ------------------------------------------------------------------------------
# --- TRACER -------------------------------------------------------------------
# ------------------------------------------------------------------------------

from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = Tracer('tr')
tr.open('/tmp/dwalkPP_','','.dat')
tr.start()
robot.after.addSignal('tr.triger')
tr.add('pg.SupportFoot','')

robot.before.addSignal('pg.SupportFoot')
robot.before.addSignal('pg.leftfootcontact')
robot.before.addSignal('pg.rightfootcontact')
robot.before.addSignal('pg.leftfootref')
robot.before.addSignal('pg.rightfootref')
robot.before.addSignal('pg.landingfootposition')
robot.before.addSignal('sot.forcesNormal')
robot.before.addSignal('dyn.lf')
robot.before.addSignal('dyn.rf')



# ------------------------------------------------------------------------------
# --- Helper functions ---------------------------------------------------------
# ------------------------------------------------------------------------------

def updateFeetCollision():
    ''' Update the box for the feet '''
    wMcomr = matrixToTuple(matrix(dyn.rf.value)*arMcomr) # world_M_ankleRight * anklRight_M_comr
    wMcoml = matrixToTuple(matrix(dyn.lf.value)*alMcoml) # wMar*arMcomr
    wMcomr_XYZ_RPY = matrixToRPY( wMcomr )               # com of RF box in (x,y,z,r,p,y) format
    wMcoml_XYZ_RPY = matrixToRPY( wMcoml )               # com of LF box in (x,y,z,r,p,y) format
    robot.viewer.updateElementConfig('boxRF', wMcomr_XYZ_RPY)   # View the right foot box
    robot.viewer.updateElementConfig('boxLF', wMcoml_XYZ_RPY)   # View the right foot box
    fclRf.setBoxTransf(wMcomr)        # Set pose of RF box (for collision detection)
    fclLf.setBoxTransf(wMcoml)        # Set pose of LF box (for collision detection)

    ''' Show contact points '''
    Prf = fclRf.contactPoints.value    # Contact points 'Pc' in the world frame 'w'
    Plf = fclLf.contactPoints.value    # Contact points 'Pc' in the world frame 'w'
    for i in range(len(Prf)):                     # Show the contact points for the right foot
        name = 'point'+str(i+1)
        robot.viewer.updateElementConfig(name, [Prf[i][0],Prf[i][1],Prf[i][2],0,0,0])
    for i in range(len(Plf)):                     # Show the contact points for the left foot
        name = 'point'+str(i+1+len(Prf))
        robot.viewer.updateElementConfig(name, [Plf[i][0],Plf[i][1],Plf[i][2],0,0,0])
    for i in range(NPt-len(Plf)-len(Prf)):        # Hide the points that are not in contact
        name = 'point'+str(i+1+len(Prf)+len(Plf))
        robot.viewer.updateElementConfig(name, [0,0,-1,0,0,0])




# ------------------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# ------------------------------------------------------------------------------

dyn.rf.recompute(0)
dyn.lf.recompute(0)

# --- Definitions for the feet sole ---
soleRF = ((0.11,-0.08,-0.08,0.11),(-0.065,-0.065,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
soleLF = ((0.11,-0.08,-0.08,0.11),(0.065,0.065,-0.045,-0.045),(-0.105,-0.105,-0.105,-0.105))
arPcomr = matrix(( average(soleRF[0]), average(soleRF[1]), -0.105 + 0.012 )).T
alPcoml = matrix(( average(soleLF[0]), average(soleLF[1]), -0.105 + 0.012 )).T
arMcomr = bmat([ [mat(eye(3)),arPcomr], [mat(zeros(3)),mat(1.)] ])
alMcoml = bmat([ [mat(eye(3)),alPcoml], [mat(zeros(3)),mat(1.)] ])

wMcomr = matrixToTuple(matrix(dyn.rf.value)*arMcomr)   # wMar*arMcomr
wMcoml = matrixToTuple(matrix(dyn.lf.value)*alMcoml)   # wMar*arMcomr
wMcomr_XYZ_RPY = matrixToRPY( wMcomr )       # com of right foot box in (x,y,z,r,p,y) format
wMcoml_XYZ_RPY = matrixToRPY( wMcoml )       # com of left foot box in (x,y,z,r,p,y) format


# --- Collision detection with the floor ----
fclRf = FclBoxMeshCollision('fclRf')
fclRf.setMinDistance(0.002)
fclRf.setBoxDimensions((0.23, 0.135, 0.024))
fclRf.setBoxTransf(wMcomr)
fclRf.setMeshFile('plane2mCentered_005.obj')
fclRf.setMeshPose((0,0,0,0,0,0))

fclLf = FclBoxMeshCollision('fclLf')
fclLf.setMinDistance(0.002)
fclLf.setBoxDimensions((0.23, 0.135, 0.024))
fclLf.setBoxTransf(wMcoml)
fclLf.setMeshFile('plane2mCentered_005.obj')
fclLf.setMeshPose((0,0,0,0,0,0))


# --- Show the initial boxes and points ---
''' Hide all the contact points (initialize them under the ground) '''
NPt = 20
for i in range(NPt):                                         # Hide the contact points
    name = 'point'+str(i+1)
    robot.viewer.updateElementConfig(name, [0,0,-1,0,0,0])
robot.viewer.updateElementConfig('boxRF', wMcomr_XYZ_RPY)      # View the right foot box
robot.viewer.updateElementConfig('boxLF', wMcoml_XYZ_RPY)      # View the right foot box


# --- For the contacts ----------------------------------------------------

#from contact_handler import ContactSelectPassingPoint

from contact_handler_new import ContactSelectPassingPointMotionReduced
#from contact_handler_2013 import ContactSelectPassingPointMotionReduced
contactSelect = ContactSelectPassingPointMotionReduced(sot, dyn)
contactSelect.setContactTasks(contactRF, contactLF)
contactSelect.setFreeSpaceTasks(taskPPrf, taskPPlf)
contactSelect.setCollisionDetection(fclRf, fclLf)
contactSelect.setRobot(robot)

contact(contactRF)
contact(contactLF)

#sot.push(taskLim.name)
sot.push(taskCom.task.name)
sot.push(taskWaist.task.name)


sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )


# --- Events -------------------------------------------------------------

attime(1000
       #,(lambda: sigset(pg.velocitydes, (0.0,0.0,0.0) )  , "" )
       ,(lambda: sigset(pg.velocitydes, (0.1,0.0,0.5) )  , "Change velocity of pattern" )
       )

attime(2000
       ,(lambda: sigset(pg.velocitydes, (0.0,0.0,0.0) )  , "Stop walking generation (velocity=0)" )
       )


robot.before.addSignal('fclRf.contactPoints')
robot.before.addSignal('fclLf.contactPoints')
