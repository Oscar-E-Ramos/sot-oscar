#============================================================================
#
#              TODO: MODIFY THE DESCRIPTION
#
#============================================================================

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.oscar import SolverMotionReduced
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime
from dynamic_graph.sot.core.utils.history import History

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd
from dynamic_graph.sot.dyninv.meta_task_capture_point import MetaTaskCapturePoint
from dynamic_graph.sot.oscar.meta_tasks_tools import FeetFollowCapturePoint
from dynamic_graph.sot.oscar.zmp_estimator import ZmpEstimator


from numpy import *


initialConfig['hrp14small'] = ( 0,0,0.6487,0,0,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,0,0,0.261799,-0.174533,0,-0.523599,0,0,0.174533,0.261799,0.174533,0,-0.523599,0,0,0.174533 )


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
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
    taskCP.update()
    #taskFollowCP.update()
    robot.increment(dt)
    attime.run(robot.control.time)

    #print taskCP.CPOutsideLimits
    #err = taskCP.feature.error.value[0:2]
    #print (err[0]**2+err[1]**2)

    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])
        robot.viewer.updateElementConfig('cp', [taskCP.task.capturePoint.value[0],
                                                taskCP.task.capturePoint.value[1],
                                                0,0,0,0])
        # robot.viewer.updateElementConfig('zmp', [taskCPGreater.task.capturePoint.value[0],
        #                                         taskCPGreater.task.capturePoint.value[1],
        #                                         0,0,0,0])
    history.record()


@loopInThread
def loop():
    inc()
runner=loop()

# --- shortcuts -------------------------------------------------
@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def dump():
    history.dumpToOpenHRP('openhrp/capturePoint')

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

taskWaist = MetaTaskDyn6d('taskWaist', dyn, 'waist', 'waist')
taskChest = MetaTaskDyn6d('taskChest', dyn, 'chest', 'chest')
taskHead  = MetaTaskDyn6d('taskHead', dyn, 'head', 'gaze')
taskrh    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
tasklh    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')
taskrf    = MetaTaskDyn6d('rf', dyn, 'rf', 'right-ankle')
tasklf    = MetaTaskDyn6d('lf', dyn, 'lf', 'left-ankle')

for task in [ taskWaist, taskChest, taskHead, taskrh, tasklh, taskrf, tasklf]:
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt
    # ADDED TO AVOID ERRORS: VERIFY IF THERE IS A BETTER WAY!!!
    task.featureDes.velocity.value=(0,0,0,0,0,0)

# CoM Task
taskCom = MetaTaskDynCom(dyn,dt)

# Posture Task
taskPosture = MetaTaskDynPosture(dyn,dt)

# Inequality Task for the CoM
taskComIneq = MetaTaskDynCom(dyn,dt,name="comIneq")
taskComIneq.task = TaskDynInequality('taskComInequality')
taskComIneq.task.add(taskComIneq.feature.name)
taskComIneq.task.dt.value = dt
plug(dyn.velocity, taskComIneq.task.qdot)

taskComIneq.feature.selec.value = "001"
taskComIneq.featureDes.errorIN.value = (0,)
taskComIneq.task.referenceInf.value = (-0.08,)     # Xmin, Ymin
taskComIneq.task.referenceSup.value = (0.13,)    # Xmin, Ymin


# Angular position and velocity limits
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


#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot    = SolverDynReduced('sot')
# sot     = SolverMotionReduced('sot')
contact = AddContactHelper(sot)

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


#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
#contactLF.support =  ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))

# For velocities
contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
contactRF.featureDes.velocity.value=(0,0,0,0,0,0)

# Imposed erordot = 0 : is there a better alternative ???
contactLF.feature.errordot.value=(0,0,0,0,0,0)
contactRF.feature.errordot.value=(0,0,0,0,0,0)


# Task for the Capture Point
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# taskCP = MetaTaskDynCom(dyn, dt, name="capturePt")
# taskCP.task = TaskCapturePoint('taskCapturePoint')
# taskCP.task.add(taskCP.feature.name)
# taskCP.task.dt.value = dt
# plug(dyn.velocity, taskCP.task.qdot)
# plug(dyn.com, taskCP.task.robotCom)

# taskCP.feature.selec.value = "011"     # It must be X,Y (always!!)
# taskCP.featureDes.errorIN.value = (0,0)

# dyn.rf.recompute(0); dyn.lf.recompute(0)
# xmax =  0.10 + (dyn.lf.value)[0][3]
# xmin = -0.08 + (dyn.lf.value)[0][3]
# ymin = -0.04 + (dyn.rf.value)[1][3]
# ymax =  0.04 + (dyn.lf.value)[1][3]
# taskCP.task.referenceInf.value = (xmin, ymin)     # Xmin, Ymin
# taskCP.task.referenceSup.value = (xmax, ymax)      # Xmax, Ymax
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# Task for the Capture Point
taskCP = MetaTaskCapturePoint(dyn, sot, dt, name="capturePt")
taskCP.limitsRF = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
taskCP.limitsLF = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
taskCP.task.capturePoint.recompute(0)

# Task for the feet to follow the CP in case it goes beyond the limits
taskFollowCP = FeetFollowCapturePoint(taskrf, contactRF, taskCP, sot)
taskFollowCP.borders = ((0.10,-0.07,-0.07,0.10),(-0.045,-0.045,0.09,0.09),(-0.105,-0.105,-0.105,-0.105))
taskFollowCP.errorx = 0.05
taskrf.gain.setConstant(1000)


# Task containing larger zone for the Capture Point
taskCPGreater = MetaTaskCapturePoint(dyn, sot, dt, name="capturePtGreater")
taskCPGreater.limitsRF = ((0.22,-0.08,-0.08,0.22),(-0.08,-0.08,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
taskCPGreater.limitsLF = ((0.22,-0.08,-0.08,0.22),(-0.07,-0.07,0.08,0.08),(-0.105,-0.105,-0.105,-0.105))




#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','dyn_','.dat')

tr.add('sot.acceleration','')
tr.add('sot.torque','')
tr.add('sot.reducedControl','')
tr.add('sot.reducedForce','')
tr.add('sot.forces','')
tr.add('sot.forcesNormal','')
tr.add('sot.reducedForce','')
tr.add('dyn.position','')
tr.add('dyn.com','')
tr.start()

robot.after.addSignal('sot.reducedForce')
robot.after.addSignal('sot.forces')
robot.after.addSignal('sot.torque')

robot.after.addSignal('tr.triger')
robot.after.addSignal(contactLF.task.name+'.error')
robot.after.addSignal('dyn.rf')
robot.after.addSignal('dyn.lf')
robot.after.addSignal('dyn.chest')
robot.after.addSignal('dyn.com')
robot.after.addSignal('sot.forcesNormal')
robot.after.addSignal('dyn.waist')

robot.after.addSignal('taskLim.normalizedPosition')
robot.after.addSignal(taskCP.task.name+'.capturePoint')
robot.after.addSignal(taskCPGreater.task.name+'.capturePoint')


history = History(dyn,1,zmp.zmp)


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

sot.clear()
contact(contactLF)
contact(contactRF)
''' To avoid problems with solverMotionReduced '''
if (cmp(sot.className,'SolverMotionReduced')==0):
    sot._RF_ddx3.value = ((0.,0.,0.,),)
    sot._LF_ddx3.value = ((0.,0.,0.,),)

sot.push(taskLim.name)
plug(robot.state,sot.position)

q0 = robot.state.value
chest0 = matrix(dyn.chest.value)[0:3,3]

taskCom.feature.selec.value = "11"
#taskCom.gain.setByPoint(100,10,0.005,0.8)
taskCom.gain.setConstant(1)

r = radians
sigset = ( lambda s, v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

# # Abusive way to overcontrol the CoM (creates problems with control)
# attime(2 ,(lambda: sot.push(taskCom.task.name),"Add CoM")
#          ,(lambda: refset(taskCom, ( 0.014, 0.001,  0.8 )), "Com to left foot")
#        )

# # Controlling the CoM in inequality
# attime( 2
#         ,(lambda: sot.push(taskComIneq.task.name),"Add CoM")
#         )

# Controlling the Capture Point in inequality
# attime( 2
#         ,(lambda: sot.push(taskCP.task.name),"Add Capture Point Inequality")
#         )


# Task trying to make the robot fall
rhDes = [0.71, -0.34, 0.9, 0,0,0]
robot.viewer.updateElementConfig('ball', rhDes)
taskChest.feature.frame('desired')
taskrf.feature.frame('desired')
attime(3 
       # ,(lambda: sot.push(taskChest.task.name), "Add Chest")
       # ,(lambda: gotoNd(taskChest, (0.25, 0, 0), "000001", gain=(10,)), "Move chest forward")
       ,(lambda: sot.push(taskrh.task.name), "Add right hand")
       ,(lambda: gotoNd(taskrh, (rhDes[0], rhDes[1], rhDes[2]), "000111"), "Move right hand forward")
       # ,(lambda: sot.push(tasklh.task.name), "Add left hand")
       # ,(lambda: gotoNd(tasklh, (0.3, 0, 0), "000111"), "Move left hand forward")
       #,(lambda: sot.push(taskCP.task.name),"Add Capture Point Inequality")
       )

# attime( 100
#         ,(lambda: sot.push(taskCP.task.name),"Add Capture Point Inequality")
#         )

attime(1000, stop, "Stopped")

#go()
