# ======================================================================================
#
#      Based on the original dynwalk.py, but modified and working using the class
#      ContactSelect to handle the contacts in the dynamic sot for either the 
#      SolverDynReduced or SolverMotionReduced.
#      - There is no task for the arms (and they are kept backwards "by default")
#
# ======================================================================================

import sys
sys.path.append('.')

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.oscar import SolverMotionReduced
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph import plug

from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom
from dynamic_graph.sot.oscar.zmp_estimator import ZmpEstimator

from numpy import *

initialConfig.clear()
initialConfig['hrp10small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0, -0.5236, 0, 0, 0, 0, 0.2618, 0.1745, 0, -0.5236, 0, 0, 0, 0 )
initialConfig['hrp14small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745,  -0.5236, 0, 0, 0, 0, 0.2618, 0.1745,  -0.5236, 0, 0, 0, 0 )


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

#robotName = 'hrp10small'
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
    contactSelect.update(pg)
    robot.increment(dt)

    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])

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

# SOT controller.
#sot = SolverOpSpace('sot')
#sot = SolverDynReduced('sot')
sot = SolverMotionReduced('sot')
sot.setSize(robotDim-6)
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

#plug(sot.control,robot.control)
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

# --- TASK RIGHT/LEFT FOOT ---------
taskRF=MetaTaskDyn6d('rf',dyn,'rf','right-ankle')
plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.gain.setConstant(2000)
#taskRF.task.controlGain.value = 5000
taskRF.featureDes.velocity.value=(0,0,0,0,0,0)
taskRF.feature.errordot.value=(0,0,0,0,0,0)

taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.gain.setConstant(2000)
#taskLF.task.controlGain.value = 5000
taskLF.featureDes.velocity.value=(0,0,0,0,0,0)
taskLF.feature.errordot.value=(0,0,0,0,0,0)

# ---- CONTACTS ---------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

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
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')
tr.add('pg.SupportFoot','')

robot.before.addSignal('pg.SupportFoot')
robot.before.addSignal('pg.leftfootcontact')
robot.before.addSignal('pg.rightfootcontact')
robot.before.addSignal('pg.leftfootref')
robot.before.addSignal('pg.rightfootref')
robot.before.addSignal('sot.forcesNormal')


# ------------------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# ------------------------------------------------------------------------------

sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support

# --- For the contacts ---------
from contact_handler import ContactSelect
contactSelect = ContactSelect(sot)
contactSelect.setContactTasks(contactRF, contactLF)
contactSelect.setFreeSpaceTasks(taskRF, taskLF)


#sot.push(taskLim.name)
sot.push(taskCom.task.name)
sot.push(taskWaist.task.name)



