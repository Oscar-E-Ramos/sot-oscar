from dynamic_graph import plug
from dynamic_graph.sot.core import GainAdaptive, OpPointModifier
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, rpy2tr
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d
from dynamic_graph.sot.oscar import TaskPassingPoint

from numpy import matrix, identity, zeros, eye, array, sqrt, radians, arccos, linalg, inner


class MetaTaskPassingPoint(MetaTask6d):
    def createTask(self):
        self.task = TaskPassingPoint('task'+self.name)
        self.task.dt.value = 1e-3
    def createGain(self):
        ''' Set the final time '''
        pass
        #self.T = 1
    def plugEverything(self):
        self.feature.setReference(self.featureDes.name)
        plug(self.dyn.signal(self.opPoint), self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint), self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.dyn.velocity, self.task.qdot)
        ''' Dummy initialization '''
        self.task.duration.value = 1
        self.task.velocityDes.value = (0,0,0,0,0,0)
        self.task.initialTime.value = 0
        #self.task.currentTime.value = 0
    
    def __init__(self,*args):
        MetaTask6d.__init__(self,*args)
        self.opPointModif = OpPointModifier('opmodif'+self.name)
        plug(self.dyn.signal(self.opPoint),self.opPointModif.signal('positionIN'))
        plug(self.dyn.signal('J'+self.opPoint),self.opPointModif.signal('jacobianIN'))
        self.opPointModif.activ = False

    @property
    def opmodif(self):
        if not self.opPointModif.activ: return False
        else: return self.opPointModif.getTransformation()

    @opmodif.setter
    def opmodif(self,m):
        if not self.opPointModif.activ:
            plug(self.opPointModif.signal('position'),self.feature.position )
            plug(self.opPointModif.signal('jacobian'),self.feature.Jq)
            self.opPointModif.activ = True
        self.opPointModif.setTransformation(m)


def goto6dPP(task, position, velocity, duration, current):
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ):
        M=eye(4)
        M[0:3,3] = position
    else:
        #print "Position 6D with rotation ... todo"
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    task.feature.selec.value = "111111"
    task.task.controlSelec.value = "111111"
    task.featureDes.position.value = matrixToTuple(M)
    task.task.duration.value = duration
    task.task.initialTime.value = current
    task.task.velocityDes.value = velocity
    task.task.resetJacobianDerivative()


def gotoNdPP(task, position, velocity, selec, duration, current):
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ):
        M=eye(4)
        M[0:3,3] = position
    else:
        #print "Position 6D with rotation ... todo"
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    if isinstance(selec,str):  
        task.feature.selec.value = selec
        task.task.controlSelec.value = selec
    else:
        task.feature.selec.value = toFlags(selec)
        task.task.controlSelec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    task.task.duration.value = duration
    task.task.initialTime.value = current
    task.task.velocityDes.value = velocity
    task.task.resetJacobianDerivative()
