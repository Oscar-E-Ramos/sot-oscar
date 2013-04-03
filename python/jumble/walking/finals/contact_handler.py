from dynamic_graph.sot.core.matrix_util import tr2rpy
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd
from dynamic_graph.sot.oscar.meta_task_passing_point import goto6dPP

from numpy import radians, sqrt, arccos


class ContactSelect:
    '''
    To handle the contact selection when using the pattern generator with the dynamic stack 
    of tasks. This class can handle both SolverDynReduced and SolverMotionReduced
    '''
    def __init__(self, sot):
        self.sot = sot
        self.leftfootcontact_prev = 1
        self.rightfootcontact_prev = 1

    def setContactTasks(self, contact_rf, contact_lf):
        self.contact_rf = contact_rf
        self.contact_lf = contact_lf
        if (cmp(self.sot.className,'SolverMotionReduced')==0):
            self.sot.signal("_"+self.contact_lf.name+"_ddx3").value = ((0.,0.,0.,),)
            self.sot.signal("_"+self.contact_rf.name+"_ddx3").value = ((0.,0.,0.,),)

    def setFreeSpaceTasks(self, task_rf, task_lf):
        self.taskrf = task_rf
        self.tasklf = task_lf

    def update(self, pg, flag=0):
        ''' If the pg is currently not working, do not do any update '''
        if( not(pg.inprocess.value) ):
            return

        ''' LF leaves floor '''
        if( (self.leftfootcontact_prev ==1) & (pg.leftfootcontact.value==0) ):
            self.sot.rmContact(self.contact_lf.name)
            self.sot.push(self.tasklf.task.name)
            self.tasklf.featureDes.value = pg.leftfootref.value

            ''' LF is swinging foot '''
        elif( (self.leftfootcontact_prev ==0) & (pg.leftfootcontact.value==0) ): 
            self.tasklf.featureDes.value = pg.leftfootref.value

            ''' LF returns to floor '''
        elif( (self.leftfootcontact_prev ==0) & (pg.leftfootcontact.value==1) ): 
            self.sot.rm(self.tasklf.task.name)
            self.sot.addContactFromTask(self.contact_lf.task.name, self.contact_lf.name)
            self.sot.signal("_"+self.contact_lf.name+"_p").value = self.contact_lf.support
            if (cmp(self.sot.className,'SolverMotionReduced')==0):
                self.sot.signal("_"+self.contact_lf.name+"_ddx3").value = ((0.,0.,0.,),)
            self.contact_lf.task.resetJacobianDerivative()

        ''' RF leaves floor '''
        if( (self.rightfootcontact_prev ==1) & (pg.rightfootcontact.value==0) ):
            self.sot.rmContact(self.contact_rf.name)
            self.sot.push(self.taskrf.task.name)
            self.taskrf.featureDes.value = pg.rightfootref.value

            ''' RF is swinging foot '''
        elif( (self.rightfootcontact_prev ==0) & (pg.rightfootcontact.value==0) ): 
            self.taskrf.featureDes.value = pg.rightfootref.value

            ''' RF returns to floor '''
        elif( (self.rightfootcontact_prev ==0) & (pg.rightfootcontact.value==1) ): 
            self.sot.rm(self.taskrf.task.name)
            self.sot.addContactFromTask(self.contact_rf.task.name, self.contact_rf.name)
            self.sot.signal("_"+self.contact_rf.name+"_p").value = self.contact_rf.support
            self.contact_rf.task.resetJacobianDerivative()
            if (cmp(self.sot.className,'SolverMotionReduced')==0):
                     self.sot.signal("_"+self.contact_rf.name+"_ddx3").value = ((0.,0.,0.,),)

        ''' Update the previous value with the current value '''
        self.leftfootcontact_prev = pg.leftfootcontact.value
        self.rightfootcontact_prev = pg.rightfootcontact.value



# ====== Contact Select used with TaskPassingPoint (next position of feet) ===================

class ContactSelectPassingPoint:
    '''
    To handle the contact selection when using the pattern generator
    with the dynamic stack of tasks, and using taskPassingPoint.
    '''
    def __init__(self, sot, dyn, height=0.05, swingT=0.69):
        ''' The system is supposed to start with both feet on the ground '''
        self.sot = sot
        self.dyn = dyn
        self.leftfootcontact_prev = 1
        self.rightfootcontact_prev = 1
        self.height = height
        self.swingT = swingT
        self.time = 0  # To control the swinging time (and get the passing point time)

    def setContactTasks(self, contact_rf, contact_lf):
        self.contact_rf = contact_rf
        self.contact_lf = contact_lf
        if (cmp(self.sot.className,'SolverMotionReduced')==0):
            self.sot.signal("_"+self.contact_lf.name+"_ddx3").value = ((0.,0.,0.,),)
            self.sot.signal("_"+self.contact_rf.name+"_ddx3").value = ((0.,0.,0.,),)

    def setFreeSpaceTasks(self, task_rf, task_lf):
        self.taskPPrf = task_rf
        self.taskPPlf = task_lf

    def update(self, pg, flag=0):
        ''' If the pg is currently not working, do not do any update '''
        if( not(pg.inprocess.value) ):
            return

        ''' LF leaves floor '''
        if( (self.leftfootcontact_prev ==1) & (pg.leftfootcontact.value==0) ):
            self.sot.rmContact(self.contact_lf.name)
            self.sot.push(self.taskPPlf.task.name)
            self.time = 0   # Initialize time as 0 
            ''' Half of the way to go for the foot up '''
            dx = (pg.landingfootposition.value[0]+self.dyn.lf.value[0][3])/2  
            dy = (pg.landingfootposition.value[1]+self.dyn.lf.value[1][3])/2
            dth = (pg.landingfootposition.value[2] + tr2rpy(self.dyn.lf.value)[0])/2
            goto6dPP(self.taskPPlf, (dx, dy, 0.155,0,0,dth), (-0.3,0,0,0,0,0), self.swingT/2,
                     self.sot.solution.time)

            ''' LF is swinging foot '''
        elif( (self.leftfootcontact_prev ==0) & (pg.leftfootcontact.value==0) ): 
            self.time = self.time+1
            if(self.time == (138/2)):
                p = pg.landingfootposition.value
                goto6dPP(self.taskPPlf, (p[0], p[1], 0.105,0,0,p[2]), (0,0,0,0,0,0), 
                         self.swingT/2, self.sot.solution.time)

            ''' LF returns to floor '''
        elif( (self.leftfootcontact_prev ==0) & (pg.leftfootcontact.value==1) ): 
            self.sot.rm(self.taskPPlf.task.name)
            self.sot.addContactFromTask(self.contact_lf.task.name, self.contact_lf.name)
            self.sot.signal("_"+self.contact_lf.name+"_p").value = self.contact_lf.support
            self.contact_lf.task.resetJacobianDerivative()
            if (cmp(self.sot.className,'SolverMotionReduced')==0):
                self.sot.signal("_"+self.contact_lf.name+"_ddx3").value = ((0.,0.,0.,),)

        ''' RF leaves floor '''
        if( (self.rightfootcontact_prev ==1) & (pg.rightfootcontact.value==0) ):
            self.sot.rmContact(self.contact_rf.name)
            self.sot.push(self.taskPPrf.task.name)
            self.time = 0
            dx = (pg.landingfootposition.value[0]+self.dyn.rf.value[0][3])/2
            dy = (pg.landingfootposition.value[1]+self.dyn.rf.value[1][3])/2
            dth = (pg.landingfootposition.value[2] + tr2rpy(self.dyn.rf.value)[0])/2
            goto6dPP(self.taskPPrf, (dx, dy, 0.155,0,0,dth), (-0.3,0,0,0,0,0), self.swingT/2,
                     self.sot.solution.time)

            ''' RF is swinging foot '''
        elif( (self.rightfootcontact_prev ==0) & (pg.rightfootcontact.value==0) ): 
            self.time = self.time+1
            if(self.time == (138/2)):
                p = pg.landingfootposition.value
                goto6dPP(self.taskPPrf, (p[0], p[1], 0.105,0,0,p[2]), (0,0,0,0,0,0), 
                         self.swingT/2, self.sot.solution.time)

            ''' RF returns to floor '''
        elif( (self.rightfootcontact_prev ==0) & (pg.rightfootcontact.value==1) ): 
            self.sot.rm(self.taskPPrf.task.name)
            self.sot.addContactFromTask(self.contact_rf.task.name, self.contact_rf.name)
            self.sot.signal("_"+self.contact_rf.name+"_p").value = self.contact_rf.support
            self.contact_rf.task.resetJacobianDerivative()
            if (cmp(self.sot.className,'SolverMotionReduced')==0):
                self.sot.signal("_"+self.contact_rf.name+"_ddx3").value = ((0.,0.,0.,),)

        ''' Update the previous value with the current value '''
        self.leftfootcontact_prev = pg.leftfootcontact.value
        self.rightfootcontact_prev = pg.rightfootcontact.value

# ============================================================================================


# def addContactFromMetaTask(sot,contact,task=None):
#     sot.addContactFromTask(contact.task.name,contact.name)
#     sot.signal("_"+contact.name+"_p").value = contact.support
#     # Added (but better to remove later, and change it in C++)
#     #sot.signal("_"+contact.name+"_ddx3").value = ((0.,0.,0.,),)
#     if task!= None: sot.rm(task.task.name)
#     contact.task.resetJacobianDerivative()

# def addContactMethod( solverClass ):
#     solverClass.addContactFromMetaTask = addContactFromMetaTask

# class AddContactPointsHelper:
#     '''
#     To be used only with SolverMotionReduced (since it is the only that handles points)
#     and has the signal ddx3
#     '''
#     def __init__(self,sot):
#         addContactMethod(sot.__class__)
#         self.sot=sot
#         self.pointTasks = [];

#     def __call__(self,*a):
#         self.sot.addContactFromMetaTask(*a)

#     def addPointsSwift(self, supPoints, contact):
#         ''' Add an arbitrary number of contact points to a body with rigid contact
#             subPoints: ((x1,...),(y1,...),(z1,...)), contact is a MetaTaskDyn6d
#         '''
#         # If contact exists, append the points to the existing ones
#         if hasattr(self.sot, '_'+contact.name+'_p'):
#             pOld = self.sot.signal("_"+contact.name+"_p").value
#             pNew = (pOld[0]+supPoints[0], pOld[1]+supPoints[1], pOld[2]+supPoints[2])
#             self.sot.signal("_"+contact.name+"_p").value = pNew
#             contact.task.resetJacobianDerivative()
#         # Else, create the contact with the specified points
#         else:
#             contact.support = supPoints
#             self.sot.addContactFromMetaTask(contact)


#     def addPoints(self, supPoints, contact, dyn, opPointRef):
#         ''' supPoints: ((x1,y1,z1),(x2,y2,z2),... )
#             Create tasks, one for each 3Dpoint
#          '''

#         # Convert the contact points to the support points format
#         # ((x1,x2...),(y1,y2,...),(z1,z2,...))
#         X=[]; Y=[]; Z=[]
#         for i in range(len(supPoints)):
#             X.append(supPoints[i][0])
#             Y.append(supPoints[i][1])
#             Z.append(supPoints[i][2])
#         Support = (tuple(X),tuple(Y),tuple(Z))
#         # print "\nSupport ", Support

#         # The contact already exists
#         if hasattr(self.sot, '_'+contact.name+'_p'):
#             supPointsOld = self.sot.signal("_"+contact.name+"_p").value    # Old supporting points
#             # print "Sup points old: ", supPointsOld
#             indexToKeep = []                                               # Indexes to keep their tasks
#             Xnew=[]; Ynew=[]; Znew=[]              # New values for X,Y,Z (support)
#             Xold=[]; Yold=[]; Zold=[]              # Old values for the support that will be kept

#             # Compare the new points with the old ones to see if they are close enough
#             for i in range(len( Support[0] )):
#                 keepNew = True
#                 for j in range(len( supPointsOld[0] )):
#                     # Get the L2 norm distance between the olds and current support points
#                     dist = sqrt( (supPointsOld[0][j]-Support[0][i])**2 + (supPointsOld[1][j]-Support[1][i])**2 + 
#                                  (supPointsOld[2][j]-Support[2][i])**2 )
                    
#                     # If the distance is less than a certain value, discard the new value, keep the old one, and 'continue'
#                     #if (dist < 0.001):
#                     if (dist < 0.05):
#                         indexToKeep.append(j)       # Add the index for the pointTasks that will be kept
#                         keepNew = False
#                         continue

#                 if (keepNew == True):
#                     Xnew.append(Support[0][i])
#                     Ynew.append(Support[1][i])
#                     Znew.append(Support[2][i])
                    
#             # print "New Values: ", Xnew, ", ", Ynew, ", ", Znew
#             # print "indexTokeep: ", indexToKeep

#               # Go through the old support contacts (in decreasing order of the index)
#             for j in reversed( range(len(supPointsOld[0])) ):
#                 # If the contact has to be kept, append it to X,Y,Zold
#                 if j in indexToKeep: 
#                     # Add the support contacts
#                     Xold.append(supPointsOld[0][j])  
#                     Yold.append(supPointsOld[1][j])
#                     Zold.append(supPointsOld[2][j])
#                 # If the contact has to be removed, remove the task associated with it
#                 else:
#                     self.pointTasks[j].task.clear()
#                     self.pointTasks.pop(j)
            
#             # print "Old Values: ", Xold, ", ", Yold, ", ", Zold 

#             # Verify that the name of the tasks (in order) correspond to their index. If so, leave them as they are, 
#             # otherwise, delete them and create a new task (to avoid name's problems when adding new tasks)
#             renameOldTasks = False
#             for i in range(len(Xold)):
#                 name = 'task' + contact.name + 'supportTask' + str(i)
#                 if (self.pointTasks[i].task.name != name):
#                     renameOldTasks = True
#                     break
#             if (renameOldTasks):
#                 for i in reversed( range(len(self.pointTasks)) ):
#                     self.pointTasks[i].task.clear()
#                     self.pointTasks.pop(i)
                
#                 for i in range(len(Xold)):
#                     name = contact.name + 'supportTask' + str(i)
#                     self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
#                     self.pointTasks[i].gain.setConstant(3000)
#                     self.pointTasks[i].opmodif = ((1,0,0,Xold[i]),(0,1,0,Yold[i]),(0,0,1,Zold[i]),(0,0,0,1)) 
#                     self.pointTasks[i].feature.position.recompute(self.sot.solution.time)
#                     p0 = self.pointTasks[i].feature.position.value
#                     gotoNd( self.pointTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
                


#             # Generate the new support composed of the old points and the new points
#             NewSupport = (tuple(Xold+Xnew), tuple(Yold+Ynew), tuple(Zold+Znew))

#             contact.support = NewSupport
#             # print "New Support: ", NewSupport
#             self.sot.signal("_"+contact.name+"_p").value = contact.support
#             #contact.task.resetJacobianDerivative()

#             Nold = len(Xold)
#             taskVectors = []

#             # print "Before adding Xnew tasks: ", self.pointTasks
#             # Only for the new points
#             for i in range(len(Xnew)):
#                 name = contact.name + 'supportTask' + str(Nold+i)
#                 self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
#                 self.pointTasks[i+Nold].gain.setConstant(3000)
#                 self.pointTasks[i+Nold].opmodif = ((1,0,0,Xnew[i]),(0,1,0,Ynew[i]),(0,0,1,Znew[i]),(0,0,0,1)) 
#                 self.pointTasks[i+Nold].feature.position.recompute(self.sot.solution.time)
#                 p0 = self.pointTasks[i+Nold].feature.position.value
#                 gotoNd( self.pointTasks[i+Nold], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
#             # print "After adding Xnew tasks: ", self.pointTasks

#             # Update the values of the tasks
#             taskVectors = []
#             for i in range(len(self.pointTasks)):
#                 self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
#                 taskVectors.append( self.pointTasks[i].task.taskVector.value, )
            
#             # print "task Vectors: ", taskVectors
#             self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)

#         # Else (the contact does not exist), create the contact with the specified points
#         else:
#             contact.support = Support
#             self.sot.addContactFromMetaTask(contact)
#             self.pointTasks = []
#             taskVectors = []
#             for i in range(len(supPoints)):
#                 name = contact.name + 'supportTask' + str(i)
#                 self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
#                 self.pointTasks[i].gain.setConstant(3000)
#                 self.pointTasks[i].opmodif = ((1,0,0,supPoints[i][0]),(0,1,0,supPoints[i][1]),(0,0,1,supPoints[i][2]),(0,0,0,1)) 
#                 self.pointTasks[i].feature.position.recompute(self.sot.solution.time)
#                 p0 = self.pointTasks[i].feature.position.value
#                 gotoNd( self.pointTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
#                 self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
#                 taskVectors.append( self.pointTasks[i].task.taskVector.value, )
        
#             self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)

    
#     def update(self, contact):
#         taskVectors = []
#         for i in range(len(self.pointTasks)):
#             self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
#             taskVectors.append( self.pointTasks[i].task.taskVector.value, )
#         self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)
 

# # Prune points that lie near to a line
# def prune(points, THRES=radians(5)):
#     #THRES = radians(5)
#     Npoints = len(points)
#     PointsOut = []
#     eliminatePoint = []
#     for i in range(Npoints):
#         norm = []
#         v = []
#         p = []
#         ang = []
#         pang = []
#         for j in range(i+1,Npoints):
#             vect = (points[j][0]-points[i][0], points[j][1]-points[i][1], points[j][2]-points[i][2])
#             norm.append( vect[0]**2+vect[1]**2+vect[2]**2 )
#             v.append( vect )
#             p.append( (i,j) )
#         for j in range(len(v)-1):
#             for k in range(j+1,len(v)):
#                 #ang.append( arccos ( (v[j][0]*v[j+1][0] + v[j][1]*v[j+1][1] + v[j][2]*v[j+1][2])/(sqrt(norm[j])*sqrt(norm[j+1])) ) )
#                 ang1 = arccos ( (v[j][0]*v[k][0] + v[j][1]*v[k][1] + v[j][2]*v[k][2])/(sqrt(norm[j])*sqrt(norm[k])) )
#                 ang.append( ang1 )
#                 pang.append( (j,k) )
#                 if ( ang1 < THRES ):
#                     if (norm[j] < norm[k]):
#                         eliminatePoint.append(p[j][1])
#                     else:
#                         eliminatePoint.append(p[k][1])
#     for i in range(Npoints):
#         if i in eliminatePoint:
#             continue
#         else:
#             PointsOut.append(points[i])
#     #return (eliminatePoint)
#     return tuple(PointsOut)
                      
