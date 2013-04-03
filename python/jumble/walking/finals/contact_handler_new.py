from numpy import *
from dynamic_graph.sot.core.matrix_util import tr2rpy, matrixToTuple
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd
from dynamic_graph.sot.oscar.meta_task_passing_point import goto6dPP
#, sqrt, arccos

class ContactSelectPassingPointMotionReduced:
    '''
    To handle the contact selection when using the pattern generator
    with the dynamic stack of tasks, and using taskPassingPoint.
    '''
    def __init__(self, sot, dyn, height=0.05, swingT=0.69):
        ''' The system is supposed to start with both feet on the ground '''
        self.sot = sot          
        self.dyn = dyn
        self.height = height
        self.swingT = swingT

        ''' Internal variables '''
        self.leftfootcontact_prev = 1   # Previous indicator of foot in contact (pattern)
        self.rightfootcontact_prev = 1
        self.swingTime = 0              # To control the swinging time (and get the passing point time)

        ''' Lists containing tasks 6d '''
        self.taskKeepContactRf = []     # Task to keep the contact points still (get the ddx3)
        self.taskKeepContactLf = []
        self.taskMoveCornerRf = []          # Tasks to take the corners of the feet to the ground
        self.taskMoveCornerLf = []

        ''' Lists containing indices '''
        self.indCornerToFloor = []         # Indices of the corners that will go to the floor
        self.indCornerInContactRf = []     # Indices to indicate which corner is already in contact
        self.indCornerInContactLf = []

        self.dictRToFloor = {}          # Dict to store the index of corner and the task
        self.dictLToFloor = {}          # Dict to store the index of corner and the task

        ''' Order: Front-right, back-right, back-left, front-left '''
        corners_rf = ((0.13,-0.10,-0.10,0.13),(-0.0775,-0.0775,0.0575,0.0575),(-0.105,-0.105,-0.105,-0.105))
        corners_lf = ((0.13,-0.10,-0.10,0.13),(0.0775,0.0775,-0.0575,-0.0575),(-0.105,-0.105,-0.105,-0.105))
        self.cornersRf = self.formatSupportToPoints(corners_rf)
        self.cornersLf = self.formatSupportToPoints(corners_lf)

    def setRobot(self, robot):
        self.robot = robot

    def setContactTasks(self, contact_rf, contact_lf):
        self.contactRf = contact_rf
        self.contactLf = contact_lf

    def setFreeSpaceTasks(self, task_rf, task_lf):
        self.taskPPrf = task_rf
        self.taskPPlf = task_lf

    def setCollisionDetection(self, collision_rf, collision_lf):
        self.collisionRf = collision_rf
        self.collisionLf = collision_lf

    def setFeetCorners(self, corners_rf, corners_lf):
        ''' Alternatively changed, otherwise, defined by default at initialization
        format of the corners:  ((x1,x2...xn),(y1,y2,...yn),(z1,z2,...zn)) '''
        self.cornersRf = self.formatSupportToPoints(corners_rf)
        self.cornersLf = self.formatSupportToPoints(corners_lf)



    def update(self, pg, flag=0):
        ''' If the pg is currently not working, do not do any update '''
        if( not(pg.inprocess.value) ):
            return

        ''' === === === === LF leaves floor === === === === '''
        if( (self.leftfootcontact_prev ==1) & (pg.leftfootcontact.value==0) ):
            self.sot.rmContact(self.contactLf.name)
            self.sot.push(self.taskPPlf.task.name)
            self.footUpwards(pg, self.taskPPlf, 'left-ankle')
            self.swingTime = 0                                       # Initialize swingTime as 0 

            ''' LF is swinging foot '''
        elif( self.leftfootcontact_prev ==0 ) : 
            self.swingTime += 1

            ''' Baaaaaad, but apparently working '''
            #print 'set velocity rf null'
            if (self.swingTime%4)==0:
                self.setVelocityNull('right-ankle')

            ''' LF going up (first half time of the swinging phase) '''
            if(self.swingTime == (138/2)):
                self.footDownwards(pg, self.taskPPlf)

            ''' LF going down (seconf half time of the swinging phase) '''
            if(self.swingTime > (138/2)):
                ''' Check if there is collision of the left foot '''
                if ( len(self.collisionLf.contactPoints.value) > 0 ): 
                    self.sot.rm(self.taskPPlf.task.name)
                    Pc = self.collisionLf.contactPoints.value                  # Contact points in world frame
                    aPc = self.convertFrames(matrix(self.dyn.lf.value).I, Pc)  # Contact points in ankle's frame
                    ''' Add contact of LF specifying the first contact point(s) '''
                    self.addPointsToContact(aPc, self.contactLf, self.taskKeepContactLf, 
                                            self.taskMoveCornerLf, self.indCornerInContactLf, 'left-ankle')
                    #print aPc

            #self.setVelocityNull('right-ankle')

            ''' === === === === RF leaves floor === === === === '''
        if( (self.rightfootcontact_prev ==1) & (pg.rightfootcontact.value==0) ):
            self.sot.rmContact(self.contactRf.name)
            self.sot.push(self.taskPPrf.task.name)
            self.footUpwards(pg, self.taskPPrf, 'right-ankle')
            self.swingTime = 0

            ''' RF is swinging foot '''
        elif( self.rightfootcontact_prev ==0 ): 
            self.swingTime += 1
            ''' RF going up (half time of the swinging phase) '''
            if(self.swingTime == (138/2)):
                self.footDownwards(pg, self.taskPPrf)

            ''' RF going down (second half time of the swinging phase) '''
            if(self.swingTime > (138/2)):
                ''' Check if the right foot is colliding with anything  '''
                #print 'rf going down'
                if ( len(self.collisionRf.contactPoints.value) > 0 ): 
                    self.sot.rm(self.taskPPrf.task.name)                       # Remove going down foot task
                    Pc = self.collisionRf.contactPoints.value                  # Contact points in world frame
                    aPc = self.convertFrames(matrix(self.dyn.rf.value).I, Pc)  # Contact points in ankle's frame
                    ''' Add contact of RF specifying the first contact point(s) '''
                    self.addPointsToContact(aPc, self.contactRf, self.taskKeepContactRf,
                                            self.taskMoveCornerRf,  self.indCornerInContactRf, 'right-ankle')

        ''' Always Update contact, if the foot is not swinging '''
        if '_'+self.contactLf.name+'_ddx3' in [s.name for s in self.sot.signals()]:
            self.updateContactKeepTasks(self.contactLf, self.taskKeepContactLf)
        if '_'+self.contactRf.name+'_ddx3' in [s.name for s in self.sot.signals()]:
            self.updateContactKeepTasks(self.contactRf, self.taskKeepContactRf)

        ''' Add contact points if they have been detected, eliminating the previous corner tasks '''
        if '_'+self.contactRf.name+'_ddx3' in [s.name for s in self.sot.signals()]:
            if ( len(self.collisionRf.contactPoints.value) > 0): 
                self.sot.rm(self.taskPPrf.task.name)
                Pc = self.collisionRf.contactPoints.value                  # Contact points in world frame
                aPc = self.convertFrames(matrix(self.dyn.rf.value).I, Pc)  # Contact points in ankle's frame
                ''' Add contact of RF specifying the first contact point(s) '''
                self.addContactRemoveCornerTasks(aPc, self.contactRf, self.taskKeepContactRf,
                                                 self.taskMoveCornerRf, self.indCornerInContactRf, 'right-ankle')
                #self.setVelocityNullAll()

        # if '_'+self.contactLf.name+'_ddx3' in [s.name for s in self.sot.signals()]:
        #     self.addContactRemoveCornerTasks(self.contactLf, self.taskContactKeepLf)
                
        ''' Update the previous value with the current value '''
        self.leftfootcontact_prev = pg.leftfootcontact.value
        self.rightfootcontact_prev = pg.rightfootcontact.value




    def addPointsToContact(self, Pc, contactF, contactKeepTasks, cornerTask, indCornerInContact, opPointRef):
        ''' Add contact points as support points, create tasks (one for each 3Dpoint),
            take the extremes fo the foot to he ground.
        - Pc: ((x1,y1,z1),(x2,y2,z2),... ) -- contact points wrt ankle
        - contactF: foot contact task (to keep the points fixed)
        - contactKeepTasks: tasks to obtain the acceleration ddx3 (one for each point)
        - cornerTask: tasks to take the corners to the ground
        - indCornerInContact: indices of the corners that are already in contact
        - opPointRef: reference point (right-ankle or left-ankle)
        '''

        #print "collision: ", Pc
        #support = self.formatPointsToSupport(Pc)

        ''' Check if the contact exists '''
        if hasattr(self.sot, '_'+contactF.name+'_p'):                 # If the contact exists
            tolerance = 0.002
            [Xnew, Ynew, Znew, Xold, Yold, Zold] = self.removeClosePoints(tolerance, Pc, contactF, contactKeepTasks)
            self.renamePointTasks( contactF, contactKeepTasks, Xold, Yold, Zold, opPointRef)

            ''' Generate the new support composed of the old points (and the new points, if there are) '''
            NewSupport = (tuple(Xold+Xnew), tuple(Yold+Ynew), tuple(Zold+Znew))
            contactF.support = NewSupport
            #print 'New support in add points to contact: ', NewSupport
            self.sot.signal("_"+contactF.name+"_p").value = contactF.support
            contactF.task.resetJacobianDerivative()
            Nold = len(Xold)

            ''' For the new points: Create one task for each point '''
            for i in range(len(Xnew)):
                name = contactF.name + 'supportTask' + str(Nold+i)
                contactKeepTasks.append( MetaTaskDyn6d(name, self.dyn, name, opPointRef) )
                contactKeepTasks[i+Nold].gain.setConstant(3000)
                contactKeepTasks[i+Nold].opmodif = ((1,0,0,Xnew[i]),(0,1,0,Ynew[i]),(0,0,1,Znew[i]),(0,0,0,1)) 
                contactKeepTasks[i+Nold].feature.position.recompute(self.sot.solution.time)
                p0 = contactKeepTasks[i+Nold].feature.position.value
                gotoNd( contactKeepTasks[i+Nold], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation

            #print NewSupport
            if (len(NewSupport[0])<3):
                ''' Find the corner points that generate the greatest polygon '''
                indCornerToFloorN = self.findCornersGreatestPolygon(NewSupport, opPointRef)  # Returns the indices
                print "corners to floor: ", indCornerToFloorN
                ''' Add the tasks to move the 'proper' corners to the floor '''
                self.addFootExtremes(indCornerToFloorN, indCornerInContact, opPointRef)

            #self.setVelocityNull(opPointRef)

            ''' The contact does not exist: create it with the specified points '''
        else:          
            self.sot.addContactFromTask(contactF.task.name, contactF.name)
            self.sot.signal("_"+contactF.name+"_p").value = self.formatPointsToSupport(Pc)
            contactF.task.resetJacobianDerivative()
            print ' -- Contact created -- '

            taskVectors = []
            [contactKeepTasks.pop() for z in xrange(len(contactKeepTasks))]    # Clear the list 'by reference'

            for i in range( len(Pc) ):
                name = contactF.name + 'SupportTask' + str(i)
                ''' Create one task for each contact point '''
                contactKeepTasks.append( MetaTaskDyn6d(name, self.dyn, name, opPointRef) )
                contactKeepTasks[i].gain.setConstant(3000)
                contactKeepTasks[i].featureDes.velocity.value=(0,0,0)
                contactKeepTasks[i].feature.errordot.value=(0,0,0)
                ''' Change the operational position to correspond to the contact point '''
                contactKeepTasks[i].opmodif = ((1,0,0,Pc[i][0]),(0,1,0,Pc[i][1]),(0,0,1,Pc[i][2]),(0,0,0,1)) 
                ''' Control the position of the task (points) to keep the current value '''
                contactKeepTasks[i].feature.position.recompute(self.sot.solution.time)
                p0 = contactKeepTasks[i].feature.position.value
                gotoNd( contactKeepTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control translation
                ''' Get the acceleration of the points (taskVector) to use it for the ddx3 in the drift '''
                contactKeepTasks[i].task.taskVector.recompute(self.sot.solution.time)
                taskVectors.append( contactKeepTasks[i].task.taskVector.value, )
    
            self.sot.signal("_"+contactF.name+"_ddx3").value = tuple(taskVectors)
            #print 'contactKeepTasks: ', contactKeepTasks

            # ''' Check the greatest polygon to add a task to move to that point '''
            # self.findCornersGreatestPolygon(NewSupport)

            #self.setVelocityNull(opPointRef)
            self.setVelocityNullAll()
            #self.setVelocityNull('left-ankle')
            #self.setVelocityNull('right-ankle')

    def updateContactKeepTasks(self, contactF, contactKeepTasks):
        ''' Update the value of the acceleration for the contact (to be kept still)
            - contactF: dyn task to keep the contact foot still
            - contactKeepTasks: tasks on the support points to get their acceleration ddx
        '''
        #print 'update, len(contactKeepTasks)=', len(contactKeepTasks)
        ''' Dirty thing to initialize (to rethink) '''
        if len(contactKeepTasks) == 0:
            self.sot.signal("_"+contactF.name+"_ddx3").value = ((0.,0.,0.,),)
            return 0

        ''' Recompute tasks to keep the contacts still '''
        taskVectors = []
        for i in range( len(contactKeepTasks) ):
            contactKeepTasks[i].task.taskVector.recompute(self.sot.solution.time)
            taskVectors.append( contactKeepTasks[i].task.taskVector.value, )

        ''' Update the value of ddx3 '''
        #self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors) # It should work like this (it doesn't)
        # THIS IS STRANGE, WHY IS THIS WORKING OK LIKE THIS ????????
        self.sot.signal("_"+contactF.name+"_ddx3").value = ((0.,0.,0.,),)  # It shouldn't work like this (it does!)


        '''
        -----------------------------------------------------------------------------
        === === === === === === ===  Auxiliary functions  === === === === === === ===
        -----------------------------------------------------------------------------
        '''

    def addContactRemoveCornerTasks(self, Pc, contactF, contactKeepTasks, cornerTask, indCornerInContact, opPointRef):
        ''' Detect the collision of the points. If a point is closer within a radius of 'th' to the 
        contactTask, then, remove that contactTask, and add the point to the contact
        - Pc: ((x1,y1,z1),(x2,y2,z2),... ) -- contact points wrt ankle
        - contactF: foot contact task (to keep the points fixed)
        - contactKeepTasks: tasks to obtain the acceleration ddx3 (one for each point)
        - cornerTask: tasks to take the corners to the ground
        - indCornerInContact: indices of the corners that are already in contact
        - opPointRef: reference point (right-ankle or left-ankle)
        '''

        ''' Get the corners values and the value of dict ''' 
        if (opPointRef=='right-ankle'):
            corner = self.cornersRf           # corner points
            dictToFloor = self.dictRToFloor   # dictionary that stores the corner tasks
        elif (opPointRef=='left-anlke'):
            corner = self.cornersLf
            dictToFloor = self.dictLToFloor

        ''' If the collision is close to a corner task point, remove the point and add it to the 
        contact foot '''
        # print '\nPc: ', Pc
        # print 'corner:', corner

        ThDist = 0.005
        for i in range(len(Pc)):
            for j in range(len(self.indCornerToFloor)):

                ''' Check if the dictionary is not empty '''
                if dictToFloor.has_key(self.indCornerToFloor[j])==False:
                    continue

                if ( self.distance(Pc[i], corner[self.indCornerToFloor[j]]) < ThDist ):
                    ''' Remove CornerToFloorTask and remove from the dictionary '''
                    # print 'i=',i,' -- Pc: ', Pc[i]
                    # print 'j=', j, '-- corner: ', corner[self.indCornerToFloor[j]]
                    # print 'dict: ', dictToFloor

                    self.sot.rm(dictToFloor[self.indCornerToFloor[j]].task.name)
                    dictToFloor.pop(self.indCornerToFloor[j])

                    ''' ---------------------------------- '''
                    ''' addToContact '''

                    tolerance = 0.002
                    [Xnew, Ynew, Znew, Xold, Yold, Zold] = self.removeClosePoints(tolerance, Pc, contactF, contactKeepTasks)
                    self.renamePointTasks( contactF, contactKeepTasks, Xold, Yold, Zold, opPointRef)

                    ''' Generate the new support composed of the old points (and the new points, if there are) '''
                    NewSupport = (tuple(Xold+Xnew), tuple(Yold+Ynew), tuple(Zold+Znew))
                    contactF.support = NewSupport
                    self.sot.signal("_"+contactF.name+"_p").value = contactF.support
                    contactF.task.resetJacobianDerivative()
                    Nold = len(Xold)

                    ''' For the new points: Create one task for each point '''
                    for k in range(len(Xnew)):
                        name = contactF.name + 'supportTask' + str(Nold+k)
                        contactKeepTasks.append( MetaTaskDyn6d(name, self.dyn, name, opPointRef) )
                        contactKeepTasks[k+Nold].gain.setConstant(3000)
                        contactKeepTasks[k+Nold].opmodif = ((1,0,0,Xnew[k]),(0,1,0,Ynew[k]),(0,0,1,Znew[k]),(0,0,0,1)) 
                        contactKeepTasks[k+Nold].feature.position.recompute(self.sot.solution.time)
                        p0 = contactKeepTasks[k+Nold].feature.position.value
                        gotoNd( contactKeepTasks[k+Nold], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation

                    #print NewSupport
                    if (len(NewSupport[0])<3):
                        ''' Find the corner points that generate the greatest polygon '''
                        indCornerToFloorN = self.findCornersGreatestPolygon(NewSupport, opPointRef)  # Returns the indices
                        print "corners to floor: ", indCornerToFloorN
                        ''' Add the tasks to move the 'proper' corners to the floor '''
                        self.addFootExtremes(indCornerToFloorN, indCornerInContact, opPointRef)

                    ''' ---------------------------------- '''


                    print 'task to be removed: ', self.indCornerToFloor[j]
                    ''' Add the corner to the in contact list '''
                    indCornerInContact.append(self.indCornerToFloor[j])        

                    self.setVelocityNullAll()

    

    def distance(self, P1, P2):
        ''' Find the euclidean distance between two points
        format of P: (x,y,z) '''
        tmp = sqrt( (P1[0]-P2[0])**2 + (P1[1]-P2[1])**2 + (P1[2]-P2[2])**2 )
        return tmp


    #def addFootExtremes(self, indCornerToFloorN, NewSupport, opPointRef, cornerTask):
    def addFootExtremes(self, indCornerToFloorN, indCornerInContactN, opPointRef):
        ''' Add tasks for the extremes 
        '''
        if (opPointRef=='right-ankle'):  Pc = self.cornersRf                # Pc: corner points
        elif (opPointRef=='left-anlke'): Pc = self.cornersLf

        #print 'corner to floor: ', self.indCornerToFloor
        #print 'corner to floor N: ', indCornerToFloorN
        ''' Discard the corner if it is already in contact '''
        for i in range(len(indCornerInContactN)):                      
            if self.indCornerToFloor.count(indCornerInContactN[i])>0:
                self.indCornerToFloor.remove(indCornerInContactN[i])
            if indCornerToFloorN.count(indCornerInContactN[i])>0:
                indCornerToFloorN.remove(indCornerInContactN[i])

        ''' See if the old values are in the new list (corners to floor) '''
        for i in range(len(self.indCornerToFloor)):
            try:
                temp = indCornerToFloorN.index(self.indCornerToFloor[i])
            except ValueError:
                self.addCornerTask(self.indCornerToFloor[i], opPointRef, Pc)

        ''' See if the new values are in the old list (corners to floor) '''
        for i in range(len(indCornerToFloorN)):
            try:
                temp = self.indCornerToFloor.index(indCornerToFloorN[i])
            except ValueError:
                self.addCornerTask( indCornerToFloorN[i], opPointRef, Pc)
        
        self.indCornerToFloor = indCornerToFloorN


    def addCornerTask( self,cornerInd, opPointRef, Pc):
        ''' Add task for a corner (extreme of a foot) '''

        name = 'cornerTask' + str(cornerInd)
        print 'add task ', name
        #print 

        ''' Choose the dictionary to be used '''
        if (opPointRef == 'right-ankle'):
            dictToFloor = self.dictRToFloor
        elif(opPointRef == 'left-ankle'):
            dictToFloor = self.dictLToFloor

        ''' Create one task for each corner '''
        dictToFloor[cornerInd] = MetaTaskDyn6d(name, self.dyn, name, opPointRef)
        dictToFloor[cornerInd].gain.setConstant(1000)
        ''' Change the operational position to correspond to the corner '''
        dictToFloor[cornerInd].opmodif = ((1,0,0,Pc[cornerInd][0]),(0,1,0,Pc[cornerInd][1]),
                                               (0,0,1,Pc[cornerInd][2]),(0,0,0,1)) 
        ''' Control the position of the task (points) to keep the current value '''
        dictToFloor[cornerInd].feature.position.recompute(self.sot.solution.time)
        p0 = dictToFloor[cornerInd].feature.position.value
        #gotoNd( cornerTask[i], (p0[0][3], p0[1][3], 0, 0, 0, 0), "000111")  # Take to the floor
        gotoNd( dictToFloor[cornerInd], (p0[0][3], p0[1][3], 0, 0, 0, 0), "000100")  # Take to the floor (z)
        self.sot.push(dictToFloor[cornerInd].task.name)


    def findCornersGreatestPolygon(self, support, opPointRef):
        ''' Get the corners of the foot 
        input: support, opPointRef
        output: (index1, index2) corners that generate the largest polygon
        '''
        if (opPointRef=='right-ankle'):
            corner = self.formatPointsToSupport(self.cornersRf)
        if (opPointRef=='left-anlke'):
            corner = self.formatPointsToSupport(self.cornersLf)
        #print "support: ", support

        ''' If just one contact point, search the greatest polygon with the corners '''
        if ( len(support[0]) == 1 ):
            area = []
            ''' For each corner of the foot, use the next clockwise corner and find the area '''
            for i in range(4):
                p1 = matrix((support[0][0], support[1][0], support[2][0]))
                p2 = matrix((corner[0][i], corner[1][i], corner[2][i]))
                if (i==3): p3 = matrix((corner[0][0], corner[1][0], corner[2][0]))
                else:      p3 = matrix((corner[0][i+1], corner[1][i+1], corner[2][i+1]))
                ''' Area of the triangle is 0.5*norm((p3-p1)x(p3-p2)) '''
                area.append( 0.5*linalg.norm(cross(p3-p1,p3-p2)) )
                # print 'i:', i, 'p1:', p1, 'p2:', p2, 'p3:', p3

            ''' Find index (and thus, the corresponding corners) for the maximum area '''
            #print area
            maxIndex1 = area.index(max(area))      # Index corresponding to largest area (1st point)
            maxIndex2 = maxIndex1+1                # 2nd point generating the largest area
            if (maxIndex2==4): maxIndex2=0
            return ((maxIndex1, maxIndex2))

            ''' If two contact points, find the area of the polygones '''
        elif ( len(support[0]) == 2 ):
            indexCorners = self.findCornersSide(corner, support)
            return indexCorners
            #print "corner side: ", cornerSide
        else:
            return ((NaN, NaN))

    def findCornersSide(self, corner, support):
        ''' Find the index of the corners that are on the side of the support that generates the
        polygon with the greatest area.
        (the assumption is that the current support is formed by two points)
        Inputs: corners (support format)
                support (support format)
        '''

        ''' Find which corners are on which side of the supporting line '''
        corner = self.formatSupportToPoints(corner)
        cornerSide1=[]; cornerSide2=[]
        indexSide1=[]; indexSide2=[]
        p1 = matrix((support[0][0], support[1][0], support[2][0]))
        p2 = matrix((support[0][1], support[1][1], support[2][1]))
        v = p1 - p2
        beta = arctan2(v[0,1], v[0,0])
        for i in range(4):
            p_i = matrix(corner[i])
            vi = p_i - p2
            th = arctan2(vi[0,0]*sin(-beta)+vi[0,1]*cos(-beta), 
                         vi[0,0]*cos(-beta)-vi[0,1]*sin(-beta))
            if (th<=0):
                cornerSide1.append(corner[i])
                indexSide1.append(i)
            else:
                cornerSide2.append(corner[i])
                indexSide2.append(i)

        ''' There are 3 corners on one side and 1 on the other side '''
        if (len(cornerSide1)==1):
            return indexSide2
        elif (len(cornerSide2)==1):
            return indexSide1
            ''' There are 2 corners on each side '''
        else:
            ''' Find the areas for the quadrilaterals '''
            A1 = self.findAreaQuadrilateral(cornerSide1, support)
            A2 = self.findAreaQuadrilateral(cornerSide2, support)
            # print 'cornerSide1: ', cornerSide1, 'Area1: ', A1
            # print 'cornerSide2: ', cornerSide2, 'Area2: ', A2
            if (A1>A2):
                return indexSide1
            elif (A2>A1):
                return indexSide2

    def findAreaQuadrilateral(self, cornerSide, support):
        ''' Find the area of a quadrilateral
        support: known to have a size of 2 points (support format)
        cornerSide: corners of the foot (points format)
        '''
        if( len(cornerSide) != 2 ): print '--- Error in findAreaQuadrilateral ---'

        ''' Get the correct order of p0,p1,p2,p3 '''
        support = self.formatSupportToPoints(support)
        p0 = matrix(support[0]); p1 = matrix(support[1])
        p2t = matrix(cornerSide[0]); p3t = matrix(cornerSide[1])
        v0 = p1-p0; v2 = p2t-p0; v3 = p3t-p0
        th2 = arccos( v0*v2.T/(linalg.norm(v0)*linalg.norm(v2)) )
        th3 = arccos( v0*v3.T/(linalg.norm(v0)*linalg.norm(v3)) )
        if (th2<th3): p2=p2t; p3=p3t
        else: p2=p3t; p3=p2t

        ''' Find the areas with the sorted ps '''
        Atemp1 = self.findAreaTriangle(p0,p1,p2)
        Atemp2 = self.findAreaTriangle(p0,p2,p3)

        ''' Return the sum of the areas of both sub-triangles '''
        return (Atemp1+Atemp2)
    

    def findAreaTriangle(self, p0,p1,p2):
        ''' Find the area given 3 points in format matrix((x,y,z)) '''
        v1 = p1-p0; v2 = p2-p0
        return ( 0.5*linalg.norm(cross(v2,v1)) )
        

    def setVelocityNull(self, opPointRef):
        ''' Project the velocity to the null space of a single foot '''
        if (opPointRef == 'right-ankle'):
            self.sot._RF_J.recompute(self.robot.control.time)
            Jc = matrix(self.sot._RF_J.value)
        elif (opPointRef == 'left-ankle'):
            self.sot._LF_J.recompute(self.robot.control.time)
            Jc = matrix(self.sot._LF_J.value)

        #dq = matrix(self.sot.velocity.value).T
        dq = matrix(self.robot.velocity.value).T
        dq1 = dq - linalg.pinv(Jc)*Jc*dq
        self.robot.setVelocity( matrixToTuple(dq1.T)[0] )

    def setVelocityNullAll(self):
        ''' Project the velocity to the null space of both
        feet, if they are defined as contact '''
        if '_RF_p' in [s.name for s in self.sot.signals()]:
            self.sot._RF_J.recompute(self.robot.control.time)
            Jcr = matrix(self.sot._RF_J.value)
            if '_LF_p' in [s.name for s in self.sot.signals()]:
                self.sot._LF_J.recompute(self.robot.control.time)
                Jcl = matrix(self.sot._LF_J.value)
                Jc = bmat([[Jcr],[Jcl]])
            else:
                Jc = Jcr
        elif '_LF_p' in [s.name for s in self.sot.signals()]:
            self.sot._LF_J.recompute(self.robot.control.time)
            Jc = matrix(self.sot._LF_J.value)

        dq = matrix(self.sot.velocity.value).T
        dq1 = dq - linalg.pinv(Jc)*Jc*dq
        self.robot.setVelocity( matrixToTuple(dq1.T)[0] )


    def removeClosePoints(self, tolerance, Pc, contactF, contactKeepTasks):
        ''' Remove close points and eliminate tasks associated with old points
        Pc: contact points (with respect to the ankle)
        contactF: contact Tasks (to keep the points fixed)
        contactKeepTasks: tasks to obtain the acceleration ddx3
        tolerance: if the distance between a new point and a previous one is smaller than
        tolerance, then the new value is eliminated and the previous closest one is kept '''

        support     = self.formatPointsToSupport(Pc)                # New support points (wrt ankle)
        supportPrev = self.sot.signal("_"+contactF.name+"_p").value  # Previous support points (wrt ankle)
        #print "support = ", support, "\nsupport prev = ", supportPrev

        tolerance_distance = tolerance
        prevIndexToKeep = []                   # Indexes to keep their tasks
        Xnew=[]; Ynew=[]; Znew=[]              # New values for X,Y,Z (support)
        Xold=[]; Yold=[]; Zold=[]              # Old values for the support that will be kept

        ''' Compare the new points with the previous ones to see if they are close enough '''
        for i in range(len( support[0] )):
            keepNew = True
            ''' For all the previous support points '''
            for p in range(len( supportPrev[0] )):
                ''' Get the L2 norm distance between the previous and the current points '''
                dist = sqrt( (supportPrev[0][p]-support[0][i])**2 + (supportPrev[1][p]-support[1][i])**2 + 
                             (supportPrev[2][p]-support[2][i])**2 )
                    
                ''' If the points are close enough, discard the new value, keep the old one, and continue '''
                if (dist < tolerance_distance):     # 0.05  #0.001
                    prevIndexToKeep.append(p)       # Add index for the contactKeepTasks that will be kept
                    keepNew = False
                    continue

            ''' If the new point was not discarded (it is not close to a previous point) '''
            if (keepNew == True):
                Xnew.append(support[0][i])
                Ynew.append(support[1][i])
                Znew.append(support[2][i])
                    
        # print "New Values: ", Xnew, ", ", Ynew, ", ", Znew
        # print "prevIndexToKeep= ", prevIndexToKeep

        ''' Go through the previous support contacts (in decreasing order of the index) '''
        #print len(contactKeepTasks)
        for p in reversed( range(len(supportPrev[0])) ):
            ''' If the contact has to be kept, append it to X,Y,Zold '''
            if p in prevIndexToKeep: 
                #print 'keep ', p
                ''' Add the support contacts '''
                Xold.append(supportPrev[0][p])  
                Yold.append(supportPrev[1][p])
                Zold.append(supportPrev[2][p])
                ''' If the contact has to be removed, remove the task associated with it '''
            else:
                print 'clear support point (task) ', p
                contactKeepTasks[p].task.clear()
                contactKeepTasks.pop(p)

        # print "Old Values: ", Xold, ", ", Yold, ", ", Zold 
        return Xnew, Ynew, Znew, Xold, Yold, Zold

    def renamePointTasks(self, contactF, contactKeepTasks, Xold, Yold, Zold, opPointRef):
        ''' Verify that the name of the tasks (in order) correspond to their index. If so, leave them as they are, 
        otherwise, delete them and create a new task (to avoid name's problems when adding new tasks) '''
        renameOldTasks = False
        ''' For all the tasks corresponding to the previous points that were kept '''
        for i in range(len(Xold)):
            name = 'task' + contactF.name + 'supportTask' + str(i)
            ''' If at least one name does not coincide, perform a renaming '''
            if (contactKeepTasks[i].task.name != name):
                renameOldTasks = True
                break
        if (renameOldTasks):
            ''' Clear the feature associated with each task '''
            for i in reversed( range(len(contactKeepTasks)) ):
                contactKeepTasks[i].task.clear()
                contactKeepTasks.pop(i)
                
            for i in range(len(Xold)):
                name = contactF.name + 'supportTask' + str(i)
                contactKeepTasks.append( MetaTaskDyn6d(name, self.dyn, name, opPointRef) )
                contactKeepTasks[i].gain.setConstant(3000)
                contactKeepTasks[i].opmodif = ((1,0,0,Xold[i]),(0,1,0,Yold[i]),(0,0,1,Zold[i]),(0,0,0,1)) 
                contactKeepTasks[i].feature.position.recompute(self.sot.solution.time)
                p0 = contactKeepTasks[i].feature.position.value
                gotoNd( contactKeepTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation


    def convertFrames(self, M, P_in):
        ''' the format of M is a 4x4 homogeneous transformation matrix
        the format of P_in is: ((x1,y1,z1), (x2,y2,z2), ... )
        the format of the output is the same as the input
        '''
        P_out = []
        for i in range(len(P_in)):
            tmp = matrixToTuple( M*matrix(P_in[i]+(1.,)).T )
            P_out.append( (tmp[0][0], tmp[1][0], tmp[2][0]) )
        return tuple(P_out)

    def cornersToWorldFrame(self, cornerWrtAnkle, M):
        ''' Input: cornerWrtAnkle: ((x1,x2,..),(y1,y2,...),(z1,z2,...))
                   M: worldMankle (ankle with respect to world
            Output: corners with respect to world: ((x1,y1,z1),(x2,y2,z2),...) '''
        corner = []
        for i in range(len(cornerWrtAnkle[0])):
            cornerWrtWorld = M*matrix((cornerWrtAnkle[0][i],cornerWrtAnkle[1][i],cornerWrtAnkle[2][i],1)).T
            corner.append( (matrixToTuple(cornerWrtWorld.T)[0])[0:3] )
        return corner

    def formatSupportToPoints(self, Pin):
        ''' Change the format from the one used for the support to the points format
        Input format:  ((x1,x2...xn),(y1,y2,...yn),(z1,z2,...zn))
        Output format: ((x1,y1,z1),(x2,y2,z2),...,(xn,yn,zn))
        '''
        Pout=[];
        for i in range( len(Pin[0]) ):
            Pout.append((Pin[0][i],Pin[1][i],Pin[2][i]))
        return tuple(Pout)

    def formatPointsToSupport(self, Pin):
        ''' Change the format from points to the support format
        Input format:  ((x1,y1,z1),(x2,y2,z2),...,(xn,yn,zn))
        Output format: ((x1,x2...xn),(y1,y2,...yn),(z1,z2,...zn))
        '''
        X=[]; Y=[]; Z=[]
        for i in range( len(Pin) ):
            X.append(Pin[i][0])
            Y.append(Pin[i][1])
            Z.append(Pin[i][2])
        return (tuple(X), tuple(Y), tuple(Z))

    def footUpwards(self, pg, taskPP, opPointRef):
        ''' Add a task to move the foot to the uppermost part which is halfway to the final 
        forward position '''
        if (opPointRef=='right-ankle'): footPose = self.dyn.rf.value
        if (opPointRef=='left-ankle'):  footPose = self.dyn.lf.value
        dx = (pg.landingfootposition.value[0]+footPose[0][3])/2
        dy = (pg.landingfootposition.value[1]+footPose[1][3])/2
        dth = (pg.landingfootposition.value[2] + tr2rpy(footPose)[0])/2
        goto6dPP(taskPP, (dx, dy, 0.155,0,0,dth), (-0.3,0,0,0,0,0), self.swingT/2, self.sot.solution.time)

    def footDownwards(self, pg, taskPP):
        ''' Add a task to move the foot to the final position specified by the pattern generator '''
        p = pg.landingfootposition.value
        goto6dPP(taskPP, (p[0], p[1], 0.105,0,0,p[2]), (0,0,0,0,0,0), self.swingT/2, self.sot.solution.time)




# ============================================================================================

# ============================================================================================

# ============================================================================================



def addContactFromMetaTask(sot,contact,task=None):
    sot.addContactFromTask(contact.task.name,contact.name)
    sot.signal("_"+contact.name+"_p").value = contact.support
    # Added to naively initialize ddx3 (but better to remove later, and change it in C++)
    sot.signal("_"+contact.name+"_ddx3").value = ((0.,0.,0.,),)
    if task!= None: sot.rm(task.task.name)
    contact.task.resetJacobianDerivative()

def addContactMethod( solverClass ):
    solverClass.addContactFromMetaTask = addContactFromMetaTask


class AddContactPointsHelperNew:
    '''
    To be used only with SolverMotionReduced (since it is the only that handles points)
    and has the signal ddx3
    '''
    def __init__(self,sot):
        addContactMethod(sot.__class__)
        self.sot=sot
        self.pointTasks = [];

    def __call__(self,*a):
        self.sot.addContactFromMetaTask(*a)

    def addPoints(self, supPoints, contact, dyn, opPointRef):
        ''' supPoints: ((x1,y1,z1),(x2,y2,z2),... )
            Create tasks, one for each 3Dpoint
         '''

        # Convert the contact points to the support points format
        # ((x1,x2...),(y1,y2,...),(z1,z2,...))
        X=[]; Y=[]; Z=[]
        for i in range(len(supPoints)):
            X.append(supPoints[i][0])
            Y.append(supPoints[i][1])
            Z.append(supPoints[i][2])
        Support = (tuple(X),tuple(Y),tuple(Z))
        # print "\nSupport ", Support

        # The contact already exists
        if hasattr(self.sot, '_'+contact.name+'_p'):
            supportPrev = self.sot.signal("_"+contact.name+"_p").value    # Old supporting points
            # print "Sup points old: ", supportPrev
            indexToKeep = []                                               # Indexes to keep their tasks
            Xnew=[]; Ynew=[]; Znew=[]              # New values for X,Y,Z (support)
            Xold=[]; Yold=[]; Zold=[]              # Old values for the support that will be kept

            # Compare the new points with the old ones to see if they are close enough
            for i in range(len( Support[0] )):
                keepNew = True
                for j in range(len( supportPrev[0] )):
                    # Get the L2 norm distance between the olds and current support points
                    dist = sqrt( (supportPrev[0][j]-Support[0][i])**2 + (supportPrev[1][j]-Support[1][i])**2 + 
                                 (supportPrev[2][j]-Support[2][i])**2 )
                    
                    # If the distance is less than a certain value, discard the new value, keep the old one, and 'continue'
                    #if (dist < 0.001):
                    if (dist < 0.05):
                        indexToKeep.append(j)       # Add the index for the pointTasks that will be kept
                        keepNew = False
                        continue

                if (keepNew == True):
                    Xnew.append(Support[0][i])
                    Ynew.append(Support[1][i])
                    Znew.append(Support[2][i])
                    
            # print "New Values: ", Xnew, ", ", Ynew, ", ", Znew
            # print "indexTokeep: ", indexToKeep

              # Go through the old support contacts (in decreasing order of the index)
            for j in reversed( range(len(supportPrev[0])) ):
                # If the contact has to be kept, append it to X,Y,Zold
                if j in indexToKeep: 
                    # Add the support contacts
                    Xold.append(supportPrev[0][j])  
                    Yold.append(supportPrev[1][j])
                    Zold.append(supportPrev[2][j])
                # If the contact has to be removed, remove the task associated with it
                else:
                    self.pointTasks[j].task.clear()
                    self.pointTasks.pop(j)
            
            # print "Old Values: ", Xold, ", ", Yold, ", ", Zold 

            # Verify that the name of the tasks (in order) correspond to their index. If so, leave them as they are, 
            # otherwise, delete them and create a new task (to avoid name's problems when adding new tasks)
            renameOldTasks = False
            for i in range(len(Xold)):
                name = 'task' + contact.name + 'supportTask' + str(i)
                if (self.pointTasks[i].task.name != name):
                    renameOldTasks = True
                    break
            if (renameOldTasks):
                for i in reversed( range(len(self.pointTasks)) ):
                    self.pointTasks[i].task.clear()
                    self.pointTasks.pop(i)
                
                for i in range(len(Xold)):
                    name = contact.name + 'supportTask' + str(i)
                    self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
                    self.pointTasks[i].gain.setConstant(3000)
                    self.pointTasks[i].opmodif = ((1,0,0,Xold[i]),(0,1,0,Yold[i]),(0,0,1,Zold[i]),(0,0,0,1)) 
                    self.pointTasks[i].feature.position.recompute(self.sot.solution.time)
                    p0 = self.pointTasks[i].feature.position.value
                    gotoNd( self.pointTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
                


            # Generate the new support composed of the old points and the new points
            NewSupport = (tuple(Xold+Xnew), tuple(Yold+Ynew), tuple(Zold+Znew))

            contact.support = NewSupport
            # print "New Support: ", NewSupport
            self.sot.signal("_"+contact.name+"_p").value = contact.support
            #contact.task.resetJacobianDerivative()

            Nold = len(Xold)
            taskVectors = []

            # print "Before adding Xnew tasks: ", self.pointTasks
            # Only for the new points
            for i in range(len(Xnew)):
                name = contact.name + 'supportTask' + str(Nold+i)
                self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
                self.pointTasks[i+Nold].gain.setConstant(3000)
                self.pointTasks[i+Nold].opmodif = ((1,0,0,Xnew[i]),(0,1,0,Ynew[i]),(0,0,1,Znew[i]),(0,0,0,1)) 
                self.pointTasks[i+Nold].feature.position.recompute(self.sot.solution.time)
                p0 = self.pointTasks[i+Nold].feature.position.value
                gotoNd( self.pointTasks[i+Nold], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
            # print "After adding Xnew tasks: ", self.pointTasks

            # Update the values of the tasks
            taskVectors = []
            for i in range(len(self.pointTasks)):
                self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
                taskVectors.append( self.pointTasks[i].task.taskVector.value, )
            
            # print "task Vectors: ", taskVectors
            self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)

        # Else (the contact does not exist), create the contact with the specified points
        else:
            contact.support = Support
            self.sot.addContactFromMetaTask(contact)
            self.pointTasks = []
            taskVectors = []
            for i in range(len(supPoints)):
                name = contact.name + 'supportTask' + str(i)
                self.pointTasks.append( MetaTaskDyn6d(name, dyn, name, opPointRef) )
                self.pointTasks[i].gain.setConstant(3000)
                self.pointTasks[i].opmodif = ((1,0,0,supPoints[i][0]),(0,1,0,supPoints[i][1]),(0,0,1,supPoints[i][2]),(0,0,0,1)) 
                self.pointTasks[i].feature.position.recompute(self.sot.solution.time)
                p0 = self.pointTasks[i].feature.position.value
                gotoNd( self.pointTasks[i], (p0[0][3], p0[1][3], p0[2][3], 0, 0, 0), "000111")  # Control only translation
                self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
                taskVectors.append( self.pointTasks[i].task.taskVector.value, )
        
            self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)

    
    def update(self, contact):
        taskVectors = []
        for i in range(len(self.pointTasks)):
            self.pointTasks[i].task.taskVector.recompute(self.sot.solution.time)
            taskVectors.append( self.pointTasks[i].task.taskVector.value, )
        self.sot.signal("_"+contact.name+"_ddx3").value = tuple(taskVectors)
