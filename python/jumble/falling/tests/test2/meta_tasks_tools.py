from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd, AddContactHelper


class FeetFollowCapturePoint(object):
    def __init__(self, taskFoot, contactFoot, taskCapturePoint, sot):
        self.taskF   = taskFoot
        self.contact = contactFoot
        self.taskCP  = taskCapturePoint
        self.sot     = sot
        self.start = True
        self.addContact = AddContactHelper(self.sot)

        ''' These borders must be set in the main program for this 
        classs to work well '''
        self.borders = None

        ''' Added to control when the foot goes up '''
        self.footUP = True

    def update(self):
        if( self.taskCP.CPOutsideLimits ):
            if( self.start ):
                ''' Starting: remove contact and add task '''
                self.sot.rmContact(self.contact.name)
                self.sot.push(self.taskF.task.name)
                print 'Contact', self.contact.name, ' removed'
                print 'Add task foot follow CP'
                self.foot0 = self.taskF.feature.position.value
                self.start = False
                
                # ''' Store the desired gain and lower the gain to move upwards '''
                # self.gain = taskF.gain.gain.value
                

            ''' Get the current borders of the foot wrt world '''
            borders_ = self.toWorldFrame(self.borders, self.taskF.feature.position.value)
            self.xmax = max([p[0] for p in borders_])
            self.xmin = min([p[0] for p in borders_])
            self.ymax = max([p[1] for p in borders_])
            self.ymin = min([p[1] for p in borders_])

            # if( self.footUP ):
            #     ''' Goes up to an intermediate position between the current capture point
            #     and the (very) initial position '''
            #     gotoNd(self.taskF, ((self.taskCP.task.capturePoint.value[0]+self.foot0[0][3])/2,
            #                         (self.taskCP.task.capturePoint.value[1]+self.foot0[1][3])/2,
            #                         0.105+0.025,0,0,0), "111111")
            #     #print 'UP'

            #     ''' Check the error between the task and the CP in x,y '''
            #     self.taskF.feature.error.recompute(self.sot.solution.time)
            #     if(((self.taskCP.task.capturePoint.value[0]+self.foot0[0][3])/2 < self.xmax) and
            #        ((self.taskCP.task.capturePoint.value[0]+self.foot0[0][3])/2 > self.xmin) and
            #        ((self.taskCP.task.capturePoint.value[1]+self.foot0[1][3])/2 < self.ymax) and
            #        ((self.taskCP.task.capturePoint.value[1]+self.foot0[1][3])/2 > self.ymin) and
            #        (abs(self.taskF.feature.error.value[2]) < 0.005)):
            #         self.footUP = False

            # if( self.footUP ):
            #     ''' Raises the foot up keeping the same (x,y) place '''
            #     gotoNd(self.taskF, (self.foot0[0][3], self.foot0[1][3],0.105+0.025,0,0,0), "111111")
            #     #print 'UP'

            #     ''' Check the error between the task and the CP in (x,y) '''
            #     self.taskF.feature.error.recompute(self.sot.solution.time)
            #     if(abs(self.taskF.feature.error.value[2]) < 0.005):
            #         self.footUP = False

            if( self.footUP ):
                ''' Raises the foot up moving it to the current x position of the CP '''
                gotoNd(self.taskF, (self.taskCP.task.capturePoint.value[0],
                                    self.foot0[1][3],0.105+0.025,0,0,0), "111111")
                #print 'UP'

                ''' Check the error between the task and the CP in (x,y) '''
                self.taskF.feature.error.recompute(self.sot.solution.time)
                if((abs(self.taskF.feature.error.value[2]) < 0.005) and
                   (abs(self.taskF.feature.error.value[0]) < 0.015)):
                    self.footUP = False


            else:
                gotoNd(self.taskF, (self.taskCP.task.capturePoint.value[0],
                                    self.taskCP.task.capturePoint.value[1],
                                    0.105,0,0,0), "111111")

                ''' Check the error between the task and the CP in x,y '''
                self.taskF.feature.error.recompute(self.sot.solution.time)
                if((self.taskCP.task.capturePoint.value[0] < self.xmax) and
                   (self.taskCP.task.capturePoint.value[0] > self.xmin) and
                   (self.taskCP.task.capturePoint.value[1] < self.ymax) and
                   (self.taskCP.task.capturePoint.value[1] > self.ymin) and
                   (abs(self.taskF.feature.error.value[2]) < 0.001)):
                    self.sot.rm(self.taskF.task.name)
                    self.addContact(self.contact)
                    print 'Remove task foot'
                    print 'Contact', self.contact.name, ' added'
                    self.start = True
                    self.footUP = True


    # def update(self):
    #     if( self.taskCP.CPOutsideLimits ):
    #         if( self.start ):
    #             ''' Starting: remove contact and add task '''
    #             self.sot.rmContact(self.contact.name)
    #             self.sot.push(self.taskF.task.name)
    #             self.start = False

    #         gotoNd(self.taskF, (self.taskCP.task.capturePoint.value[0],
    #                             self.taskCP.task.capturePoint.value[1],
    #                             0.105,0,0,0), "111111")

    #         ''' Check the error between the task and the CP in x,y '''
    #         borders_ = self.toWorldFrame(self.borders, self.taskF.feature.position.value)
    #         self.xmax = max([p[0] for p in borders_])
    #         self.xmin = min([p[0] for p in borders_])
    #         self.ymax = max([p[1] for p in borders_])
    #         self.ymin = min([p[1] for p in borders_])

    #         # error = self.taskCP.feature.error.value[0:2]
    #         # if( ((error[0]**2)+(error[1]**2)) < 0.001):
    #         if((self.taskCP.task.capturePoint.value[0] < self.xmax) and
    #            (self.taskCP.task.capturePoint.value[0] > self.xmin) and
    #            (self.taskCP.task.capturePoint.value[1] < self.ymax) and
    #            (self.taskCP.task.capturePoint.value[1] > self.ymin) ):
    #             self.sot.rm(self.taskF.task.name)
    #             self.addContact(self.contact)
    #             self.start = True
                
    def toWorldFrame(self, P, M):
        '''
        Transforms the (x,y) elements of the points in P according to the homogeneous
        transformation M
        -    Input
        -       P: ((x1,x2,x3,x4),(y1,y2,y3,y4),(z1,z2,z3,z4))
        -       M: Tuple containing the homogeneous transformation matrix
        -    Ouput
        -       res: ((x1,y1),(x2,y2),(x3,y3),(x4,y4))
        '''
        res = ()
        for i in range(len(P[0])):
            x = M[0][0]*P[0][i]+M[0][1]*P[1][i]+M[0][2]*P[2][i]+M[0][3]
            y = M[1][0]*P[0][i]+M[1][1]*P[1][i]+M[1][2]*P[2][i]+M[1][3]
            res += ((x,y),)
        return res
