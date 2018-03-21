import sys
import os
import math

import numpy as np
#import numba as nb

from osgeo import gdal
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.uic import *

#The Shape class holds relevant information for loop of coordinates. 
#The coordinates are stored as lists, with each list containing an unbroken line moving either upwards 
#or downwards. The lists are read in alternating fashion, starting with the first down list, then the first up
#list, then the second down list, etc. There will always be an equal number of up and down lists.

#There is a variable (lastLine) to store the last set of coordinates added and sorted into the lists, and an
#array (segmentBuffer) to store coordinates that are about to be added, and an
#array (innerLoops) of sub loops representing holes in the shape.
class Shape():
        def __init__(self, d, u):
            self.down = []
            self.up = []
            
            self.down.append(d)
            self.up.append(u)
            
            #Divided into: innerLoops[ loop[ downLists[ list [ segment[ leftCoord[x,y], rightCoord[x,y] ]+ ]+ ], upLists[ list [ segment[ leftCoord[x,y], rightCoord[x,y] ]+ ]+ ] ]+ ]
            self.innerLoops = []
            
            #The last line is divided into interleaving left and right (up and down) coordinates of each segment, without actually putting brackets around each segment
            self.lastLine = []
            
            #Divided into: segmentBuffer[ segList[ segment[ leftCoord[x,y], rightCoord[x,y] ]+ ]+ ]
            self.segmentBuffer = [0 for x in range(len(self.lastLine)/2)]
        
        #Merge the given shape with this shape. Merge by wrapping the edge pairs of this
        #shape around the mergee's edge pairs in a way depending on the index of the connected segments 
        #in the last line.
        #Then merge the segment buffers by appending the tail of the mergee's buffer to this shape's buffer.
        def merge(self, s):
            #Get index of edge lists to insert merger's edge lists into
            insIndex = self.findEdgePair(self.lastLine[-2][0])
            
            #Get index of merger's edge lists to split and rearrange around
            splIndex = s.findEdgePair(s.lastLine[0][0])

            #Insert merger's down edges from split point into this shape's down edges
            self.down[insIndex+1:insIndex+1] = s.down[splIndex+1:]
            
            #Insert merger's up edges from split point into this shape's down edges
            self.up[insIndex:insIndex] = s.up[splIndex:]
            
            insIndex += len(s.down[splIndex+1:])
            
            #Insert remaining merger's down edges into this shapes down edges from point after edges prviously added
            self.down[insIndex+1:insIndex+1] = s.down[:splIndex+1]
            
            #Insert remaining merger's down edges into this shapes down edges from point after edges prviously added
            self.up[insIndex+1:insIndex+1] = s.up[:splIndex]
           
            #Add elements from merging segment buffer to this segment buffer
            for elem in s.segmentBuffer[1:]:
                self.segmentBuffer.append(elem)
                
            #Append merging shape's inner loops list to this shape's inner loops list
            for loop in s.innerLoops:
                self.innerLoops.append(loop)
        
        #Put segments in buffer to be added to up/down lists later. This is so we can rearrange edge lists
        #due to loop extraction and merging before adding new lines. For each segment in the last
        #line, we check if the passed segment is touching it. If it is, we add the segment  to its corresponding 
        #index in a buffer with the same number of elements as segments in the last line.
        def bufferSegment(self, segment):
            #Check if segment is connected to segment in last line
            segDown = segment[0][0] #Leftmost x value of segment
            segUp = segment[1][0] #Rightmost x value of segment
            
            leftIdx = 0
            #Find last segment left coordinate is connected to and add to correpsonding index in segment buffer
            for i, lastSegRight in enumerate(self.lastLine[1::2]):
                #Left edge of segment is connected here if left edge position is less than last right edge position
                if segDown < lastSegRight[0] and segUp > self.lastLine[i * 2][0]:
                    #Add to segment buffer, start new array if no other segments connected to this last segment
                    if self.segmentBuffer[i] == 0:
                        self.segmentBuffer[i] = [[segment[0], 0]]
                    else:
                        self.segmentBuffer[i].append([segment[0], 0])
                    leftIdx = i * 2
                    break
            else:
                return False
            #Find last segment right coordinate is connected to and  add to correpsonding index in segment buffer
            for i, lastSegLeft in enumerate(self.lastLine[-2::-2]):
                j = len(self.lastLine)/2 - 1 - i
                #Right edge of segment is connected here if right edge position is greater than last left edge position
                if segUp > lastSegLeft[0]:
                    #Add to segment buffer, start new array if no other segments connected to this last segment
                    if self.segmentBuffer[j] == 0:
                        self.segmentBuffer[j] = [[0, segment[1]]]
                        
                        #If right edge of this segment connected to different segment than left edge, we have a loop that needs to be extracted
                        leftPair = self.findEdgePair(self.lastLine[leftIdx][0])
                        rightPair = self.findEdgePair(self.lastLine[j * 2][0])
                        
                        innerEdges = [self.down[(leftPair + 1):(rightPair + 1)], self.up[leftPair:rightPair]]
                        
                        del self.down[(leftPair + 1):(rightPair + 1)]
                        del self.up[leftPair:rightPair]
                        
                        #Also need to reform segmentBuffer
                        del self.segmentBuffer[leftIdx/2 + 1:j + 1]
                        self.segmentBuffer[leftIdx/2][-1][1] = segment[1]
                        
                        #Build loops from extracted inner edges and add to innerLoops
                        u = 0
                        d = 1
                        for edge in innerEdges[0][1:-1]:
                            if edge[-1] in self.lastLine[::2]:
                                 self.innerLoops.append([innerEdges[0][u:d + 1], innerEdges[1][u:d + 1]])
                                 u = d + 1
                            d += 1
                        self.innerLoops.append([innerEdges[0][u:], innerEdges[1][u:]])
                        
                        #Also need to reform lastLine
                        self.lastLine[leftIdx + 1] = self.lastLine[j*2+ 1]
                        del self.lastLine[leftIdx + 2:j*2 + 2]
                    else:
                        self.segmentBuffer[j][-1][1] = segment[1]
                    return True
            
            return False
        
        #Find index of edge pair given x value of left coord
        def findEdgePair(self, leftX):
            for i, edge in enumerate(self.down):
                if leftX == edge[-1][0]:
                    return i
        
        #Add segments in segment buffer into edge pairs. If more than one segment is connected 
        #to a segment added in the last line, start a new pair of edge lists
        def addLines(self):
            if all(x == 0 for x in self.segmentBuffer):
                return
            #Each index in the buffer corresponds to a possible previous segment that new segments can be attached to
            for i, segList in enumerate(self.segmentBuffer):
                #Indicies that equal 0 mean there were no segments attached here
                if segList == 0:
                    continue
                #Get index of edge pair to recieve segments in list
                idx = self.findEdgePair(self.lastLine[i*2][0])
                
                #Append left side of first segment to left edge and right side of last segment to right edge
                #If there's only one segment the first and last segments are the same segment
                self.down[idx].append(segList[0][0])
                self.up[idx].append(segList[-1][1])
                
                #If there's more than one segment, we need to start new edge lists with the unused sides of the 
                #first and last segments, then add any more segments in between
                if len(segList) > 1:
                    self.up.insert(idx, [ segList[0][1] ])
                    self.down.insert(idx + 1, [ segList[-1][0] ])
                    
                    for seg in segList[1:-1]:
                       self.down.insert(idx + 1, [ seg[0] ])
                       self.up.insert(idx + 1, [ seg[1] ])
                
                #Finally repopulate last line with new segments and reset segment buffer
                self.lastLine = []
                for seg in segList:
                    self.lastLine.append(seg[0])
                    self.lastLine.append(seg[1])
                self.resetBuffer()
                
        #Reset segment buffer by initialzing zeroes equal to half the length of the last line. Each index represents a segment in the last line
        def resetBuffer(self):
            self.segmentBuffer = [0 for x in range(len(self.lastLine)/2)]


class GPURasterConverter(QDialog):
    
    def __init__(self):
        super(GPURasterConverter, self).__init__()
        
        self.initUI()
        
    #Initialize UI
    def initUI(self):      
        loadUi(os.path.join(sys.path[0], 'BUCK_raster_converter_dialog.ui'), self)
        
        #Connect event listeners to functions
        self.openPathButton.clicked.connect(self.showOpenDialog)
        self.savePathButton.clicked.connect(self.showSaveDialog)
        self.tolSlider.valueChanged.connect(self.showTolerance)
        self.button_box.accepted.connect(self.execute)
        
        #Render
        self.show()
        
    #Prompt user to get path for raster file to open
    def showOpenDialog(self):
        fname = QFileDialog.getOpenFileName(self, 'Open file', '/home')
        self.inputEdit.setText(fname[0])
    
    #Prompt user to set path for output file
    def showSaveDialog(self):
        fname = QFileDialog.getSaveFileName(self, "Select ouput file", "", "*.shp")
        self.outputEdit.setText(fname[0])
        
    #Display tolerance number set by moving slider
    def showTolerance(self):
        self.valueLabel.setText(str(self.tolSlider.value()))
    
    #Read file and sort coordinates into loops to make polygonal shapefile
    def execute(self):
        #Get raw array of data from raster file
        print("Reading...")
        raster = gdal.Open(self.inputEdit.text())
        ra2D = raster.ReadAsArray() #2D array
        print("Done.")
        
        #Read metadata to help find coordinates of each pixel
        file = open(self.inputEdit.text())
        
        cols = float(file.readline().split()[1])
        rows = float(file.readline().split()[1])
        x = float(file.readline().split()[1]) #Gives x coord of leftmost row
        yl = float(file.readline().split()[1]) #Gives y coord of bottom row
        cellSize = float(file.readline().split()[1])
        noData = float(file.readline().split()[1])
        
        #Convert y coord to top row so can now use top left corner as origin
        y = yl + (cellSize * rows)
        
        print("cols: {}rows: {}x: {}y: {}cell size: {}no data value: {}").format(str(cols) + '\n', str(rows) + '\n', str(x) + '\n', str(y) + '\n', str(cellSize) + '\n', str(noData) + '\n')
        
        file.close()
        
        #Put pixel values into groups based on user defined tolerance
        groups = int(100/self.tolSlider.value())
        print("Groups (based on tolerance): " + str(groups))
        max = np.amax(ra2D)
        print("Max value: " + str(max))
        
        rasterArray = (ra2D/max) * groups
        noData = (noData/max) * groups
        print("New no_data value: " + str(int(noData)) + "\n")
        
#        print ("Buffer testing...")
#        self.bufferTest()
#        print("\nMerge testing...")
#        self.mergeTest()
#        print("\nAdding lines testing...")
#        self.addTest()
        
        #Find coordinates for edge pixels on each line of raster array
        print("Scanning for edges...")
        scanGroups = [0 for x in range(groups)]
        self.scanLines(rasterArray, noData, scanGroups, cellSize, x, y)
        print("Finished scanning.\n")
        
        
        #Build shapes from scanned lines
        print("Beginning edge connection...")
        shapeGroups = [0 for x in range(groups)]
        
        for i, group in enumerate(scanGroups):
            print("Connecting value " + str(i + 1) + " shapes...")
            
            for scanLine in group:
                if shapeGroups[i] == 0:
                    #New to python - couldn't fix contstructor in time
                    shape = Shape([], [])
                    shape.down = [ [ scanLine[0][0] ] ]
                    shape.up = [ [ scanLine[0][1] ] ]
                    shape.lastLine = [ scanLine[0][0], scanLine[0][1] ]
                    shape.resetBuffer()
                    shapeGroups[i] = [shape]
                    
                    for segment in scanLine[1:]:
                        shape = Shape([], [])
                        shape.down = [ [ segment[0] ] ]
                        shape.up = [ [ segment[1] ] ]
                        shape.lastLine = [ segment[0], segment[1] ]
                        shape.resetBuffer()
                        shapeGroups[i].append(shape)
                 
                else:
                    m = []
                    for segment in scanLine:
                        newShape = True
                        
                        for shape in shapeGroups[i]:
                            if shape.bufferSegment(segment):
                                m.append(shape)
                                
                                if len(m) > 1:
                                    shapeGroups[i].remove(shape)
                                    m[0].merge(shape)
                                    m = [m[0]]
                                newShape = False
                         
                        if newShape:
                            idx = self.findShapeIndex(shapeGroups[i], segment)
                            shape = Shape([], [])
                            shape.down = [ [ segment[0] ] ]
                            shape.up = [ [ segment[1] ] ]
                            shape.lastLine = [ segment[0], segment[1] ]
                            shape.resetBuffer()
                            shapeGroups[i].insert(idx, Shape(segment[0], segment[1]))
                        
                        
                    for shape in shapeGroups[i]:
                        shape.addLines()
            print("Created " + str(len(shapeGroups[i])) + " shapes")
    
    def findShapeIndex(self, shapeList, segment):
        for i, shape in enumerate(shapeList):
            if all(x == 0 for x in shape.segmentBuffer):
                continue
            else:
                if segment[0][0] > shape.lastLine[-1][0]:
                    return i
        return len(shapeList) - 1
        
    #Scan rows of raster array and sort edge coordinates into groups depending on tolerance
    def scanLines(self, rasterArray, noData, scanGroups, cellSize, originX, originY):
        reading = False #Switch to help detmermine what end of the segment we need to look out for
        end = len(rasterArray[0]) #Used to help close off segments that are still reading when they get to end of line
        
        for group in range(len(scanGroups)):
            print("Scanning value " + str(group + 1) + "...")
            size = 0
            for i, row in enumerate(rasterArray):
                scanLine = []
                segment = [0, 0]
                
                for j, col in enumerate(row): 
                    if int(col) == int(noData) and reading == False:
                        continue
                    elif math.ceil(col) == group + 1 and reading == False:
                        reading = True
                        segment[0] = [originX + (cellSize * j) - (cellSize/2), originY - (cellSize * i)]
                    elif (math.ceil(col) < group + 1 or i == end - 1) and reading == True:
                        reading = False
                        segment[1] = [originX + (cellSize * j) - (cellSize/2), originY - (cellSize * i)]
                        scanLine.append(segment)
                        
                        size += 1
                        segment = [0, 0]
                    
                if scanGroups[group] == 0:
                    scanGroups[group] = scanLine
                else:
                    if scanLine != []:
                        scanGroups[group].append(scanLine)
            print("{} segments, {} lines found").format(size, len(scanGroups[group]))
            
    #=======================TESTING=======================#
    
    #Buffer some segments to a shape
    def bufferTest(self):
        down = [ [ [0, 0], [0, -1], [0, -2] ], [ [8, -1], [8, -2] ] ]
        up = [ [ [3, -1], [3, -2] ], [ [14, 0], [14, -1], [14, -2] ] ]
        
        shape = Shape([], [])
        shape.down = down
        shape.up = up
        shape.lastLine = [ [0, -2], [3, -2], [8, -2], [14, -2] ]
        shape.resetBuffer()
        
        passedBuffer = [0, 0]
        if not self.checkBuffer(passedBuffer, shape, "Buffer creation test"):
            return
        
        segment1 = [ [2, -3], [7, -3] ]
        segment2 = [ [9, -3], [10, -3] ]
        segment3 = [ [13, -3], [15, -3] ]
        
        
        shape.bufferSegment(segment1)
        
        passedBuffer = [ [ [ [2, -3], [7, - 3] ] ], 0 ]
        if not self.checkBuffer(passedBuffer, shape, "Simple single segment buffer test"):
            return 
        
        shape.bufferSegment(segment2)
        shape.bufferSegment(segment3)
        
        passedBuffer = [ [ [ [2, -3], [7, - 3] ] ], [ [ [9, -3], [10, -3] ], [ [13, -3], [15, -3] ] ] ]
        if not self.checkBuffer(passedBuffer, shape, "Simple double segment buffer test"):
            return
         
        segment4 = [ [18, -3], [19, -3] ]
        segment5 = [ [-3, -3], [-1, -3] ]
        shape.bufferSegment(segment4)
        shape.bufferSegment(segment5)
        if not self.checkBuffer(passedBuffer, shape, "Buffer ignore test"):
            return
            
        segment = [ [2, -3], [12, -3] ]
        shape.resetBuffer()
        
        shape.bufferSegment(segment)
        passedDown = [ [ [0, 0], [0, -1], [0, -2] ] ]
        passedUp = [ [ [14, 0], [14, -1], [14, -2] ] ]
        passedLoops = [ [ [ [ [8, -1], [8, -2] ] ], [ [ [3, -1], [3, -2] ] ] ] ]
        passedBuffer = [ [ [ [2, -3], [12, -3] ] ] ]
        if not self.checkLoopRemoval(passedDown, passedUp, passedLoops, shape, "Loop removal test"):
            return
        if not self.checkBuffer(passedBuffer, shape, "Segment causing loop removal test"):
            return
            
        down = [ [ [0, 0], [0, -1], [0, -2], [0, -3] ], [ [7, -1], [7, -2] ], [ [25, -1], [25, -2], [25, -3] ], [ [35, -1], [35, -2], [35, -3] ] ]
        up = [ [ [3, -1], [3, -2], [3, -3] ], [ [12, -1], [12, -2] ], [ [30, -1], [30, -2], [30, -3] ], [ [40, 0], [40, -1], [40, -2], [-40, -3] ] ]
        
        shape = Shape([], [])
        shape.down = down
        shape.up = up
        shape.lastLine = [ [0, -3], [3, -3], [25, -3], [30, -3], [35, -3], [40, -3] ]
        shape.resetBuffer()
        
        segment = [ [2, -4], [37, -4] ]
        
        shape.bufferSegment(segment)
        
        passedDown = [ [ [0, 0], [0, -1], [0, -2], [0, -3] ] ]
        passedUp = [ [ [40, 0], [40, -1], [40, -2], [-40, -3] ] ] 
        passedLoops = [ [ [ [ [7, -1], [7, -2] ], [ [25, -1], [25, -2], [25, -3] ] ], [ [ [3, -1], [3, -2], [3, -3] ], [ [12, -1], [12, -2] ] ] ],  [ [ [ [35, -1], [35, -2], [35, -3] ] ],  [ [ [30, -1], [30, -2], [30, -3] ] ] ] ]
        passedBuffer = [ [ [ [ 2, -4], [37, -4] ] ] ]
        if not self.checkLoopRemoval(passedDown, passedUp, passedLoops, shape, "Multiple loop removal test"):
            return
        if not self.checkBuffer(passedBuffer, shape, "Segment causing multiple loop removal test"):
            return
        

    
    #Merge two different shapes with a segment between them
    def mergeTest(self):
        down = [ [0, 0], [0, -1], [0, -2] ]
        up = [ [3, 0], [3, -1], [3, -2] ]
        
        shape1 = Shape(down, up)
        shape1.lastLine = [ [0, -2], [3, -2] ]
        shape1.resetBuffer()
        
        down = [ [6, 0], [6, -1], [6, -2] ]
        up = [ [9, 0], [9, -1], [9, -2] ]
        
        shape2 = Shape(down, up)
        shape2.lastLine = [ [6, -2], [9, -2] ]
        shape2.resetBuffer()
        
        segment = [ [2, -3], [7, -3] ]
        
        shape1.bufferSegment(segment)
        shape2.bufferSegment(segment)
        
        shape1.merge(shape2)
        
        passedMergeDown = [ [ [0, 0], [0, -1], [0, -2] ], [ [6, 0], [6, -1], [6, -2] ] ]
        passedMergeUp = [ [ [9, 0], [9, -1], [9, -2] ], [ [3, 0], [3, -1], [3, -2] ] ]
        passedBuffer = [ [ [ [2, -3], [7, -3] ] ] ]
        if not self.checkMerge([], passedMergeDown, passedMergeUp, passedBuffer, shape1, "Simple merge test"):
            return
            
        down = [ [ [0, 0], [0, -1], [0, -2] ], [ [10, -1], [10, -2], [10, -3] ], [ [17, -1], [17, -2] ], [ [35, -1], [35, -2], [35, -3] ], [ [45, -1], [45, -2], [45, -3] ], [ [53, -1], [53, -2] ] ]
        up = [ [ [3, 0], [3, -1], [3, -2] ], [ [13, -1], [13, -2], [13, -3] ], [ [22, -1], [22, -2] ], [ [40, -1], [40, -2], [40, -3] ], [ [50, -1], [50, -2], [-50, -3] ], [ [57, 0], [57, -1], [57, -2] ] ]
        
        shape1 = Shape([], [])
        shape1.down = down
        shape1.up = up
        shape1.lastLine = [ [10, -3], [13, -3], [35, -3], [40, -3], [45, -3], [50, -3] ]
        shape1.resetBuffer()
        
        down = [ [ [60, 0], [60, -1], [60, -2] ], [ [70, 0], [70, -1], [70, -2], [70, -3] ], [ [77, -1], [77, -2] ], [ [95, -1], [95, -2], [95, -3] ], [ [105, -1], [105, -2], [105, -3] ], [ [113, -1], [113, -2] ] ]
        up = [ [ [63, 0], [63, -1], [63, -2] ], [ [73, -1], [73, -2], [73, -3] ], [ [82, -1], [82, -2] ], [ [100, -1], [100, -2], [100, -3] ], [ [110, -1], [110, -2], [-110, -3] ], [ [117, 0], [117, -1], [117, -2] ] ]
        
        shape2 = Shape([], [])
        shape2.down = down
        shape2.up = up
        shape2.lastLine = [ [70, -3], [73, -3], [95, -3], [100, -3], [105, -3], [110, -3] ]
        shape2.resetBuffer()
        
        segment = [ [37, -4], [97, -4] ]
        
        shape1.bufferSegment(segment)
        shape2.bufferSegment(segment)
        
        shape1.merge(shape2)
        
        passedMergeLoops = [ [ [ [ [45, -1], [45, -2], [45, -3] ] ], [ [ [40, -1], [40, -2], [40, -3] ] ] ], [ [ [ [77, -1], [77, -2] ], [ [95, -1], [95, -2], [95, -3] ] ], [ [ [73, -1], [73, -2], [73, -3] ], [ [82, -1], [82, -2] ] ] ] ]
        passedMergeDown = [ [ [0, 0], [0, -1], [0, -2] ], [ [10, -1], [10, -2], [10, -3] ], [ [17, -1], [17, -2] ], [ [35, -1], [35, -2], [35, -3] ], [ [105, -1], [105, -2], [105, -3] ], [ [113, -1], [113, -2] ], [ [60, 0], [60, -1], [60, -2] ], [ [70, 0], [70, -1], [70, -2], [70, -3] ], [ [53, -1], [53, -2] ] ]
        passedMergeUp = [ [ [3, 0], [3, -1], [3, -2] ], [ [13, -1], [13, -2], [13, -3] ], [ [22, -1], [22, -2] ], [ [100, -1], [100, -2], [100, -3] ], [ [110, -1], [110, -2], [-110, -3] ], [ [117, 0], [117, -1], [117, -2] ], [ [63, 0], [63, -1], [63, -2] ], [ [50, -1], [50, -2], [-50, -3] ], [ [57, 0], [57, -1], [57, -2] ] ]
        passedBuffer = [0, [ [ [37, -4], [97, -4] ] ], 0]
        if not self.checkMerge(passedMergeLoops, passedMergeDown, passedMergeUp, passedBuffer, shape1, "Complex merge test"):
            return
    
    #Add segments to a shape using its segment buffer
    def addTest(self):
        down = [ [5, 0], [5, -1], [5, -2] ]
        up = [ [8, 0], [8, -1], [8, -2] ]
        
        shape = Shape(down, up)
        shape.lastLine = [ [5, -2], [8, -2] ]
        shape.resetBuffer()
        
        segment = [ [2, -3], [10, -3] ]
        
        shape.bufferSegment(segment)
        shape.addLines()
        
        passedDown = [ [ [5, 0], [5, -1], [5, -2], [2, -3] ] ]
        passedUp = [ [ [8, 0], [8, -1], [8, -2], [10, -3] ] ]
        if not self.checkAdd(passedDown, passedUp, shape, "Single segment add test"):
            return
            
        segment1 = [ [3, -4], [6, -4] ]
        segment2 = [ [8, -4], [14, -4] ]
        
        shape.bufferSegment(segment1)
        shape.bufferSegment(segment2)
        shape.addLines()
        
        passedDown = [ [ [5, 0], [5, -1], [5, -2], [2, -3], [3, -4] ], [ [8, -4] ] ]
        passedUp = [ [ [6, -4] ], [ [8, 0], [8, -1], [8, -2], [10, -3], [14, -4] ] ]
        if not self.checkAdd(passedDown, passedUp, shape, "Double segment add test"):
            return
            
        segment1 = [ [1, -5], [4, -5] ]
        segment2 = [ [5, -5], [9, -5] ]
        segment3 = [ [10, -5], [16, -5] ]
        
        shape.bufferSegment(segment1)
        shape.bufferSegment(segment2)
        shape.bufferSegment(segment3)
        shape.addLines()
        
        passedDown = [ [ [5, 0], [5, -1], [5, -2], [2, -3], [3, -4], [1, -5] ], [ [5, -5] ], [ [10, -5] ] ]
        passedUp = [ [ [4, -5] ], [ [9, -5] ], [ [8, 0], [8, -1], [8, -2], [10, -3], [14, -4], [16, -5] ] ]
        if not self.checkAdd(passedDown, passedUp, shape, "Triple segment add test"):
            return
        
    #Used for buffer insertion scenarios. Check shape buffer against passed buffer then print and return results
    def checkBuffer(self, passedBuffer, shape, testString):
        if passedBuffer == shape.segmentBuffer:
            print(testString + " passed")
            return True
        else:
            print(testString + " failed")
            print("Correct buffer: {}\nShape buffer: {}\n").format(passedBuffer, shape.segmentBuffer)
        
        return False
    
    #Used for loop extraction scenarios. Check reordered edge lists and inner loop list against passed variables then print and return results
    def checkLoopRemoval(self, passedDown, passedUp, passedLoops, shape, testString):
        if passedDown == shape.down and passedUp == shape.up and passedLoops == shape.innerLoops:
            print (testString + " passed")
            return True
        else:
            print(testString + " failed")
            print("Correct inner loop: {}\nInner loop list: {}\n").format(passedLoops, shape.innerLoops)
            print("Correct down edge list: {}\nDown edge list: {}\n").format(passedDown, shape.down)
            print("Correct up edge list: {}\nUp edge list: {}\n").format(passedUp, shape.up)
        
        return False
    
    #Used for shape merging scenarios. Check edglists, segment buffer and inner loops against passed variables then print and return results
    def checkMerge(self, passedMergeLoops, passedMergeDown, passedMergeUp, passedBuffer, shape, testString):
        if shape.down == passedMergeDown and shape.up == passedMergeUp and shape.segmentBuffer == passedBuffer:
            print(testString + " passed")
            return True
        else:
            print(testString + " failed")
            print("Correct inner loop list: {}\nInner loop list: {}\n").format(passedMergeLoops, shape.innerLoops)
            print("Correct down list: {}\nDown list: {}\n").format(passedMergeDown, shape.down)
            print("Correct up list: {}\nUp list: {}\n").format(passedMergeUp, shape.up)
            print("Correct buffer: {}\nShape buffer: {}\n").format(passedBuffer, shape.segmentBuffer)
        
        return False
        
    #Used for adding segment scenarios. Check edglists against passed edgelists then print and return results
    def checkAdd(self, passedDown, passedUp, shape, testString):
        if passedDown == shape.down and passedUp == shape.up:
            print (testString + " passed")
            return True
        else:
            print(testString + " failed")
            print("Correct down edge list: {}\nDown edge list: {}\n").format(passedDown, shape.down)
            print("Correct up edge list: {}\nUp edge list: {}\n").format(passedUp, shape.up)
        
        return False
    
    #=======================END TESTING=======================#
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    GPURC = GPURasterConverter()
    sys.exit(app.exec_())