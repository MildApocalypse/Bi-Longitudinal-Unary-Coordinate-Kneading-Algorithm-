# Bi-Longitudinal-Unary-Coordinate-Kneading-Algorithm- (B.U.C.K Algorithm)
This algorithm is an experimental exploration into the parallelization of the creation of coordinate loops to be written into 
shapefiles. This prototype was written with python using Anaconda 2 on Windows 10. At the time of this readme I am still new to python and this was my first real project using it, so please excuse all the non - pythonic coding.

The inspiration of this algorithm comes from CPU - based 3D rendering techniques called scanline rendering. This algorithm works by reading each row of the raster and creating edglists from the coordinates obtained by picking out the starts and ends of rows of adjacent pixels (segments). These edglists are built sequentially from checking each row of segments against the previous row and inserting into the growing and changing pairs of edgelists through logic taking various cases into account. Each shape is made up of a set of bi-longitudinal edge lists, i.e. an equal number of lists that read sequentially downwards and upwards. An alternate name for this algorithm is the Reverse Scanline algorithm.

The algorithm has four steps and one preliminary step:

- Step 0: Scan the image
  - Scan the raster grid row by row and collect arrays of segments. Segments are strings of laterally adjacent pixels of the same value.   Group these segments into different tolerances depending on the number of polygon values desired in the final shapefile.
This is the only parallelizable step. i.e: one thread per row, one block (group of threads) per tolerance group.


- Step 1: Buffer segments\Create new shapes
  - Iterate through each segment on each row. If this segment is touching another segment on the row above it, it needs to be added to the shape the above segment is a part of. Add this segment to this shapes segment buffer. If this segment is not touching any segment on the row above it, a new shape can be created.

- Step 2: Remove inner loops
  - If this segment is touching two or more segments on the row above it that belong to the same shape, an inner loop is created. Extract the loop from the edgelists and put in a separate list within the shape object.

- Step 3: Merge shapes
  - If this segment is touching two or more segments on the row above it that belong to different shapes, the shapes need to be merged. Merge each shape needing to be merged one at a time from right to left, merging each shape with the shape to its left until only one shape is left. Due to the loop removal of the previous step, there should be only one segment being touched by the buffered segment per shape. 

- Step 4: Add segments
  - Once all loops and shape merges caused by this row of segments have been resolved, lines can be transferred from the segment buffer into the edge lists. In the cases of multiple segments touching the same previous segment in the row above, new edglists must be created for that shape.

This algorithm is a prototype and as such does not actually include the aforementioned parallelization and shapefile creation. It can currently only read and construct shape objects composed of edge lists from ASCII Grid files.
