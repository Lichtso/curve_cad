# Features
[Overview Video](https://github.com/Lichtso/curve_cad/blob/gallery/trailer.mp4?raw=true)



## Context Menu > Bezier CAD

### Fillet
Select spline points to round them by inserting arcs. Only works on corners between straight lines with handle type "vector".

### Boolean
Select exactly two bezier splines to combine their area into a new one by union, intersection or difference.

### Intersection
Select at least two segments to split them at their intersections.

### Merge Ends
Merges two end control points into one while preserving the relative positions of their handles.

### Subdivide
Blender only allows for uniform subdivision.
This operator enables you to select multiple segments and split them at custom parameters simultaneously.

### Array
Blenders array modifier only works on the resulting mesh not on the curves themselves.
This operator enables you to select splines in edit mode to duplicate, shift and connect them.

### Circle
Add a bezier circle which completes the selected segment into a full circle.

### Length
Measures the arc length of the selected bezier segments.



## Add > Toolpath

### Offset Curve
Creates a toolpath with a defined offset to the selected splines.

### Slice Mesh
Creates a toolpath with a defined offset to the selected mesh.

### Truncate
Add an "empty" of type "cube" and transform it to your wishes.
Select some curve objects and the empty cube last so that it becomes the active object.
Now, apply the truncate operator to cutoff all the polygon splines inside the curves at the empty cubes boundaries.
Optionally, movements to the top of the volume (Z-hops) can be added.

### Rect Macro
Creates a toolpath for a rectangular area.

### Drill Macro
Creates a toolpath for a circular hole.


## File > Export

### SVG (for Laser Cutting)
- Fills and Colors
- Selection Only / All Visible
- Viewport Projection / Local Orthographic Projection
- Absolute / Relative Coords
- SVG Path: Move To, Line To, Curve To, Close
- Units

### G-Code (for CNC Milling)
Exports the active curve.
It must contain exactly one spline / polygon which must be non cyclic.
- Feed Rate: The "softbody goal weight" is used to encode this information per vertex
- Seek: Fast movement
- Line: Slow movement while cutting
- Circle: Reconstructed from bezier curves
