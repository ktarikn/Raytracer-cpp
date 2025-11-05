# Raytracer-cpp
Raytracing renderer created for CENG477 Introduction to Computer Graphics Course Homework


Creates PPM images using CPU, Raytracing and C++. Utilizes KD-Tree structure. Reads scene data from xml.

example xml structure:

<Scene>
<BackgroundColor>R G B</BackgroundColor>
<ShadowRayEpsilon>X</ShadowRayEpsilon>
<MaxRecursionDepth>N</MaxRecursionDepth>
<Cameras>
<Camera id=”Cid”>
<Position>X Y Z</Position>
<Gaze>X Y Z</Gaze>
<Up> X Y Z </Up>
<NearPlane>Left Right Bottom Top</NearPlane>
<NearDistance>X</NearDistance>
<ImageResolution>Width Height</ImageResolution>
<ImageName>ImageName.ppm</ImageName>
</Camera>
</Cameras>
<Lights>
<AmbientLight>X Y Z</AmbientLight>
<PointLight id=”Lid”>
<Position>X Y Z</Position>
<Intensity>X Y Z</Intensity>
</PointLight>
</Lights>
<Materials>
<Material id=”Mid” [type=”mirror”]>
<AmbientReflectance>X Y Z</AmbientReflectance>
<DiffuseReflectance>X Y Z</DiffuseReflectance>
<SpecularReflectance>X Y Z</SpecularReflectance>
<MirrorReflectance>X Y Z</MirrorReflectance>
<PhongExponent>X</PhongExponent>
</Material>
</Materials>
<VertexData>
V1X V1Y V1Z
V2X V2Y V2Z
.....................
</VertexData>
<Objects>
<Mesh id=”Meid”>
<Material>N</Material>
<Faces>
F1 V 1 F1 V 2 F1 V 3
F2 V 1 F2 V 2 F2 V 3
.........................
</Faces>
</Mesh>
<Triangle id=”Tid”>
<Material>N</Material>
<Indices>
V1 V2 V3
</Indices>
</Triangle>
<Sphere id=”Sid”>
<Material>N</Material>
<Center>N</Center>
<Radius>X</Radius>
</Sphere>
</Objects>
</Scene>
