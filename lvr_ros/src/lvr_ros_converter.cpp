/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * lvr_ros_converter.cpp
 *
 * Author: Henning Deeken <hdeeken@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 */

#include "lvr_ros_converter.h"

namespace lvr_ros_converter
{

lvr_ros::PointNormals LvrRosConverter::convertPointBufferPtrToPointNormalsMessage(lvr::PointBufferPtr buffer,
                                                                                    ros::Time stamp,
                                                                                    string frame)
{
    size_t numPoints;
    lvr::floatArr points = buffer->getPointArray(numPoints);

    size_t numNormals;
    lvr::floatArr pointnormals = buffer->getPointNormalArray(numNormals);

    lvr_ros::PointNormals normalsMsg;
    normalsMsg.header.stamp = stamp;
    normalsMsg.header.frame_id = frame;
    normalsMsg.points.resize(numPoints);
    normalsMsg.normals.resize(numNormals);

    if (numPoints)
    {
        for (int i = 0, j = 0; i < numPoints; i++, j += 3)
        {
            normalsMsg.points[i].x = points[j];
            normalsMsg.points[i].y = points[j + 1];
            normalsMsg.points[i].z = points[j + 2];
        }
    }

    if (numNormals)
    {
        for (int i = 0, j = 0; i < numNormals; i++, j += 3)
        {
            normalsMsg.normals[i].x = pointnormals[j];
            normalsMsg.normals[i].y = pointnormals[j + 1];
            normalsMsg.normals[i].z = pointnormals[j + 2];
        }
    }

    return normalsMsg;
}

lvr_ros::Textures LvrRosConverter::convertMeshBufferToTextureMessage(lvr::MeshBufferPtr buffer,
                                                                       ros::Time stamp,
                                                                       string frame)
{
    GroupVector textureMaterials;
    GroupVector colorMaterials;
    generateMaterialGroupsFromMeshBuffer(buffer, textureMaterials, colorMaterials);

    size_t numVertices;
    size_t numVertexTextureCoordinates;
    size_t numTextures;

    lvr::coord3fArr vertices = buffer->getIndexedVertexArray(numVertices);
    lvr::coord3fArr vertexTextureCoordinates = buffer->getIndexedVertexTextureCoordinateArray(numVertexTextureCoordinates);
    lvr::textureArr textures = buffer->getTextureArray(numTextures);

    lvr_ros::Textures texturesMsg;
    texturesMsg.header.stamp = stamp;
    texturesMsg.header.frame_id = frame;
    texturesMsg.texturecoords.resize(numVertices);
    texturesMsg.materialgroups.resize(textureMaterials.size() + colorMaterials.size());

    for (int i = 0; i < numVertices; i++)
    {
        // texture stuff
        texturesMsg.texturecoords[i].x = vertexTextureCoordinates[i][0];
        texturesMsg.texturecoords[i].y = 1 - vertexTextureCoordinates[i][1];
        texturesMsg.texturecoords[i].z = vertexTextureCoordinates[i][2];
    }

    // fill materialgroups message type with material groups WITH texture
    for (int i = 0; i < textureMaterials.size(); i++) 
    {
        MaterialGroupPtr g = textureMaterials[i];

        int width  = textures[g->texture_index]->m_width;
	int height = textures[g->texture_index]->m_height;

        // fill image in materialgroup message
        texturesMsg.materialgroups[i].image.width  = width;
        texturesMsg.materialgroups[i].image.height = height;
        texturesMsg.materialgroups[i].image.data.resize(width * height * 3);

        for (int j = 0; j < width * height * 3; j++)
        {
            texturesMsg.materialgroups[i].image.data[j] = (unsigned int8_t) textures[g->texture_index]->m_pixels[j];
        }

        // fill default color in message
        texturesMsg.materialgroups[i].defaultcolor.r = g->r;
        texturesMsg.materialgroups[i].defaultcolor.g = g->g;
        texturesMsg.materialgroups[i].defaultcolor.b = g->b;

        //TODO: put in intensity values
        texturesMsg.materialgroups[i].defaultcolor.a = 1;
        texturesMsg.materialgroups[i].face_indices.resize(g->faceBuffer.size());

        // fill associated faces of texture in message
        for (int k = 0; k < g->faceBuffer.size(); k++)
        {
            texturesMsg.materialgroups[i].face_indices[k] = g->faceBuffer[k];
        }
        g->faceBuffer.clear();
    }

    // fill materialgroups WITHOUT texture in message
    for (int i = 0; i < colorMaterials.size(); i++) 
    {
        MaterialGroupPtr g = colorMaterials[i];
        int index = i + textureMaterials.size();
        // fill default color in message
        texturesMsg.materialgroups[index].defaultcolor.r = g->r;
        texturesMsg.materialgroups[index].defaultcolor.g = g->g;
        texturesMsg.materialgroups[index].defaultcolor.b = g->b;
        texturesMsg.materialgroups[index].image.width  = 0;
        texturesMsg.materialgroups[index].image.height = 0;
        //TODO: put in intensity values
        texturesMsg.materialgroups[index].defaultcolor.a = 1;
        texturesMsg.materialgroups[index].face_indices.resize(g->faceBuffer.size());

        // fill associated faces of texture in message
        for (int k = 0; k < g->faceBuffer.size(); k++)
        {
            texturesMsg.materialgroups[index].face_indices[k] = g->faceBuffer[k];
        }
        g->faceBuffer.clear();
    }
    textureMaterials.clear();
    colorMaterials.clear();
    return texturesMsg;
}

lvr_ros::LabeledFaces LvrRosConverter::convertMeshBufferToLabeledFacesMessage(lvr::MeshBufferPtr buffer, ros::Time stamp, string frame)
{
    lvr::labeledFacesMap labeledFacesMap = buffer->getLabeledFacesMap();
    lvr_ros::LabeledFaces labeledFacesMsg;
    labeledFacesMsg.header.stamp = stamp;
    labeledFacesMsg.header.frame_id = frame;

    return labeledFacesMsg;
}

lvr_ros::TriangleMeshGeometry LvrRosConverter::convertMeshBufferToMeshMessage(lvr::MeshBufferPtr buffer,
                                                                                ros::Time stamp,
                                                                                string frame)
{
    size_t numVertices;
    lvr::coord3fArr verticesArray = buffer->getIndexedVertexArray(numVertices);
    size_t numFaces;
    lvr::uintArr facesArray = buffer->getFaceArray(numFaces);

    lvr_ros::TriangleMeshGeometry mesh;
    mesh.header.stamp = stamp;
    mesh.header.frame_id = frame;
    mesh.vertices.resize(numVertices);
    mesh.faces.resize(numFaces);

    if (numVertices)
    {
        for (unsigned int i = 0; i < numVertices; i++)
        {
            mesh.vertices[i].x = verticesArray[i].x;
            mesh.vertices[i].y = verticesArray[i].y;
            mesh.vertices[i].z = verticesArray[i].z;
        }

        for (unsigned int i = 0; i < numFaces; i++)
        {
            mesh.faces[i].i = facesArray[i * 3];
            mesh.faces[i].j = facesArray[i * 3 + 1];
            mesh.faces[i].k = facesArray[i * 3 + 2];
        }
    }

    return mesh;
}

lvr_ros::TriangleMeshGeometry LvrRosConverter::readTriangleMeshGeometry(string path, string frame)
{
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel(path);
    lvr::MeshBufferPtr input_mesh;

    // Parse loaded data
    if (!model)
    {
        cout << "IO Error: Unable to parse " << path << endl;
        exit(-1);
    }

    input_mesh = model->m_mesh;

	lvr_ros::TriangleMeshGeometry mesh = convertMeshBufferToMeshMessage(input_mesh, ros::Time::now(), frame);

    return mesh;
}

void LvrRosConverter::generateMaterialGroupsFromMeshBuffer(lvr::MeshBufferPtr buffer,
                                                           GroupVector &textureMaterials,
                                                           GroupVector &colorMaterials)
{
    int countera = 0;
    size_t numMaterials;
    size_t numFaceMaterialIndices;
    size_t numFaces;

    lvr::materialArr faceMaterials = buffer->getMaterialArray(numMaterials);
    lvr::uintArr faceMaterialIndices = buffer->getFaceMaterialIndexArray(numFaceMaterialIndices);
    lvr::uintArr facesArray = buffer->getFaceArray(numFaces);
 
    std::map<int, MaterialGroupPtr > texMatMap;
    std::map<lvr::Vertex<lvr::uchar>, MaterialGroupPtr > colorMatMap;

    // Iterate over face material buffer and
    // sort faces by their material
    for (unsigned int i = 0; i < numMaterials; i++)
    {
        std::map<int, MaterialGroupPtr>::iterator texIt;
        std::map<lvr::Vertex<lvr::uchar>, MaterialGroupPtr >::iterator colIt;

        // Get material by index and lookup in map. If present
        // add face index to the corresponding group. Create a new
        // group if none was found. For efficient rendering we have to
        // create groups by color and texture index,
        lvr::Material* m = faceMaterials[i];

        if (m->texture_index != -1)
        {

            texIt = texMatMap.find(m->texture_index);
            if (texIt == texMatMap.end())
            {
                MaterialGroupPtr g = MaterialGroupPtr(new MaterialGroup());
                g->texture_index = m->texture_index;
                g->r = 1;
                g->g = 1;
                g->b = 1;
                textureMaterials.push_back(g);
                texMatMap[m->texture_index] = g;
                countera++;
            }
        }

        else
        {
            lvr::Vertex<lvr::uchar> coloru = lvr::Vertex<lvr::uchar>(m->r, m->g, m->b);
            colIt = colorMatMap.find(coloru);

            if (colIt == colorMatMap.end())
            {
                MaterialGroupPtr g = MaterialGroupPtr(new MaterialGroup());
                g->texture_index = m->texture_index;
                g->r = m->r;
                g->g = m->g; 
                g->b = m->b;
                colorMaterials.push_back(g);
                colorMatMap[coloru] = g;
                countera++;
            }
        }
    }

    // fill MaterialGroups with associated faces
    for (unsigned int i = 0; i < numFaces; i++)
    {
        std::map<int, MaterialGroupPtr>::iterator texIt;
        std::map<lvr::Vertex<lvr::uchar>, MaterialGroupPtr >::iterator colIt;
        
        lvr::Material* m = faceMaterials[faceMaterialIndices[i]];
        
        if (m->texture_index != -1)
        {
            texIt = texMatMap.find(m->texture_index);
            texIt->second->faceBuffer.push_back(i);
        }

        else
        {
            colIt = colorMatMap.find(lvr::Vertex<lvr::uchar>(m->r, m->g, m->b));
            colIt->second->faceBuffer.push_back(i);
        }
    }
}

lvr::ModelPtr LvrRosConverter::convertPointCloud2PtrToModelPtr(const sensor_msgs::PointCloud2::ConstPtr& pc, int& numPoints)
{
    int numColors;
    int numIntensities;

    float* pointsBuf = getPointArrFromCloud2(pc, numPoints);
    uint8_t* pointColorsBuf = getPointCloud2Colors(pc, numColors);
    float* pointIntensitiesBuf = getPointCloud2Intensities(pc, numIntensities);

    lvr::floatArr pointConfidences;

    int nanCount = 0;
    for (int i = 0; i < numPoints; i++)
    {
        float x = pointsBuf[i * 3 + 0];
        float y = pointsBuf[i * 3 + 1];
        float z = pointsBuf[i * 3 + 2];
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            nanCount++;
        }
    }

    // ignore nans
    numPoints -= nanCount;
    numColors -= nanCount;
    numIntensities -= nanCount;

    if (numPoints <= 0)
    {
        lvr::ModelPtr model(new lvr::Model(lvr::PointBufferPtr(new lvr::PointBuffer)));
        return model;
    }

    lvr::floatArr points;
    lvr::ucharArr pointColors;
    lvr::floatArr pointIntensities;
//    lvr::floatArr pointConfidences;

    points.reset(new float[numPoints * 3]);
    
    if (numColors > 0)
    {
        pointColors.reset(new uint8_t[numColors * 3]);
    }

    if (numIntensities > 0)
    {
        pointIntensities.reset(new float[numIntensities]);
    }

    for (int i = 0, j = 0; i < (numPoints + nanCount); i++)
    {
        float x = pointsBuf[i * 3 + 0];
        float y = pointsBuf[i * 3 + 1];
        float z = pointsBuf[i * 3 + 2];
        if (!(std::isnan(x) || std::isnan(y) || std::isnan(z)))
        {
            points[j * 3 + 0] = x;
            points[j * 3 + 1] = y;
            points[j * 3 + 2] = z;

            if (numColors == numPoints)
            {
                pointColors[j * 3 + 0] = pointColorsBuf[i * 3 + 0];
                pointColors[j * 3 + 1] = pointColorsBuf[i * 3 + 1];
                pointColors[j * 3 + 2] = pointColorsBuf[i * 3 + 2];
            }

            if (numIntensities == numPoints)
            {
                pointIntensities[j] = pointIntensitiesBuf[i];
            }
            
            j++;
        }
    }

    delete[] pointsBuf;
    delete[] pointColorsBuf;
    delete[] pointIntensitiesBuf;

    ROS_INFO("Ignored %i nans", nanCount);
    ROS_INFO("numPoints: %i, numColors: %i", numPoints, numColors);

    lvr::ModelPtr model(new lvr::Model(lvr::PointBufferPtr(new lvr::PointBuffer)));
    model->m_pointCloud->setPointArray(points, numPoints);

    if (numColors > 0)
    {
        model->m_pointCloud->setPointColorArray(pointColors, numColors);
    }

    if (numIntensities > 0)
    {
        model->m_pointCloud->setPointIntensityArray(pointIntensities, numIntensities);
    }

    // model->m_pointCloud->setPointConfidenceArray(pointConfidences, 0);
    return model;
}

lvr::MeshBuffer LvrRosConverter::convertMeshMessageToMeshBuffer(lvr_ros::TriangleMeshGeometry mesh)
{
    lvr::MeshBuffer mesh_buffer;
    vector<float> vertices;

    for (unsigned int i = 0; i < mesh.vertices.size(); i++)
    {
        vertices.push_back((float) mesh.vertices[i].x);
        vertices.push_back((float) mesh.vertices[i].y);
        vertices.push_back((float) mesh.vertices[i].z);	
    }

    mesh_buffer.setVertexArray(vertices);

    vector<unsigned int> faces;

    for (unsigned int i = 0; i < mesh.faces.size(); i++)
    {
        faces.push_back((unsigned int) mesh.faces[i].i);
        faces.push_back((unsigned int) mesh.faces[i].j);
        faces.push_back((unsigned int) mesh.faces[i].k);
    }

    mesh_buffer.setFaceArray(faces);

    return mesh_buffer;	
}

lvr::MeshBufferPtr LvrRosConverter::readMeshBufferPtr(string path)
{
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel(path);
    lvr::MeshBufferPtr input_mesh;

    // Parse loaded data
    if (!model)
    {
        cout << "IO Error: Unable to parse " << path << endl;
        exit(-1);
    }	
    return model->m_mesh;	
}

void LvrRosConverter::writeMeshBufferPtr(lvr::MeshBufferPtr mesh, string path)
{
    ROS_INFO("Writing mesh to %s", path.c_str());
    lvr::ModelPtr m(new lvr::Model(mesh));
    lvr::ModelFactory::saveModel(m, path);
}

void LvrRosConverter::writeTriangleMeshGeometry(lvr_ros::TriangleMeshGeometry mesh, string path)
{
    ROS_INFO("Writing mesh to %s", path.c_str());
    lvr::MeshBuffer buffer = convertMeshMessageToMeshBuffer(mesh); 
    lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer); 

    lvr::ModelPtr m(new lvr::Model(buffer_ptr));
    lvr::ModelFactory::saveModel(m, path);
}

uint8_t* LvrRosConverter::getPointCloud2Colors(const sensor_msgs::PointCloud2ConstPtr& pc,
                                               int& numColors)
{
    int width = pc->width;
    int height = pc->height;
    numColors = width * height;

    int pointLength = pc->point_step;

    // get offset
    int offset, datatype;
    datatype = getPointCloud2ColorOffset(pc, offset);


    // Keine rgb-Daten vorhanden?
    if (offset == -1)
    {
        numColors = -1;
        return NULL;
    }
                

   if (datatype != pc->fields[0].FLOAT32)
   {
       ROS_ERROR("Color data format not supported! >%i<", datatype);
       numColors = -1;
       return NULL;
   }

   uint8_t* colorData = new uint8_t[numColors * 3];
   for (int i = 0; i < numColors; i++)
   {
       uint8_t r, g, b;

       // get the packed color value
       float val = *((float*) &(pc->data[i * pointLength + offset]));

       // r g b        // TODO
       r = *((unsigned char*) (&val) + 2);
       g = *((unsigned char*) (&val) + 1);
       b = *((unsigned char*) (&val) + 0);
                
       colorData[i * 3 + 0] = r;
       colorData[i * 3 + 1] = g;
       colorData[i * 3 + 2] = b;
    }
        
    return colorData;
}

int LvrRosConverter::getPointCloud2ColorOffset(const sensor_msgs::PointCloud2ConstPtr& pc,
                                               int& rgb_offset)
{
    // set failure data
    int datatype = -1;
    rgb_offset = -1;

    // search for r / g / b fields
    for (int i = 0; i < pc->fields.size(); i++)
    {
        sensor_msgs::PointField pf = pc->fields[i];

        if (pf.name == "rgb")
        {
            rgb_offset = pf.offset;
            datatype = pf.datatype;
        }
    }

    return datatype;
}

float* LvrRosConverter::getPointCloud2Intensities(const sensor_msgs::PointCloud2ConstPtr& pc,
                                                  int& numIntensities)
{
    int width = pc->width;
    int height = pc->height;
    numIntensities = width * height;

    int pointLength = pc->point_step;

    int offset, datatype;
    datatype = getPointCloud2IntensitiesOffset(pc, offset);

    if (offset == -1)
    {
        numIntensities = -1;
        return NULL;
    }

    if (datatype != pc->fields[0].FLOAT32)          //TODO find out correct datatype for intensity data !!
    {
        ROS_ERROR( "Intensity data format not supported! >%i<", datatype);
        numIntensities = -1;
        return NULL;
    }

    float* intensityData = new float[numIntensities];
    for (int i = 0; i < numIntensities; i++)
    {
        // get the intensity value
        float val = *((float*) &(pc->data[i * pointLength + offset]));
        intensityData[i] = val;
    }

    return intensityData;
}

int LvrRosConverter::getPointCloud2IntensitiesOffset(const sensor_msgs::PointCloud2ConstPtr& pc,
                                                     int& intensities_offset)
{
    int datatype = -1;
    intensities_offset = -1;

    for (int i=0; i < pc->fields.size(); i++)
    {
        sensor_msgs::PointField pf = pc->fields[i];
        if (pf.name == "intensities")
        {
            intensities_offset = pf.offset;
            datatype = pf.datatype;
        }
    }
}

int LvrRosConverter::getPointCloud2XYZOffsets(const sensor_msgs::PointCloud2ConstPtr& pc,
                                              int& xo, int& yo, int&zo)
{
    // set failure data
    int datatype = -1;
    xo = yo = zo = -1;

    // search for x/y/z fields
    for (int i = 0; i < pc->fields.size(); i++)
    {
        sensor_msgs::PointField pf = pc->fields[i];

        if (pf.name == "x")
        {
            xo = pf.offset;
            datatype = pf.datatype;
        }

        if (pf.name == "y")
        {
            yo = pf.offset;
        }

        if (pf.name == "z")
        {
            zo = pf.offset;
        }
    }

    return datatype;
}

float* LvrRosConverter::getPointArrFromCloud2(const sensor_msgs::PointCloud2ConstPtr& pc,
                                              int &numPoints)
{
    // INFO: doesn't ignore nan-data anymore as color value is extracted
    //       seperately - there'd be no correct mapping possible

    int width = pc->width;
    int height = pc->height;
    int pointLength = pc->point_step;

    numPoints = width * height;

    int xo, yo, zo; // offsets
    int dataType = getPointCloud2XYZOffsets(pc, xo, yo, zo);

    // empty float[] of matching size
    float* pointData = new float[numPoints * 3];

    for (int i = 0; i < numPoints; i++)
    {
        float x, y, z;
        // interpret the adress of the 
        // i*pointLength+offset
        // byte-entry in the data blob as a
        // float* / int* / ...
        // and get the value its pointing to.

        if (dataType == pc->fields[0].INT8)
        {
            x = (float) *((char*) &(pc->data[i * pointLength + xo]));
            y = (float) *((char*) &(pc->data[i * pointLength + yo]));
            z = (float) *((char*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].UINT8)
        {
            x = (float) *((unsigned char*) &(pc->data[i * pointLength + xo]));
            y = (float) *((unsigned char*) &(pc->data[i * pointLength + yo]));
            z = (float) *((unsigned char*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].INT16)
        {
            x = (float) *((short int*) &(pc->data[i * pointLength + xo]));
            y = (float) *((short int*) &(pc->data[i * pointLength + yo]));
            z = (float) *((short int*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].UINT16)
        {
            x = (float) *((unsigned short int*) &(pc->data[i * pointLength + xo]));
            y = (float) *((unsigned short int*) &(pc->data[i * pointLength + yo]));
            z = (float) *((unsigned short int*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].INT32)
        {
            x = (float) *((int*) &(pc->data[i * pointLength + xo]));
            y = (float) *((int*) &(pc->data[i * pointLength + yo]));
            z = (float) *((int*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].UINT32)
        {
            x = (float) *((unsigned int*) &(pc->data[i * pointLength + xo]));
            y = (float) *((unsigned int*) &(pc->data[i * pointLength + yo]));
            z = (float) *((unsigned int*) &(pc->data[i * pointLength + zo]));
        }

        else if (dataType == pc->fields[0].FLOAT32)
        {
            x = *((float*) &(pc->data[i * pointLength + xo]));
            y = *((float*) &(pc->data[i * pointLength + yo]));
            z = *((float*) &(pc->data[i * pointLength + zo]));
        }

        else
        {
            ROS_ERROR("Point datatype >%i< not supported!", dataType);
            delete[] pointData;
            return NULL;
        }

        pointData[i * 3]     = x;
        pointData[i * 3 + 1] = y;
        pointData[i * 3 + 2] = z;
    }

    return pointData;
}

float* LvrRosConverter::ucharToFloatColor(lvr::ucharArr ucharColor, int length)
{
    float* floatColor = new float[length];

    ROS_INFO("in ucharToFloatColor: length: %i", length);

    for (int i = 0; i < length; i++)
    {
        floatColor[i] = ucharColor[i] / 255.;

        // test:
        if (floatColor[i] < 0. || floatColor[i] > 1.)
        {
            printf("ERROR converting color data! \n");
        }
    }

    return floatColor;
}
} // end namespace
