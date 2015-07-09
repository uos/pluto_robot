/*
 * lvr_ros_converter.h
 *
 *  Created on: 30.04.2014
 */

#ifndef LVRROSCONVERTER_H_
#define LVRROSCONVERTER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include "io/Model.hpp"
#include "io/PointBuffer.hpp"
#include "io/MeshBuffer.hpp"
#include "io/PLYIO.hpp"
#include "io/DataStruct.hpp"
#include "io/ModelFactory.hpp"

#include "texture/Texture.hpp"
#include "geometry/Vertex.hpp"

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <lvr_ros/Mesh.h>
#include <lvr_ros/Face.h>
#include <lvr_ros/TriangleMeshGeometry.h>
#include <lvr_ros/LabeledFaces.h>
#include <lvr_ros/PointNormals.h>
#include <lvr_ros/Textures.h>

namespace lvr_ros_converter
{

struct MaterialGroup
{
    int                          texture_index;
    lvr::uchar                   r;
    lvr::uchar                   g;
    lvr::uchar                   b;
    std::vector<unsigned int>    faceBuffer;
};

typedef std::vector<boost::shared_ptr<MaterialGroup> > GroupVector;
typedef boost::shared_ptr<MaterialGroup> MaterialGroupPtr;

/**
 * @brief Class to convert LVR-Types to ROS-Messages.
 */
class LvrRosConverter
{

public:
    /**
     * @brief Constructor
     */
    LvrRosConverter(){};

    /**
     * @brief Destructor
     */
    ~LvrRosConverter(){};

    /**
     * @brief Convert LVR-PointBuffer to Ros-PointNormalsMessage
     *
     * @param buffer  LVR-PointBuffer
     * @parem stamp   TimeStamp
     * @param frame   Needed Ros-TF-Frame
     *
     * @return Ros-PointNormalsMessage
     */
    lvr_ros::PointNormals convertPointBufferPtrToPointNormalsMessage(lvr::PointBufferPtr buffer, ros::Time stamp, string frame);

    /**
     * @brief Convert LVR-MeshBuffer to Ros-TexturesMessage
     *
     * @param buffer  LVR-MeshBuffer
     * @param stamp   TimeStamp
     * @param frame   Needed Ros-TF-Frame
     *
     * @return Ros-TexturesMessage
     */
    lvr_ros::Textures convertMeshBufferToTextureMessage(lvr::MeshBufferPtr buffer, ros::Time stamp, string frame);

    /**
     * @brief Convert LVR-MeshBuffer to Ros-TriangleMeshGeometryMessage
     *
     * @param buffer  LVR-MeshBuffer
     * @parem stamp   TimeStamp
     * @param frame   Needed Ros-TF-Frame
     *
     * @return Ros-TriangleMeshGeometryMessage
     */
    lvr_ros::TriangleMeshGeometry convertMeshBufferToMeshMessage(lvr::MeshBufferPtr buffer, ros::Time stamp, string frame);

    /**
     * @brief Convert LVR-MeshBuffer to Ros-LabeledFacesMessage
     *
     * @param buffer  LVR-MeshBuffer
     * @parem stamp   TimeStamp
     * @param frame   Needed Ros-TF-Frame
     *
     * @return Ros-LabeledFacesMessage
     */
    lvr_ros::LabeledFaces convertMeshBufferToLabeledFacesMessage(lvr::MeshBufferPtr buffer, ros::Time stamp, string frame);

    /**
     * @brief Creates a Ros-TriangleMeshGeometryMessage from a file
     *
     * @param path    Path to a MeshFile
     * @param frame   Needed Ros-TF-Frame
     *
     * @return Ros-TriangleMeshGeometryMessage
     */
    lvr_ros::TriangleMeshGeometry readTriangleMeshGeometry(string path, string frame);

    /**
     * @brief Creates MaterialGroups from a LVR-MeshBuffer
     *
     * @param buffer             LVR-MeshBuffer
     * @parem textureMaterials   Vector with textures
     * @param colorMaterials     Vector with colors
     */
    void generateMaterialGroupsFromMeshBuffer(lvr::MeshBufferPtr buffer, GroupVector &textureMaterials, GroupVector &colorMaterials);

    /**
     * @brief Convert a Ros-PointCloud2Message to a LVR-ModelPointr
     *
     * @param pc          Ros-PointCloud2Message
     * @parem numPoints   Number of points with textures
     *
     * @return LVR-ModelPointr
     */
    lvr::ModelPtr convertPointCloud2PtrToModelPtr(const sensor_msgs::PointCloud2::ConstPtr& pc, int& numPoints);

    /**
     * @brief Convert a Ros-TriangleMeshGeometryMessage to a LVR-MeshBuffer
     *
     * @param mesh    RIS.TriangleMeshGeometryMessage
     *
     * @return LVR-MeshBuffer
     */
    lvr::MeshBuffer convertMeshMessageToMeshBuffer(lvr_ros::TriangleMeshGeometry mesh);

    /**
     * @brief Creates a LVR-MeshBufferPointer from a file
     *
     * @param path    Path to a MeshFile
     *
     * @return LVR-MeshBufferPointer
     */
    lvr::MeshBufferPtr readMeshBufferPtr(string path);

    /**
     * @brief Writes a LVR-MeshBufferPointer to a file
     *
     * @param mesh   LVR-MeshBufferPointer
     * @param path   Path to a MeshFile
     */
    void writeMeshBufferPtr(lvr::MeshBufferPtr mesh, string path);

    /**
     * @brief Writes a ROS-TriangleMeshGeometryMessage to a file
     *
     * @param mesh   ROS-TriangleMeshGeometryMessage
     * @param path   Path to a MeshFile
     */
    void writeTriangleMeshGeometry(lvr_ros::TriangleMeshGeometry mesh, string path);

    /**
     * @brief Gets the Colors of a Ros-PointCloud2Message
     *
     * @param pc          Ros-PointCloud2Message
     * @param numColors   Number of Colors
     *
     * @param Pointer to the colors-Array
     */
    uint8_t* getPointCloud2Colors(const sensor_msgs::PointCloud2ConstPtr& pc, int& numColors);

    /**
     * @brief Gets the Colors-Offset of a Ros-PointCloud2Message
     *
     * @param pc          Ros-PointCloud2Message
     * @param rgb_offset  RGB-Offset
     *
     * @param Datatype
     */
    int getPointCloud2ColorOffset(const sensor_msgs::PointCloud2ConstPtr& pc, int& rgb_offset);

    /**
     * @brief Gets the Intensities of a Ros-PointCloud2Message
     *
     * @param pc               Ros-PointCloud2Message
     * @param numIntensities   Number of Intensities
     *
     * @param Pointer to the intensities-Array
     */
    float* getPointCloud2Intensities(const sensor_msgs::PointCloud2ConstPtr& pc, int& numIntensities);

    /**
     * @brief Gets the Intensities-Offsets of a Ros-PointCloud2Message
     *
     * @param pc                   Ros-PointCloud2Message
     * @param intensities_offset   Intensities-Offset
     *
     * @param Datatype
     */
    int getPointCloud2IntensitiesOffset(const sensor_msgs::PointCloud2ConstPtr& pc, int& intensities_offset);

    /**
     * @brief Gets the XYZ-Offsets of a Ros-PointCloud2Message
     *
     * @param pc    Ros-PointCloud2Message
     * @param xo    X-Offset
     * @param yo    Y-Offset
     * @param zo    Z-Offset
     *
     * @param Datatype
     */
    int getPointCloud2XYZOffsets(const sensor_msgs::PointCloud2ConstPtr& pc, int& xo, int& yo, int&zo);

    /**
     * @brief Gets the Points-Array of a Ros-PointCloud2Message
     *
     * @param pc          Ros-PointCloud2Message
     * @param numPoints   Number of points
     *
     * @param Pointer to the Points-Array
     */   
    float* getPointArrFromCloud2(const sensor_msgs::PointCloud2ConstPtr& pc, int &numPoints);

    /**
     * @brief Converts ucharArray to float-Array
     *
     * @param ucharColor    ucharColor-Array
     * @param length        Length of the array
     *
     * @param Pointer to the converted Array
     */  
    float* ucharToFloatColor(lvr::ucharArr ucharColor, int length);
};

} // end namespace

#endif /* LVRROSCONVERTER_H_ */
