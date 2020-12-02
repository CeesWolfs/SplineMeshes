#include "Mesh.hpp"

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

/*
* Default construct the mesh the a unit cube
*/
Mesh::Mesh(/* args */) {
    // Add 8 initial vertices
    vertices.push_back({ 0.0,0.0,0.0 });
    vertices.push_back({ 1.0,0.0,0.0 });
    vertices.push_back({ 1.0,1.0,0.0 });
    vertices.push_back({ 0.0,1.0,0.0 });
    vertices.push_back({ 0.0,0.0,1.0 });
    vertices.push_back({ 1.0,0.0,1.0 });
    vertices.push_back({ 1.0,1.0,1.0 });
    vertices.push_back({ 0.0,1.0,1.0 });
    // Add initial 3D cuboid
    cuboids.push_back({ 0,1,2,3,4,5,6,7 });

    // Add 6 border faces
    for (int i = 0; i < 6; i++) {
        F2f.push_back(halfFace(border_id));
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        V2lV.push_back({ 0,i });
    }
}

const std::vector<Vertex>& Mesh::getVertices() const {
    return vertices;
}
const std::vector<Cuboid>& Mesh::getCuboids() const {
    return cuboids;
}

const std::vector<halfFace>& Mesh::getF2f() const {
    return F2f;
}

const std::vector<localVertex>& Mesh::getV2lV() const {
    return V2lV;
}

void Mesh::Save(const std::string& filename)
{
    std::filebuf fb_binary;
    fb_binary.open(filename + ".ply", std::ios::out);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + filename);
    tinyply::PlyFile file;
    file.add_properties_to_element("vertex", { "x", "y", "z" },
        tinyply::Type::FLOAT32, vertices.size(), reinterpret_cast<uint8_t*>(vertices.data()), tinyply::Type::INVALID, 0);
    std::vector<uint32_t> face_vert_ids;
    face_vert_ids.reserve(6 * cuboids.size());
    for(const auto& cub : cuboids)
    {
        for (size_t i = 0; i < 6; i++)
        {
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][0]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][1]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][2]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][3]]);
        }
    }
    file.add_properties_to_element("face", { "vertex_indices" },
        tinyply::Type::UINT32, cuboids.size() * 6, reinterpret_cast<uint8_t*>(face_vert_ids.data()), tinyply::Type::UINT8, 4);
    // Write a text file
    file.write(outstream_binary, false);
}

void Mesh::splitHalfFace(const halfFace toSplit, const halfFace lower, const halfFace higher, const Axis split_axis, const Vertex& split_point)
{
    auto twin = Twin(toSplit);
    if (twin.isBorder()) return;
    if (twin.isSubdivided()) {
        // Divide the subFaces over the to new halfFaces
        auto split_res = sft.splitTree(F2f[1], split_axis, split_point, lower , higher, F2f);
        sft.updateParent(split_res.first, lower);
        sft.updateParent(split_res.second, higher);
        Twin(lower) = split_res.first;
        Twin(higher) = split_res.second;
    }
    else {
        // Just split the twin
        Twin(toSplit) = sft.splitHalfFace(twin, twin, split_axis, split_point, lower, higher);
    }
}

void Mesh::updateHalfFace(const halfFace hf, const halfFace new_hf, const Vertex& middle)
{
    auto twin = Twin(hf);
    if (twin.isBorder()) return;
    if (twin.isSubdivided())
    {
        // Update the parent of the nodes
        sft.updateParent(twin, new_hf);
        // Update all the subFaces
        for (auto it = sft.begin(twin); it != sft.end(); ++it) {
            updateTwin(twin, hf, new_hf, middle);
        }
    }
    else {
        updateTwin(twin, hf, new_hf, middle);
    }
}

void Mesh::updateTwin(const halfFace twin, const halfFace old_hf, const halfFace new_hf, const Vertex& middle)
{
    if (twin.isSubdivided()) {
        auto it = sft.find(twin, old_hf, middle);
        *it = new_hf;
    }
    else {
        Twin(twin) = new_hf;
    }

}


halfFace& Mesh::Twin(const halfFace& hf) {
    return F2f[hf.getCuboid() * 6 + hf.getLocalId()];
}

const halfFace& Mesh::Twin(const halfFace& hf) const
{
    return F2f[hf.getCuboid() * 6 + hf.getLocalId()];
}


bool Mesh::mergeVertexIfExists(const Vertex& v, uint32_t& vref) {
    auto it = std::find(this->vertices.begin(), this->vertices.end(), v);
    // If element was found
    if (it != this->vertices.end())
    {
        // calculate the index and return
        vref = (it - this->vertices.begin());
        return true;
    }
    else {
        return false;
    }
}

// TODO: this function is supposed to refactor the code of helper functions for searching vertex in one function, avoiding duplicate code
//bool Mesh::seachVertexByAxis(const std::vector<halfFace>& hfts, const Axis splitAxis, const Vertex& toFind) const {
//    const halfFace hft1 = hfts.at(0);
//    const halfFace hft2 = hfts.at(1);
//    const halfFace hft3 = hfts.at(2);
//    const halfFace hft4 = hfts.at(3);
// 
//    halfFace hft_req1(-1, -1);
//    halfFace hft_req2(-1, -1);
//    halfFace hft_req1_twin(-1, -1);
//    halfFace hft_req2_twin(-1, -1);
// 
//    if (splitAxis == Axis::x) {
//        // getting the requried half faces in the bottom cuboid
//        const halfFace hft_req1 = Twin(Twin(halfFace(hft1.getCuboid(), 4))); // front twin face
//        const halfFace hft_req2 = Twin(Twin(halfFace(hft1.getCuboid(), 2))); // back twin face
// 
//        // also checking twin half faces of the half faces in the bottom cuboid
//        const halfFace hft_req1_twin = Twin(hft_req1); // twin face of the twin face req1
//        const halfFace hft_req2_twin = Twin(hft_req2); // twin face of the twin face req2
//    }
//    else if (splitAxis == Axis::y) {
//        // getting the requried half faces in the bottom cuboid
//        const halfFace hf1_right = ;
//        const halfFace hf1_left = ;
// 
//        // also checking twin half faces of the half faces in the bottom cuboid
//        const halfFace hf1_right_twin = Twin(hf1_right);
//        const halfFace hf1_left_twin = Twin(hf1_left);
// 
//        // getting the requried half faces in the bottom cuboid
//        const halfFace hft_req1 = Twin(Twin(halfFace(hf1.getCuboid(), 3))); // right twin face
//        const halfFace hft_req2 = Twin(Twin(halfFace(hf1.getCuboid(), 5))); // left twin face
// 
//        // also checking twin half faces of the half faces in the bottom cuboid
//        const halfFace hft_req1_twin = Twin(hft_req1); // twin face of the twin face req
//        const halfFace hft_req2_twin = Twin(hft_req2); // twin face of the twin face req2
//    }
//    else if (splitAxis == Axis::z) {
// 
//    }
//    
//    if (splitAxis == Axis::x || splitAxis == Axis::y) {
//        // TODO:
//    }
//    else if (splitAxis == Axis::z) {
//        // TODO:
//    }
// 
//    return false;
//}

bool Mesh::findVertexAxisX(
    const halfFace& hf1, // bottom half face 
    const halfFace& hf2, // upper half face
    const halfFace& hf3, // back half face
    const halfFace& hf4, // front half face
    const Vertex& vToFind
) const {
    
    // check twin hf1
    if (!hf1.isBorder()) {
        if (hf1.isSubdivided()) {
            // TODO: traverse tree and get the half faces that are required
        }

        // getting the requried half faces in the bottom cuboid
        const halfFace hf1_front = Twin(Twin(halfFace(hf1.getCuboid(), 4)));
        const halfFace hf1_back = Twin(Twin(halfFace(hf1.getCuboid(), 2)));

        // also checking twin half faces of the half faces in the bottom cuboid
        const halfFace hf1_front_twin = Twin(hf1_front);
        const halfFace hf1_back_twin = Twin(hf1_back);

        if (hf1_front.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_front, vToFind)) {
                return true;
            }
        }
        if (hf1_back.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_back, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf1_front_twin.isBorder()) {
            if (hf1_front_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_front_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf1_back_twin.isBorder()) {
            if (hf1_back_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_back_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // checks hf1 ends here

    // check twin hf2
    if (!hf2.isBorder()) {
        if (hf2.isSubdivided()) {
            // TODO: traverse tree and get the half faces that are required
        }

        // getting the requried half faces in the top cuboid
        const halfFace hf2_back = Twin(Twin(halfFace(hf2.getCuboid(), 2)));
        const halfFace hf2_front = Twin(Twin(halfFace(hf2.getCuboid(), 4)));

        // also checking twin half faces of the half faces in the top cuboid
        const halfFace hf2_back_twin = Twin(hf2_back);
        const halfFace hf2_front_twin = Twin(hf2_front);

        if (hf2_back.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf2_back, vToFind)) {
                return true;
            }
        }
        if (hf2_front.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf2_front, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf2_back_twin.isBorder()) {
            if (hf2_back_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf2_back_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf2_front_twin.isBorder()) {
            if (hf2_front_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf2_front_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // twin hf2 checks ends here

    // check twin hf3
    if (!hf3.isBorder()) {
        if (hf3.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf3, vToFind)) {
                return true;
            }
        }
    } // checks for hf3 ends here

    // check twin hf4
    if (!hf4.isBorder()) {
        if (hf4.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf4, vToFind)) {
                return true;
            }
        }
    } // checks for hf4 ends here

    return false;
}

bool Mesh::findVertexAxisY(
    const halfFace& hf1, // bottom half face
    const halfFace& hf2, // upper half face
    const halfFace& hf3, // right half face
    const halfFace& hf4, // left half face
    const Vertex& vToFind
) const {

    // check twin hf1
    if (!hf1.isBorder()) {
        if (hf1.isSubdivided()) {
            // TODO: traverse tree and get the correct half faces in which to search for the vertex
        }
        // getting the requried half faces in the bottom cuboid
        const halfFace hf1_right = Twin(Twin(halfFace(hf1.getCuboid(), 3)));
        const halfFace hf1_left = Twin(Twin(halfFace(hf1.getCuboid(), 5)));

        // also checking twin half faces of the half faces in the bottom cuboid
        const halfFace hf1_right_twin = Twin(hf1_right);
        const halfFace hf1_left_twin = Twin(hf1_left);

        if (hf1_right.isSubdivided()) {
            //       check whether hf1_front has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_right, vToFind)) {
                return true;
            }
        }
        if (hf1_left.isSubdivided()) {
            //       check whether hf1_back has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_left, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf1_right_twin.isBorder()) {
            if (hf1_right_twin.isSubdivided()) {
                //       check whether hf1_front has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_right_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf1_left_twin.isBorder()) {
            if (hf1_left_twin.isSubdivided()) {
                //       check whether hf1_back has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_left_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // checks hf1 ends here

    // check twin hf2
    if (!hf2.isBorder()) {
        if (hf2.isSubdivided()) {
            // TODO: traverse tree and get the correct half faces in which to search for the vertex
        }

        // getting the requried original half faces in the top cuboid
        const halfFace hf2_left = Twin(Twin(halfFace(hf2.getCuboid(), 5)));
        const halfFace hf2_right = Twin(Twin(halfFace(hf2.getCuboid(), 3)));

        // also checking twin half faces of the half faces in the top cuboid
        const halfFace hf2_left_twin = Twin(hf2_left);
        const halfFace hf2_right_twin = Twin(hf2_right);

        if (hf2_left.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf2_left, vToFind)) {
                return true;
            }
        }
        if (hf2_right.isSubdivided()) {
            //       check whether this half face has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf2_right, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf2_left_twin.isBorder()) {
            if (hf2_left_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf2_left_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf2_right_twin.isBorder()) {
            if (hf2_right_twin.isSubdivided()) {
                //       check whether this half face has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf2_right_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // twin hf2 checks ends here

    // check twin hf3
    if (!hf3.isBorder()) {
        if (hf3.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf3, vToFind)) {
                return true;
            }
        }
    } // checks for hf3 ends here

    // check twin hf4
    if (!hf4.isBorder()) {
        if (hf4.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf4, vToFind)) {
                return true;
            }
        }
    } // checks for hf4 ends here

    return false;
}

bool Mesh::findVertexAxisZ(
    const halfFace& hf1, // back half face
    const halfFace& hf2, // right half face
    const halfFace& hf3, // front half face
    const halfFace& hf4, // left half face 
    const Vertex& vToFind
) const {

    // check twin hf1
    if (!hf1.isBorder()) {
        if (hf1.isSubdivided()) {
            // TODO: search for the vertex in this half face. If vertex is found return true.
        }
        // getting the requried half faces in the bottom cuboid
        const halfFace hf1_right = Twin(Twin(halfFace(hf1.getCuboid(), 3)));
        const halfFace hf1_left = Twin(Twin(halfFace(hf1.getCuboid(), 5)));

        // also checking twin half faces of the half faces in the bottom cuboid
        const halfFace hf1_right_twin = Twin(hf1_right);
        const halfFace hf1_left_twin = Twin(hf1_left);

        if (hf1_right.isSubdivided()) {
            //       check whether hf1_front has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_right, vToFind)) {
                return true;
            }
        }
        if (hf1_left.isSubdivided()) {
            //       check whether hf1_back has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf1_left, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf1_right_twin.isBorder()) {
            if (hf1_right_twin.isSubdivided()) {
                //       check whether hf1_front has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_right_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf1_left_twin.isBorder()) {
            if (hf1_left_twin.isSubdivided()) {
                //       check whether hf1_back has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf1_left_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // checks for hf1 ends here

    // check twin hf2
    if (!hf2.isBorder()) {
        if (hf2.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf2, vToFind)) {
                return true;
            }
        }
    } // checks for hf2 ends here

    // check twin hf3
    if (!hf3.isBorder()) {
        if (hf3.isSubdivided()) {
            // TODO: search for the vertex in this half face. If vertex is found return true.
        }

        // getting the requried half faces in the bottom cuboid
        const halfFace hf3_right = Twin(Twin(halfFace(hf3.getCuboid(), 3)));
        const halfFace hf3_left = Twin(Twin(halfFace(hf3.getCuboid(), 5)));

        // also checking twin half faces of the half faces in the bottom cuboid
        const halfFace hf3_right_twin = Twin(hf3_right);
        const halfFace hf3_left_twin = Twin(hf3_left);

        if (hf3_right.isSubdivided()) {
            //       check whether hf3_right has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf3_right, vToFind)) {
                return true;
            }
        }
        if (hf3_left.isSubdivided()) {
            //       check whether hf3_left has a split that contains vertex.
            //       if so then return true
            if (sft.findVertex(hf3_left, vToFind)) {
                return true;
            }
        }
        // check borders for the twins only as there are only cases where the twins can be borders
        // check twins only if they are not part of the border
        if (!hf3_right_twin.isBorder()) {
            if (hf3_right_twin.isSubdivided()) {
                //       check whether hf3_right_twin has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf3_right_twin, vToFind)) {
                    return true;
                }
            }
        }
        if (!hf3_left_twin.isBorder()) {
            if (hf3_left_twin.isSubdivided()) {
                //       check whether hf3_left_twin has a split that contains vertex.
                //       if so then return true
                if (sft.findVertex(hf3_left_twin, vToFind)) {
                    return true;
                }
            }
        }
    } // checks for hf3 ends here

    // check twin hf4
    if (!hf4.isBorder()) {
        if (hf4.isSubdivided()) {
            //  search for the vertex in this half face. If vertex is found return true.
            if (sft.findVertex(hf4, vToFind)) {
                return true;
            }
        }
    } // checks for hf4 ends here

    return false;
}

bool Mesh::mergeVertexIfExistsNew(
    const Vertex& v,
    uint32_t& vref,
    const uint32_t cuboid_id,
    const Axis split_axis
) {

    // TODO: check whether border checks are necessary
    // TODO: make this function shorter by adding helper functions and re-organize the code to make it more readable 
    // TODO: check whether there are half faces that cannot exist yet. So then maybe we should check whether they exist before doing the isBorder or isSubdivided checks?

    // init default hf vars
    halfFace hf1(-1, -1);
    halfFace hf2(-1, -1);
    halfFace hf3(-1, -1);
    halfFace hf4(-1, -1);

    if (split_axis == Axis::x) { // This is the axis of splitYZ

        // use twin half faces of {v1,v2,v3,v4}, {v1,v2,v5,v6}, {v3,v4,v7,v8}, {v5,v6,v7,v8}
        // to check neighbouring cuboids
        hf1 = Twin(halfFace(cuboid_id, 0)); // bottom half face
        hf2 = Twin(halfFace(cuboid_id, 1)); // upper half face
        hf3 = Twin(halfFace(cuboid_id, 2)); // back half face
        hf4 = Twin(halfFace(cuboid_id, 4)); // front half face

        const bool vertexOnXAxisFound = findVertexAxisX(hf1, hf2, hf3, hf4, v);
        if (vertexOnXAxisFound) {
            return true;
        }
        
    } // end axis x checks

    else if (split_axis == Axis::y) { // This is the axis of splitXZ
        // use twin half faces of {v1,v2,v3,v4}, {v2,v3,v6,v7}, {v1,v4,v5,v8}, {v5,v6,v7,v8}
        // to check neighbouring cuboids
        hf1 = Twin(halfFace(cuboid_id, 0)); // bottom half face
        hf2 = Twin(halfFace(cuboid_id, 1)); // upper half face
        hf3 = Twin(halfFace(cuboid_id, 3)); // right half face
        hf4 = Twin(halfFace(cuboid_id, 5)); // left half face
        
        const bool vertexOnYAxisFound = findVertexAxisY(hf1, hf2, hf3, hf4, v);
        if (vertexOnYAxisFound) {
            return true;
        }

    } // end axis y checks

    else if (split_axis == Axis::z) { // This is the axis of splitXY
        // use twin half faces of {v2,v3,v6,v7}, {v1,v2,v5,v6}, {v1,v4,v5,v8}, {v1,v2,v3,v4}
        // to check neighbouring cuboids
        hf1 = Twin(halfFace(cuboid_id, 2)); // back half face
        hf2 = Twin(halfFace(cuboid_id, 3)); // right half face
        hf3 = Twin(halfFace(cuboid_id, 4)); // front half face
        hf4 = Twin(halfFace(cuboid_id, 5)); // left half face 

        const bool vertexOnZAxisFound = findVertexAxisZ(hf1, hf2, hf3, hf4, v);
        if (vertexOnZAxisFound) {
            return true;
        }

    } // end axis z checks

    // if none of the checks returned true, then vertex does not exist yet
    return false;
}


void Mesh::addHalfFaces(const uint32_t cuboid_id, const Axis split_axis) {
    // For now just push back the twins of the original element
    (split_axis == Axis::z) ? F2f.push_back(halfFace(cuboid_id, 1)) : F2f.push_back(Twin(halfFace(cuboid_id, 0)));
    F2f.push_back(Twin(halfFace(cuboid_id, 1)));
    F2f.push_back(Twin(halfFace(cuboid_id, 2)));
    F2f.push_back(Twin(halfFace(cuboid_id, 3)));
    (split_axis == Axis::y) ? F2f.push_back((halfFace(cuboid_id, 2))) : F2f.push_back(Twin(halfFace(cuboid_id, 4)));
    (split_axis == Axis::x) ? F2f.push_back(halfFace(cuboid_id, 3)) : F2f.push_back(Twin(halfFace(cuboid_id, 5)));
}



uint32_t Mesh::SplitAlongXY(uint32_t cuboid_id, float z_split) {
    // border checks
    if ((z_split <= vertices[cuboids[cuboid_id].v1].z) || z_split >= vertices[cuboids[cuboid_id].v5].z) {
        return -1; // Splitpoint not in cuboid
    }
    const uint32_t new_cuboid_id = cuboids.size();

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v2];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v3];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v4];

    // All the new vertices
    const Vertex v1_new = { v1_old.x, v1_old.y, z_split };
    const Vertex v2_new = { v2_old.x, v2_old.y, z_split };
    const Vertex v3_new = { v3_old.x, v3_old.y, z_split };
    const Vertex v4_new = { v4_old.x, v4_old.y, z_split };

    const auto middle = (v1_new + v3_new) / 2;

    // Check whether the new vertices already exist in the vectors array.
    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        // New vertex push it back, and push back V2lV
        v1_idx = vertices.size();
        vertices.push_back(v1_new);
        V2lV.push_back({new_cuboid_id, 1});
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 2 });
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        v3_idx = vertices.size();
        vertices.push_back(v3_new);
        V2lV.push_back({ new_cuboid_id, 3 });
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        v4_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 4 });
        vertices.push_back(v4_new);
    }

    // Update old cuboid vertices
    // Cuboid old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(cuboids[cuboid_id]);
    cuboids[cuboid_id].v5 = v1_idx;
    cuboids[cuboid_id].v6 = v2_idx;
    cuboids[cuboid_id].v7 = v3_idx;
    cuboids[cuboid_id].v8 = v4_idx;

    // Update new cuboid vertices
    Cuboid& new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_idx;
    new_cuboid.v2 = v2_idx;
    new_cuboid.v3 = v3_idx;
    new_cuboid.v4 = v4_idx;

    addHalfFaces(cuboid_id, Axis::z);

    // TODO: Update V2f half faces 

    // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 2), halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 3), halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 4), halfFace(cuboid_id, 4), halfFace(new_cuboid_id, 4), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 5), halfFace(cuboid_id, 5), halfFace(new_cuboid_id, 5), Axis::z, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 1)) = halfFace(new_cuboid_id, 0);
    
    return new_cuboid_id;
}


uint32_t Mesh::SplitAlongYZ(uint32_t cuboid_id, float x_split) {
    // border checks
    if ((x_split <= vertices[cuboids[cuboid_id].v1].x) || x_split >= vertices[cuboids[cuboid_id].v2].x) {
        return -1; // Splitpoint not in cuboid
    }

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v4];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v8];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v5];

    // All the new vertices
    const Vertex v1_new = { x_split, v1_old.y, v1_old.z };
    const Vertex v2_new = { x_split, v2_old.y, v2_old.z };
    const Vertex v3_new = { x_split, v3_old.y, v3_old.z };
    const Vertex v4_new = { x_split, v4_old.y, v4_old.z };

    const auto middle = (v1_new + v3_new) / 2;

    const uint32_t new_cuboid_id = cuboids.size();

    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        v1_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 1 });
        vertices.push_back(v1_new);
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 4 });
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        v3_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 8 });
        vertices.push_back(v3_new);
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        v4_idx = vertices.size();
        vertices.push_back(v4_new);
        V2lV.push_back({ new_cuboid_id, 5 });
    }

     // Update old cuboid vertices
    // Cuboid& old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(cuboids[cuboid_id]);

    cuboids[cuboid_id].v1 = v1_idx;
    cuboids[cuboid_id].v4 = v2_idx;
    cuboids[cuboid_id].v8 = v3_idx;
    cuboids[cuboid_id].v5 = v4_idx;

    // Update new cuboid vertices
    Cuboid &new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v2 = v1_idx;
    new_cuboid.v3 = v2_idx;
    new_cuboid.v7 = v3_idx;
    new_cuboid.v6 = v4_idx;

    addHalfFaces(cuboid_id, Axis::x);

    // TODO: Update V2f half faces 

     // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 0), halfFace(cuboid_id, 0), halfFace(new_cuboid_id, 0), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 1), halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 2), halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 4), halfFace(cuboid_id, 4), halfFace(new_cuboid_id, 4), Axis::x, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 3)) = halfFace(new_cuboid_id, 5);
    
    return new_cuboid_id;
}

uint32_t Mesh::SplitAlongXZ(uint32_t cuboid_id, float y_split) {
    // border checks
    if ((y_split <= vertices[cuboids[cuboid_id].v2].y) || y_split >= vertices[cuboids[cuboid_id].v3].y) {
        return -1; // Splitpoint not in cuboid
    }

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v2];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v6];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v5];

    // All the new vertices
    const Vertex v1_new = { v1_old.x, y_split, v1_old.z };
    const Vertex v2_new = { v2_old.x, y_split, v2_old.z };
    const Vertex v3_new = { v3_old.x, y_split, v3_old.z };
    const Vertex v4_new = { v4_old.x, y_split, v4_old.z };

    const auto middle = (v1_new + v3_new) / 2;

    const uint32_t new_cuboid_id = cuboids.size();

    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        v1_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 1 });
        vertices.push_back(v1_new);
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 2 });
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        v3_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 6 });
        vertices.push_back(v3_new);
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        v4_idx = vertices.size();
        vertices.push_back(v4_new);
        V2lV.push_back({ new_cuboid_id, 5 });
    }

     // Update old cuboid vertices
    // Cuboid old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(cuboids[cuboid_id]);
    cuboids[cuboid_id].v1 = v1_idx;
    cuboids[cuboid_id].v2 = v2_idx;
    cuboids[cuboid_id].v6 = v3_idx;
    cuboids[cuboid_id].v5 = v4_idx;

    // Update new cuboid vertices
    Cuboid new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v4 = v1_idx;
    new_cuboid.v3 = v2_idx;
    new_cuboid.v7 = v3_idx;
    new_cuboid.v8 = v4_idx;

    addHalfFaces(cuboid_id, Axis::y);

    // TODO: Update V2f half faces 

    // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 0), halfFace(cuboid_id, 0), halfFace(new_cuboid_id, 0), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 1), halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 3), halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 5), halfFace(cuboid_id, 5), halfFace(new_cuboid_id, 5), Axis::y, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 2)) = halfFace(new_cuboid_id, 4);

    return new_cuboid_id;
}

Mesh::~Mesh() {

}