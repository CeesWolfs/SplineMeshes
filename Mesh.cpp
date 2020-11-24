#include "Mesh.hpp"
#include "SubFaceTree.hpp"

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
        V2f.push_back({ 0, (uint8_t)i });
    }
}

halfFace Mesh::Twin(const halfFace& hf) const {
    return F2f[hf.id];
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


bool Mesh::mergeVertexIfExistsNew(
    const Vertex& v,
    uint32_t& vref,
    const uint32_t cuboid_id,
    const Axis split_axis,
    SubFaceTree& sft
) {

    // TODO: check whether border checks are necessary (except for twins because that is necessary)
    // TODO: make this function shorter by adding helper functions and re-organize the code to make it more readable 

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

        // check twin hf1
        if (!hf1.isBorder()) {
            if (hf1.isSubdivided()) {
                // TODO: traverse tree and get the half faces that are required
            }

            // getting the requried half faces in the bottom cuboid
            const halfFace hf1_front = halfFace(hf1.getCuboid(), 2);
            const halfFace hf1_back = halfFace(hf1.getCuboid(), 4);

            // also checking twin half faces of the half faces in the bottom cuboid
            const halfFace hf1_front_twin = Twin(hf1_front);
            const halfFace hf1_back_twin = Twin(hf1_back);

            if (hf1_front.isSubdivided()) {
                // TODO: check whether this half face has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            if (hf1_back.isSubdivided()) {
                // TODO: check whether this half face has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            // check borders for the twins only as there are only cases where the twins can be borders
            // check twins only if there are not part of the border
            if (!hf1_front_twin.isBorder()) {
                if (hf1_front_twin.isSubdivided()) {
                    // TODO: check whether this half face has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
            if (!hf1_back_twin.isBorder()) {
                if (hf1_back_twin.isSubdivided()) {
                    // TODO: check whether this half face has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
        } // checks hf1 ends here

        // check twin hf2
        if (!hf2.isBorder()) {
            if (hf2.isSubdivided()) {
                // TODO: traverse tree and get the half faces that are required
            }

            // getting the requried half faces in the top cuboid
            const halfFace hf2_back = halfFace(hf2.getCuboid(), 2);
            const halfFace hf2_front = halfFace(hf2.getCuboid(), 4);

            // also checking twin half faces of the half faces in the top cuboid
            const halfFace hf2_back_twin = Twin(hf2_back);
            const halfFace hf2_front_twin = Twin(hf2_front);

            if (hf2_back.isSubdivided()) {
                // TODO: check whether this half face has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            if (hf2_front.isSubdivided()) {
                // TODO: check whether this half face has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            // check borders for the twins only as there are only cases where the twins can be borders
            // check twins only if there are not part of the border
            if (!hf2_back_twin.isBorder()) {
                if (hf2_back_twin.isSubdivided()) {
                    // TODO: check whether this half face has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
            if (!hf2_front_twin.isBorder()) {
                if (hf2_front_twin.isSubdivided()) {
                    // TODO: check whether this half face has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
        } // twin hf2 checks ends here

        // check twin hf3
        if (!hf3.isBorder()) {
            if (hf3.isSubdivided()) {
                // TODO: search for the vertex in this half face. If vertex is found return true.
            }
        } // checks for hf3 ends here

        if (!hf4.isBorder()) {
            if (hf4.isSubdivided()) {
                // TODO: search for the vertex in this half face. If vertex is found return true.
            }
        }
        
    } // end axis x checks
    else if (split_axis == Axis::y) { // This is the axis of splitXZ
        // use twin half faces of {v1,v2,v3,v4}, {v2,v3,v6,v7}, {v1,v4,v5,v8}, {v5,v6,v7,v8}
        // to check neighbouring cuboids
        hf1 = Twin(halfFace(cuboid_id, 0)); // bottom half face
        hf2 = Twin(halfFace(cuboid_id, 1)); // upper half face
        hf3 = Twin(halfFace(cuboid_id, 3)); // right half face
        hf4 = Twin(halfFace(cuboid_id, 5)); // left half face
        
        // check twin hf1
        if (!hf1.isBorder()) {
            if (hf1.isSubdivided()) {
                // TODO: traverse tree and get the half faces that are required
            }
            // getting the requried half faces in the bottom cuboid
            const halfFace hf1_right = halfFace(hf1.getCuboid(), 3);
            const halfFace hf1_left = halfFace(hf1.getCuboid(), 5);

            // also checking twin half faces of the half faces in the bottom cuboid
            const halfFace hf1_right_twin = Twin(hf1_right);
            const halfFace hf1_left_twin = Twin(hf1_left);

            if (hf1_right.isSubdivided()) {
                // TODO: check whether hf1_front has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            if (hf1_left.isSubdivided()) {
                // TODO: check whether hf1_back has a split that contains vertex vref.
                //       if so then return true
                // TODO: SubFaceIterator iter = sft.find();
            }
            // check borders for the twins only as there are only cases where the twins can be borders
            // check twins only if there are not part of the border
            if (!hf1_right_twin.isBorder()) {
                if (hf1_right_twin.isSubdivided()) {
                    // TODO: check whether hf1_front has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
            if (!hf1_left_twin.isBorder()) {
                if (hf1_left_twin.isSubdivided()) {
                    // TODO: check whether hf1_back has a split that contains vertex vref.
                    //       if so then return true
                    // TODO: SubFaceIterator iter = sft.find();
                }
            }
        } // checks hf1 ends here

        // TODO:

    } // end axis y checks
    else if (split_axis == Axis::z) { // This is the axis of splitXY
        // use twin half faces of {v2,v3,v6,v7}, {v1,v2,v5,v6}, {v1,v4,v5,v8}, {v1,v2,v3,v4}
        // to check neighbouring cuboids
        hf1 = Twin(halfFace(cuboid_id, 2)); // back half face
        hf2 = Twin(halfFace(cuboid_id, 3)); // right half face
        hf3 = Twin(halfFace(cuboid_id, 4)); // front half face
        hf4 = Twin(halfFace(cuboid_id, 5)); // left half face 

        // TODO:

    } // end axis z checks

    // if none of the checks returned true, then vertex does not exist yet
    return false;
}

/**
* This method adds 6 half faces to the new cuboid. 
*/
void Mesh::addHalfFaces(const uint32_t cuboid_id, const Axis split_axis) {
    if (split_axis == Axis::x) {


        // For the new cuboid point the lower face to the top face of the old element
        F2f.push_back(halfFace(cuboid_id, 1));

        // For now just push back the twins of the original element
        F2f.push_back(Twin(halfFace(cuboid_id, 1)));
        F2f.push_back(Twin(halfFace(cuboid_id, 2)));
        F2f.push_back(Twin(halfFace(cuboid_id, 3)));
        F2f.push_back(Twin(halfFace(cuboid_id, 4)));
        F2f.push_back(Twin(halfFace(cuboid_id, 5)));
        
    }
    else if (split_axis == Axis::y) {

    }
    else if (split_axis == Axis::z) {

    }
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

    // Check whether the new vertices already exist in the vectors array.
    uint32_t v1_idx = vertices.size();
    uint32_t v2_idx = v1_idx + 1;
    uint32_t v3_idx = v2_idx + 2;
    uint32_t v4_idx = v3_idx + 3;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        vertices.push_back(v1_new);
        v2_idx--;
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        vertices.push_back(v2_new);
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        vertices.push_back(v3_new);
        v4_idx--;
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        vertices.push_back(v4_new);
    }

    // Update old cuboid vertices
    Cuboid old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(old_cuboid);

    old_cuboid.v5 = v1_idx;
    old_cuboid.v6 = v2_idx;
    old_cuboid.v7 = v3_idx;
    old_cuboid.v8 = v4_idx;

    // Update new cuboid vertices
    Cuboid new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_idx;
    new_cuboid.v2 = v2_idx;
    new_cuboid.v3 = v3_idx;
    new_cuboid.v4 = v4_idx;

    addHalfFaces(cuboid_id, Axis::z);

    // TODO: Update V2f half faces 

    // TODO: Update the twin faces (mark them as subdivided).

    return new_cuboid_id;
}


uint32_t Mesh::SplitAlongYZ(uint32_t cuboid_id, float x_split) {
    // border checks
    if ((x_split <= vertices[cuboids[cuboid_id].v1].x) || x_split >= vertices[cuboids[cuboid_id].v2].x) {
        return -1; // Splitpoint not in cuboid
    }

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v2];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v3];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v4];

    // All the new vertices
    const Vertex v1_new = { x_split, v1_old.y, v1_old.z };
    const Vertex v2_new = { x_split, v2_old.y, v2_old.z };
    const Vertex v3_new = { x_split, v3_old.y, v3_old.z };
    const Vertex v4_new = { x_split, v4_old.y, v4_old.z };

    uint32_t v1_idx = vertices.size();
    uint32_t v2_idx = v1_idx + 1;
    uint32_t v3_idx = v2_idx + 2;
    uint32_t v4_idx = v3_idx + 3;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        vertices.push_back(v1_new);
        v2_idx--;
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        vertices.push_back(v2_new);
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        vertices.push_back(v3_new);
        v4_idx--;
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        vertices.push_back(v4_new);
    }

     // Update old cuboid vertices
    const uint32_t new_cuboid_id = cuboids.size();
    Cuboid& old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(old_cuboid);

    old_cuboid.v1 = v1_idx;
    old_cuboid.v4 = v2_idx;
    old_cuboid.v8 = v3_idx;
    old_cuboid.v5 = v4_idx;

    // Update new cuboid vertices
    Cuboid &new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v2 = v1_idx;
    new_cuboid.v3 = v2_idx;
    new_cuboid.v7 = v3_idx;
    new_cuboid.v6 = v4_idx;

    addHalfFaces(cuboid_id, Axis::x);

    // TODO: Update V2f half faces 

    // TODO: Update the twin faces (mark them as subdivided).
    
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
    const Vertex v3_old = vertices[cuboids[cuboid_id].v3];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v4];

    // All the new vertices
    const Vertex v1_new = { v1_old.x, y_split, v1_old.z };
    const Vertex v2_new = { v2_old.x, y_split, v2_old.z };
    const Vertex v3_new = { v3_old.x, y_split, v3_old.z };
    const Vertex v4_new = { v4_old.x, y_split, v4_old.z };


    uint32_t v1_idx = vertices.size();
    uint32_t v2_idx = v1_idx + 1;
    uint32_t v3_idx = v2_idx + 2;
    uint32_t v4_idx = v3_idx + 3;

    if (!mergeVertexIfExists(v1_new, v1_idx)) {
        vertices.push_back(v1_new);
        v2_idx--;
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v2_new, v2_idx)) {
        vertices.push_back(v2_new);
        v3_idx--;
        v4_idx--;
    }
    if (!mergeVertexIfExists(v3_new, v3_idx)) {
        vertices.push_back(v3_new);
        v4_idx--;
    }
    if (!mergeVertexIfExists(v4_new, v4_idx)) {
        vertices.push_back(v4_new);
    }

     // Update old cuboid vertices
    const uint32_t new_cuboid_id = cuboids.size();
    Cuboid old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(old_cuboid);
    old_cuboid.v1 = v1_idx;
    old_cuboid.v2 = v2_idx;
    old_cuboid.v6 = v3_idx;
    old_cuboid.v5 = v4_idx;

    // Update new cuboid vertices
    Cuboid new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v4 = v1_idx;
    new_cuboid.v3 = v2_idx;
    new_cuboid.v7 = v3_idx;
    new_cuboid.v8 = v4_idx;

    addHalfFaces(cuboid_id, Axis::y);

    // TODO: Update V2f half faces 

    // TODO: Update the twin faces (mark them as subdivided).

    return new_cuboid_id;
}

Mesh::~Mesh() {

}