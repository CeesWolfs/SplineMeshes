#include "Mesh.hpp"

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
    cuboids.push_back({0,1,2,3,4,5,6,7});

    // Add 6 border faces
    for (int i=0;i<6;i++) {
        F2f.push_back(halfFace(border_id));
        V2f.push_back({ 0,i });
    }
}

/**
 * Returns the twin half face of the given half face "hf"
*/
halfFace Mesh::Twin(const halfFace hf) const {
    // TODO: maybe use F2f.at(hf.id) instead ? So that also out-of-range lookups are checked by the vector. 
    return F2f[hf.id];
}

/**
 * Create a new cuboid above the split line, and let its bottom face point to top face bottom element
 * Split the four original faces in two, update all twin faces
 * Create 4 new vertices, merge if vertex already exists in neigboring element
 * 
*/
uint32_t Mesh::SplitAlongXY(uint32_t cuboid_id, float z_split) {

    // border checks
    if ((z_split <= vertices[cuboids[cuboid_id].v1].z) || z_split >= vertices[cuboids[cuboid_id].v5].z) {
			return -1; // Splitpoint not in cuboid
	}

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

    // if a vertex is not available yet in the verties vector, then add it.
    bool notcontains = std::find(vertices.begin(), vertices.end(), v1_new) == vertices.end();

    // Update old cuboid vertices
    uint32_t v1_new_ref = vertices.size();
    uint32_t v2_new_ref = v1_new_ref + 1;
    uint32_t v3_new_ref = v1_new_ref + 2;
    uint32_t v4_new_ref = v1_new_ref + 3;

    if (notcontains) {
        vertices.push_back(v1_new);
        vertices.push_back(v2_new);
        vertices.push_back(v3_new);
        vertices.push_back(v4_new);
    } else {
        return -1;
    }

    const uint32_t new_cuboid_id = cuboids.size();
    Cuboid old_cuboid = cuboids[cuboid_id];
    cuboids.push_back(old_cuboid);

    old_cuboid.v5 = v1_new_ref;
    old_cuboid.v6 = v2_new_ref;
    old_cuboid.v7 = v3_new_ref;
    old_cuboid.v8 = v4_new_ref;

    // Update new cuboid vertices
    Cuboid new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_new_ref;
    new_cuboid.v2 = v2_new_ref;
    new_cuboid.v3 = v3_new_ref;
    new_cuboid.v4 = v4_new_ref;

    // For the new cuboid point the lower face to the top face of the old element
    F2f.push_back(halfFace(cuboid_id, 1));

    // For now just push back the twins of the original element
    F2f.push_back(Twin(halfFace(cuboid_id, 1)));
    F2f.push_back(Twin(halfFace(cuboid_id, 2)));
    F2f.push_back(Twin(halfFace(cuboid_id, 3)));
    F2f.push_back(Twin(halfFace(cuboid_id, 4)));
    F2f.push_back(Twin(halfFace(cuboid_id, 5)));

    // TODO: Update V2f half faces 

    // TODO: Update the twin faces with the correct vertices.

    return new_cuboid_id;
}

/**
 * Create a new cuboid to the left of split line, and let its right face point to left face right element.
 * Split the four original faces in two, update all twin faces.
 * Create 4 new vertices, merge if vertex already exists in neigboring element.
 * 
*/
uint32_t Mesh::SplitAlongYZ(uint32_t cuboid_id, float x_split) {
    const uint32_t new_cuboid_id = cuboids.size();
    
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

    // TODO:
}

/**
 * w.r.t XZ plane, create a new cuboid on the side of set of vertices with highest coordinates split line, 
 * and let its right face point to left face right element.
 * Split the four original faces in two, update all twin faces.
 * Create 4 new vertices, merge if vertex already exists in neigboring element.
 * 
*/
uint32_t Mesh::SplitAlongXZ(uint32_t cuboid_id, float y_split) {
    const uint32_t new_cuboid_id = cuboids.size();
    
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

    // TODO:
}