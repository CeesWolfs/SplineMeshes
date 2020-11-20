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
uint32_t Mesh::SplitAlongXY(uint32_t elem_id, float z_split) {
    const uint32_t new_elem_id = cuboids.size();
    // All the new vertices
    const Vertex v1_new = { vertices[cuboids[elem_id].v1].x, vertices[cuboids[elem_id].v1].y, z_split };
    const Vertex v2_new = { vertices[cuboids[elem_id].v2].x, vertices[cuboids[elem_id].v2].y, z_split };
    const Vertex v3_new = { vertices[cuboids[elem_id].v3].x, vertices[cuboids[elem_id].v3].y, z_split };
    const Vertex v4_new = { vertices[cuboids[elem_id].v4].x, vertices[cuboids[elem_id].v4].y, z_split };
    // For the new cuboid point the lower face to the top face of the old element
    F2f.push_back(halfFace(elem_id, 1));
    // For now just push back the twins of the original element
    F2f.push_back(Twin(halfFace(elem_id, 1)));
    F2f.push_back(Twin(halfFace(elem_id, 2)));
    F2f.push_back(Twin(halfFace(elem_id, 3)));
    F2f.push_back(Twin(halfFace(elem_id, 4)));
    F2f.push_back(Twin(halfFace(elem_id, 5)));
}

/**
 * Create a new cuboid to the left of split line, and let its right face point to left face right element.
 * Split the four original faces in two, update all twin faces.
 * Create 4 new vertices, merge if vertex already exists in neigboring element.
 * 
*/
uint32_t Mesh::SplitAlongYZ(uint32_t elem_id, float x_split) {
    const uint32_t new_elem_id = cuboids.size();
    // All the new vertices
    const Vertex v1_new = { vertices[cuboids[elem_id].v1].x, vertices[cuboids[elem_id].v1].y, x_split };
    const Vertex v2_new = { vertices[cuboids[elem_id].v2].x, vertices[cuboids[elem_id].v2].y, x_split };
    const Vertex v3_new = { vertices[cuboids[elem_id].v3].x, vertices[cuboids[elem_id].v3].y, x_split };
    const Vertex v4_new = { vertices[cuboids[elem_id].v4].x, vertices[cuboids[elem_id].v4].y, x_split };

    // TODO:
}

/**
 * w.r.t XZ plane, create a new cuboid on the side of set of vertices with highest coordinates split line, 
 * and let its right face point to left face right element.
 * Split the four original faces in two, update all twin faces.
 * Create 4 new vertices, merge if vertex already exists in neigboring element.
 * 
*/
uint32_t Mesh::SplitAlongXZ(uint32_t elem_id, float y_split) {
    // TODO
}