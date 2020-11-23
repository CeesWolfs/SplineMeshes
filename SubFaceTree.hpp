#ifndef _SUBFACETREE_HPP
#define _SUBFACETREE_HPP
#include "Types.hpp"
#include <cassert>
#include <vector>
#include <type_traits>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>

typedef struct _node
{
    halfFace parent;
    float split_coord;
    Axis split_axis;
    halfFace lower_child;
    halfFace top_child;
} Node;
class SubFaceIterator;

/*  SubFaceTree class, stores all subfaces in the mesh
*   As such multiple trees are actually stored in the object
*   An iterator for a specific tree can be obtained via the begin method
*/
class SubFaceTree
{
private:
    
public:
    std::vector<Node> nodes; 
    SubFaceTree(/* args */);
    static uint32_t toNodeIndex(halfFace from) {return from.id >> 3;}
    auto find(halfFace start_node, halfFace toFind, Vertex toFindmiddle);
    void splitHalfFace(halfFace start_node, halfFace toSplit, Axis split_axis, Vertex split_point, halfFace lower, halfFace higher);
    void removeNode(uint32_t node_index, std::vector<halfFace>& F2f);
    // Obtain an iterator to iterate through the subHalfFace tree starting at the start node
    SubFaceIterator begin(uint32_t start_node_index);
    SubFaceIterator end();
    ~SubFaceTree();
};

static inline bool isHigher(halfFace face) {
    return face.getLocalId() == 7;
}

class SubFaceIterator
{
private:
    SubFaceTree& tree;
    uint32_t node_index;
    bool at_lower;
public:
    SubFaceIterator (SubFaceTree& tree, uint32_t index, bool lower) : tree{tree}, node_index{index}, at_lower{lower} {}
    halfFace& operator*() const {
        return (at_lower ? tree.nodes[node_index].lower_child : tree.nodes[node_index].top_child);
    }
    const bool operator==(const SubFaceIterator& other) const {
        return other.node_index == node_index && other.at_lower == at_lower;
    }
    SubFaceIterator& operator++() {
        if (at_lower) {
            at_lower = false;
            if(tree.nodes[node_index].top_child.isSubdivided()) {
                node_index = tree.toNodeIndex(tree.nodes[node_index].top_child);
                // Move down until lower child no longer split
                while(tree.nodes[node_index].lower_child.isSubdivided()) {
                    node_index = tree.toNodeIndex(tree.nodes[node_index].lower_child);
                }
            }
            return *this;
        }
        // We are already at the higher element, move up the chain
        while(isHigher(tree.nodes[node_index].parent)) {
            node_index = tree.toNodeIndex(tree.nodes[node_index].parent);
        }
        if(tree.nodes[node_index].parent.isSubdivided()) return tree.end();
    }
};
#endif // !_SUBFACETREE_HPP

