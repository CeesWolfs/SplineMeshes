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
    SubFaceTree(/* args */) = default;
    static uint32_t toNodeIndex(halfFace from) {return from.id >> 3;}
    SubFaceIterator find(halfFace start_node, halfFace toFind, Vertex toFindmiddle);
    halfFace splitHalfFace(const halfFace start_node, const halfFace twin, const Axis split_axis ,const Vertex split_point,const halfFace lower,const halfFace higher);
    void removeNode(uint32_t node_index, std::vector<halfFace>& F2f);
    // Obtain an iterator to iterate through the subHalfFace tree starting at the start node
    SubFaceIterator begin(halfFace start_node);
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
    halfFace toIndex() const {
        return halfFace(node_index, at_lower ? 6 : 7);
    }
    halfFace& operator*() const {
        return (at_lower ? tree.nodes[node_index].lower_child : tree.nodes[node_index].top_child);
    }
    bool operator==(const SubFaceIterator& other) const {
        return other.node_index == node_index && other.at_lower == at_lower;
    }
    bool operator!=(const SubFaceIterator& other) const {
        return other.node_index != node_index || other.at_lower != at_lower;
    }
    SubFaceIterator& operator++() {
        // If we are at the higher child of a node
        if (!at_lower) {
            if (!tree.nodes[node_index].parent.isSubdivided()) {
                // If we cannot move up the chain anymore, we have come to the end
                node_index = -1;
                return *this;
            }
            // Move up the chain until we are no longer at a higher element
            while (isHigher(tree.nodes[node_index].parent)) {
                node_index = tree.toNodeIndex(tree.nodes[node_index].parent);
                if (!tree.nodes[node_index].parent.isSubdivided()) {
                    // If we cannot move up the chain anymore, we have come to the end
                    node_index = -1;
                    return *this;
                }
            }
            node_index = tree.toNodeIndex(tree.nodes[node_index].parent);
        }
        // Lower child so move to the right
        at_lower = false;
        // The right points to a new node, follow down
        if (tree.nodes[node_index].top_child.isSubdivided()) {
            node_index = tree.toNodeIndex(tree.nodes[node_index].top_child);
            // Move down until lower child no longer split
            while (tree.nodes[node_index].lower_child.isSubdivided()) {
                node_index = tree.toNodeIndex(tree.nodes[node_index].lower_child);
            }
            // If we have moved down we start at lower again
            at_lower = true;
        }
        return *this;
    }
};
#endif // !_SUBFACETREE_HPP

