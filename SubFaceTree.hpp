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

typedef std::pair<halfFace, halfFace> HalfFacePair;
template<typename TreeType>
class SubFaceIterator;

/*  SubFaceTree class, stores all subfaces in the mesh
*   The Trees are a kind of  2D adaptive KD-trees where levels can be split at any axis
*   The difference with a regular KD-tree true is that we now care about the boxes, not about the points
*   As Multiple trees are actually stored in the object
*   An iterator for a specific tree can be obtained via the begin method
*/
class SubFaceTree
{
private:
    void updateSubTreeTwins(const halfFace head, const halfFace old_hf, const halfFace new_hf, const Vertex& split_point, std::vector<halfFace>& F2f);
public:
    void updateParent(const halfFace node, halfFace new_parent) {if(!node.isSubdivided()) return; nodes[toNodeIndex(node)].parent = new_parent;}
    std::vector<Node> nodes; 
    SubFaceTree(/* args */) = default;
    SubFaceTree(const SubFaceTree&) = delete; // prevent expensive accidental copies
    static uint32_t toNodeIndex(halfFace from) {return from.id >> 3;}
    SubFaceIterator<SubFaceTree> find(halfFace start_node, const halfFace toFind, const Vertex& toFindmiddle);

    /* 
        [description] searches the tree for the vertex that we need to find starting from start_node half face.
        [returns] true if vertex is found, otherwise false. Also returns the lower halfFace within the vertex is found
    */
    bool findVertex(halfFace start_node, const Vertex& vertexToFind) const;
    bool findVertexBorder(halfFace start_node, const Vertex& vertexToFind, const Axis splitAxis, halfFace& found) const;
    halfFace splitHalfFace(const halfFace start_node, const halfFace twin, const Axis split_axis, const Vertex& split_point,const halfFace lower,const halfFace higher);
    // Split a SubFaceTree in two along a split, returns the two start nodes, also splits twin faces automatically if necassary
    HalfFacePair splitTree(const halfFace tree_head, const Axis split_axis, const Vertex& split_point, const halfFace lower, const halfFace higher, std::vector<halfFace>& F2f);
    void removeNode(uint32_t node_index, std::vector<halfFace>& F2f);
    // Obtain an iterator to iterate through the subHalfFace tree starting at the start node
    SubFaceIterator<SubFaceTree> begin(halfFace start_node);
    SubFaceIterator<SubFaceTree> end();
    // Obtain a constant iterator to iterate through the subHalfFace tree starting at the start node
    SubFaceIterator<const SubFaceTree> cbegin(halfFace start_node) const;
    SubFaceIterator<const SubFaceTree> cend() const;
    ~SubFaceTree();
};

static inline bool isHigher(halfFace face) {
    return face.getLocalId() == 7;
}

template<typename TreeType>
class SubFaceIterator
{
private:
    TreeType* tree;
    uint32_t node_index;
    bool at_lower;
public:
    SubFaceIterator(TreeType* tree, uint32_t index, bool lower) : tree{ tree }, node_index{ index }, at_lower{ lower } {}
    halfFace toIndex() const {
        return halfFace(node_index, at_lower ? 6 : 7);
    }
    auto& operator*() const {
        return (at_lower ? tree->nodes[node_index].lower_child : tree->nodes[node_index].top_child);
    }
    bool operator==(const SubFaceIterator& other) const {
        return other.node_index == node_index && other.at_lower == at_lower;
    }
    bool operator!=(const SubFaceIterator& other) const {
        return other.node_index != node_index || other.at_lower != at_lower;
    }
    auto& operator++() {
        // If we are at the higher child of a node
        if (!at_lower) {
            if (!tree->nodes[node_index].parent.isSubdivided()) {
                // If we cannot move up the chain anymore, we have come to the end
                node_index = -1;
                return *this;
            }
            // Move up the chain until we are no longer at a higher element
            while (isHigher(tree->nodes[node_index].parent)) {
                node_index = tree->toNodeIndex(tree->nodes[node_index].parent);
                if (!tree->nodes[node_index].parent.isSubdivided()) {
                    // If we cannot move up the chain anymore, we have come to the end
                    node_index = -1;
                    return *this;
                }
            }
            node_index = tree->toNodeIndex(tree->nodes[node_index].parent);
        }
        // Lower child so move to the right
        at_lower = false;
        // The right points to a new node, follow down
        if (tree->nodes[node_index].top_child.isSubdivided()) {
            node_index = tree->toNodeIndex(tree->nodes[node_index].top_child);
            // Move down until lower child no longer split
            while (tree->nodes[node_index].lower_child.isSubdivided()) {
                node_index = tree->toNodeIndex(tree->nodes[node_index].lower_child);
            }
            // If we have moved down we start at lower again
            at_lower = true;
        }
        return *this;
    }
};

#endif // !_SUBFACETREE_HPP

